/*
Unit tests for the connection smoothing in sv_smooth.c.

A tiny self-contained harness: each CHECK reports and counts a failure
instead of aborting, so one run shows everything that is wrong. The queue
is simulated with an array of arrival times, the way the server keeps it in
its delayed packet list.
*/

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "sv_smooth.h"

static int failures;
static int checks;

#define CHECK(cond) \
	do { \
		checks++; \
		if (!(cond)) { \
			failures++; \
			printf ("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
		} \
	} while (0)

#define CHECK_NEAR(a, b) CHECK (fabs ((a) - (b)) < 1e-9)

#define MS(x) ((x) / 1000.0)

/* A queue of arrival times, oldest first. */
typedef struct
{
	double arrived[64];
	int    count;
} queue_t;

static void queue_push (queue_t *q, double arrived)
{
	q->arrived[q->count++] = arrived;
}

static double queue_pop (queue_t *q)
{
	double head = q->arrived[0];

	memmove (q->arrived, q->arrived + 1, (q->count - 1) * sizeof (q->arrived[0]));
	q->count--;
	return head;
}

/* A packet arrives; returns 1 when it went straight through. */
static int arrive (smooth_t *s, queue_t *q, double now, const smooth_config_t *cfg)
{
	Smooth_Arrived (s, now, cfg);
	if (!q->count && Smooth_PassThrough (s, now, cfg))
		return 1;
	queue_push (q, now);
	return 0;
}

/* Releases whatever is due at `now`, the way SV_ReadPackets does. Returns
   how many packets were processed; stale ones are dropped first. */
static int release (smooth_t *s, queue_t *q, double now, const smooth_config_t *cfg, double *released_at)
{
	int processed = 0;

	while (q->count)
	{
		double arrived = q->arrived[0];

		if (Smooth_Stale (now, arrived, cfg))
		{
			queue_pop (q);
			Smooth_Dropped (s, now);
			continue;
		}
		if (!Smooth_Due (s, now))
			break;
		queue_pop (q);
		Smooth_Released (s, now, arrived, q->count > 0, q->count ? q->arrived[0] : 0, cfg);
		if (released_at)
			released_at[processed] = now;
		processed++;
	}
	return processed;
}

static void test_packets_on_schedule_pass_straight_through (void)
{
	smooth_config_t cfg;
	smooth_t s;
	queue_t q = {{0}, 0};

	Smooth_Config (&cfg, 10, 50, 200, 10);
	Smooth_Init (&s);
	CHECK (arrive (&s, &q, 1.000, &cfg));
	CHECK (arrive (&s, &q, 1.010, &cfg));
	CHECK (arrive (&s, &q, 1.025, &cfg));
	CHECK (q.count == 0);
}

static void test_early_packet_waits_for_its_slot (void)
{
	smooth_config_t cfg;
	smooth_t s;
	queue_t q = {{0}, 0};

	Smooth_Config (&cfg, 10, 50, 200, 10);
	Smooth_Init (&s);
	CHECK (arrive (&s, &q, 1.000, &cfg));
	/* 4 ms early: held until the slot at 1.010. */
	CHECK (!arrive (&s, &q, 1.006, &cfg));
	CHECK_NEAR (s.next_release, 1.010);
	CHECK (release (&s, &q, 1.009, &cfg, NULL) == 0);
	CHECK (release (&s, &q, 1.010, &cfg, NULL) == 1);
	/* The line stays paced relative to the previous release. */
	CHECK (!arrive (&s, &q, 1.012, &cfg));
	CHECK (release (&s, &q, 1.019, &cfg, NULL) == 0);
	CHECK (release (&s, &q, 1.020, &cfg, NULL) == 1);
	CHECK (s.dropped == 0);
}

static void test_burst_is_spread_and_catch_up_doubles_the_rate (void)
{
	/* 10 ms spacing until the backlog is over 50 ms old, then 5 ms. */
	static const double expected[12] = {
		1.010, 1.020, 1.030, 1.040, 1.050, 1.060,
		1.065, 1.070, 1.075, 1.080, 1.085, 1.090,
	};
	double released_at[64];
	smooth_config_t cfg;
	smooth_t s;
	queue_t q = {{0}, 0};
	double t;
	int i, n = 0;

	Smooth_Config (&cfg, 10, 50, 200, 10);
	Smooth_Init (&s);
	CHECK (arrive (&s, &q, 1.0, &cfg));
	for (i = 0; i < 12; i++)
		CHECK (!arrive (&s, &q, 1.0, &cfg));

	for (t = 1.001; q.count && t < 2.0; t += 0.001)
		n += release (&s, &q, t, &cfg, released_at + n);

	CHECK (n == 12);
	for (i = 0; i < 12; i++)
	{
		CHECK (fabs (released_at[i] - expected[i]) < 0.0005);
		if (fabs (released_at[i] - expected[i]) >= 0.0005)
			printf ("  packet %d released at %.4f, expected %.4f\n", i + 1, released_at[i], expected[i]);
	}
	CHECK (s.dropped == 0);
}

static void test_stale_packets_are_dropped_oldest_first (void)
{
	smooth_config_t cfg;
	smooth_t s;
	queue_t q = {{0}, 0};

	Smooth_Config (&cfg, 10, 50, 200, 10);
	Smooth_Init (&s);
	CHECK (arrive (&s, &q, 1.000, &cfg));
	CHECK (!arrive (&s, &q, 1.000, &cfg));
	CHECK (!arrive (&s, &q, 1.000, &cfg));
	CHECK (!arrive (&s, &q, 1.000, &cfg));
	CHECK (!arrive (&s, &q, 1.100, &cfg));

	/* Nothing was released for 250 ms: the first three are stale, the later one is not. */
	CHECK (release (&s, &q, 1.250, &cfg, NULL) == 1);
	CHECK (s.dropped == 3);
	CHECK (q.count == 0);
	CHECK (Smooth_SeriesCount (&s.drops, 1.250) == 3);
}

/* Runs `seconds` of a 77 packets/s client whose uplink only transmits every
   `slot` seconds (0: straight away), releasing exactly when due. Fills
   `sends` with the send times and returns how many there were. */
static int simulate (smooth_t *s, queue_t *q, const smooth_config_t *cfg, double t0, double seconds, double slot, double *sends, int max_sends)
{
	double client_interval = 1.0 / 77.0, next_send = t0, end = t0 + seconds;
	int n = 0;

	for (;;)
	{
		double arrival = slot > 0 ? t0 + slot * ceil ((next_send - t0) / slot - 1e-9) : next_send;

		if (q->count && s->next_release <= arrival)
		{
			double due = s->next_release;
			int released = release (s, q, due, cfg, NULL);

			while (released-- > 0 && n < max_sends)
				sends[n++] = due;
			continue;
		}
		if (arrival >= end)
			break;
		if (arrive (s, q, arrival, cfg) && n < max_sends)
			sends[n++] = arrival;
		next_send += client_interval;
	}
	return n;
}

static void test_interval_is_the_measured_rate_once_known (void)
{
	smooth_config_t cfg;
	smooth_t s;
	double t = 1.0;
	int i;

	Smooth_Config (&cfg, 13, 50, 200, 10);
	Smooth_Init (&s);
	for (i = 0; i < 32; i++)
	{
		Smooth_Arrived (&s, t, &cfg);
		t += 0.040;
	}
	/* 31 gaps so far: still the configured interval. */
	CHECK_NEAR (Smooth_Interval (&s, t, &cfg), 0.013);
	Smooth_Arrived (&s, t, &cfg);
	CHECK_NEAR (Smooth_Interval (&s, t, &cfg), 0.040 * 1.002);
	/* A stall is not a rate signal. */
	t += 0.5;
	Smooth_Arrived (&s, t, &cfg);
	CHECK_NEAR (Smooth_Interval (&s, t, &cfg), 0.040 * 1.002);
	/* Neither is a burst: 32 gaps of nothing leave the configured interval in place. */
	Smooth_Init (&s);
	for (i = 0; i <= 32; i++)
		Smooth_Arrived (&s, 1.0, &cfg);
	CHECK_NEAR (Smooth_Interval (&s, 1.0, &cfg), 0.013);
	/* Unsmoothed clients are not measured. */
	Smooth_Init (&s);
	for (i = 0; i < 100; i++)
		Smooth_Arrived (&s, 1.0 + i * 0.013, NULL);
	CHECK_NEAR (Smooth_Interval (&s, 2.3, &cfg), 0.013);
}

static void test_clumped_arrivals_leave_at_the_client_rate (void)
{
	/* 77 packets/s through an uplink with a 20 ms slot: after the rate is
	   measured and the schedule has locked on, every packet leaves on a slot,
	   the clumping is gone and the queue never runs dry. */
	static double sends[1024];
	smooth_config_t cfg;
	smooth_summary_t wait;
	smooth_t s;
	queue_t q = {{0}, 0};
	double t0 = 1.0, sum = 0, sumsq = 0, max = 0, mean, sd;
	int n, i, count = 0;

	Smooth_Config (&cfg, 13, 50, 200, 10);
	Smooth_Init (&s);
	n = simulate (&s, &q, &cfg, t0, 8.0, 0.020, sends, 1024);
	CHECK (n > 500);
	for (i = 1; i < n; i++)
	{
		double gap = (sends[i] - sends[i - 1]) * 1000.0;

		if (sends[i - 1] < t0 + 3.0)
			continue;
		count++;
		sum += gap;
		sumsq += gap * gap;
		if (gap > max)
			max = gap;
	}
	mean = sum / count;
	sd = sqrt (sumsq / count - mean * mean);
	CHECK (fabs (mean - 1000.0 / 77.0) < 0.1);
	CHECK (sd < 0.2);
	CHECK (max < 13.5);
	CHECK (s.dropped == 0);
	if (sd >= 0.2 || max >= 13.5)
		printf ("  gaps: mean %.3f sd %.3f max %.3f ms\n", mean, sd, max);
	/* The queue holds about one uplink slot, not more. */
	Smooth_SeriesSummary (&s.wait, t0 + 8.0, &wait);
	CHECK (wait.mean < 20.0);
}

static void test_backlog_drains_at_the_measured_rate (void)
{
	smooth_config_t cfg;
	smooth_t s;
	queue_t q = {{0}, 0};
	double t = 1.0, empty_at = 0, client_interval = 1.0 / 77.0;
	int i;

	Smooth_Config (&cfg, 13, 500, 1000, 10);
	Smooth_Init (&s);

	/* A stall delivers four packets at once, then the client is steady. */
	CHECK (arrive (&s, &q, t, &cfg));
	for (i = 0; i < 3; i++)
		CHECK (!arrive (&s, &q, t, &cfg));
	CHECK (q.count == 3);

	for (i = 4; i < 256; i++)
	{
		t += client_interval;
		while (q.count && s.next_release <= t)
			release (&s, &q, s.next_release, &cfg, NULL);
		arrive (&s, &q, t, &cfg);
		if (!q.count && !empty_at)
			empty_at = t;
	}
	/* 39 ms of slack goes at up to 10% of a slot per packet once the first
	   half-second period is over. */
	CHECK (empty_at > 0);
	CHECK (empty_at - 1.0 <= 1.5);
	CHECK (s.dropped == 0);
}

static void test_series_summary_and_window (void)
{
	smooth_series_t series;
	smooth_summary_t summary;

	memset (&series, 0, sizeof (series));
	Smooth_SeriesRecord (&series, 100.0, 10);
	Smooth_SeriesRecord (&series, 100.5, 12);
	Smooth_SeriesRecord (&series, 101.0, 14);
	Smooth_SeriesRecord (&series, 101.5, 16);

	Smooth_SeriesSummary (&series, 104.0, &summary);
	CHECK (summary.count == 4);
	CHECK_NEAR (summary.mean, 13.0);
	CHECK_NEAR (summary.stddev, sqrt (5.0));
	CHECK_NEAR (summary.max, 16.0);

	/* The second the samples were taken in falls out of the window after five seconds. */
	Smooth_SeriesSummary (&series, 104.9, &summary);
	CHECK (summary.count == 4);
	Smooth_SeriesSummary (&series, 105.0, &summary);
	CHECK (summary.count == 2);
	Smooth_SeriesSummary (&series, 106.0, &summary);
	CHECK (summary.count == 0);
	CHECK (Smooth_SeriesCount (&series, 104.9) == 4);

	/* A bucket is reused once its second comes round again. */
	Smooth_SeriesRecord (&series, 105.2, 1);
	Smooth_SeriesSummary (&series, 105.2, &summary);
	CHECK (summary.count == 3);
	CHECK_NEAR (summary.max, 16.0);
}

static void test_unsmoothed_packets_are_measured (void)
{
	smooth_summary_t summary;
	smooth_t s;

	Smooth_Init (&s);
	Smooth_Arrived (&s, 1.000, NULL);
	Smooth_Sent (&s, 1.000, 1.000);
	Smooth_Arrived (&s, 1.013, NULL);
	Smooth_Sent (&s, 1.013, 1.013);
	Smooth_Arrived (&s, 1.033, NULL);
	Smooth_Sent (&s, 1.040, 1.033);

	CHECK (Smooth_SeriesCount (&s.arrivals, 1.040) == 3);
	Smooth_SeriesSummary (&s.arrival_gap, 1.040, &summary);
	CHECK (summary.count == 2);
	CHECK_NEAR (summary.max, 20.0);
	Smooth_SeriesSummary (&s.wait, 1.040, &summary);
	CHECK (summary.count == 3);
	CHECK_NEAR (summary.max, 7.0);
	Smooth_SeriesSummary (&s.send_gap, 1.040, &summary);
	CHECK_NEAR (summary.max, 27.0);
}

static void test_duplicates_are_counted_apart (void)
{
	/* The owner keeps duplicates out of the packet calls; here only their
	   count is recorded and the packet statistics stay untouched. */
	smooth_summary_t summary;
	smooth_t s;

	Smooth_Init (&s);
	Smooth_Arrived (&s, 1.000, NULL);
	Smooth_Sent (&s, 1.000, 1.000);
	Smooth_Duplicate (&s, 1.000);
	Smooth_Arrived (&s, 1.013, NULL);
	Smooth_Sent (&s, 1.013, 1.013);
	Smooth_Duplicate (&s, 1.013);

	CHECK (Smooth_SeriesCount (&s.arrivals, 1.013) == 2);
	CHECK (Smooth_SeriesCount (&s.dupes, 1.013) == 2);
	Smooth_SeriesSummary (&s.arrival_gap, 1.013, &summary);
	CHECK (summary.count == 1);
	CHECK_NEAR (summary.mean, 13.0);
	Smooth_SeriesSummary (&s.send_gap, 1.013, &summary);
	CHECK (summary.count == 1);
	CHECK_NEAR (summary.mean, 13.0);
	CHECK (Smooth_SeriesCount (&s.dupes, 12.0) == 0);
}

static void test_config_clamps_nonsense_values (void)
{
	smooth_config_t cfg;

	Smooth_Config (&cfg, 0, -5, -1, -1);
	CHECK_NEAR (cfg.interval, 0.0001);
	CHECK_NEAR (cfg.catchup, 0);
	CHECK_NEAR (cfg.max_delay, 0);
	CHECK_NEAR (cfg.drain, 0);
	Smooth_Config (&cfg, 12.5, 50, 200, 10);
	CHECK_NEAR (cfg.interval, 0.0125);
	CHECK_NEAR (cfg.catchup, 0.05);
	CHECK_NEAR (cfg.max_delay, 0.2);
	CHECK_NEAR (cfg.drain, 0.1);
	Smooth_Config (&cfg, 5000, 0, 0, 500);
	CHECK_NEAR (cfg.interval, 1.0);
	CHECK_NEAR (cfg.drain, 1.0);
}

int main (void)
{
	test_packets_on_schedule_pass_straight_through ();
	test_early_packet_waits_for_its_slot ();
	test_burst_is_spread_and_catch_up_doubles_the_rate ();
	test_stale_packets_are_dropped_oldest_first ();
	test_interval_is_the_measured_rate_once_known ();
	test_clumped_arrivals_leave_at_the_client_rate ();
	test_backlog_drains_at_the_measured_rate ();
	test_series_summary_and_window ();
	test_unsmoothed_packets_are_measured ();
	test_duplicates_are_counted_apart ();
	test_config_clamps_nonsense_values ();

	printf ("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
