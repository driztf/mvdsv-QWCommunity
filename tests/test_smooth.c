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
	Smooth_Arrived (s, now);
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

	Smooth_Config (&cfg, 10, 50, 200);
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

	Smooth_Config (&cfg, 10, 50, 200);
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

	Smooth_Config (&cfg, 10, 50, 200);
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

	Smooth_Config (&cfg, 10, 50, 200);
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

static void test_backlog_drains_when_interval_is_shorter_than_client_rate (void)
{
	/* Client sends every 13 ms; we release every 12 ms. */
	smooth_config_t cfg;
	smooth_t s;
	queue_t q = {{0}, 0};
	double t = 1.0, empty_at = 0;
	int i;

	Smooth_Config (&cfg, 12, 500, 1000);
	Smooth_Init (&s);

	/* A stall delivers four packets at once. */
	CHECK (arrive (&s, &q, t, &cfg));
	for (i = 0; i < 3; i++)
		CHECK (!arrive (&s, &q, t, &cfg));
	CHECK (q.count == 3);

	/* Steady 13 ms arrivals from then on: each release frees 1 ms of backlog. */
	for (i = 0; i < 60; i++)
	{
		double step;

		for (step = 0.001; step <= 0.013; step += 0.001)
			release (&s, &q, t + step, &cfg, NULL);
		t += 0.013;
		arrive (&s, &q, t, &cfg);
		if (!q.count && !empty_at)
			empty_at = t;
	}
	CHECK (empty_at > 0);
	CHECK (empty_at - 1.0 <= 40 * 0.013);
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

	Smooth_SeriesSummary (&series, 105.0, &summary);
	CHECK (summary.count == 4);
	CHECK_NEAR (summary.mean, 13.0);
	CHECK_NEAR (summary.stddev, sqrt (5.0));
	CHECK_NEAR (summary.max, 16.0);

	/* The second the samples were taken in falls out of the window after ten seconds. */
	Smooth_SeriesSummary (&series, 109.9, &summary);
	CHECK (summary.count == 4);
	Smooth_SeriesSummary (&series, 110.0, &summary);
	CHECK (summary.count == 2);
	Smooth_SeriesSummary (&series, 111.0, &summary);
	CHECK (summary.count == 0);
	CHECK (Smooth_SeriesCount (&series, 105.0) == 4);

	/* A bucket is reused once its second comes round again. */
	Smooth_SeriesRecord (&series, 110.2, 1);
	Smooth_SeriesSummary (&series, 110.2, &summary);
	CHECK (summary.count == 3);
	CHECK_NEAR (summary.max, 16.0);
}

static void test_unsmoothed_packets_are_measured (void)
{
	smooth_summary_t summary;
	smooth_t s;

	Smooth_Init (&s);
	Smooth_Arrived (&s, 1.000);
	Smooth_Sent (&s, 1.000, 1.000);
	Smooth_Arrived (&s, 1.013);
	Smooth_Sent (&s, 1.013, 1.013);
	Smooth_Arrived (&s, 1.033);
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
	Smooth_Arrived (&s, 1.000);
	Smooth_Sent (&s, 1.000, 1.000);
	Smooth_Duplicate (&s, 1.000);
	Smooth_Arrived (&s, 1.013);
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

	Smooth_Config (&cfg, 0, -5, -1);
	CHECK_NEAR (cfg.interval, 0.0001);
	CHECK_NEAR (cfg.catchup, 0);
	CHECK_NEAR (cfg.max_delay, 0);
	Smooth_Config (&cfg, 12.5, 50, 200);
	CHECK_NEAR (cfg.interval, 0.0125);
	CHECK_NEAR (cfg.catchup, 0.05);
	CHECK_NEAR (cfg.max_delay, 0.2);
	Smooth_Config (&cfg, 5000, 0, 0);
	CHECK_NEAR (cfg.interval, 1.0);
}

int main (void)
{
	test_packets_on_schedule_pass_straight_through ();
	test_early_packet_waits_for_its_slot ();
	test_burst_is_spread_and_catch_up_doubles_the_rate ();
	test_stale_packets_are_dropped_oldest_first ();
	test_backlog_drains_when_interval_is_shorter_than_client_rate ();
	test_series_summary_and_window ();
	test_unsmoothed_packets_are_measured ();
	test_duplicates_are_counted_apart ();
	test_config_clamps_nonsense_values ();

	printf ("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
