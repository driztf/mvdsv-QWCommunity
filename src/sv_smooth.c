/*
Connection smoothing, see sv_smooth.h.
*/

#include <math.h>
#include <string.h>

#include "sv_smooth.h"

#define MS(seconds) ((seconds) * 1000.0)

/* Arrival gaps needed before the measured rate replaces the configured
   interval, and the time they must span: a burst of packets arriving at
   once says nothing about the rate. */
#define MIN_RATE_SAMPLES 32
#define MIN_RATE_SPAN    1.0
/* The measured interval is stretched by this much: a slightly long interval
   only builds slack, which is drained, while a short one runs the queue dry
   and re-syncs the schedule to a clumped arrival. */
#define RATE_MARGIN      1.002
/* How often the slack in the queue is measured and scheduled for draining. */
#define SLACK_PERIOD     0.5
/* Slack is shaved off each slot by this share of what is left, so a lot of
   slack goes quickly and the last of it gently, between a floor (as a
   fraction of the interval) and the configured cap. */
#define DRAIN_SHARE      0.25
#define DRAIN_FLOOR      0.02

void Smooth_Init (smooth_t *s)
{
	int i;

	memset (s, 0, sizeof (*s));
	for (i = 0; i < SMOOTH_WINDOW_SECONDS; i++)
	{
		s->arrival_gap.buckets[i].second = -1;
		s->send_gap.buckets[i].second = -1;
		s->wait.buckets[i].second = -1;
		s->rate.buckets[i].second = -1;
		s->arrivals.buckets[i].second = -1;
		s->dupes.buckets[i].second = -1;
		s->drops.buckets[i].second = -1;
	}
}

void Smooth_Duplicate (smooth_t *s, double now)
{
	Smooth_SeriesRecord (&s->dupes, now, 0);
}

void Smooth_Config (smooth_config_t *cfg, double interval_ms, double catchup_ms, double max_delay_ms, double drain_percent)
{
	if (!(interval_ms > 0.1))
		interval_ms = 0.1; /* also catches NaN */
	if (interval_ms > 1000)
		interval_ms = 1000;
	if (!(catchup_ms >= 0))
		catchup_ms = 0;
	if (!(max_delay_ms >= 0))
		max_delay_ms = 0;
	if (!(drain_percent >= 0))
		drain_percent = 0;
	if (drain_percent > 100)
		drain_percent = 100;

	cfg->interval = interval_ms / 1000.0;
	cfg->catchup = catchup_ms / 1000.0;
	cfg->max_delay = max_delay_ms / 1000.0;
	cfg->drain = drain_percent / 100.0;
}

void Smooth_Arrived (smooth_t *s, double now, const smooth_config_t *cfg)
{
	if (s->last_arrival > 0)
	{
		double gap = now - s->last_arrival;

		Smooth_SeriesRecord (&s->arrival_gap, now, MS (gap));
		if (cfg && gap <= cfg->max_delay)
			Smooth_SeriesRecord (&s->rate, now, MS (gap));
	}
	s->last_arrival = now;
	Smooth_SeriesRecord (&s->arrivals, now, 0);
}

double Smooth_Interval (const smooth_t *s, double now, const smooth_config_t *cfg)
{
	smooth_summary_t rate;
	double interval;

	Smooth_SeriesSummary (&s->rate, now, &rate);
	if (rate.count < MIN_RATE_SAMPLES || rate.mean * rate.count < MS (MIN_RATE_SPAN))
		return cfg->interval;
	interval = rate.mean * RATE_MARGIN / 1000.0;
	return interval < 0.001 ? 0.001 : interval > 1.0 ? 1.0 : interval;
}

/* A packet was released having waited `wait`; keeps track of the slack and,
   once a period is over, schedules it for draining. */
static void Smooth_NoteRelease (smooth_t *s, double now, double wait, const smooth_config_t *cfg)
{
	if (!s->period_start)
		s->period_start = now;
	if (!s->have_slack || wait < s->slack)
		s->slack = wait;
	s->have_slack = 1;
	if (now - s->period_start >= SLACK_PERIOD)
	{
		s->drain += s->slack < cfg->catchup ? s->slack : cfg->catchup;
		s->have_slack = 0;
		s->period_start = now;
	}
}

void Smooth_Sent (smooth_t *s, double now, double arrived)
{
	if (s->last_send > 0)
		Smooth_SeriesRecord (&s->send_gap, now, MS (now - s->last_send));
	s->last_send = now;
	Smooth_SeriesRecord (&s->wait, now, MS (now - arrived));
}

int Smooth_PassThrough (smooth_t *s, double now, const smooth_config_t *cfg)
{
	if (s->next_release > now)
		return 0;

	s->next_release = now + Smooth_Interval (s, now, cfg);
	Smooth_NoteRelease (s, now, 0, cfg);
	Smooth_Sent (s, now, now);
	return 1;
}

int Smooth_Due (const smooth_t *s, double now)
{
	/* Slot times are sums of millisecond intervals; a microsecond of slack
	   keeps rounding from pushing a release to the next wakeup. */
	return now + 1e-6 >= s->next_release;
}

void Smooth_Released (smooth_t *s, double now, double arrived, int more_queued, double next_arrived, const smooth_config_t *cfg)
{
	/* Slots are counted from when the last one was due, not from now, so a
	   late wakeup catches up on the slots it missed. */
	double due = s->next_release > 0 ? s->next_release : now;
	double backlog = more_queued ? now - next_arrived : 0;
	double interval = Smooth_Interval (s, now, cfg);
	double slot = backlog > cfg->catchup ? interval / 2 : interval;
	double shave = s->drain * DRAIN_SHARE;

	Smooth_NoteRelease (s, now, now - arrived, cfg);
	if (shave < interval * DRAIN_FLOOR)
		shave = interval * DRAIN_FLOOR;
	if (shave > interval * cfg->drain)
		shave = interval * cfg->drain;
	if (shave > s->drain)
		shave = s->drain;
	s->drain -= shave;
	s->next_release = due + slot - shave;
	Smooth_Sent (s, now, arrived);
}

int Smooth_Stale (double now, double arrived, const smooth_config_t *cfg)
{
	return now - arrived > cfg->max_delay;
}

void Smooth_Dropped (smooth_t *s, double now)
{
	s->dropped++;
	Smooth_SeriesRecord (&s->drops, now, 0);
}

void Smooth_Reset (smooth_t *s)
{
	s->next_release = 0;
	s->have_slack = 0;
	s->period_start = 0;
	s->drain = 0;
}

static smooth_bucket_t *Smooth_Bucket (smooth_series_t *series, int second)
{
	smooth_bucket_t *bucket = &series->buckets[(unsigned) second % SMOOTH_WINDOW_SECONDS];

	if (bucket->second != second)
	{
		bucket->second = second;
		bucket->count = 0;
		bucket->sum = 0;
		bucket->sumsq = 0;
		bucket->max = 0;
	}
	return bucket;
}

void Smooth_SeriesRecord (smooth_series_t *series, double now, double value)
{
	smooth_bucket_t *bucket = Smooth_Bucket (series, (int) floor (now));

	bucket->count++;
	bucket->sum += value;
	bucket->sumsq += value * value;
	if (value > bucket->max)
		bucket->max = value;
}

/* Whether a bucket holds samples from the window ending at `now`. */
static int Smooth_BucketInWindow (const smooth_bucket_t *bucket, double now)
{
	int second = (int) floor (now);

	return bucket->second >= 0 && bucket->second <= second && bucket->second > second - SMOOTH_WINDOW_SECONDS;
}

void Smooth_SeriesSummary (const smooth_series_t *series, double now, smooth_summary_t *out)
{
	double sum = 0, sumsq = 0, max = 0, mean, variance;
	int count = 0, i;

	for (i = 0; i < SMOOTH_WINDOW_SECONDS; i++)
	{
		const smooth_bucket_t *bucket = &series->buckets[i];

		if (!Smooth_BucketInWindow (bucket, now))
			continue;
		count += bucket->count;
		sum += bucket->sum;
		sumsq += bucket->sumsq;
		if (bucket->max > max)
			max = bucket->max;
	}

	memset (out, 0, sizeof (*out));
	out->count = count;
	if (!count)
		return;

	mean = sum / count;
	variance = sumsq / count - mean * mean;
	out->mean = mean;
	out->stddev = variance > 0 ? sqrt (variance) : 0;
	out->max = max;
}

int Smooth_SeriesCount (const smooth_series_t *series, double now)
{
	int count = 0, i;

	for (i = 0; i < SMOOTH_WINDOW_SECONDS; i++)
		if (Smooth_BucketInWindow (&series->buckets[i], now))
			count += series->buckets[i].count;
	return count;
}
