/*
Connection smoothing: a per-client jitter buffer that enforces a minimum interval
between a client's packets before the server processes them.

QuakeWorld clients send a fixed number of packets per second (77, one every
13 ms), and links such as cellular uplinks deliver them in clumps. Each packet
is processed no earlier than one interval after the previous one, so packets
that arrive on time pass straight through and only the ones arriving early
(right behind a late one) wait. The interval is the client's own send
interval, measured from its arrivals over the last few seconds (the configured
interval serves until enough have been seen), so the releases run at the
client's rate and the queue holds only what the clumping needs. Whatever slack
builds up beyond that, because the estimate is a touch long or a stall let the
queue grow, is found as the smallest wait over the last second and shaved off
the following slots a little at a time. Once a backlog grows past a threshold
the queue drains at double rate to recover from a latency burst, and packets
that have waited past a hard limit are discarded, oldest first, so the client
skips ahead rather than falling ever further behind.

Clients may send every packet twice for loss protection (cl_c2sdupe), and
a jittery link may deliver packets out of order. A packet whose netchan
sequence number does not exceed the highest seen so far is either such a
duplicate or a straggler; a later packet has already arrived, so it serves
no purpose here and the owner discards it, keeping it out of the calls below
except Smooth_Duplicate, which only counts it.

This file does not depend on the rest of the server, so it can be unit
tested on its own (see tests/test_smooth.c). The caller owns the packet
queue; the functions here only decide when its head may go and keep the
statistics. Times are absolute seconds on any monotonic clock; a time of 0
means "never".
*/

#ifndef SV_SMOOTH_H
#define SV_SMOOTH_H

#define SMOOTH_WINDOW_SECONDS 5

typedef struct
{
	double interval;  /* spacing between processed packets until the client's rate is measured, seconds */
	double catchup;   /* backlog age above which the queue drains at double rate */
	double max_delay; /* packets queued longer than this are dropped */
	double drain;     /* the most a slot may be shortened to drain slack, as a fraction of the interval */
} smooth_config_t;

/* Samples from one second. */
typedef struct
{
	int    second;  /* floor(time) of the samples held; -1 when unused */
	int    count;
	double sum;
	double sumsq;
	double max;
} smooth_bucket_t;

/* Samples over the last SMOOTH_WINDOW_SECONDS, one bucket per second. */
typedef struct
{
	smooth_bucket_t buckets[SMOOTH_WINDOW_SECONDS];
} smooth_series_t;

typedef struct
{
	int    count;
	double mean;
	double stddev;
	double max;
} smooth_summary_t;

typedef struct
{
	int      active;       /* whether the owner is currently pacing this client; owner maintained */
	double   next_release; /* when the next queued packet may go; 0 when idle */
	double   last_arrival; /* 0 before the first packet */
	double   last_send;
	unsigned dropped;      /* over the life of the connection */
	unsigned last_sequence; /* highest netchan sequence seen, for spotting duplicates and stragglers; owner maintained */
	int      have_sequence;
	double   slack;        /* smallest wait of a release since period_start */
	int      have_slack;
	double   period_start;
	double   drain;        /* slack still to be shaved off coming slots, seconds */

	smooth_series_t arrival_gap; /* ms between packets arriving from the client */
	smooth_series_t send_gap;    /* ms between packets handed to the server */
	smooth_series_t wait;        /* ms each packet spent queued */
	smooth_series_t rate;        /* arrival gaps the rate is measured from; stalls left out */
	smooth_series_t arrivals;    /* one sample per packet; only the count matters */
	smooth_series_t dupes;       /* one sample per duplicate; only the count matters */
	smooth_series_t drops;       /* one sample per drop; only the count matters */
} smooth_t;

void Smooth_Init (smooth_t *s);
/* Builds a config from the console values, clamping nonsense. */
void Smooth_Config (smooth_config_t *cfg, double interval_ms, double catchup_ms, double max_delay_ms, double drain_percent);

/* A packet arrived from the client at `now`; `cfg` may be NULL when the
   client is not being smoothed (its rate is then not measured). */
void Smooth_Arrived (smooth_t *s, double now, const smooth_config_t *cfg);
/* The spacing between releases: the client's measured send interval once
   enough arrivals have been seen, the configured one before that. */
double Smooth_Interval (const smooth_t *s, double now, const smooth_config_t *cfg);
/* A copy of the previous packet arrived at `now`; statistics only. */
void Smooth_Duplicate (smooth_t *s, double now);
/* With the queue empty, whether a packet arriving at `now` may be processed
   at once. Claims the slot when it may; queue the packet otherwise. */
int  Smooth_PassThrough (smooth_t *s, double now, const smooth_config_t *cfg);
/* Whether the head of the queue may be processed at `now`. */
int  Smooth_Due (const smooth_t *s, double now);
/* The head, which arrived at `arrived`, was processed at `now`. When more
   packets are queued, `next_arrived` is the arrival time of the new head. */
void Smooth_Released (smooth_t *s, double now, double arrived, int more_queued, double next_arrived, const smooth_config_t *cfg);
/* Whether a packet that arrived at `arrived` has waited too long. */
int  Smooth_Stale (double now, double arrived, const smooth_config_t *cfg);
void Smooth_Dropped (smooth_t *s, double now);
/* Records a packet processed outside of pacing (statistics only). */
void Smooth_Sent (smooth_t *s, double now, double arrived);
/* Forgets the current slot, e.g. when the queue was flushed. */
void Smooth_Reset (smooth_t *s);

void Smooth_SeriesRecord (smooth_series_t *series, double now, double value);
void Smooth_SeriesSummary (const smooth_series_t *series, double now, smooth_summary_t *out);
int  Smooth_SeriesCount (const smooth_series_t *series, double now);

#endif /* SV_SMOOTH_H */
