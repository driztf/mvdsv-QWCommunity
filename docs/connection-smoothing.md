# Connection smoothing

A per-client jitter buffer on the server that re-spaces packets arriving from
clients on links that deliver them in clumps. This document describes the
behaviour, the reasoning behind each design decision, and how it was tested.
The implementation lives in `src/sv_smooth.c` (the decision logic, with no
server dependencies) and `src/sv_main.c` (the integration with the client
packet queue).

## The problem

A QuakeWorld client sends its movement commands at a fixed rate, 77 packets
per second, one every 1000/77 ms. Some links, notably cellular uplinks, do
not transmit continuously: they get a transmission opportunity only every so
often, and everything generated since the last opportunity leaves together.
The server then sees packets arrive in clumps: two or three at once, then a
gap. Each packet carries a movement command with its own frame time, so the
server runs the player's physics in fits and starts, and the player appears
to stutter or warp to everyone else, while their own prediction keeps
disagreeing with the server.

The client cannot fix this: the clumping happens after the packets leave it.
The server can, because it knows when each packet was generated (they arrive
at a steady rate on average) and can process them on that schedule instead of
as they land.

## What it does

A smoothed client's packets are processed no earlier than one *interval*
after the previous one. A packet that arrives on time is processed at once; a
packet that arrives early, right behind a late one, waits for its slot. The
queue used for this already existed: `sv_minping` keeps a delayed packet list
per client (`client_t.packets`), and smoothing reuses it, so no new buffers
or memory management were needed. A smoothed client is not also subject to
the `sv_minping` delay, since the two keep different clocks (see below).

Three refinements keep the queue healthy:

* **The interval is the client's own send rate**, measured from its arrivals.
* **Slack in the queue is drained** a little at a time.
* **A large backlog drains at double rate**, and packets that have waited too
  long are dropped, oldest first, so the client skips ahead instead of
  falling ever further behind.

Duplicate packets, which clients send for loss protection, and packets the
link delivers out of order are counted and discarded on arrival rather than
being treated as extra traffic.

## Design decisions and why

### Pace at the measured rate, not a fixed interval

The first version used a fixed interval slightly shorter than the client's
rate (12 ms against 13), on the reasoning that a backlog would then drain by
the difference on every packet. It did drain, but it also meant the queue
ran dry every dozen packets. Whenever the queue is empty, the next packet
passes straight through at its arrival time, and on a clumped link that
arrival is aligned to the uplink's slot, not to the client's schedule. So
once per drain cycle the schedule re-synchronised to a clumped arrival, with
one long gap.

A simulation of the schedule under perfect timing, with a 77 packets/s
client whose uplink transmits every 20 ms, showed this was the whole story.
Output gaps in ms:

| Schedule                                  | avg   | sd   | max  |
|-------------------------------------------|-------|------|------|
| 12 ms interval, exact wake-ups            | 12.99 | 1.72 | 16.0 |
| 12 ms interval, wake-ups on a 1 ms grid   | 12.99 | 1.72 | 16.0 |
| 12.8 ms interval                          | 12.99 | 0.44 | 14.0 |
| interval equal to the client rate         | 12.99 | 0.11 | 13.0 |

The measured 1.4 to 1.7 ms standard deviation on a live server sat on the
first row, and timer resolution (second row) made no difference, so the fix
was the interval itself. The interval is now the mean arrival gap over the
statistics window. Because the mean over hundreds of gaps is unbiased by the
clumping (the clumps only move packets within the window, not the total), it
converges on the client's true rate.

Two guards apply. Gaps longer than `sv_smooth_maxdelay` are left out of the
estimate, because a stall is not a rate signal and would otherwise bias the
interval long for the whole window. And the estimate is only trusted once 32
gaps spanning at least a second have been seen: a burst of packets arriving
at once produces dozens of zero gaps that say nothing about the rate. Until
then `sv_smooth_interval` is used, whose default is exactly 1000/77 ms.

The measured interval carries a 0.2 % margin. The two ways of being wrong
are not symmetric: an interval slightly too long only builds slack, which
the drain removes silently, while one slightly too short runs the queue dry
and re-syncs the schedule to a clumped arrival, which is the jitter this
whole mechanism exists to remove.

### Drain slack by shortening slots, sized by the smallest recent wait

With the schedule at the client's rate, the queue neither grows nor shrinks
on its own, so something has to remove whatever slack builds up: the margin
above, a stall that let packets pile up, or a phase set by an unlucky first
packet. The question is how to tell slack from the buffering the clumping
genuinely needs, since draining the latter would run the queue dry again.

The answer is the smallest wait among the releases of the last half second.
If every packet in that period waited at least *m* ms, the schedule can be
moved *m* ms earlier without any packet being due before it has arrived,
and that *m* is precisely the slack. A pass-through counts as a wait of
zero, so a period containing one has no slack to give.

Rather than jumping the schedule by *m* at once, which would itself be a
visible short gap, the slack is shaved off the following slots. Each slot
loses a quarter of what remains, which drains a large backlog quickly and
the last of it gently, floored at 2 % of a slot so it does not tail off
forever, and capped at `sv_smooth_drain` percent of a slot (default 10) so
recovery from a lag event finishes in well under a second without making
any single slot unreasonably short. The cap is a cvar because how fast to
recover is a matter of taste that server operators may want to tune.

### Catch-up and dropping

Above the slack drain sit two coarser mechanisms inherited from the first
version. A backlog older than `sv_smooth_catchup` ms (50) drains at double
rate: after a real latency burst the player is better served by getting
back in sync quickly than by having every queued command spread out. And a
packet that has waited longer than `sv_smooth_maxdelay` ms (200) is dropped,
oldest first, because by then its movement command is stale and processing
it would only delay the fresher ones behind it; the client's own prediction
already assumed those frames happened.

### Duplicate packets are discarded

Clients can send every packet twice (`cl_c2sdupe`) so that a single lost
datagram does not cost a frame. On the wire this is two identical datagrams
with the same netchan sequence number, microseconds apart. Treated naively,
such a client looks like it sends 154 packets/s, which no interval built
for 77 can ever drain; the first live test showed exactly this, with the
queue pinned in catch-up mode. A packet whose sequence number does not
exceed the highest seen so far is therefore recognised as a duplicate and
discarded on arrival, counted in the statistics but never queued or
processed. The copy exists to survive loss on the way to the server; once
the original has arrived it has done its job, and the netchan would reject
it as out of order anyway. The same test catches a packet the link reorders
behind a newer one: a plain check against only the previous packet's
sequence would let it through, where it would take a pacing slot and skew
the rate estimate, only for the netchan to reject it as out of order. A proxy in front of the server should instead pass copies on
with their originals, since there may still be loss on its own link to the
server, which is what qwfwd does.

### Who is smoothed

Smoothing is always available to clients; `sv_smooth` selects how it is
applied: `0` clients that opt in with `setinfo smooth 1`, `1` (the default)
everyone, with no way for a client to opt out, `2` everyone except clients
that opt out with `setinfo smooth 0`. Smoothing everyone is the default
because on a link that delivers packets on time it costs nothing (see
below), so most players never notice it, while the ones on poor links get
it without having to know it exists. A server can restrict it to players
who ask, or let players who know better switch it off. Clients can change
their setting while connected, and the queue is handed over cleanly when
they do.

### Clocks and wake-ups

The delayed packet queue was designed for `sv_minping` and stamps packets
with `realtime`, which stops while the server is paused. Smoothing stamps
them with `curtime`, the wall clock, because its schedule must keep pace
with the client's regardless of pause. When a client switches between the
two mechanisms the queue is drained first so the stamps never mix.

The server's main loop sleeps in `select()` for up to `sys_select_timeout`
when idle. A queued packet whose slot falls inside that sleep would be
processed late, so the timeout is shortened to the earliest pending slot
across all smoothed clients (`SV_SmoothSleepMs`). A late wake-up does not
shift the schedule: slots are counted from when the previous one was due,
so the next packet catches up rather than everything drifting late.

## Cost

Smoothing trades latency for regularity. A smoothed client's commands are
processed on average about half an uplink slot later than they arrive,
about 10 ms for a 20 ms slot, and the queue is sized by the clumping, so
the cost is proportional to how bad the link is and is zero on a link that
delivers packets on time, which is why it can be on for everyone by default.

## Observability

`smoothstats` prints, per client over the last 5 seconds: whether they are
smoothed, packets and duplicates, the average, standard deviation and
maximum gap between packets as they arrive and as they are processed, the
average and maximum time packets spent queued, the current queue depth and
drops. The arrival columns show a client's jitter as it reaches the server; the
processing columns show what smoothing made of it. The window is short so
that the effect of a change, or of a lag event, is visible within seconds.

## Cvars

| Cvar                 | Default   | Meaning |
|----------------------|-----------|---------|
| `sv_smooth`          | 1         | 0 clients with `setinfo smooth 1`, 1 everyone, 2 everyone except `setinfo smooth 0` |
| `sv_smooth_interval` | 1000/77   | ms between processed packets until the client's rate is measured |
| `sv_smooth_drain`    | 10        | most a slot is shortened to drain slack, percent |
| `sv_smooth_catchup`  | 50        | ms of backlog above which the queue drains at double rate |
| `sv_smooth_maxdelay` | 200       | ms after which a queued packet is dropped |

## Testing

`tests/test_smooth.c` is a self-contained unit test of the decision logic,
run by `ctest` and by the `test` job in CI. It checks pass-through, waiting
for a slot, the burst schedule with catch-up, stale drops, the rate
estimate (including that stalls and bursts do not fool it), an 8 second
simulation of a clumped 77 packets/s client whose settled output must have
a mean gap within 0.1 ms of 1000/77, a standard deviation under 0.2 ms and
never run dry, backlog recovery within 1.5 s, the statistics window, and
duplicate accounting.

End to end, the feature was validated with ezQuake's
`cl_delay_packet_upstream_rate`, which simulates a slot-limited uplink on
the client. With the rate at 50 (20 ms slots) and the client opted in,
`smoothstats` showed arrivals with a gap standard deviation of 9.5 ms and
processing with 1.4 ms under the fixed 12 ms interval, the figure that
prompted the move to the measured rate (the simulation above predicts about
0.1 ms for it); with the client not opted in the processing columns
matched the arrival columns exactly. The same test exposed the duplicate packet
problem described above.
