# Soak telemetry cadence investigation

## Measured A/B result

The Ethernet comparison used the same relayed path, firmware and host scheduling
class for both legs. Only the DUT state polling interval changed; peer, monitor,
role and health polling stayed fixed. Both legs had zero gate failures, missing
packet counters, unanswered frames, comparator increments, send failures or
rebonds.

| DUT state interval | Exact UTC interval | Accepted/answered frames | Native frame-stamp gaps >=250 ms | Arrival p90/p99/max |
| --- | --- | --- | --- | --- |
| 1 s | 21:36:29.546176–21:46:40.609416 | 3023/3023 | 64/3022 (2.12%) | 236/298/403 ms |
| 0.5 s | 21:46:41.976616–21:56:53.531401 | 2898/2898 | 319/2897 (11.01%) | 273/334/422 ms |

The settled subsets showed the same direction (2.28% versus 11.47%). This is a
substantial cadence-associated timing effect; frame timestamps do not directly
measure CPU preemption. No safety deadline breach was observed. Sustained 2 Hz
state polling is therefore rejected as the remedy for observation expiry.

Evidence is under `/tmp/opencode/pstop-soak-20260910/poll-cadence-ab-20260910/`.
The previous 1 Hz Ethernet four-hour pass remains historical baseline evidence.

## Deadline-aware sampling

`tools/soak_stability.py --poll-sec 1 --adaptive-dut-poll` keeps the normal 1 Hz
cadence and may request an earlier state sample before the existing reply-evidence
certificate expires. The collector already evaluates after processing arrivals
and approximately every 50 ms. The approximately one-second term in observation
events 482/786/794 was the age of the previous sample, not an evaluation tick.

The optional scheduler uses the same valid sample's raw reply age and conservative
generation bound to calculate expiry. It starts an early read with a lead of
`max(0.2 s, previous HTTP duration + 0.1 s)`, subject to:

- At least 0.5 seconds since the previous state request started.
- At most six early reads in a rolling minute and 60 in a rolling hour.
- One existing state worker: no overlapping or replacement state requests.
- Re-phasing the next regular tick from the early request's start, rather than
  immediately following it with the old regular tick.

These are scheduling limits, not additional acceptance allowances. Invalid or
expired samples are still rejected. Reaching a duty cap keeps regular polling;
it does not grant credit or extend the 1600 ms bound. A blocked early request may
cause observation coverage to expire, as before. The 1.8-second HTTP timeout
marks unusable evidence but cannot cancel a blocked Python network call.
An early response captured after its stored prior expiry explicitly retains that
expiry failure, even if it arrives between supervisor ticks and has a fresh body.
Capture at or before expiry is not made late merely by subsequent processing.

Acceptance, normalization, clock/frontier certification and closing checks are
unchanged. Firmware is unchanged. Peer/auxiliary cadences are unchanged.

## Evidence and validation

The phase intent records the option and its limits. State records retain early
request context and actual start/end times. `scheduler.jsonl` records early
requests, cap hits and whether results completed/were processed before the old
certificate expired; a result's timing is not itself proof that it was valid.
`collector.jsonl` exposes cumulative early/cap counts and rolling early counts.

Offline tests cover normal 1 Hz load, recorded age/latency-shaped delay cases,
minute/hour limits, auxiliary polling, invalid timestamps, and a blocked early
worker retaining the existing expiry failure. A boundary regression checks
responses captured1ms before, exactly at, and1ms after the prior expiry.
Simulated responses are scheduler
tests, not invented intermediate hardware evidence.

Hardware validation of the new option is required before qualification restart:
check both transports, actual request rate/early fraction, HTTP and core load,
counter changes, and peer-confirmed frame timing on comparable settled paths.
All comparison time is excluded from qualification. Both four-hour modes restart
on the final reviewed configuration with no carried-over clean time.

## Final-source hardware results

Collector SHA-256:
`b55094f1eaf00a0ebc8587f271cbbd0ee40502df991aa25370898c2efacfd7f4`.
Both pilots retained matching before/after source fingerprints and source copies.
Firmware remained `v1.2-29-g901cafd`; peer configuration remained `c51a8398...`.
The final source passed 105 offline tests and the repository's pre-commit checks.

| Mode | Exact UTC interval | Clean seconds | Gate failures | Early state requests |
| --- | --- | --- | --- | --- |
| USB | 22:47:20.315500–22:57:32.287203 | 603.931 | 0 | 1/613 (0.16%) |
| Ethernet | 22:59:33.115167–23:09:45.194687 | 603.759 | 0 | 0/613 |

Neither pilot had comparator, send-failure or rebond increments. The USB early
request completed at22:55:15.270368 and was processed at15.293523, before the
previous expiry at approximately15.418156. The late-result guard was verified
offline, not exercised by this on-time event.

Independent packet review found3042 USB frames answered with no missing counters;
Ethernet had2992 unique counters answered and one duplicate frame. Maximum
arrival gaps were452ms and320ms respectively, with no machine safety events.
Native timestamp-gap rates were1.18% (USB) and4.58% (Ethernet). These exceed some
earlier same-transport references and remain recorded uncertainty; zero/sparse
early reads do not establish an added-query load regression. They are descriptive
measurements, not new acceptance thresholds or failure exclusions.

Nominal1Hz with capped early refreshes is adopted for fresh qualification, with
first-early-read and first30min reviews in each mode. Evidence and peer review:
`/tmp/opencode/pstop-soak-20260910/poll-adaptive-final-20260910/`.
