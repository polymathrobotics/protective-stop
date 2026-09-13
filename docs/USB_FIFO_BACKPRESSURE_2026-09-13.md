# USB FIFO backpressure recovery — 2026-09-13

## Evidence and qualification disposition

Firmware `0b803a562` (`34b1a82`) completed Ethernet qualification with
14403.312835514429 consecutive clean seconds and zero failures. Independent
review of the full sampled interval, 00:34:32.687312–04:34:43.821598 UTC,
found 70379 contiguous replied-to heartbeats and nine reconciled duplicates.

USB did not complete qualification. `usb_tx_can_xmit_fail` increased once near
04:35:13 and again near 05:45:24. Each increase produced a counter-change event
and an acceptance-fault event; four event records describe two observed drops.
All affected callbacks ran within 10 ms, with no pool, queue, lifetime, or
integrity failures. Both events overlapped a roughly 20 KB diagnostic peer-list
HTTP response. The first response's tail stalled for 1.459557 seconds; the
second request took 1.252850 seconds. The configured TCP retransmission timeout
is 1500 ms. Temporary NCM buffer pressure and TCP recovery are strongly
supported; an unsubmitted packet is not visible in a host capture, so its exact
identity is not proved. The peer observed no missing inner heartbeat in either
event window. That does not rule out a recovered outer-transport loss.

The campaign was paused and ownership released at 05:46:43 UTC. An engineering
reproducer offered a 60-second staggered workload: state at 1 Hz, monitor and
role at 0.2 Hz, health every 30 seconds, and the full peer list at 1 Hz. Its
recorded response interval was 05:56:15.497196–05:57:14.996118 UTC. It completed
58 peer-list requests, missing two scheduled slots during slow responses,
and reproduced two more `can_xmit_fail` drops: 3235 submitted versus 3233 copied.
There were no HTTP errors; peer-list maximum latency was 1234.616 ms. The peer
reviewed 294 contiguous replied-to heartbeats, maximum arrival gap 325 ms.
This is engineering evidence, not qualification credit.

Private evidence resides under
`/tmp/opencode/pstop-soak-20260910/campaign-20260913T003413Z/`,
`capture-archive/20260913T043913Z/`, and `usb-burst-20260913T055615Z/`.
Raw captures, firmware images, credentials, and coredumps are not committed.

## Ordered retention instead of pre-copy loss

This design supersedes the per-request closure policy documented in
`USB_TX_RECOVERY_2026-09-12.md`:

- Keep the 16 resident 1536-byte PSRAM payload slots and 100 ms lifetime.
- Publish a single-producer/single-consumer FIFO using release/acquire head and
  tail indices. A slot cannot be reused until the consumer publishes its head.
- Queue at most one TinyUSB drain. The producer's nonblocking kick handles the
  normal path; a resident 2 ms `ESP_TIMER_TASK` callback supplies busy retries
  and recovers a push racing with the drain's final flag clear.
- The timer only checks scheduling metadata and attempts zero-wait deferral.
  Only the TinyUSB consumer reads, copies, cancels, or releases queued payloads.
- Drain a bounded tail snapshot. Never bypass an NCM-busy head or move it behind
  a newer frame. Retain it until capacity returns or its original lifetime
  expires. The callback wrapper and synchronous vendor-copy validation remain.
- Netif disable still advances the epoch. The consumer cancels old-epoch
  requests; restarting the netif cannot resurrect them.
- A full producer snapshot gets one bounded head re-read, never a spin/wait.
  An impossible published non-pending head is retained and reported as an
  integrity fault rather than skipped: safe progress cannot be assumed after
  ownership corruption.

The existing shared event queue remains 64 entries, NCM IN/OUT remain 4/2 NTBs
of 3200 bytes, and TinyUSB priority remains 7 below the safety tasks. The NTB
pool is not enlarged: the prior internal-heap low-water mark was about 17 KB.

Both application defaults reduce `CONFIG_LWIP_TCP_SND_BUF_DEFAULT` from 32768
to 11520 bytes, eight 1440-byte segments. This bounds each connection's burst
while the existing FIFO drains. TCP receive window, retransmission timeout,
protocol timing, and NCM storage are unchanged. This throughput tradeoff needs
before/after hardware measurement and both-mode requalification.

## Counter contract and tests

`usb_tx_ncm_busy_retries` reports retained-head capacity probes. A frame that
waits and is copied before expiry was not dropped. Expiry remains `cancelled`;
expiry after NCM backpressure also increments `can_xmit_fail`. Pool overflow
remains `exhausted`. Failed zero-wait kicks still increment the hard-gated
`defer_full` counter, even when a later timer tick recovers the retained FIFO.
All existing terminal-drop gates remain. The legacy per-packet `defer_cap`
counter is zero by construction with one drain; FIFO overflow is accounted for
by `exhausted` instead. Per-frame callback counts and latency histograms count
retirement once, including the full waiting time rather than each retry probe.

Tests compile the actual adapter and guarded TinyUSB queue extension. They
exercise initialization failures, lost wakeups, competing kick sources, timer
callbacks during a drain, FIFO and clock wrap, five busy probes followed by
ordered delivery, busy expiry, failed retry kicks, epoch changes, and a mixed
14-frame bulk/two-heartbeat burst with three of four capacity probes busy.
The first sanitized C11/Werror run passed 878 checks.
The bounded capacity re-read/interleave case brought the sanitized total to
941 passing checks. The unchanged soak collector's 107 tests also pass.

The patch requires independent review, fresh builds, hardware steady/burst
comparisons, controlled cuts, pilots, and fresh four-hour windows in both modes.
The steady measurement retains its zero-increment requirement for latency bins
at or above 50 ms. Burst tests may show retained waits below 100 ms; terminal
drops, expiry, and pool overflow are still failures.
The prior Ethernet result remains historical once firmware changes. No timeout
increase, observation-gap credit, startup exception, or diagnostic-load removal
is part of this remedy. The peer's frozen five-second state poll remains a load
contributor in every comparison.
