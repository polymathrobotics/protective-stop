# USB transmit ownership repair

## Failure mechanism

Espressif esp_tinyusb 2.2.1 publishes a caller's stack packet through one global
pointer and shares a completion event bit between calls. A timed-out deferred
callback can observe a later request. A late completion bit can also make a new
request return success before copying it: the omitted aggregate `result` field
is zero-initialized, hence ESP_OK. The post-timeout semaphore wait and the
blocking TinyUSB defer enqueue can hold TCPIP indefinitely. Clearing the global
pointer alone does not repair those lifetime, signalling or locking problems.

## Project-owned replacement

The tether no longer calls `tinyusb_net_send_sync`. It copies each submitted
Ethernet frame into a fixed, resident PSRAM pool and attempts a **zero-wait**
TinyUSB defer. ESP_OK means queue acceptance, not USB copy or wire delivery.
There is no completion wait on TCPIP, no shared finished bit, and no caller
buffer retained after return.

Each slot follows `FREE -> FILLING -> PENDING -> RUNNING -> FREE`. Successful
enqueue transfers ownership to exactly one callback. Failed enqueue transfers
nothing and immediately frees the slot. The callback releases only after its
last slot access. Atomics publish metadata/payload and prevent reuse while a
closure still refers to it. No timeout reaper reuses an outstanding context.

The callback drops a frame if it is at least 100ms old before claim, the netif
epoch changed, USB is unmounted, or NCM cannot accept it. It never retries a
dropped frame. A claim inside TTL may finish later if preempted; TTL is not a
100ms wire-delivery guarantee. NCM can retain a copied datagram in an NTB until
the host accepts the IN transfer.

The netif glue checks lwIP-core ownership and counts/rejects violations without
panicking. A busy guard rejects concurrent or reentrant producers. Together with the USB FIFO and no callback retries,
this preserves frame publication order. Capturing the epoch at send entry
prevents a stop/start interrupting publication from assigning an old send to
the new netif epoch.

## Bounded resources and queue headroom

There are 16 slots of 1536 bytes: **24KiB of PSRAM**, allocated once before
netif startup, with fixed metadata. At most **16 of our callbacks** may be
pending in TinyUSB's **64-entry** shared queue. The cap is derived as queue/4
from a shared limits header and checked in both the host test and extension,
leaving 48 entries for DCD/other events. This does not guarantee that
other producers cannot fill the remaining space.

Capacity rejection is counted, never waited out. If the USB task stops draining
accepted callbacks, occupied slots remain bounded and admission fails fast;
when it drains again, expired slots are released. The resident pool survives
netif stop/start. The project leaves the TinyUSB driver installed, so it does
not destroy the underlying USB queue while these references exist.

No encrypted-frame size heuristic reserves slots for presumed safety traffic.
Admission/capacity counters and the callback histogram will determine whether
task priority or scheduling needs a further measured change.

## Nonblocking defer and callback linkage

TinyUSB 0.21.0~1 exposes only a void defer whose FreeRTOS enqueue waits forever.
The project builds a same-translation-unit extension, `tud_defer_func_try`,
which uses `xQueueSendToBack(..., 0)`, reports acceptance and calls the original
event hook only on success. A null device queue also fails explicitly.

The build-generated source includes the original `usbd.c` and the small tracked
extension **inside the original TinyUSB target**, retaining its private flags.
Managed sources are not edited. Configuration requires the audited original
source SHA-256
`38de54351d81878e6543f2b022d0c39f45d28c8f0888491796398dec45e727be`
and exactly one original source replacement. Explicit managed-component
dependencies and a target-existence check enforce configuration order. The
original source's compile flags/options/definitions/include directories are
copied in its target directory, including `-Wno-type-limits`.

The upstream TinyUSB CMakeLists.txt is also SHA-guarded
(`4198bde29dfe2d6db0564a1527707647a3f79de8036520adfbb5c13f585efd8c`),
and known unsupported compile-affecting source properties are rejected.
Coverage is by guarded build rules and explicit checks, not an asserted
enumeration of all possible CMake properties. Upstream drift requires review.

The `tud_network_xmit_cb` link wrapper recognizes pool references by exact
address range/alignment and copies their owned bytes. Other references forward
to the original callback, including its vendor free-buffer contract. No private
Espressif packet-structure layout is assumed. Both projects run a post-link
disassembly check proving NCM calls the wrapper, legacy forwarding resolves,
and the adapter calls the nonblocking defer extension.

## Observation and acceptance

`/state.json` and `/api/health` expose:

- submitted/copy counts;
- cancelled, unexpected-error, rejected-unavailable, unmounted-drop,
  pool-exhausted, defer-full, defer-cap,
  can-xmit-false and invalid-callback counts;
- enqueue-to-callback-entry buckets: <10, <50, <100 and >=100ms;
- occupied slots, pending callbacks, oldest queued age and last TX-callback age.

TX-callback progress is not an idle USB-task heartbeat. Fast callbacks with
can-xmit failures suggest NCM/host-read pressure; growing queued age with absent
callback progress indicates device-side starvation or lack of queue draining.
Pair these signals with usbmon and peer captures rather than infer a cause
from one counter.

The seven unexpected/drop counters remain hard failures in both modes, even
when the peer stays armed. NCM capacity rejection is a real lost datagram and
stays gating. Rejected-unavailable and unmounted-drop are **hard failures during
USB qualification**, including cable events; they are warning-only in Ethernet
mode. Deliberate setup transitions occur outside qualification and are settled
before baseline capture. Queue acceptance cannot report a later drop
synchronously, so these counters are essential; no USB-window losses are excluded.
Existing safety deadlines and observation-gap rules are unchanged.

The state JSON's uptime is now sampled after all reply timestamps, eliminating
the negative-age formatting race. Buffer capacity was increased to accommodate
the additional diagnostics; no freshness grace was added. Health-response
capacity uses a format/uint32-width bound and reserves the closing brace.
Overflow returns a complete HTTP 500 JSON error rather than a truncated body.

## Verification and deployment boundary

Host tests compile the actual adapter and defer extension against a controlled
queue/clock: owned-buffer isolation, delayed callback after later submissions,
TTL boundaries, rejected enqueue, no hook on rejection, IRQ headroom cap,
stalled-queue drain/recovery, NCM capacity failure/recovery, epoch changes,
callback completion before enqueue returns, reentrant producer rejection,
legacy forwarding and error counters. The queue-tuned suite has **358 checks** and
passes warnings-as-errors with Address/UndefinedBehaviorSanitizers.

The soak suite includes a regression for deferred drops while the peer stays
healthy plus an explicit USB/Ethernet availability-counter matrix (**107 tests**).
An integrated remote build passed the final-link check;
fresh final builds, independent diff review and controlled hardware pilots are
required before qualification. No USB-task priority change has been made.

Independent final review accepted immutable patch `1efa96a0755f...` (82230 bytes;
full SHA-256 `1efa96a0755f6f3c16e5f2519ce876875821d4ae92aed260ada2373f56b7d2ad`).
The reviewer independently passed all 284 checks with C11 warnings-as-errors,
GNU17 ASan/UBSan, and `-O3 -flto -Wshadow -Wconversion -Werror`; both vendor SHA
guards matched. Required R1/R2 and source/build/JSON follow-ups were re-reviewed
and accepted. The next step is a fresh committed build and controlled DUT test.

Both four-hour windows must use the same final reviewed firmware/configuration;
these software tests and build checks are not hardware soak passes.

## Hardware-driven admission tuning

The first deployed build `9333e7314` used a 16-entry USB event queue and a
four-closure cap. Controlled Ethernet/USB isolation tests demonstrated STOP
within 2 seconds, but nominal USB polling exposed persistent admission loss.
The corrected-auth baseline, 22:46:07.507968-22:47:07.535763 UTC September 12,
recorded 2225 submissions/copies and **161 cap rejections**, with no other drop
increments or HTTP errors. Callback histogram delta was `[2101,121,3,0]`.
Peer capture saw 299 accepted heartbeat frames with contiguous counters and a
296ms maximum arrival gap: TCP recovery masked the lower-layer drops, which
correctly remained qualification failures.

The next candidate changes only queue headroom/admission: queue 64, cap 16,
same resident payload pool and 100ms TTL. The wrapper sets the queue macro
before including the guarded original `usbd.c`, inside its original target;
conflicting configuration fails compilation. No vendor files are modified.

TinyUSB priority **5**, affinity **core 1**, and NCM buffers **IN 4 / OUT 2**
remain unchanged. Raising USB above TCPIP 18 would also outrank both roles'
safety tasks at priority 8. The steady baseline had no >=100ms expirations,
so such a scheduling change is not justified by this measurement. Repeat the
same 60-second protocol on the queue-only candidate before pilots; keep every
drop/error counter hard-gated according to its existing mode scope.
