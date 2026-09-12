# Health-aware home-DERP retry candidate

September 12 update: the USB ownership repair and state-clock correction are
now implemented alongside this patch; see [the USB repair note](USB_TX_RECOVERY_2026-09-12.md).
The build hashes below identify the earlier health-only candidate, not the
combined image that will be freshly built for deployment.

Branch: `fix/soak-recovery`, carrying the existing soak collector changes and
investigation reports. Hardware remains PAUSED with qualification BLOCKED.

## Defect and change

Paused PSTOP06 diagnostics found an empty priority peer while its machine peer
was registered for application health tracking. The periodic home-DERP retry
gate previously checked only the configured priority peer, selecting the
60-second management-only interval for this safety configuration.

The candidate selects **configured priority OR any registered health-peer IP**.
An unhealthy, relay-bound, unbonded, or not-yet-admitted target still qualifies.
Fleet and ordinary reachability pins alone do not qualify.

`ml_safety_retry.h` contains the host-testable membership and policy helpers.
WG per-peer membership and the any-registered query share that predicate;
DERP calls the WG query. Membership IPs are C11 atomic. The new reader ignores
the `healthy` flag and does not attempt to make `(ip, healthy)` coherent.

The existing 5/5/10/60-second periodic ladder, strict due-time comparison,
immediate reconnect kick, safety deadlines and protocol behavior are preserved.
This repairs eligibility after an immediate reconnect attempt fails; it does
not establish the cause of the 43.12-second relay-cluster reconnect, restore
the missing direct path, or stabilize the site WAN. Pair 8's 836/2827 ms home
reconnects do not demonstrate failure of this periodic gate.

## Accepted policy decisions and release note

**Burst is preserved across eligibility changes.** Late registration does not
replenish an exhausted retry budget: burst >=3 still selects 60 seconds. With
burst 0 and more than 5 seconds since the last periodic attempt, registration
can make the next evaluation due immediately. Last unregistration removes fast
eligibility unless a configured priority peer remains.

**Attempt-based fallback is preserved for every safety class.** On an
**unlocked, ownerless, health-only device**, the home can now move to an
**unproven fallback candidate** sooner: roughly 80 seconds in an instantaneous-
failure model, versus roughly four minutes with 60-second intervals. The
existing picker prefers a live aux region, then the compiled default, then a
safety want-set region; the latter two need no reachability proof. This is an
explicitly accepted consequence of applying the existing safety retry policy
to health-only peers. A reachability-proof requirement would be a separate change.

Those times assume last-periodic timestamp zero and immediate failures. Tests
produce attempts at 5001, 10002, 20003 and 80004 ms. A pending asynchronous
connect keeps the slot non-IDLE until completion/failure/timeout and can delay
later attempts. The model is not a hardware timing guarantee.

PSTOP06 has raw region override **9**, which forbids the home move. A re-home
owner also blocks fallback migration. LOCK still permits one rescue aux kick
after the attempt threshold when the pool is dark, a candidate and non-home
slot are available, and no rescue is pending. Tests cover these gates,
duplicate suppression and slot zero becoming aux after a home move.

## Concurrency and capacity

The fixed-lifetime registration array has one writer in each shipped role:
remote health notifications occur in `comparator_task`; machine-role
notification/removal occur in `seen_publish`, called only by its comparator.
Repository call-site inspection and independent review confirmed those paths.
Atomic IPs define the cross-task membership accesses; they do not make the
existing scan-then-write registration algorithm multi-writer safe. Future
callers must preserve writer serialization or introduce synchronization.

The existing 16-entry capacity and full-table behavior are unchanged. This
patch covers registered entries, not refused registrations or application
unregistration policy. A full table already contains registered safety peers,
so it still makes the any-peer query true.

The header is C-only and currently included only by C translation units.
Independent host checks found the old/new entry layout identical (size 8,
alignment 4) and 32-bit atomics lock-free. The any-peer scan deliberately uses
the same per-peer predicate; concurrent removal can postpone observation to a
later evaluation. The extra eager rescue free-slot scan is read-only and bounded.

## Verification and independent review

- Host suite passed: **148 new retry-policy checks**, 5117 clock-guard checks,
  21 crypto checks, 24 demote-verdict checks and auxiliary-channel tests.
- `python3 -B -m unittest discover -s tools -p 'test_soak*.py'`: **105 passed**.
- Scoped repository pre-commit hooks passed after C formatting.
- ESP-IDF v5.5.4 remote build passed in
  `/tmp/opencode/soak-recovery-firmware-build`; copied sdkconfig matches the
  original byte-for-byte. The qualified build was retained.
- Machine-role build passed in `/tmp/opencode/soak-recovery-machn-build`, using
  machine defaults because this checkout had no machine sdkconfig. This is a
  compile check, not a validated machine deployment configuration.

Remote binary SHA-256:
`881aa1bb119f8c23e70f38c04bdab8ce6e36b8bc4c5f5345a5fd82e2f83ec712`.
Machine-role compile-check binary SHA-256:
`08f97b3176e49da4b8ecddd27800b09801bd8aa418a7178ff3394b8af0a557df`.
Neither was flashed. Hashes identify these uncommitted-source artifacts more
precisely than their git-derived version strings.

Independent peer verdict: **ACCEPT for DUT deployment**, subject to fresh
qualification and the fallback release note above. The peer independently
verified the focused patch (19373 bytes, SHA-256
`af23151b63ae1cb9ea8d6bf4b9578d15235e2a28c80076fa526fcd2256e55b55`),
applied it on a throwaway worktree of upstream `4ce8094`, and passed all 148
checks with C11 warnings-as-errors and GNU17 Address/UndefinedBehaviorSanitizers.
Gate equivalence, immediate kick, burst preservation, LOCK, re-home ownership,
rescue bookkeeping and the single-writer paths were reviewed.

## Remaining upstream PR

Upstream `main` is `4ce8094`, with squash merges #114 and #115; Graphify has
refreshed to that commit. Remaining
[PR #117](https://github.com/polymathrobotics/protective-stop/pull/117), head
`f86f3be`, already merged that main. Its four-file refetch-backoff/diagnostic
diff has 73 additions and 2 deletions. Its tree matches the `d4b86ad` merge
already incorporated into the soak work.

It remains relevant: it reduces **control-plane refetches**, whereas this
candidate changes **home-DERP reconnect eligibility**. Two corrections are
warranted before merging #117:

1. “Four rounds in the first 10 minutes” is incorrect. After the first request,
   the interval doubles to 180s; subsequent minimum spacings are 180, 360, 720,
   1440, 1800s. Requests two through four are therefore no earlier than roughly
   3, 9 and 21 minutes relative to the first. Scheduler gating can delay them.
2. The `next in` log prints before doubling/capping and reports the prior
   interval. Log after the update or label it as the prior interval.

This review checked source and ancestry, not a fresh GitHub CI verdict. No
upstream PR was edited or pushed. GitHub CLI lacked authentication, so public
read-only API results and fetched git objects supplied the review evidence.

## Next validation boundary

Controlled DUT recovery testing must correlate actual periodic attempts and
home/aux outcomes with discovery and gateway evidence. The USB send-lifetime
repair and staged state-snapshot timestamp correction remain separate work.
Any final firmware/configuration change requires both four-hour qualification
windows to restart from zero under the existing acceptance rules.
