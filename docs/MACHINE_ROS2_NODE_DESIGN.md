# Machine ROS 2 Lifecycle Node — Design v0.2

Status: **READY TO IMPLEMENT** (2026-08-01). Decisions locked from Q&A:
role = **bridge + control**; software backend = **node hosts pstop_c itself**;
distro = **Jazzy**; interfaces = **machine-state + relay/fault + remotes/RTT**;
arming stays **remote-only** (§2, §8). Implementation decisions resolved in §10:
extend `protective_stop_msg`, in-repo `ros2/` workspace, **full control-proxy**
hardware backend, keep `machine_app_runner`.

---

## 1. Purpose & scope

A managed (lifecycle) ROS 2 node that presents **one uniform ROS 2 surface**
over a pstop *machine*, regardless of which of the two machine implementations
is behind it:

- **software** — the node **is** the machine: it links the certified `pstop_c`
  library and runs a machine instance in-process, binding UDP `:8890` so remotes
  bond directly to it (today's `host/machine_app_runner`, absorbed into the node).
- **hardware** — the ESP32 `machn` unit is the machine; the node is an HTTP
  client that polls its `/state.json` at 5 Hz and drives its admin API.

The backend is chosen at **configure time** from the parameter file; everything
above the backend seam (topics, services, diagnostics, lifecycle) is identical.

### What this node is NOT
It is a **bridge + control** surface, **not the safety authority**. The relays
(hardware) and the `pstop_c` fail-safe logic (software) enforce STOP
independently of ROS 2. Loss of this node, its host, or the DDS graph **must not
produce unsafe motion**: with the software backend the machine dies with the
node, so remotes stop receiving replies and **fail-safe STOP** on their own
heartbeat timeout; with the hardware backend the ESP32 keeps enforcing while
ROS 2 merely goes blind. This property is a design invariant, re-checked in §8.

What the node **does** owe, because the pure ROS 2 (software) machine has no
final element of its own, is an unambiguous published statement of the desired
state — STOP or OK — carried on a stamped heartbeat a consumer can watchdog.
That is `~/pstop_hb`; see §5 "Stop signal". Publishing it does not make the node
the safety authority (SR-M-04), it makes the machine's verdict actionable by
whatever the integrator allocates the actuation duty to.

---

## 2. Control surface (what "bridge + control" means here)

Allowed control, none of which can *create* an unsafe RUN state:

- **Reconfigure timing** — heartbeat period, `max_missed`, `min_stop_ms` — via
  dynamic parameters. Tightening these can only make the machine *more*
  conservative at runtime; validation (§4) refuses values looser than the
  compile-time safe envelope. Rates are read at configure/activate and need a
  restart.
- **Target/peer management** — which remotes the software machine expects, or
  (hardware) proxied peer-slot config through the ESP32 admin API.
- **Lifecycle** — activate/deactivate the bridge (deactivate drives the machine
  to its safe state, §3).

Explicitly **out of scope**: arming. Arming a machine to RUN requires a physical
STOP→OK gesture on a bonded remote (pstop_c `#59` ownership semantics). Exposing
"arm" over ROS 2 would move the human-in-the-loop guarantee into software; we
keep it on the remotes. The node can *observe* and *report* arm state, never
assert it.

---

## 3. Lifecycle model (`rclcpp_lifecycle::LifecycleNode`)

| Transition | Software backend | Hardware backend | Safety note |
|---|---|---|---|
| `on_configure` | load+validate params, construct backend, create pub/sub/services **inactive**, build `pstop_c` machine (no socket yet) | same, construct HTTP client, resolve device URL | no I/O, no motion authority yet |
| `on_activate`  | bind UDP `:8890`, start the **machine loop thread**, activate publishers, start state-publish timer | start 5 Hz poll timer, activate publishers | machine begins servicing remotes / polling |
| `on_deactivate`| **stop the machine loop** → remotes stop getting replies → they fail-safe STOP; deactivate publishers | stop polling; ESP32 keeps enforcing | deactivation is a *safe* action by construction |
| `on_cleanup`   | destroy machine, socket, pubs | destroy HTTP client, pubs | back to `unconfigured` |
| `on_shutdown`  | ensure loop stopped, release | ensure polling stopped | idempotent safe teardown |
| `on_error`     | stop loop, publish `UNSTABLE`, → `errorprocessing` | mark backend unreachable, publish `UNSTABLE` | fault ⇒ conservative state |

Bringup is **self-managed**: the read-only `autostart` parameter (default
**true**) makes the node configure and activate itself in its constructor, so a
bare `ros2 run protective_stop_machine machine_bridge_node` comes up active.
Set `autostart:=false` to hold it in `unconfigured` for manual or orchestrated lifecycle control

---

## 4. Parameters — file-driven, validated, partly dynamic

Loaded on start from a **ROS 2 params YAML** (`--ros-args --params-file`), the
idiomatic mechanism. We use **`generate_parameter_library`** (PickNik, standard
on Jazzy) to declare params in one YAML, auto-generate a typed struct with
**validation (ranges/enums)**, `read_only` flags, and a dynamic-update hook —
eliminating hand-rolled `declare_parameter` + validation boilerplate.

**Static (`read_only`, require reconfigure):**
`autostart`, `backend` (`software|hardware`), `machine_id`, `frame_id`, software
`bind_addr`/`port`, software `default_stop_only`/`operators` (operator
authorization, SR-SYS-09 — an unlisted remote is stop-only), hardware
`device_url`/`admin_user`, the `rates.*` group (`state_poll_hz`,
`publish_rate_hz`, `diagnostics_rate_hz` — read at configure/activate), and the
optional fleet groups `announce.*` and `fleet.*`.

**Dynamic (runtime-settable, re-validated on set):** the `timing.*` group only —
`heartbeat_ms` (∈ [50, 1000]), `max_missed` (∈ [1, 5]), `min_stop_ms` (≥ 100).

Validation rule: a dynamic set that would **loosen** the safety envelope past the
compile-time floor is **rejected** (the set-callback returns `successful=false`
with a reason), so ROS 2 can only make the machine safer, never less safe.

**Fleet check-in (`announce.*`, software backend only, opt-in):**
`announce.url`, `announce.key_file`, `announce.name`, `announce.interval_s` —
all `read_only`, and **default DISABLED** (empty `url`). When a URL is set, the
node runs a background thread (started on `on_activate`, stopped on
`deactivate`/`error`/`shutdown`) that POSTs a bearer-authenticated JSON check-in
every `interval_s`, so the software machine appears in the fleet console's
machines/overview exactly like the host runner (`host/machine_app_runner.c`
`[announce]`) and the ESP32 machn. The body's core `{"name","port"}` matches the
host runner byte-for-byte (the console keys on `name` and attributes the source
IP itself); additive fields carry the machine's status for the overview
(`device_type:"machine"`, `machine_id`, `running`, `active_remotes`, and a
per-remote `remotes[]` summary). Deliberately **off the safety path**: its own
thread, libcurl only, log-only on failure — a dead console never affects pstop.
Prefer the **environment** for the deployment secrets: `PSTOP_ANNOUNCE_URL` and
`PSTOP_ANNOUNCE_KEY_FILE` override the params so URL/key stay out of committed
config (same convention as the host runner). See `announce.hpp` /
`src/announce.cpp`; payload builder is header-inline + unit-tested
(`test/test_announce.cpp`, network-free).

---

## 5. Interfaces

### Topics (QoS chosen per data semantics)
| Topic | Type | QoS | Notes |
|---|---|---|---|
| `~/machine_state` | `protective_stop_msg/ProtectiveStopStatus` (ACTIVE/DEACTIVATED/UNSTABLE) + reason string | **reliable, transient_local (latched)**, depth 1 | publish on-change **and** at `publish_rate_hz` heartbeat; late subscribers get current state |
| `~/relay_status` | new `MachineRelayStatus` (relay_stop, relay_fault_a/b, mismatch, run) | reliable, depth 5 | hardware: from `/state.json`; software: relays N/A → fields flagged `not_applicable` |
| `~/remotes` | new `BondedRemoteArray` (id, bond_state, reply_age_ms, loop_rtt_ms, disco_rtt_ms, rebonds) | reliable, depth 5 | uniform across backends |
| `~/pstop_hb` | `protective_stop_msg/ProtectiveStopHeartbeat` (`stamp`, `stop`) | **reliable, volatile**, depth 1 | the **desired-state signal + liveness** — see "Stop signal" below. Republished every `publish_rate_hz` tick regardless of change; deliberately **not** transient_local |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` via `diagnostic_updater` | default | backend reachability, bond count, faults, mismatch, loop jitter |

### Stop signal (what a downstream consumer acts on)

On the hardware backend the final element is the `machn` relay pair and this
node is telemetry. On the **pure ROS 2 (software) machine there is no final
element in scope** (SR-M-04, HARA A-07/H-12) — the actuation duty is allocated
to a downstream consumer, so the node's *published* signal is the only thing
that consumer can act on. It must therefore be unambiguous and self-dating:

- **Desired state, not a status narrative.** `~/pstop_hb.stop` is a single
  boolean — `true` = drive to STOP, `false` = cleared to run. It is taken from
  the machine's own cleared-to-run verdict, so `DEACTIVATED` (stopped, or
  awaiting the arming gesture) and `UNSTABLE` (backend unreachable) **both**
  resolve to `stop = true`. FMEA M03-1's "downstream consumer must treat
  STOP/UNSTABLE as de-energize" is applied at the source rather than re-derived,
  possibly differently, by every consumer.
- **`~/machine_state` is not the actuation input.** It stays the operator /
  telemetry view. It carries no stamp, and its tri-state deliberately reports
  *why* the machine is not running — a distinction a gate must not act on
  differently.
- **Heartbeat.** The same message carries `stamp` and is republished every tick
  whether or not anything changed, so a consumer fails safe on silence or
  staleness without the node having to assert anything. **Absence is a stop.**
- **Explicit final stop.** `on_deactivate`, `on_shutdown` and `on_error` each
  publish one last `stop = true` before the publishers go down, so an orderly
  teardown produces a positive stop instead of relying on the consumer's timeout.
- **Volatile by construction.** A late-joining consumer must never be handed a
  latched historical `stop = false` and open its gate on it; it waits for a live
  sample.

This makes the signal *usable*; it does not make it *rated*. SR-M-04 still
stands — a software-machine deployment is non-safety-rated and the final-element
integrity remains the integrator's (HARA A-07).

Interfaces ship as **`protective_stop_msg`** (`ros2/protective_stop_msg/`),
moved out of `archive/` so there is a single interface package. It keeps the six
original remote-side messages and the `ProtectiveStop` service — including
`ProtectiveStopHeartbeat`, reused unchanged for `~/pstop_hb` — and adds **three**
machine-side ones: `MachineRelayStatus`, `BondedRemote` and `BondedRemoteArray`,
so remote and machine share one interface package.

### Diagnostics
`diagnostic_updater` publishes OK/WARN/ERROR: backend unreachable ⇒ ERROR (but
note the machine still enforces independently); relay fault or mismatch ⇒ WARN/
ERROR; loop-period jitter over budget (software) ⇒ WARN.

---

## 6. Threading & determinism

Clean separation of **pure logic** from **ROS glue** (matches the repo's module
discipline):

- **Software backend** runs the `pstop_c` machine loop in a **dedicated
  `std::thread` at a deterministic period**, *off* the ROS 2 executor, so DDS/
  callback jitter never perturbs the safety heartbeat. It writes an immutable
  **state snapshot** (double-buffered / mutex-guarded POD) that ROS timers read.
- **Hardware backend** polls on a ROS timer in a **`Reentrant` callback group**
  so a slow HTTP round-trip can't stall state publication. *(superseded)*
- Publishers/services live in a `MutuallyExclusive` group. A `MultiThreaded
  Executor` runs the node. *(superseded)*

  > **[Reconciled 2026-08-05]** Both bullets above describe a threading model the
  > code does not have, and the node is **deliberately not** built that way.
  > Ground truth:
  >
  > - The hardware backend polls on its **own `std::thread`**
  >   (`hardware_backend.cpp` `poll_loop`), not a ROS timer in a callback group —
  >   so the "slow HTTP can't stall publication" property holds by the same
  >   off-executor mechanism the software backend uses, not by reentrancy.
  > - The node declares **no callback groups**, so every ROS callback
  >   (`publish_tick`, the lifecycle transitions, `diagnostic_updater`, the
  >   parameter set handler) lives in the node's default **`MutuallyExclusive`**
  >   group, and the code-generated executable spins a
  >   **`SingleThreadedExecutor`** (rclcpp_components default).
  >
  > **Single-threaded execution is a design requirement, not an accident.**
  > `publish_tick` and the lifecycle transitions both touch the publisher handles
  > and `last_snapshot_`, and `pub_timer_->cancel()` does not wait for an
  > in-flight callback. Serialising every ROS callback on one thread is what
  > makes that safe without locks: `on_cleanup`/`on_error` can only `reset()` a
  > publisher on the same thread that would otherwise be inside `publish_tick`.
  > Introducing a `MultiThreadedExecutor`, or moving any callback into a
  > `Reentrant` group, re-opens a use-after-free on all four publishers and a
  > data race on `last_snapshot_`, and **must** be accompanied by explicit
  > lifetime/state guards. Off-executor work belongs on a backend thread behind
  > `IMachineBackend`'s mutex-guarded snapshot, which is where it already is.
- Backends implement one interface:

  ```cpp
  class IMachineBackend {            // pure, no rclcpp
  virtual void start() = 0;        // bind/socket or begin polling
    virtual void stop()  = 0;        // → safe state
    virtual MachineSnapshot snapshot() const = 0;   // thread-safe read
    virtual bool configure(const MachineTiming&) = 0; // runtime reconfig
  };
  ```

  `SoftwareMachineBackend` and `HardwareMachineBackend` are the two impls; the
node depends only on `IMachineBackend`.

---

## 8. Safety analysis (invariant re-check)

| Failure | Software backend | Hardware backend |
|---|---|---|
| Node process dies | machine dies → remotes time out → **STOP**; `~/pstop_hb` also goes silent → a consumer watchdogging it stops | ESP32 unaffected → still enforces; ROS 2 blind (diagnostics ERROR on next consumer) |
| `on_deactivate` | machine loop stops → remotes **STOP**; one explicit `stop = true` heartbeat is published before the publishers go down | polling stops; ESP32 enforces |
| DDS/host partition | irrelevant to enforcement (UDP bond is separate from DDS); a partitioned consumer sees `~/pstop_hb` stop arriving → **STOP** | irrelevant; ESP32 enforces |
| Backend unreachable | n/a (in-process) | publish `UNSTABLE` + diag ERROR; **no** motion authority asserted |
| Param set loosening safety | rejected by validator | rejected by validator |

Invariant holds: **no ROS 2 failure path creates or sustains RUN.** The node can
only observe, report, and tighten.

---

## 9. Testing

- **gtest** on `SoftwareMachineBackend`/`HardwareMachineBackend` behind
  `IMachineBackend` with a mock transport / mock HTTP — no ROS spin needed
  (pure-logic tests).
- **launch_testing** for lifecycle transitions and the safety invariants
  (deactivate ⇒ remotes STOP; bad param ⇒ rejected).
- Reuse the existing wire-protocol suites (`pstop_multi_remote_test.py`,
  `stop_reset_battery.py`) against the **software backend hosted in the node** to
  prove parity with `machine_app_runner`.

---

## 10. Decisions (resolved 2026-08-01)

1. **Message ownership** — **extend `protective_stop_msg`** (moved out of
   `archive/` into `ros2/`); add `MachineRelayStatus` + `BondedRemote` +
   `BondedRemoteArray` there so remote and machine share one interface package.
2. **Package location** — **in-repo `ros2/` colcon workspace** at repo root. The
   archived packages already live here and CI can build it; accepts re-adding
   ROS 2 build deps that #53 slimmed out.
3. **Hardware backend scope** — **full control proxy**: the hardware backend not
   only publishes state but proxies config/peer-slot changes to the ESP32 via
   its admin API, so timing reconfiguration and target management work uniformly
   across both backends (§5). Still bounded by the safety validator (§4): proxied
   config can only tighten, never arm.
4. **`machine_app_runner`** — **kept** as a headless / CI tool. The node's
   software backend shares its pstop_c glue, but the runner stays because the
   wire-protocol suites (`pstop_multi_remote_test.py`, `stop_reset_battery.py`)
   depend on it.

## 11. Rollout note (perf / fleet, 2026-08-01)
Firmware perf track is `-O2`-only for the near term (validated); `-O3`-on-crypto
is deferred to its own change. Verified builds are uploaded to the fleet build
server so the flashing station's "latest" tracks real work; "starring" not
adopted (station uses newest).

---

## 12. Running the node and overriding parameters

Defaults (and the SR-M-01 validation ranges) live in the
`generate_parameter_library` source `src/protective_stop_machine_params.yaml`
and are compiled in, so a bare run comes up active:

```sh
ros2 run protective_stop_machine machine_bridge_node
```

To override, pass a params file:

```sh
ros2 run protective_stop_machine machine_bridge_node \
  --ros-args --params-file my_overrides.yaml
```

```yaml
/machine_bridge:
  ros__parameters:
    autostart: true             # read_only; false holds the node in unconfigured
    backend: software           # software | hardware   (read_only)
    machine_id: 0x01020304      # read_only
    frame_id: pstop_machine

    software:                   # used when backend == software
      bind_addr: 0.0.0.0
      port: 8890                # read_only
      # Operator authorization (SAFETY, SR-SYS-09). A bonded remote is accepted
      # and heartbeat-monitored but STOP-ONLY by default: it may command STOP,
      # never re-arm. Only a remote whose 32-bit pstop id is listed in
      # `operators` gets re-arm authority. Empty (the default) = every remote is
      # stop-only = maximally safe, and the machine can never be armed.
      default_stop_only: true   # read_only
      # operators: [30234300]   # read_only; ids granted re-arm (default: none)

    hardware:                   # used when backend == hardware
      device_url: http://100.84.155.111
      admin_user: admin         # password via env, never in the file

    timing:                     # the only dynamic group; validated on every set
      heartbeat_ms: 400         # [50, 1000]
      max_missed: 5             # [1, 5] — default 5, matches machn (timeout ~2.0s)
      min_stop_ms: 500          # >= 100

    rates:                      # read_only; read at configure/activate
      state_poll_hz: 5.0        # hardware poll
      publish_rate_hz: 10.0     # also the ~/pstop_hb rate
      diagnostics_rate_hz: 1.0

    # Optional fleet console check-in (software backend only, both default
    # DISABLED on an empty url). Prefer the ENVIRONMENT for the secrets:
    # PSTOP_ANNOUNCE_URL / PSTOP_ANNOUNCE_KEY_FILE and
    # PSTOP_FLEET_CHECKIN_URL / PSTOP_FLEET_API_KEY_FILE override these.
    announce:                   # lightweight overview check-in
      url: ''                   # read_only; empty = disabled
      key_file: ''              # read_only; chmod-600 bearer token
      name: ''                  # read_only; empty = this host's hostname
      interval_s: 60            # read_only

    fleet:                      # device check-in (device_type="machine")
      checkin_url: ''           # read_only; base url, /api/v1/checkin appended
      api_key_file: ''          # read_only
      check_interval_s: 300     # read_only; [60, 300]
```
