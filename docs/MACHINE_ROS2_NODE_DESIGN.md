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

---

## 2. Control surface (what "bridge + control" means here)

Allowed control, none of which can *create* an unsafe RUN state:

- **Reconfigure timing** — heartbeat period, `max_missed`, `min_stop_ms`, poll
  rate — via dynamic parameters. Tightening
  these can only make the machine *more* conservative at runtime; validation
  (§4) refuses values looser than the compile-time safe envelope.
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

Bringup is self-managed: the `autostart` parameter (default true) makes the node
configure and activate itself in its constructor, so a bare `ros2 run` comes up
active with no `nav2_lifecycle_manager` or launch event handlers. Set
`autostart:=false` to hold the node in `unconfigured` for manual/orchestrated
lifecycle control (tests, bench, an external lifecycle manager) — useful when an
operator needs the node configured but not yet servicing.

---

## 4. Parameters — file-driven, validated, partly dynamic

Loaded on start from a **ROS 2 params YAML** (`--ros-args --params-file`), the
idiomatic mechanism. We use **`generate_parameter_library`** (PickNik, standard
on Jazzy) to declare params in one YAML, auto-generate a typed struct with
**validation (ranges/enums)**, `read_only` flags, and a dynamic-update hook —
eliminating hand-rolled `declare_parameter` + validation boilerplate.

**Static (`read_only`, require reconfigure):**
`backend` (`software|hardware`), `machine_id`, software `bind_addr`/`port`,
software `default_stop_only`/`operators` (operator authorization — see below),
hardware `device_url`/`admin_user`, `frame_id`, topic QoS depths.

**Dynamic (runtime-settable, re-validated on set):**
`heartbeat_ms` (≥ safe floor), `max_missed` (≥ 1), `min_stop_ms`,
`state_poll_hz`, `publish_rate_hz`, `diagnostics_rate_hz`.

Validation rule: a dynamic set that would **loosen** the safety envelope past the
compile-time floor is **rejected** (the set-callback returns `successful=false`
with a reason), so ROS 2 can only make the machine safer, never less safe.

A `config_file` convenience param may point at a machine-specific YAML the node
merges in `on_configure` for operators who prefer one file per unit; the
ROS-native params-file remains the source of truth.

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
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` via `diagnostic_updater` | default | backend reachability, bond count, faults, mismatch, loop jitter |

Reuse `protective_stop_msg` (restore from `archive/`); add the two machine-side
messages there so remote/machine share one interface package.

### Services (bridge + control)
| Service | Type | Effect |
|---|---|---|
| `~/get_status` | `Trigger`-like | synchronous snapshot for scripting |

Runtime timing reconfiguration is done through dynamic parameters (validated
against the safety floor on `on_set_parameters`), not a dedicated service.

No `arm` service (see §2).

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
  so a slow HTTP round-trip can't stall state publication.
- Publishers/services live in a `MutuallyExclusive` group. A `MultiThreaded
  Executor` runs the node.
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

## 7. Package layout

```
protective_stop_machine/                 # ament_cmake, C++17
  include/protective_stop_machine/
    machine_bridge_node.hpp
    machine_backend.hpp                  # IMachineBackend + POD snapshot/timing
    software_backend.hpp
    hardware_backend.hpp
  src/
    machine_bridge_node.cpp              # lifecycle + ROS glue only
    software_backend.cpp                 # links pstop_c
    hardware_backend.cpp                 # libcurl + nlohmann/json
    protective_stop_machine_params.yaml  # generate_parameter_library source + defaults
  test/                                   # gtest (backend logic) + launch_testing (transitions)
protective_stop_msg/                      # restored from archive/, + MachineRelayStatus, BondedRemoteArray
```

**Dependencies (Jazzy):** `rclcpp`, `rclcpp_lifecycle`, `lifecycle_msgs`,
`protective_stop_msg`, `diagnostic_updater`, `generate_parameter_library`,
`std_srvs`. Software backend vendors `pstop_c` sources (as `host/` does — the
certified lib is **not** modified). Hardware backend: `libcurl` +
`nlohmann_json` (both rosdep-available).

---

## 8. Safety analysis (invariant re-check)

| Failure | Software backend | Hardware backend |
|---|---|---|
| Node process dies | machine dies → remotes time out → **STOP** | ESP32 unaffected → still enforces; ROS 2 blind (diagnostics ERROR on next consumer) |
| `on_deactivate` | machine loop stops → remotes **STOP** | polling stops; ESP32 enforces |
| DDS/host partition | irrelevant to enforcement (UDP bond is separate from DDS) | irrelevant; ESP32 enforces |
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

1. **Message ownership** — **extend `protective_stop_msg`** (restored from
   `archive/`); add `MachineRelayStatus` + `BondedRemoteArray` there so remote
   and machine share one interface package.
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

There is no launch file — the node is a composable component with a
code-generated executable. Defaults (and the SR-M-01 validation ranges) live in
the `generate_parameter_library` source `src/protective_stop_machine_params.yaml`
and are compiled in, so `autostart` defaults to true and a bare run comes up
active:

```sh
ros2 run protective_stop_machine machine_bridge_node
```

To override any parameter, pass your own params file:

```sh
ros2 run protective_stop_machine machine_bridge_node \
  --ros-args --params-file my_overrides.yaml
```

where `my_overrides.yaml` looks like:

```yaml
/machine_bridge:
  ros__parameters:
    backend: software           # software | hardware   (read_only)
    machine_id: 0x01020304      # read_only
    frame_id: pstop_machine

    software:                   # used when backend == software
      bind_addr: 0.0.0.0
      port: 8890                # read_only
      # Operator authorization (SAFETY). A bonded remote is accepted and
      # heartbeat-monitored but STOP-ONLY by default: it may command STOP but may
      # NEVER re-arm (STOP -> OK). Only a remote whose 32-bit pstop id is listed
      # in `operators` gets re-arm authority. Empty list (the default) = every
      # remote is stop-only = maximally safe out of the box.
      default_stop_only: true   # read_only
      # operators: [30234300]   # read_only; 32-bit pstop ids granted re-arm (default: none)

    hardware:                   # used when backend == hardware
      device_url: http://100.84.155.111
      admin_user: admin         # password via env, never in the file

    timing:                     # dynamic, validated against safe floor
      heartbeat_ms: 400
      max_missed: 3
      min_stop_ms: 500

    rates:
      state_poll_hz: 5.0        # hardware poll
      publish_rate_hz: 10.0
      diagnostics_rate_hz: 1.0

    # Optional fleet check-in (software backend only). Default DISABLED (empty
    # url) so a non-fleet deployment is unaffected. Prefer the ENVIRONMENT for
    # the URL + key file so secrets stay out of committed config —
    # PSTOP_ANNOUNCE_URL and PSTOP_ANNOUNCE_KEY_FILE override these two keys.
    announce:
      url: ''                   # http://host[:port]/path — empty = disabled (read_only)
      key_file: ''              # path whose first line is the bearer token, chmod 600 (read_only)
      name: ''                  # console display name; empty = this host's hostname (read_only)
      interval_s: 60            # seconds between check-ins (read_only)
```
