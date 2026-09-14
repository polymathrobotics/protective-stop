# Armed-soak collector

`tools/soak_stability.py` records and verifies one transport phase. It issues
GETs, retains raw samples/events, and fails closed on missing or stale evidence.
Arming, transport isolation and recovery belong to an external bench controller.

```sh
export ML_ADMIN_USER=admin
export ML_ADMIN_PASSWORD='your-device-password'
python3 tools/soak_stability.py \
  --dut http://DUT_HOST --peer http://PEER_HOST:8895 \
  --machine-id 0x01020393 --remote-id 0x01D7F344 --slot 3 \
  --dut-vpn-ip DUT_TAILSCALE_IP --iface usb --clean-seconds 14400 \
  --poll-sec 1 --peer-poll-sec 0.5 --adaptive-dut-poll --out /tmp/usb-soak
```

Run Ethernet separately with its DUT URL, `--iface ethernet`, and a new output
directory. Replace the example IDs/addresses with the configured bench values.

## Peer contract

This is a contract-specific bench tool. A compatible observer must expose
`/status.json`, `/config.json`, and `/events.jsonl?after=N&limit=1000` using the
`nested-soak-2026-09-10` schema. A generic ROS dashboard is insufficient.

The adapter checks selected-machine/remote identity, actual armed state,
accepted-frame/processing evidence, recorder health, monotonic event sequences,
and configuration provenance/generation. Event records must be complete and
newline-terminated. The normalization helpers and test fixtures specify the
fields. Host clocks must be synchronized within the acknowledged uncertainty.

## Credit and output

- The packet/reply freshness budget is 1600 ms (400 ms × 4 missed windows).
- Faults, coverage gaps and process restarts reset consecutive credit. Recovery
  requires five healthy seconds; historical evidence never earns new credit.
- Adaptive reads retain the existing expiry: minimum interval 0.5 s, at most
  six per minute and sixty per hour. Late results cannot repair expired credit.
- Completion requires post-cutoff configuration/health checks and processing
  watermarks. Exit 0 is a verified pass; 2 is incomplete/error; 130/143 interrupted.
- JSONL streams and event snapshots preserve the evidence. An optional
  `--failure-hook` receives the event filename; its actions are external to this
  collector. Recordings can contain sensitive diagnostics; keep them private.

Sampled telemetry does not measure physical STOP latency. Review controlled
cuts against the last accepted protocol frame separately.

```sh
python3 -B -m unittest discover -s tools -p 'test_soak*.py'
```

Tests cover clock bounds, cumulative evidence, gaps/restarts, failed workers,
adaptive scheduling and closing fences. Hardware qualification is a separate
activity; the existence of the tool or a passing unit suite is not a soak pass.
