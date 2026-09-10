# Network-owner handoff: recurring upstream resets during PSTOP06 soak

## Requested investigation

Please correlate the UTC windows below with gateway/WAN health checks, failover
and recovery decisions, routing policy and NAT session logs. Confirm whether the
two observed public addresses are intended egress paths, and whether established
TCP sessions can change external source address during a path change.

The exact gateway policy is **not yet known**. The evidence supports a shared
upstream egress/NAT disruption; it does not identify a particular router defect.
No gateway configuration has been changed during this investigation.

## Observed windows on 2026-09-10

| Pair | First disturbance, UTC | Second disturbance, UTC |
| --- | --- | --- |
| 1 | 05:23:53 | 05:25:02 |
| 2 | 08:29:13–14 | 08:30:21–22 |
| 3 | 15:39:04–05 | 15:40:12–13 |
| 4 | 16:08:24 | 16:09:32–33 |
| 5 | 17:05:24–25 | 17:06:32–33 |

Times identify observed packets/disruptions, not exact router decision times.
Each pair is approximately 68 seconds apart; pair-to-pair spacing varies.

## Topology and evidence

- Bench: `hq0-dev-test01`, LAN `10.74.30.176`, default gateway `10.74.28.1`.
- DUT USB-NCM: `10.43.0.122`, forwarded/NATed by the bench during USB mode.
- Remote ROS machine: Tailscale `100.110.35.58`, at another site.
- Public endpoint observations repeatedly alternate between `192.184.222.186`
  and `104.59.120.100`; cached old endpoints remain in Tailscale's reported list.
- The DUT's DERP and control-plane TCP connections receive RSTs. Matching packets
  are visible arriving on the bench uplink before reaching USB, with the normal
  one-hop TTL decrement.
- The bench's **own** Tailscale relay, control and log-upload TCP connections
  reset in the same windows. The remote machine site's link remained healthy.
- USB URBs continue successfully during inspected outages; no global USB freeze
  or device reboot explains these shared resets.

These events are distinct from the separately documented USB management-response
duplication/loss and comparator-timing investigations.

## Impact and qualification policy

The machine stops when its 1600 ms silence budget is exceeded. Inspected STOP
latencies remained below the required two-second limit, but the interruptions
invalidate the consecutive-clean USB qualification window and require re-arming.
Events below that silence budget can still invalidate qualification through send
failures, return-path loss or missing observation coverage.

Ethernet completed a verified four-hour clean window on the same DUT build.
No bench reset/public-endpoint-change entries were found during that passed
window, so it is **not** evidence of immunity to WAN failover.

The USB campaign continues. WAN events are not excluded, and timeouts have not
been relaxed to obtain a pass. A stable upstream path, or identification and
correction of the recurring disruption, is needed for reliable qualification.

## Retained evidence

Bench root: `/tmp/opencode/pstop-soak-20260910/`. It contains per-event
assessments, raw telemetry, `host-tailscaled.log`, USB/uplink captures and usbmon.
`capture-archive/20260910T171457Z/manifest.json` records SHA-256 hashes for a
1.074 GB snapshot of 45 capture files outside the rolling buffers.
The peer independently archived its pcaps, event journal and evidence bundles.
See also `SOAK_INVESTIGATION_2026-09-10.md` and the campaign manifests.
