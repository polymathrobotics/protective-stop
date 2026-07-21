#!/usr/bin/env python3
"""Chaos / soak stress test for dual_core_safety on the Waveshare ESP32-S3-ETH.

Continuously probes the device's reachability + telemetry while injecting chaos
events (reboots, Ethernet drops, WiFi failover, USB / WireGuard toggles, cable
flaps), then reports stability metrics: ping success rate, RTT distribution,
lockstep mismatches, unexpected reboots (crashes), heap-floor, and pstop reply
rate.

Reachability is measured via HTTP GET /state.json (tries the Ethernet IP, then
the WiFi IP) — a stronger test than ICMP since it exercises the app + lwIP path.
ICMP RTT is sampled on whichever IP is currently answering.

Restores of disrupted interfaces are idempotent and retried until confirmed, so
the device is never left in a disrupted state even if a chaos window overlaps a
reboot. Reboots are gated on uptime > 140 s so the safety layer's rapid-boot
counter (rolls back firmware after 3 boots within 120 s) is never tripped.

Usage:  python3 chaos_soak.py [duration_seconds]      (default 2400 = 40 min)
"""
import urllib.request, json, subprocess, time, re, sys, base64

TS  = "100.122.25.95"        # Tailscale (STATIC) — primary control channel. The chip's
                             # eth lease is work-LAN-dependent and this host is usually off
                             # that LAN, so Tailscale is the only reliable path. RTT here
                             # reflects the DERP/WG tunnel (~100ms), not the local link.
USB = "10.42.0.109"          # USB-NCM tether (host-shared); reachable once failover brings it up
ETH = "10.74.29.167"         # chip's current work-LAN eth IP (NOT reachable from this host)
WIFI = "192.168.107.97"      # stale (old LAN) — kept only as a last-ditch fallback
AUTH = "Basic " + base64.b64encode(b"admin:microlink").decode()
DUR = int(sys.argv[1]) if len(sys.argv) > 1 else 2400
SAMPLE = 2.0          # seconds between probes
CHAOS_GAP = 95        # seconds between chaos events
REBOOT_MIN_UPTIME = 140  # don't reboot below this (avoid rapid-boot rollback)


def log(m):
    print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)


def state(timeout=4):
    """Return (dict, host, http_rtt_ms) from /state.json. Tailscale FIRST: this
    host is off the chip's work LAN, so its eth/wifi IPs are unreachable here and
    Tailscale (static) is the dependable channel that rides whatever uplink is
    active — so crash/reboot detection (reading uptime) never loses the device,
    even across an Ethernet drop. http_rtt over TS reflects the tunnel, not the
    local link; the crash/failover/heap metrics are what this soak validates."""
    for h in (TS, USB, ETH, WIFI):
        try:
            t0 = time.time()
            r = urllib.request.urlopen(f"http://{h}/state.json", timeout=timeout)
            return json.loads(r.read()), h, (time.time() - t0) * 1000.0
        except Exception:
            continue
    return None, None, None


def icmp(host):
    try:
        o = subprocess.run(["ping", "-c", "1", "-W", "1", host],
                           capture_output=True, text=True, timeout=3)
        if o.returncode == 0:
            m = re.search(r"time[=<]([\d.]+)", o.stdout)
            return float(m.group(1)) if m else 0.0
    except Exception:
        pass
    return None


def post(path, host, auth=False, timeout=10):
    try:
        req = urllib.request.Request(f"http://{host}{path}", method="POST")
        if auth:
            req.add_header("Authorization", AUTH)
        urllib.request.urlopen(req, timeout=timeout)
        return True
    except Exception:
        return False


def host_now():
    _, h, _ = state(2)
    return h or ETH


# ---- idempotent "ensure" closures: return True once the target state is confirmed ----
def ensure_field(field, want_on):
    def f():
        d, h, _ = state(2)
        if not d:
            return False                       # unreachable — retry later
        if bool(d.get(field)) == want_on:
            return True                        # confirmed
        path = {"eth_en": "/api/iface/eth", "wifi_en": "/api/iface/wifi",
                "usbncm_en": "/api/iface/usb", "wg_paused": "/api/wg"}[field]
        post(path, h)
        return False                           # toggled; confirm next cycle
    return f


def fmt(vs):
    if not vs:
        return "n/a"
    s = sorted(vs)
    n = len(s)
    q = lambda p: s[min(n - 1, int(p * n))]
    mean = sum(s) / n
    return (f"n={n} min={s[0]:.1f} mean={mean:.1f} p50={q(0.5):.1f} "
            f"p95={q(0.95):.1f} p99={q(0.99):.1f} max={s[-1]:.1f}")


# pending restores: [fire_time, ensure_fn, desc, expire_time]
pending = []
def defer(delay, fn, desc, window=70):
    t = time.time()
    pending.append([t + delay, fn, desc, t + delay + window])


M = dict(samples=0, reach=0, icmp=[], http=[], mm_max=0, heap_min=10**9,
         reboots_obs=0, reboots_int=0, max_unreach=0, events=0,
         ps_sent0=None, ps_repl0=None, ps_sent=0, ps_repl=0,
         rebonds0=None, rebonds_last=0)

# raw per-sample log for post-hoc percentiles / plotting
SAMP = open("/tmp/chaos_samples.csv", "w")
SAMP.write("t,reach,icmp_ms,http_ms,active,mm,heap_min,rebonds,eth_en,wifi_en\n")

CHAOS = ["eth_short", "wg_disrupt", "eth_long", "usb_toggle", "reboot",
         "wifi_manual", "eth_flap", "reboot"]

start = time.time()
next_chaos = start + 25
last_up = None
last_reboot_t = 0
unreach_streak = 0
ci = 0

log(f"=== CHAOS SOAK: {DUR}s ({DUR//60} min), chaos every {CHAOS_GAP}s ===")

while time.time() - start < DUR:
    c0 = time.time()
    try:
        # fire / retry deferred restores
        for item in pending[:]:
            if c0 >= item[0]:
                ok = item[1]()
                if ok or c0 > item[3]:
                    if not ok:
                        log(f"  (restore EXPIRED unconfirmed: {item[2]})")
                    else:
                        log(f"  (restored: {item[2]})")
                    pending.remove(item)

        # sample
        M['samples'] += 1
        d, h, hrtt = state()
        if d:
            M['reach'] += 1
            unreach_streak = 0
            if hrtt:
                M['http'].append(hrtt)
            r = icmp(h)
            if r is not None:
                M['icmp'].append(r)
            M['mm_max'] = max(M['mm_max'], d.get("pstop_mismatch", 0) or 0)
            hm = d.get("heap_min_int") or 0
            if hm:
                M['heap_min'] = min(M['heap_min'], hm)
            up = (d.get("uptime_ms", 0) or 0) // 1000
            ps, pr = d.get("pstop_sent", 0) or 0, d.get("pstop_replies", 0) or 0
            if M['ps_sent0'] is None:
                M['ps_sent0'], M['ps_repl0'] = ps, pr
            M['ps_sent'], M['ps_repl'] = ps, pr
            rb = d.get("pstop_rebonds", 0) or 0
            if M['rebonds0'] is None:
                M['rebonds0'] = rb
            M['rebonds_last'] = rb
            if last_up is not None and up < last_up - 5 and up < 40:
                M['reboots_obs'] += 1
                unexpected = (c0 - last_reboot_t) > 50
                log(f"  >>> REBOOT detected uptime {last_up}->{up}s "
                    f"{'[UNEXPECTED — crash]' if unexpected else '[intentional]'}")
            last_up = up
            SAMP.write(f"{int(c0-start)},1,{r if r is not None else ''},{hrtt:.1f},"
                       f"{d.get('active_iface')},{d.get('pstop_mismatch',0) or 0},{hm},{rb},"
                       f"{1 if d.get('eth_en') else 0},{1 if d.get('wifi_en') else 0}\n")
        else:
            unreach_streak += 1
            M['max_unreach'] = max(M['max_unreach'], unreach_streak)
            SAMP.write(f"{int(c0-start)},0,,,,,,,,\n")

        # chaos event
        if c0 >= next_chaos:
            act = CHAOS[ci % len(CHAOS)]
            ci += 1
            d, h, _ = state(2)
            up = ((d or {}).get("uptime_ms", 0) or 0) // 1000
            h = h or ETH
            if act == "reboot":
                if up > REBOOT_MIN_UPTIME:
                    post("/admin/api/restart", h, auth=True)
                    M['reboots_int'] += 1
                    last_reboot_t = c0
                    log(f"CHAOS #{ci}: REBOOT (uptime was {up}s)")
                    M['events'] += 1
                else:
                    log(f"CHAOS #{ci}: reboot SKIPPED (uptime {up}s < {REBOOT_MIN_UPTIME})")
            elif act == "eth_short":
                post("/api/iface/eth", h) if d and d.get("eth_en") else None
                defer(12, ensure_field("eth_en", True), "eth on")
                log(f"CHAOS #{ci}: Ethernet DROP 12s"); M['events'] += 1
            elif act == "eth_long":
                post("/api/iface/eth", h) if d and d.get("eth_en") else None
                defer(40, ensure_field("eth_en", True), "eth on")
                log(f"CHAOS #{ci}: Ethernet DROP 40s (force WiFi failover)"); M['events'] += 1
            elif act == "eth_flap":
                post("/api/iface/eth", h) if d and d.get("eth_en") else None
                defer(4, ensure_field("eth_en", True), "flap up")
                defer(8, ensure_field("eth_en", False), "flap down")
                defer(12, ensure_field("eth_en", True), "flap final up")
                log(f"CHAOS #{ci}: Ethernet FLAP"); M['events'] += 1
            elif act == "wifi_manual":
                post("/api/iface/wifi", h) if d and not d.get("wifi_en") else None
                defer(20, ensure_field("wifi_en", False), "wifi off (let supervisor own it)")
                log(f"CHAOS #{ci}: WiFi manual enable 20s"); M['events'] += 1
            elif act == "usb_toggle":
                post("/api/iface/usb", h)
                defer(20, ensure_field("usbncm_en", False), "usb off")
                log(f"CHAOS #{ci}: USB-NCM toggle 20s"); M['events'] += 1
            elif act == "wg_disrupt":
                post("/api/wg", h)
                defer(15, ensure_field("wg_paused", False), "wg resume")
                log(f"CHAOS #{ci}: WireGuard pause 15s"); M['events'] += 1
            next_chaos = c0 + CHAOS_GAP

        # periodic summary every ~60 s
        if M['samples'] % 30 == 0:
            SAMP.flush()
            sr = 100.0 * M['reach'] / M['samples']
            log(f"... t={int(c0-start)}s reach={sr:.1f}% icmp[{fmt(M['icmp'])}] "
                f"mm_max={M['mm_max']} reboots(obs/int)={M['reboots_obs']}/{M['reboots_int']} "
                f"heap_min={M['heap_min']} max_unreach={M['max_unreach']*SAMPLE:.0f}s")
    except Exception as e:
        log(f"  (sampler exception: {e})")
    time.sleep(max(0, SAMPLE - (time.time() - c0)))

# leave the device in a clean state: Ethernet on, USB off, WG running
log("=== restoring clean state (eth on, usb off, wg unpaused) ===")
for fn, desc in ((ensure_field("eth_en", True), "eth on"),
                 (ensure_field("usbncm_en", False), "usb off"),
                 (ensure_field("wg_paused", False), "wg run")):
    for _ in range(15):
        if fn():
            break
        time.sleep(2)

sr = 100.0 * M['reach'] / max(1, M['samples'])
psent = M['ps_sent'] - (M['ps_sent0'] or 0)
prepl = M['ps_repl'] - (M['ps_repl0'] or 0)
prate = 100.0 * prepl / psent if psent else 0.0
crashes = max(0, M['reboots_obs'] - M['reboots_int'])
log("=" * 64)
log("=== SOAK FINAL REPORT ===")
log(f"duration           : {int(time.time()-start)}s   samples: {M['samples']}")
log(f"reachability       : {sr:.2f}%  ({M['reach']}/{M['samples']})  "
    f"[downtime incl. intentional reboots/outages]")
log(f"longest outage     : {M['max_unreach']*SAMPLE:.0f}s")
log(f"ICMP ping RTT (ms) : {fmt(M['icmp'])}")
log(f"HTTP RTT (ms)      : {fmt(M['http'])}")
log(f"pstop path         : sent+{psent} replies+{prepl}  reply-rate {prate:.1f}% "
    f"(note: dual-core redundant send, ~50% nominal)")
rebonds_d = M['rebonds_last'] - (M['rebonds0'] or 0)
log(f"pstop rebonds      : +{rebonds_d}  (link re-syncs; chaos-driven is expected, "
    f"a flat baseline is the goal)")
log(f"lockstep mismatch  : max {M['mm_max']}")
log(f"reboots            : observed {M['reboots_obs']}  intentional {M['reboots_int']}  "
    f"==> UNEXPECTED/CRASHES: {crashes}")
log(f"internal heap_min  : {M['heap_min']} bytes (low-water over soak)")
log(f"chaos events fired : {M['events']}")
verdict = "PASS" if (crashes == 0 and M['mm_max'] <= 2 and M['heap_min'] > 8000) else "REVIEW"
log(f"VERDICT            : {verdict}")
log(f"raw samples        : /tmp/chaos_samples.csv")
log("=" * 64)
SAMP.close()
