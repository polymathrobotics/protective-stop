#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
"""flash_station.py — production flashing station for pstop remotes.

Runs as an unattended loop: plug in a blank ESP32-S3, it auto-detects the
chip in download mode, flashes the app, waits for the unit to boot and join
Tailscale, prints its tailnet IP, then makes the operator confirm the button
+ LED ring work before the unit is called good. Unplug it, plug in the next
one — repeat.

The app image comes from a pluggable **build source**. If an optional local
plugin `tools/app_source.py` is present, the station pulls the latest verified
app from it by default (sha256-verified) and always flashes the freshest build;
that plugin is intentionally NOT part of this repo. With no plugin (or with
--from-image) the station flashes the locally-staged
tools/production_image/pstop_remote.bin. The stable boot trio (bootloader,
partition-table, ota_data) always comes from tools/production_image/.

A live progress bar tracks each phase; every unit ends in a clear PASS/FAIL
line and the session keeps a running tally. Nothing here rebuilds firmware or
needs ESP-IDF — just esptool + the image.

    tools/flash_station.py                 # loop; use the build-source plugin if present
    tools/flash_station.py --from-image    # flash the locally staged app, no fetch
    tools/flash_station.py --fw-sha 40f2   # pin a specific build by sha prefix (plugin)
    tools/flash_station.py --once          # flash a single unit and exit
    tools/flash_station.py --erase         # full chip-erase before each flash
    tools/flash_station.py --selftest      # exercise the plumbing, no hardware

tools/production_image/ carries secrets (Tailscale key, WiFi creds, admin
password) and is git-ignored — never commit it.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time
import urllib.request

# Optional LOCAL build-source plugin (e.g. an internal build server). It is
# git-ignored and not shipped with this repo; when present it lets the station
# flash the latest verified build. Contract: LABEL, status()->(ok,msg),
# latest(project, pin_sha)->img|None, download(img, dest)->(ok,err).
try:
    import app_source as _prov  # noqa: E402  (local-only, git-ignored)
except Exception:
    _prov = None

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, "production_image")
# Offsets mirror tools/flash_pstop.sh — keep the two in sync.
IMAGES = [
    (0x0, "bootloader.bin"),
    (0x8000, "partition-table.bin"),
    (0x19000, "ota_data_initial.bin"),
    (0x20000, "pstop_remote.bin"),
]
CHIP = "esp32s3"
BAUD = "460800"
ESP_VID = "303a"          # Espressif
RUNNING_PID = "4001"      # a booted pstop app (USB-CDC) — NOT flashable
HOST_PREFIX = "pstop-01"  # node name = pstop-01<mac24>, mac24 = last 3 MAC bytes
BAR_W = 34


# --- tiny ANSI helpers ----------------------------------------------------
class C:
    G = "\033[32m"; R = "\033[31m"; Y = "\033[33m"; B = "\033[36m"
    DIM = "\033[2m"; BOLD = "\033[1m"; X = "\033[0m"


def bar(frac, phase, extra=""):
    """Render an in-place progress bar. frac in [0,1]."""
    frac = max(0.0, min(1.0, frac))
    fill = int(BAR_W * frac)
    b = "█" * fill + "░" * (BAR_W - fill)
    sys.stdout.write(f"\r  {C.B}{b}{C.X} {int(frac*100):3d}%  {phase:<12}{extra}   ")
    sys.stdout.flush()


def line(msg):
    sys.stdout.write("\r\033[K" + msg + "\n")
    sys.stdout.flush()


# --- esptool resolution + v4/v5 flag differences --------------------------
def esptool_cmd():
    for c in (["esptool"], ["esptool.py"], [sys.executable, "-m", "esptool"]):
        try:
            subprocess.run(c + ["version"], capture_output=True, timeout=10)
            return c
        except Exception:
            continue
    sys.exit(f"{C.R}ERROR: esptool not found — 'pip install esptool'.{C.X}")


def esptool_v5(cmd):
    try:
        out = subprocess.run(cmd + ["version"], capture_output=True, text=True, timeout=10)
        m = re.search(r"(\d+)\.\d+", out.stdout + out.stderr)
        return bool(m) and int(m.group(1)) >= 5
    except Exception:
        return False


# --- device detection -----------------------------------------------------
def udev_prop(port, key):
    try:
        out = subprocess.run(["udevadm", "info", "-q", "property", "-n", port],
                             capture_output=True, text=True, timeout=5).stdout
        m = re.search(rf"^{key}=(.*)$", out, re.M)
        return m.group(1) if m else ""
    except Exception:
        return ""


def download_ports():
    """Espressif chips currently sitting in download mode (flashable)."""
    ports = []
    for base in ("/dev/ttyACM", "/dev/ttyUSB"):
        for n in range(8):
            p = f"{base}{n}"
            if not os.path.exists(p):
                continue
            if udev_prop(p, "ID_VENDOR_ID") != ESP_VID:
                continue
            if udev_prop(p, "ID_MODEL_ID") == RUNNING_PID:
                continue  # booted app, not download mode
            ports.append(p)
    return ports


def wait_for_blank(known):
    """Block until a download-mode port appears that isn't in `known`."""
    spin = "|/-\\"
    i = 0
    while True:
        for p in download_ports():
            if p not in known:
                return p
        sys.stdout.write(f"\r  {C.DIM}waiting for a blank ESP32 in download mode "
                         f"{spin[i % 4]}{C.X}   ")
        sys.stdout.flush()
        i += 1
        time.sleep(0.4)


def read_mac(cmd, port):
    """Return (mac_str, mac24_hex) or (None, None)."""
    try:
        out = subprocess.run(cmd + ["--chip", CHIP, "-p", port, "read_mac"],
                             capture_output=True, text=True, timeout=30)
        m = re.search(r"MAC:\s*([0-9a-fA-F:]{17})", out.stdout + out.stderr)
        if not m:
            return None, None
        mac = m.group(1).lower()
        mac24 = mac.replace(":", "")[-6:]
        return mac, mac24
    except Exception:
        return None, None


# --- the flash, with a byte-accurate progress bar -------------------------
def flash(cmd, port, v5, erase, on_progress):
    for _, fn in IMAGES:
        if not os.path.isfile(os.path.join(IMG, fn)):
            return False, f"missing {IMG}/{fn} — stage a build first"

    sizes = {base: os.path.getsize(os.path.join(IMG, fn)) for base, fn in IMAGES}
    total = sum(sizes.values())
    ordered = sorted(sizes.items())  # (base, size) low→high

    def done_bytes(addr):
        acc = 0
        for base, size in ordered:
            if addr >= base + size:
                acc += size
            elif addr >= base:
                acc += addr - base
        return acc

    o = (("write-flash", "erase-flash", "--flash-mode", "--flash-size",
          "--flash-freq", "default-reset", "hard-reset") if v5 else
         ("write_flash", "erase_flash", "--flash_mode", "--flash_size",
          "--flash_freq", "default_reset", "hard_reset"))
    WF, EF, MODE, SIZE, FREQ, BEFORE, AFTER = o

    if erase:
        on_progress(0.0, "erase")
        r = subprocess.run(cmd + ["--chip", CHIP, "-p", port, "-b", BAUD, EF],
                           capture_output=True, text=True)
        if r.returncode != 0:
            return False, "erase failed: " + (r.stderr or r.stdout)[-200:].strip()

    args = cmd + ["--chip", CHIP, "-p", port, "-b", BAUD,
                  "--before", BEFORE, "--after", AFTER,
                  WF, MODE, "dio", SIZE, "8MB", FREQ, "80m"]
    for base, fn in IMAGES:
        args += [hex(base), os.path.join(IMG, fn)]

    proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)
    tail = []
    buf = ""
    while True:
        ch = proc.stdout.read(1)
        if ch == "":
            break
        if ch in ("\r", "\n"):
            if buf.strip():
                tail.append(buf.strip())
                tail[:] = tail[-8:]
                m = re.search(r"Writing at 0x([0-9a-fA-F]+)", buf)
                if m:
                    on_progress(done_bytes(int(m.group(1), 16)) / total, "flash")
            buf = ""
        else:
            buf += ch
    proc.wait()
    if proc.returncode != 0:
        return False, "esptool: " + " | ".join(tail[-4:])
    on_progress(1.0, "flash")
    return True, ""


# --- Tailscale IP discovery ----------------------------------------------
def ts_status_hosts():
    try:
        out = subprocess.run(["tailscale", "status"], capture_output=True,
                             text=True, timeout=10).stdout
    except Exception:
        return {}
    hosts = {}
    for ln in out.splitlines():
        f = ln.split()
        if len(f) >= 2 and f[1].startswith("pstop-"):
            hosts[f[1]] = "offline" not in ln.lower()
    return hosts


def wait_for_ip(mac24, pre_hosts, timeout, on_progress):
    """Wait until this unit's node is online; return (ip, host) or (None, host)."""
    want = HOST_PREFIX + mac24 if mac24 else None
    t0 = time.time()
    while time.time() - t0 < timeout:
        on_progress(min(0.99, (time.time() - t0) / timeout), "tailscale")
        hosts = ts_status_hosts()
        # exact match on the derived hostname (preferred)
        if want and hosts.get(want):
            ip = ts_ip(want)
            if ip:
                on_progress(1.0, "tailscale")
                return ip, want
        # fallback: any pstop node that is newly present and online
        for h, up in hosts.items():
            if up and h not in pre_hosts:
                ip = ts_ip(h)
                if ip:
                    on_progress(1.0, "tailscale")
                    return ip, h
        time.sleep(2)
    return None, want


def ts_ip(host):
    try:
        out = subprocess.run(["tailscale", "ip", "-4", host],
                             capture_output=True, text=True, timeout=8)
        ip = out.stdout.strip().splitlines()
        return ip[0] if ip else None
    except Exception:
        return None


def unit_state(ip):
    """Return the unit's /state.json dict, or None."""
    try:
        with urllib.request.urlopen(f"http://{ip}/state.json", timeout=4) as r:
            return json.loads(r.read().decode())
    except Exception:
        return None


# --- operator button/LED sign-off ----------------------------------------
def confirm_hardware(ip):
    """Manual QA gate: operator verifies the button + LED ring physically."""
    line(f"  {C.BOLD}Manual check:{C.X} open {C.B}http://{ip}/{C.X} and on the unit:")
    line(f"    1. Watch the LED ring — it should light (yellow=disconnected → "
         f"green/colour once bonded).")
    line(f"    2. Press the STOP button — the ring should react and the page's "
         f"status should change.")
    while True:
        ans = input(f"  {C.BOLD}Do the button AND LED work? [y/N/r=retry link] {C.X}").strip().lower()
        if ans == "r":
            line(f"    → http://{ip}/   (tailnet IP {ip})")
            continue
        if ans in ("y", "yes"):
            return True, ""
        note = input("  Note the failure (what's wrong): ").strip()
        return False, note or "operator reported button/LED not working"


# --- one full unit cycle --------------------------------------------------
def flash_one(cmd, v5, port, erase, ip_timeout, app_ver="?"):
    line(f"{C.BOLD}▶ unit on {port}{C.X}  (app {app_ver})")
    bar(0.02, "read-mac")
    mac, mac24 = read_mac(cmd, port)
    if not mac24:
        line(f"  {C.R}✗ chip not responding on {port} (bad cable / not in download "
             f"mode).{C.X}")
        return False, {"port": port, "stage": "read-mac"}
    host = HOST_PREFIX + mac24
    line(f"  MAC {mac}  →  node {C.B}{host}{C.X}")

    pre = ts_status_hosts()

    span = {"flash": (0.05, 0.75), "tailscale": (0.75, 1.0)}

    def prog(frac, phase):
        lo, hi = span.get(phase, (0.05, 0.75))
        if phase == "erase":
            lo, hi = 0.03, 0.05
        elif phase == "read-mac":
            lo, hi = 0.0, 0.05
        bar(lo + (hi - lo) * frac, phase)

    ok, err = flash(cmd, port, v5, erase, prog)
    if not ok:
        line(f"  {C.R}✗ FLASH FAILED: {err}{C.X}")
        return False, {"port": port, "node": host, "stage": "flash", "err": err}
    line(f"  {C.G}✓ flashed{C.X} — rebooting, joining Tailscale…")

    ip, found = wait_for_ip(mac24, pre, ip_timeout, prog)
    if not ip:
        line(f"  {C.R}✗ no tailnet IP for {host} within {ip_timeout}s "
             f"(check DHCP / Tailscale key / Ethernet link).{C.X}")
        return False, {"port": port, "node": host, "stage": "tailscale"}
    if found != host:
        line(f"  {C.Y}! joined as {found} (expected {host}) — verify identity.{C.X}")
    st = unit_state(ip)
    line(f"  {C.G}✓ online at {C.BOLD}{ip}{C.X}{C.G} "
         f"({'admin page reachable' if st else 'no HTTP yet — may still be booting'}){C.X}")
    # confirm the running build is the one we flashed
    run_fw = (st or {}).get("fw_ver")
    if app_ver != "?" and run_fw and not run_fw.startswith(app_ver):
        line(f"  {C.Y}! running fw_ver {run_fw} != flashed {app_ver} — "
             f"boot slot / stale image?{C.X}")

    good, note = confirm_hardware(ip)
    if not good:
        line(f"  {C.R}✗ HARDWARE CHECK FAILED: {note}{C.X}")
        return False, {"port": port, "node": host, "ip": ip, "stage": "hw-check", "err": note}
    line(f"  {C.G}{C.BOLD}✓ PASS{C.X}{C.G}  {host} @ {ip} — button + LED confirmed.{C.X}\n")
    return True, {"port": port, "node": host, "ip": ip}


# --- build source (optional plugin) --------------------------------------
APP_BIN = os.path.join(IMG, "pstop_remote.bin")   # IMAGES[-1] target
BOOT_TRIO = ["bootloader.bin", "partition-table.bin", "ota_data_initial.bin"]


def source_refresh(project, pin_sha, current_sha):
    """Via the plugin, refresh APP_BIN to the latest build if it changed.
    Returns (img_dict, changed_bool); (None, False) if unavailable or on error."""
    if _prov is None:
        return None, False
    try:
        img = _prov.latest(project, pin_sha)
    except Exception as e:
        line(f"  {C.R}build source: {e}{C.X}")
        return None, False
    if not img:
        return None, False
    if img.get("sha256") == current_sha and os.path.isfile(APP_BIN):
        return img, False
    ok, err = _prov.download(img, APP_BIN)
    if not ok:
        line(f"  {C.R}build source: {err}{C.X}")
        return None, False
    return img, True


# --- selftest (no hardware) ----------------------------------------------
def selftest(cmd, v5):
    print(f"{C.BOLD}flash_station selftest{C.X}")
    print(f"  esptool: {' '.join(cmd)}  (v5 flags: {v5})")
    miss = [fn for _, fn in IMAGES if not os.path.isfile(os.path.join(IMG, fn))]
    print(f"  image dir {IMG}: " +
          (f"{C.G}complete{C.X}" if not miss else f"{C.Y}missing {miss}{C.X}"))
    print(f"  download-mode ports now: {download_ports() or 'none'}")
    hosts = ts_status_hosts()
    print(f"  tailscale sees {len(hosts)} pstop nodes; "
          f"online: {[h for h, up in hosts.items() if up] or 'none'}")
    for h in list(hosts)[:1]:
        print(f"  ts_ip({h}) = {ts_ip(h)}")
    if _prov is None:
        print(f"  build source: {C.Y}no plugin (tools/app_source.py) — "
              f"staged image only{C.X}")
    else:
        ok, msg = _prov.status()
        col = C.G if ok else C.Y
        print(f"  build source: {col}{_prov.LABEL}: {msg}{C.X}")
    print("  progress-bar demo:")
    for i in range(0, 101, 5):
        bar(i / 100, "demo")
        time.sleep(0.01)
    line(f"  {C.G}✓ selftest done{C.X}")


def main():
    ap = argparse.ArgumentParser(description="Production flashing station for pstop remotes.")
    ap.add_argument("--once", action="store_true", help="flash one unit and exit")
    ap.add_argument("--erase", action="store_true", help="full chip erase before each flash")
    ap.add_argument("--ip-timeout", type=int, default=90, help="seconds to wait for a tailnet IP")
    ap.add_argument("--from-image", action="store_true",
                    help="flash the locally-staged app; ignore the build-source plugin")
    ap.add_argument("--fw-sha", metavar="PREFIX",
                    help="pin a specific build by sha256 prefix (build-source plugin)")
    ap.add_argument("--project", default="pstop_remote",
                    help="build-source project name (default pstop_remote)")
    ap.add_argument("--no-refresh", action="store_true",
                    help="fetch latest once at startup, don't re-check per unit")
    ap.add_argument("--selftest", action="store_true", help="exercise plumbing, no hardware")
    args = ap.parse_args()

    cmd = esptool_cmd()
    v5 = esptool_v5(cmd)

    if args.selftest:
        selftest(cmd, v5)
        return

    # The stable boot trio always comes from production_image/ (the plugin
    # supplies only the app). Require it regardless of source.
    boot_miss = [fn for fn in BOOT_TRIO if not os.path.isfile(os.path.join(IMG, fn))]
    if boot_miss:
        sys.exit(f"{C.R}ERROR: boot images {boot_miss} not staged in {IMG} "
                 f"(stage bootloader/partition-table/ota_data once).{C.X}")

    print(f"{C.BOLD}=== pstop flashing station ==={C.X}")

    # --- resolve the app source: build-source plugin (default) or staged image ---
    use_plugin = _prov is not None and not args.from_image
    app_sha = ""          # sha of the app currently in APP_BIN
    app_ver = "?"
    if use_plugin:
        img, _ = source_refresh(args.project, args.fw_sha, app_sha)
        if not img:
            sys.exit(f"{C.R}ERROR: {_prov.LABEL} could not supply an app image "
                     f"(project {args.project}"
                     f"{', sha '+args.fw_sha if args.fw_sha else ''}).{C.X}\n"
                     f"  Fix the source, or pass --from-image to flash the staged app.")
        app_sha, app_ver = img["sha256"], img["version"]
        print(f"  app: {C.G}{_prov.LABEL} {app_ver}{C.X} ({app_sha[:12]}, "
              f"{'starred' if img.get('starred') else 'newest'})"
              + (" [pinned]" if args.fw_sha else ""))
    else:
        if not os.path.isfile(APP_BIN):
            sys.exit(f"{C.R}ERROR: no app image — {APP_BIN} is not staged"
                     + (" and no build-source plugin is present" if _prov is None
                        else "") + f".{C.X}")
        why = "--from-image" if args.from_image else "no build-source plugin"
        print(f"  app: {C.Y}local staged image{C.X} ({why})")
    print("  Plug in a blank ESP32-S3 (download mode). Ctrl-C to quit.\n")

    n_ok = n_fail = 0
    fails = []
    # Start empty: running units enumerate as 303a:4001 and are already excluded
    # by download_ports(), so anything flashable now is a blank chip we should
    # flash. After each cycle `known` is reset to the still-present flashable
    # ports, so a chip that failed (and stays in download mode) isn't
    # re-flashed in a loop — unplug/replug to retry.
    known = set()
    try:
        while True:
            port = wait_for_blank(known)
            # keep "latest" honest: re-check the source before each unit unless
            # pinned or told not to (cheap — one small query; re-download only
            # if the sha changed).
            if use_plugin and not args.no_refresh and not args.fw_sha:
                img, changed = source_refresh(args.project, None, app_sha)
                if img and changed:
                    app_sha, app_ver = img["sha256"], img["version"]
                    line(f"{C.B}  {_prov.LABEL}: newer build → {app_ver} "
                         f"({app_sha[:12]}); flashing it.{C.X}")
            line("")
            ok, info = flash_one(cmd, v5, port, args.erase, args.ip_timeout, app_ver)
            if ok:
                n_ok += 1
            else:
                n_fail += 1
                fails.append(info)
            # this port stays "known" until the unit is unplugged, so we don't
            # re-flash the same chip; a new blank chip = a new port.
            known = set(download_ports())
            print(f"{C.DIM}  session: {C.G}{n_ok} PASS{C.DIM}, "
                  f"{C.R}{n_fail} FAIL{C.DIM}. Unplug this unit, plug in the next.{C.X}\n")
            if args.once:
                break
    except KeyboardInterrupt:
        print()
    print(f"\n{C.BOLD}=== session summary ==={C.X}  "
          f"{C.G}{n_ok} passed{C.X}, {C.R}{n_fail} failed{C.X}")
    for f in fails:
        print(f"  {C.R}✗{C.X} {f.get('node', f.get('port'))} — failed at "
              f"{f['stage']}" + (f": {f['err']}" if f.get("err") else ""))
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
