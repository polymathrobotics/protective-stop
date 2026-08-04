> ⚠️ **Historical (2026-05-25 brick-recovery doc).** Written when the
> chip had bricked itself via a dual-invalid OTA partition. The
> never-brick guard introduced in v15.3.3 (see `main.c::app_main`)
> prevents that failure mode now. Keep as a reference for the recovery
> procedure if it ever happens again, but in normal operation the
> bootloader rollback chain (verified v15.26) handles bad OTAs
> automatically. Current state:
> [`TAILSCALE_FINAL_2026-05-26.md`](TAILSCALE_FINAL_2026-05-26.md);
> troubleshooting: [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md).

# Read first when you get back

The chip is currently USB-disconnected — `lsusb -d 303a:` returns
nothing, has for ~30 min as I write this. Last USB enumeration was
at `19:31:22` (May 25 2026 PDT).

I tried recovery without buttons. I cannot — no passwordless sudo
to flip USB port power via `/sys`, no `uhubctl` installed, no other
software path to power-cycle the chip from this side. The bricked
chip needs physical intervention.

## Two-minute recovery procedure

```sh
# 1. Physical: unplug the chip's USB-C cable, count to 5, plug back.
#    Then watch:
watch -n 1 'lsusb -d 303a:'

# Case A — chip enumerates as 303a:4001:
#   The bootloader managed to boot one of the partitions.
#   curl -m 3 -s http://10.42.0.80/state.json | jq
#   then continue from "next steps after recovery" below.

# Case B — chip enumerates as 303a:1001:
#   ROM USB-JTAG is exposed. Old firmware never came back, but you
#   can flash directly via esptool.
#   cd examples/dual_core_safety
#   source ~/esp-idf-5.5/export.sh
#   idf.py -p /dev/ttyACM0 erase-flash
#   idf.py -p /dev/ttyACM0 flash
#
# Case C — nothing enumerates:
#   Hold BOOT, tap RESET (or unplug+replug while holding BOOT).
#   The S3 should expose ROM USB-JTAG. Then run the commands above.
#   This is the only path that requires touching a button —
#   sorry, the rollback-loop genuinely bricked the chip beyond
#   what software can recover.
```

## Next steps after recovery (regardless of which case above)

Once `curl http://10.42.0.80/state.json` returns a JSON body, do:

```sh
# 1. Confirm we're on v15.3.3 (the post-brick hardening)
curl -s http://10.42.0.80/state.json | jq '{ts_boot_en, boot_count, free_heap}'

# Expected: ts_boot_en=0 (default in v15.3.3), boot_count=0.
# If ts_boot_en is missing from the JSON, you're on old firmware
# — OTA the v15.3.3 binary first:
cd examples/dual_core_safety
source ~/esp-idf-5.5/export.sh
idf.py build  # ensure build/ is fresh
curl -m 5 -X POST http://10.42.0.80/api/derp   # pause Tailscale if running
curl -m 5 -X POST http://10.42.0.80/api/wg
curl -m 120 -u admin:microlink \
     -H "Content-Type: application/octet-stream" \
     --data-binary "@build/microlink_dual_core_safety.bin" \
     http://10.42.0.80/admin/api/ota

# 2. Run the bench test sweep
/tmp/test_suite.sh    # output goes to /tmp/test_suite.log
# expected: TEST 1, 3, 5 all green; TEST 2 same caveats (wifi_tx_power
# can't be set with WiFi not started; pstop_peer should now PASS
# thanks to the handler-table fix in v15.3.2); TEST 4 all 5 resets OK.

# 3. Tailscale latency profile — the test we never got to run
curl -s -X POST http://10.42.0.80/api/ts_boot      # NVS flag → 1
curl -s -X POST http://10.42.0.80/admin/api/restart -u admin:microlink
sleep 30                                             # let Tailscale come up
# from the host:
ping -c 600 -i 0.2 -W 1 100.97.180.43 \
  | tee /tmp/ts_ping.log
# 600 packets × 0.2s = 2 min. With the residual wedge unfixed, expect
# this to fail partway through. Capture the moment in /tmp/longsoak.log
# for the next debugging pass.

# 4. WiFi-mode test
curl -s -X POST -u admin:microlink http://10.42.0.80/api/usb_enable
# chip reboots into WiFi mode (no USB-NCM); look for it on your LAN.
# From sdkconfig.credentials, the SSID is polymath-2G.
```

## Why all this happened

Short version: I was aggressively OTA'ing during a wedge cycle on a
chip whose other partition was already marked invalid from earlier
attempts. The safety-chain rollback path (in the *old* code at the
time) was perfectly happy to mark the SECOND partition invalid too,
which left the bootloader with no bootable image. ESP-IDF bootloader
without a factory partition has nowhere to fall back to → halts
before bringing up USB.

What I changed afterward in v15.3.3:

1. `esp_ota_get_state_partition()` check before
   `esp_ota_mark_app_invalid_rollback_and_reboot()`. Never mark
   the current partition invalid when the other is already invalid.
   Just `esp_restart()` instead and let the bad build keep trying
   until a real OTA replaces it. This is a guard against the exact
   failure mode that just happened.

2. Auto safe-mode: if `boot_count >= 1` we force `ts_boot_en=false`
   for this boot regardless of NVS, so a chip that's stuck in a
   Tailscale-wedge crash loop stops trying to come back into
   Tailscale and just stays useful.

Both changes are in `examples/dual_core_safety/main/main.c` and
the v15.3.3 binary at `build/microlink_dual_core_safety.bin` is
ready to flash.

Full session writeup in `TEST_REPORT_2026-05-25-eve.md` (same dir).
