# Upload-time idle watchdog reset

The 23:10:11 UTC preflight upload of the priority-7 candidate did not complete.
The DUT remained on `v1.2-30-g0c112bd` / `1e4c97c6e`; it did not verify the new
image. Only one OTA POST was issued. No automatic retry or reset was performed
by the uploader.

The preserved coredump, decoded with the matching old-image ELF, identifies:

- watchdog failure: **IDLE1 (CPU 1)**;
- interrupted task: **httpd**, not interrupt context;
- stack: `handler_ota -> esp_ota_write -> esp_partition_write -> esp_flash_write`
  and SPI cache/other-core synchronization, during a 4096-byte write;
- crash image: **1e4c97c6e**, not the new priority-7 image.

OTA status reported running/boot partition `ota_1`, state VALID, target `ota_0`,
and `rollback_occurred=false`. The failed execution had not reached OTA image
validation/boot selection. This is not evidence that priority 7 failed at boot.
The coredump locates the failure; it does not by itself measure the duration of
one flash write versus the cumulative uninterrupted receive/write loop.

The subsequent 23:12:34 software reset is explained by the retained log and
existing recovery ladder: a crash-count-1 boot uses DERP-only mode; after 120s
of healthy uptime `bc_clear_task` clears the crash counter and restarts to retry
direct UDP. It was not a second upload or an unexplained operator action.

## Remedy and validation

Both HTTP-push and HTTP-client-pull OTA loops now block for **one RTOS tick
after each successful chunk**.
On a fast link, `httpd_req_recv` need not block; the former tight receive/write
loop provided no guaranteed scheduling opportunity for idle tasks. `taskYIELD`
is insufficient because it need not schedule lower-priority tasks. No watchdog
subscription, watchdog timeout, or protective-stop deadline is relaxed.

To install the fix through the still-running old handler, use a recorded paced
upload (4096-byte chunks at approximately 32KiB/s). This lets the old receiver
block between chunks. Build from the latest reviewed commit, verify the flashed
identity and retained settings, then validate the repaired handler with a
full-speed preflight upload before proceeding to the steady-load test and soak.

Evidence is private under
`/tmp/opencode/pstop-soak-20260910/ota-investigation-20260912T231610Z/`:
the 64836-byte coredump has SHA-256
`b165e78e92400a1f05404ac983ce07c81260dd4dabf73b52124a09beba32980f`.
Raw memory and decoded register/task reports are not included in the PR.
The two previously successful uploads and this failure are retained; successful
retesting must not erase the intermittent failure from the record.

The retained coredump intentionally survives subsequent software reboots.
`crash_present=1` with the old `crash_sha=1e4c97c6e` is therefore expected after
a successful update; verify the new running identity, clean SW reset and lack
of a new crash instead of erasing the old diagnostic to obtain a zero flag.
