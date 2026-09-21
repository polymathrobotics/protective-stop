<!--
SPDX-FileCopyrightText: 2026 Polymath Robotics
SPDX-License-Identifier: CC-BY-4.0
-->

# Quickstart: one remote, one laptop, over Tailscale

End state: a Protective Stop remote on your desk, tethered to your laptop by
USB, talking over your own Tailscale network to a ROS 2 node on the laptop.
Press the button, the node reports STOP. Hold and release, it arms.

No firmware toolchain is needed: you flash a prebuilt image from the
[releases page](https://github.com/polymathrobotics/protective-stop/releases)
and configure it from a web page. Budget about 20 minutes plus the ROS 2
install. (Building the firmware yourself is Appendix A.)

## 0. What you need

| Item | Notes |
|---|---|
| Assembled remote | Waveshare ESP32-S3-ETH + NKK FF01 switch + LED ring, see [`hardware/README.md`](../hardware/README.md) and [`hardware/ASSEMBLY.md`](../hardware/ASSEMBLY.md). A bare ESP32-S3-ETH board works for everything except the button steps. |
| USB-C data cable | Powers the remote and carries its network. |
| Laptop | Ubuntu 24.04 with internet. Ubuntu 22.04 also works (use ROS 2 Humble). |
| Python 3 | For `esptool` (flashing). `python3 -m pip install esptool` |
| ROS 2 Jazzy | [Install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). `ros-jazzy-ros-base` is enough. |
| Tailscale account | Free tier is fine. |

Everything below is copy-paste. Lines marked `expect:` are what success
looks like.

## 1. Tailscale account and auth key

1. Create an account at <https://login.tailscale.com/start>.
2. Generate an auth key at <https://login.tailscale.com/admin/settings/keys>
   → **Generate auth key**. Settings that matter:

   | Setting | Value | Why |
   |---|---|---|
   | Reusable | **on** | The firmware re-sends the key on every re-registration. |
   | Ephemeral | **off** | The remote keeps its node identity in flash; ephemeral nodes get deleted. |
   | Pre-approved | **on** (if device approval is enabled) | The remote has no browser to approve itself with. |
   | Tags | optional | See [`TAILSCALE_ISOLATION.md`](TAILSCALE_ISOLATION.md) for a locked-down fleet policy. Not needed for this guide. |

3. Copy the key (`tskey-auth-…`). You will paste it into the remote's admin
   page in step 5.

After the remote appears on your tailnet (step 5), open its row in the
[Machines](https://login.tailscale.com/admin/machines) page → **⋯** →
**Disable key expiry**. Otherwise the remote silently drops off the tailnet
in 180 days.

## 2. Laptop: join Tailscale

```sh
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip -4                      # expect: 100.x.y.z
```

Keep that address; it is `$LAPTOP_TS` below.

## 3. Laptop: USB tether

The remote's USB port is a network adapter (CDC-NCM). Your laptop must own the
link: take `10.42.0.1`, hand the remote a DHCP lease, and NAT its traffic to
the internet so it can reach Tailscale. One script does that (NetworkManager or
systemd-networkd, detected automatically). You need the repository anyway for
the ROS 2 node in step 6:

```sh
git clone https://github.com/polymathrobotics/protective-stop.git
cd protective-stop
host/setup/install.sh
```

Nothing to verify yet; the interface `esp-pstop0` appears the first time a
running remote is plugged in. Details and Windows/macOS notes:
[`USB_NCM_SETUP.md`](USB_NCM_SETUP.md).

## 4. Flash the release image

Download the remote's **full-flash** image and the checksum file from the
[latest release](https://github.com/polymathrobotics/protective-stop/releases/latest)
(`pstop_remote-<version>-public-fullflash.bin` and `SHA256SUMS`), then:

```sh
sha256sum -c SHA256SUMS --ignore-missing   # expect: pstop_remote-…-fullflash.bin: OK
```

Plug the remote in. A blank board is already in download mode:

```sh
ls /dev/ttyACM*                            # expect: /dev/ttyACM0
python3 -m esptool --chip esp32s3 -p /dev/ttyACM0 -b 460800 write_flash 0x0 pstop_remote-*-public-fullflash.bin
# expect: … Hash of data verified. … Hard resetting via RTS pin…
```

If `esptool` cannot connect, hold **BOOT**, tap **RESET**, release BOOT, retry.

The full-flash image contains everything (bootloader, partition table, app)
and writes the whole flash from address 0 — including the settings area, which
it leaves **blank**. It is a **factory image**: flashing it onto a unit that
was already provisioned erases its Tailscale identity and key, machine peer,
role and health counters (the unit comes back as a new machine on the tailnet).
To *update* a running unit instead, send the app image over the network
(`…-public.bin`, step 9, "Update a running unit") — that keeps every setting.

The image is built from the release tag **without any credentials**: no
Tailscale key, no WiFi, admin password `microlink`. You add the key next.

## 5. First boot: give the remote its Tailscale key

Unplug and replug the remote after flashing (so the tether re-enumerates
under the new interface name), then find it on the tether:

```sh
ip addr show esp-pstop0              # expect: inet 10.42.0.1/24  (within ~10 s)
ip neigh show dev esp-pstop0         # expect: 10.42.0.X … REACHABLE   (the remote)
```

The small onboard LED also blinks the last octet of the remote's IP in green,
digit by digit. Open **`http://10.42.0.X/admin`** in a browser (user `admin`,
password `microlink`), go to **Settings**, paste the auth key from step 1 into
**Tailscale Auth Key**, **Save**, then **Restart**. The same thing from the
shell:

```sh
DEV=10.42.0.X                        # from ip neigh above
curl -u admin:microlink -H 'Content-Type: application/json' \
     -d '{"auth_key":"tskey-auth-…"}' -X POST "http://$DEV/admin/api/settings"   # expect: {"ok":true…}
curl -u admin:microlink -X POST "http://$DEV/admin/api/restart"
```

Within about a minute of the restart:

```sh
tailscale status | grep pstop-       # expect: 100.a.b.c  pstop-01xxxxxx  …
```

The LED ring plays a dim purple sweep, then settles on solid **white** (no
machine configured). Record the identifiers you will need:

```sh
REMOTE_HOST=$(tailscale status | awk '/pstop-01/{print $2; exit}')   # e.g. pstop-01d7f344
REMOTE=$(tailscale ip -4 "$REMOTE_HOST")                              # e.g. 100.a.b.c
REMOTE_ID=$(printf '%d' 0x${REMOTE_HOST#pstop-})                      # e.g. 30929732
echo "$REMOTE_HOST $REMOTE $REMOTE_ID"
curl -s "http://$REMOTE/state.json" | grep -o '"ml_state":[0-9]'      # expect: "ml_state":4
```

`ml_state` 4 means the Tailscale session is up. The hostname suffix is the
remote's 32-bit device ID in hex; the ROS node wants it in decimal, hence
`REMOTE_ID`. Now go back to the Tailscale admin console and **disable key
expiry** on the new machine (step 1).

The key is stored in the remote's settings area: it survives OTA updates and
app-image flashes, not the full-flash image (Appendix D). The admin page is reachable only from the USB tether,
a wired LAN, or your tailnet; to use a password other than `microlink`, build
the image yourself (Appendix A).

## 6. Laptop: build and run the ROS 2 node

```sh
source /opt/ros/jazzy/setup.bash
sudo apt install -y ros-jazzy-generate-parameter-library ros-jazzy-diagnostic-updater \
                    ros-jazzy-rclcpp-lifecycle ros-jazzy-rclcpp-components libcurl4-openssl-dev
cd ros2                              # inside the protective-stop clone from step 3
colcon build --packages-up-to protective_stop_machine
source install/setup.bash
```

Build from `ros2/`, not the repo root. The defaults are all you need (the node
listens on UDP 8890 on every interface and admits every remote):

```sh
ros2 run protective_stop_machine machine_bridge_node
```

To restrict *which remotes may bond at all* (optional), give it an allowlist
and/or denylist of 32-bit remote ids:

```sh
cat > pstop_machine.yaml <<EOF
/machine_bridge:
  ros__parameters:
    software:
      allowlist: [$REMOTE_ID]       # only these may bond (empty = everyone)
      denylist: []                  # never these
EOF
ros2 run protective_stop_machine machine_bridge_node --ros-args --params-file pstop_machine.yaml
```

In a second terminal (source both setup files again):

```sh
ros2 lifecycle get /machine_bridge   # expect: active [3]
```

If `ufw` is enabled on the laptop:
`sudo ufw allow in on tailscale0 to any port 8890 proto udp`.

## 7. Pair the remote to the laptop

The remote initiates; tell it where the machine is:

```sh
curl -X POST "http://$REMOTE/api/pstop_peer?ip=$LAPTOP_TS&port=8890"
# expect: {"ok":true,...}    ring: white -> blue within a few seconds
```

A new remote is **stop-only**: it can stop the machine but never arm it.
Promote it once (applies live, no reboot):

```sh
curl -u admin:microlink -X POST "http://$REMOTE/api/role?role=operator"
# expect: {"ok":true,"role":"operator","message":"applied"}
```

That is the only gate: the remote decides its own role and the machine honours
it. `role=stop_only` demotes it again at any time — an armed machine keeps
running, but refuses the next re-arm until an `operator` remote does STOP → OK.

## 8. Test station

Third terminal, node topics:

```sh
ros2 topic echo /machine_bridge/remotes       # expect: device_id "01xxxxxx", bond_state 2, stop_only false
ros2 topic echo /machine_bridge/machine_state # expect: status 1, "need_stop (awaiting arming gesture)"
ros2 topic echo /pstop_hb                     # expect: stop: true at ~10 Hz
```

| Action | Expect |
|---|---|
| Press the button, hold ≥ 0.5 s, twist to release | `machine_state` → `status: 0`, `"armed (cleared to run)"`; `/pstop_hb` → `stop: false`; ring **green** |
| Press the button | `status: 1`; `stop: true` within one heartbeat (~200 ms); ring **red** |
| Release (twist) | Stays stopped until the next hold-and-release |
| Unplug the USB cable while armed | `status: 1` within ~2 s; remote disappears from `/remotes` |
| Replug | Ring blue; hold-and-release to re-arm |

`/pstop_hb` is the signal for the rest of your robot stack: `stop: false` at
10 Hz means cleared to run; anything else, including silence, means stop.

Remote-side counters, if a step does not match:

```sh
curl -s "http://$REMOTE/state.json" | python3 -m json.tool | grep -E 'pstop_(sent|replies)|ml_state|role'
curl -s "http://$REMOTE/api/health"                 # lifetime counters: presses, uptime, boots
```

`pstop_sent` and `pstop_replies` climbing together means the bond is healthy.

No hardware yet? `cd tools && uv run python pstop_test_remote.py --port 8890`
is a software remote that runs the same arming sequence against the node; see
[`TESTING.md`](TESTING.md) and [`../tools/README.md`](../tools/README.md) for
the one-time uv setup.

## 9. Troubleshooting

| Symptom | Check |
|---|---|
| `esptool` cannot connect | Hold **BOOT**, tap **RESET**, release BOOT, retry. `lsusb \| grep 303a` — `303a:1001` is download mode (good for flashing), `303a:4001` is a running remote. |
| No `esp-pstop0` after replug | `lsusb \| grep 303a` shows `303a:4001`? `nmcli con show esp-pstop` exists? Re-run `host/setup/install.sh`. |
| `ip neigh` shows nothing | Wait 10 s after replug; the remote needs a DHCP lease from the laptop first. `sudo nmcli con show esp-pstop \| grep ipv4.method` → `shared`. |
| Admin page refuses the key / no `pstop-` in `tailscale status` after 2 min | Key wrong, single-use, or expired: `curl -u admin:microlink http://10.42.0.X/admin/api/status` → `state`. Device approval on and key not pre-approved? Approve it in the console. |
| `ml_state` stuck at 0–3 | Laptop has no internet, or NAT not active (`ipv4.method` above). |
| Ring stays white after `pstop_peer` | POST failed; re-run and read the JSON. |
| Ring blue, never green | `/api/role` on the remote returns `operator`? Node running? `/machine_bridge/remotes` shows `stop_only: true` while the remote announces stop-only. |
| Remote row shows `REJECTED` | The node's `allowlist`/`denylist` refused the bond. Fix the list, then press **Rebond** on the remote (or `POST /api/pstop_peers?slot=0&rebond=1`). |
| Ring red pulsing (slow) | Peer configured but unreachable: node down, wrong `$LAPTOP_TS`, or ufw. `tailscale ping $REMOTE` from the laptop. |
| Ring purple | One switch loop open while the other is closed: wiring fault. See [`hardware/README.md`](../hardware/README.md). |
| Update a running unit | `curl -u admin:microlink --data-binary @pstop_remote-<version>-public.bin -X POST "http://$REMOTE/admin/api/ota"` — the **app** image, over the network; the unit reboots into it and keeps all settings. |
| Reflash a running unit by cable | It has no serial port while running: hold BOOT, tap RESET (or `curl -u admin:microlink -X POST "http://$REMOTE/api/enter_download?confirm=1"`), then flash. The full-flash image resets it to factory (step 4); to keep settings, use the OTA row above instead. |
| Start over | Flash the full-flash image, or `python3 -m esptool --chip esp32s3 -p /dev/ttyACM0 erase_flash`; either wipes the Tailscale identity, key, peer, role and health counters, and the remote comes back as a new machine on the tailnet. |

More: [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md), [`API.md`](API.md).

---

## Appendix A: build the firmware yourself

Needed only to bake your own credentials into the image (a different admin
password, WiFi, a Tailscale key without the admin-page step), or to change the
firmware. Install ESP-IDF **v5.5**
([guide](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/get-started/linux-macos-setup.html);
older IDF will not build — the USB tether needs a 5.5 fix), then:

```sh
cp firmware/sdkconfig.credentials.example firmware/sdkconfig.credentials
$EDITOR firmware/sdkconfig.credentials
```

Set two values, leave the rest:

```
CONFIG_ML_TAILSCALE_AUTH_KEY="tskey-auth-…"     # from step 1
CONFIG_ML_ADMIN_PASSWORD="choose-something"     # protects /admin and role changes
```

WiFi is not needed on the USB tether. The fleet/OTA fields are for a
management backend this repo does not ship; leave them as they are.

```sh
cd firmware
. ~/esp/esp-idf/export.sh            # wherever you installed IDF 5.5
idf.py build
grep CONFIG_ML_TAILSCALE_AUTH_KEY sdkconfig   # expect: your key, not the XXXXX placeholder
idf.py -p /dev/ttyACM0 flash
```

If the grep shows the placeholder, the build reused a stale `sdkconfig`:
`rm sdkconfig && idf.py build`, then grep again. `sdkconfig.credentials` is
only read when `sdkconfig` is (re)generated. Do not run `idf.py monitor`: the
USB port becomes the network tether a few seconds into boot and the serial
console goes quiet by design. With the key baked in, skip the admin-page part
of step 5. Never publish an image built with a credentials file — every value
in it is compiled in as plain text (`tools/release_guard.sh` refuses such
images).

## Appendix B: machine side without ROS

The plain-C runner is the fastest way to see the protocol work and prints
every state change, which the ROS node does not:

```sh
cd host && make
./machine_app_runner machine.toml   # admits every remote; [policy] allowlist/denylist to restrict
# expect: machine_app_runner listening on 0.0.0.0:8890
#         pstop 0x01XXXXXX -> BOND
#         ARMED by 0x01XXXXXX: STOP held 804 ms (policy minimum 500 ms)
```

Same `pstop_peer` and `role` steps as sections 7–8. Details:
[`host/README.md`](../host/README.md).

## Appendix C: Ethernet, PoE, or WiFi instead of USB

The remote tries uplinks in order: Ethernet (6 s DHCP wait), USB tether, WiFi.

- **Ethernet / PoE**: plug into any DHCP LAN with internet; skip step 3. Find
  the remote's LAN IP from your router (or the LED blink, blue on Ethernet) and
  use it for the admin page in step 5; once Tailscale is up, use `$REMOTE`. Ethernet wins
  over USB whenever both are connected, including hot-plug on a running unit;
  pulling it falls back to USB with one ~5 s Tailscale re-register that a
  bonded machine rides through.
- **WiFi**: enter the network under **Settings** on the admin page (or set
  `CONFIG_ML_WIFI_SSID` / `CONFIG_ML_WIFI_PASSWORD` when building yourself).
  A remote that **boots** with neither Ethernet nor USB and cannot join WiFi
  within 60 s opens its own access point `microlink-XXYYZZ` (password
  `microlink`) with the admin page at `http://192.168.4.1/admin`, where WiFi
  and the auth key can be set without a cable. This is a boot-time fallback
  only: a unit that loses its wired/USB link while running falls back to the
  configured WiFi but does not open the access point — power-cycle it to get
  there.
- Two remotes on one laptop by USB: only the first gets `esp-pstop0`; see
  [`USB_NCM_SETUP.md`](USB_NCM_SETUP.md).

## Appendix D: what persists where

| Data | Lives in | Survives OTA / app-image flash | Survives full-flash image / `erase_flash` |
|---|---|---|---|
| Tailscale auth key, WiFi | NVS if set via `/admin/` (or the firmware image when built with credentials) | yes (NVS wins over image) | no |
| Admin password | firmware image only (`microlink` in release builds) | yes | yes (it is in the image) |
| Tailscale node identity | NVS | yes | no (new machine on tailnet) |
| Machine peer, self-role | NVS | yes | no |
| Lifetime health counters (`/api/health`) | NVS | yes | no |
| Admission lists (`allowlist`/`denylist`) | laptop: `pstop_machine.yaml` / `machine.toml` | n/a (not on the remote) | n/a |

The full-flash image writes the settings area blank (the release notes call it
the factory image); `erase_flash` wipes everything. OTA and app-image flashes
never touch the settings area.

Changing the key baked into a new image does not replace a key that was ever
saved through the admin page; NVS takes priority at boot.
