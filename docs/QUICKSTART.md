<!--
SPDX-FileCopyrightText: 2026 Polymath Robotics
SPDX-License-Identifier: CC-BY-4.0
-->

# Quickstart: one remote, one laptop, over Tailscale

End state: a Protective Stop remote on your desk, tethered to your laptop by
USB, talking over your own Tailscale network to a ROS 2 node on the laptop.
Press the button, the node reports STOP. Hold and release, it arms.

Budget about 45 minutes, most of it waiting on the ESP-IDF install and the
first build.

## 0. What you need

| Item | Notes |
|---|---|
| Assembled remote | Waveshare ESP32-S3-ETH + NKK FF01 switch + LED ring, see [`hardware/README.md`](../hardware/README.md) and [`hardware/ASSEMBLY.md`](../hardware/ASSEMBLY.md). A bare ESP32-S3-ETH board works for everything except the button steps. |
| USB-C data cable | Powers the remote and carries its network. |
| Laptop | Ubuntu 24.04 with internet. Ubuntu 22.04 also works (use ROS 2 Humble). |
| ESP-IDF **v5.5** | [Install guide](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/get-started/linux-macos-setup.html). Older IDF will not build (the USB tether needs a 5.5 fix). |
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

3. Copy the key (`tskey-auth-…`). You will paste it into the firmware config
   in step 4.

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
systemd-networkd, detected automatically):

```sh
git clone https://github.com/polymathrobotics/protective-stop.git
cd protective-stop
host/setup/install.sh
```

Nothing to verify yet; the interface `esp-pstop0` appears the first time a
running remote is plugged in. Details and Windows/macOS notes:
[`USB_NCM_SETUP.md`](USB_NCM_SETUP.md).

## 4. Build and flash the firmware

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
ADMIN_PW='choose-something'          # same value; used by curl below
cd firmware
. ~/esp/esp-idf/export.sh            # wherever you installed IDF 5.5
idf.py build
grep CONFIG_ML_TAILSCALE_AUTH_KEY sdkconfig   # expect: your key, not the XXXXX placeholder
```

If the grep shows the placeholder, the build reused a stale `sdkconfig`:
`rm sdkconfig && idf.py build`, then grep again. `sdkconfig.credentials` is
only read when `sdkconfig` is (re)generated.

Plug the remote in. A blank board is already in download mode:

```sh
ls /dev/ttyACM*                      # expect: /dev/ttyACM0
idf.py -p /dev/ttyACM0 flash
```

If `flash` cannot connect, hold **BOOT**, tap **RESET**, release BOOT, retry.
Do not run `idf.py monitor`: the USB port becomes the network tether a few
seconds into boot and the serial console goes quiet by design.

## 5. First boot

Unplug and replug the remote after flashing (so the tether re-enumerates
under the new interface name).

<!-- VERIFY: ring colour and time-to-tailnet on a fresh unit -->

```sh
ip addr show esp-pstop0              # expect: inet 10.42.0.1/24  (within ~10 s)
tailscale status | grep pstop-       # expect: 100.a.b.c  pstop-01xxxxxx  …  (within ~60 s)
```

The LED ring plays a dim purple sweep, then settles on solid **white** (no
machine configured). The small onboard LED shows network state.

Record the two identifiers you will need:

```sh
REMOTE_HOST=$(tailscale status | awk '/pstop-01/{print $2; exit}')   # e.g. pstop-01d7f344
REMOTE=$(tailscale ip -4 "$REMOTE_HOST")                              # e.g. 100.a.b.c
REMOTE_ID=$(printf '%d' 0x${REMOTE_HOST#pstop-})                      # e.g. 30929732
echo "$REMOTE_HOST $REMOTE $REMOTE_ID"
curl -s "http://$REMOTE/state.json" | grep -o '"ml_state":[0-9]'      # expect: "ml_state":4
```

`ml_state` 4 means the Tailscale session is up. The hostname suffix is the
remote's 32-bit device ID in hex; the ROS node wants it in decimal, hence
`REMOTE_ID`.

Now go back to the Tailscale admin console and **disable key expiry** on the
new machine (step 1).

## 6. Laptop: build and run the ROS 2 node

```sh
cd ..                                # back to the repo root
source /opt/ros/jazzy/setup.bash
sudo apt install -y ros-jazzy-generate-parameter-library ros-jazzy-diagnostic-updater \
                    ros-jazzy-rclcpp-lifecycle ros-jazzy-rclcpp-components libcurl4-openssl-dev
cd ros2
colcon build --packages-up-to protective_stop_machine
source install/setup.bash
```

Build from `ros2/`, not the repo root. Write the node's parameters (the node
listens on UDP 8890 on every interface by default):

```sh
cat > pstop_machine.yaml <<EOF
/machine_bridge:
  ros__parameters:
    software:
      operators: [$REMOTE_ID]       # remotes allowed to ARM; everyone else is stop-only
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
Promote it once (reboots the remote, ~20 s):

```sh
curl -u "admin:$ADMIN_PW" -X POST "http://$REMOTE/api/role?role=operator"
```

Arming needs both this and the `operators` entry from step 6. Either one alone
leaves the remote stop-only.

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
| Unplug the USB cable while armed | `status: 1` within ~2 s (400 ms × 5 missed heartbeats); remote disappears from `/remotes` |
| Replug | Ring blue; hold-and-release to re-arm |

`/pstop_hb` is the signal for the rest of your robot stack: `stop: false` at
10 Hz means cleared to run; anything else, including silence, means stop.

Remote-side counters, if a step does not match:

```sh
curl -s "http://$REMOTE/state.json" | python3 -m json.tool | grep -E 'pstop_(sent|replies)|ml_state|role'
```

`pstop_sent` and `pstop_replies` climbing together means the bond is healthy.

No hardware yet? `python3 tools/pstop_test_remote.py --port 8890` is a
software remote that runs the same arming sequence against the node (add its
id `16909185` to `operators` first); see [`TESTING.md`](TESTING.md).

## 9. Troubleshooting

| Symptom | Check |
|---|---|
| No `esp-pstop0` after replug | `lsusb \| grep 303a` — `303a:4001` is a running remote, `303a:1001` is download mode (flash failed or BOOT held). `nmcli con show esp-pstop` exists? Re-run `host/setup/install.sh`. |
| `esp-pstop0` up, no `pstop-` in `tailscale status` after 2 min | Key wrong or single-use: `curl -u admin:PW http://10.42.0.X/admin/api/status` (find `10.42.0.X` with `ip neigh show dev esp-pstop0`) → `state`. Fix the key via the admin page at `http://10.42.0.X/admin/` without reflashing. Device approval on and key not pre-approved? Approve it in the console. |
| `ml_state` stuck at 0–3 | Laptop has no internet, or NAT not active: `sudo nmcli con show esp-pstop \| grep ipv4.method` → `shared`. |
| Ring stays white after `pstop_peer` | POST failed; re-run and read the JSON. |
| Ring blue, never green | Two gates: `/api/role` returns `operator`? `REMOTE_ID` in `operators`? Node running? `/machine_bridge/remotes` shows `stop_only: true` while either gate is open. |
| Ring red pulsing (slow) | Peer configured but unreachable: node down, wrong `$LAPTOP_TS`, or ufw. `tailscale ping $REMOTE` from the laptop. |
| Ring purple | One switch loop open while the other is closed: wiring fault. See [`hardware/README.md`](../hardware/README.md). |
| Need to reflash a running unit | It has no serial port. Hold BOOT, tap RESET, then `idf.py -p /dev/ttyACM0 flash`; or `curl -u admin:PW -X POST "http://$REMOTE/api/enter_download?confirm=1"`. Settings in flash (key, peer, role) survive a reflash. |
| Start over | `idf.py -p /dev/ttyACM0 erase-flash` wipes everything including the Tailscale identity; the remote comes back as a new machine on the tailnet. |

More: [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md), [`API.md`](API.md).

---

## Appendix A: machine side without ROS

The plain-C runner is the fastest way to see the protocol work and prints
every state change, which the ROS node does not:

```sh
cd host && make
$EDITOR machine.toml     # add an [[operator]] block with device_id = 0x01xxxxxx, stop_only = false
./machine_app_runner machine.toml
# expect: machine_app_runner listening on 0.0.0.0:8890
#         pstop 0x01XXXXXX -> BOND
#         ARMED by 0x01XXXXXX: STOP held 804 ms (policy minimum 500 ms)
```

Same `pstop_peer` and `role` steps as sections 7–8. Details:
[`host/README.md`](../host/README.md).

## Appendix B: Ethernet, PoE, or WiFi instead of USB

The remote tries uplinks in order: Ethernet (6 s DHCP wait), USB tether, WiFi.

- **Ethernet / PoE**: plug into any DHCP LAN with internet; skip step 3. Find
  the remote's LAN IP from your router or, once Tailscale is up, just use
  `$REMOTE`. Every command in steps 5–8 works over the tailnet address.
- **WiFi**: set `CONFIG_ML_WIFI_SSID` / `CONFIG_ML_WIFI_PASSWORD` in
  `sdkconfig.credentials` before building. If WiFi fails for 60 s the remote
  opens its own access point `microlink-XXYYZZ` (password `microlink`) with the
  admin page at the gateway address, where WiFi and the auth key can be fixed
  without a reflash. <!-- VERIFY: AP gateway address, expected 192.168.4.1 -->
- Two remotes on one laptop by USB: only the first gets `esp-pstop0`; see
  [`USB_NCM_SETUP.md`](USB_NCM_SETUP.md).

## Appendix C: what persists where

| Data | Lives in | Survives `idf.py flash` / OTA | Survives `erase-flash` |
|---|---|---|---|
| Tailscale auth key, WiFi, admin password | firmware image (from `sdkconfig.credentials`) or NVS if set via `/admin/` | yes (NVS wins over image) | no |
| Tailscale node identity | NVS | yes | no (new machine on tailnet) |
| Machine peer, self-role | NVS | yes | no |
| Operator allowlist (`operators`) | laptop: `pstop_machine.yaml` / `machine.toml` | n/a (not on the remote) | n/a |

Changing the key baked into a new image does not replace a key that was ever
saved through the admin page; NVS takes priority at boot.
