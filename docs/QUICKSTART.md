<!--
SPDX-FileCopyrightText: 2026 Polymath Robotics
SPDX-License-Identifier: CC-BY-4.0
-->

# Quickstart: one remote, one laptop

Where you end up: a Protective Stop remote on your desk, on your own Tailscale
network, bonded to a ROS 2 machine node running on your laptop. Press the
button and the node reports STOP; hold and release and it arms.

The remote reaches the laptop either over **Ethernet** (plug it into the same
LAN, or a PoE port) or over **USB** (the cable is both power and network). Pick
one; each step says what differs. No firmware toolchain is needed: you flash a
prebuilt image and configure it from a web page. Allow about 20 minutes plus
the ROS 2 install.

## 0. What you need

| Item | Notes |
|---|---|
| Assembled remote | Waveshare ESP32-S3-ETH + NKK FF01 switch + LED ring: [`hardware/README.md`](../hardware/README.md), [`hardware/ASSEMBLY.md`](../hardware/ASSEMBLY.md). A bare board works for everything except the button steps. |
| USB-C data cable | Power, and the network if you use USB. |
| Ethernet cable and a DHCP LAN with internet | Only for the Ethernet path. A PoE switch port powers the remote too. |
| Laptop | Ubuntu 24.04 with internet (22.04 works with ROS 2 Humble). |
| Python 3 | Step 3 installs `esptool` with it. |
| ROS 2 Jazzy | [Install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html); `ros-jazzy-ros-base` is enough. |
| Tailscale account | Free tier is fine. |

Everything below is copy-paste. Lines marked `expect:` show what success looks
like.

## 1. Tailscale: account, laptop, auth key

```sh
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip -4                      # expect: 100.x.y.z
```

Keep that address; it is `$LAPTOP_TS` below.

Then create an auth key for the remote at
<https://login.tailscale.com/admin/settings/keys> → **Generate auth key**:
**Reusable on**, **Ephemeral off**, **Pre-approved on** if your tailnet uses
device approval. Copy the `tskey-auth-…` value; you paste it into the remote in
step 4. (Why these settings, and tags for a locked-down fleet: Appendix B.)

## 2. Clone the repo and prepare the laptop

```sh
git clone https://github.com/polymathrobotics/protective-stop.git
cd protective-stop
```

**USB path only** — make the laptop own the USB link (address, DHCP for the
remote, internet sharing), once per laptop:

```sh
host/setup/install.sh
```

Ethernet path: nothing to prepare; the remote gets its address from your LAN.
(WiFi, or moving a remote between uplinks later: Appendix A.)

## 3. Flash the remote

Into an empty folder, download three files from the
[latest release](https://github.com/polymathrobotics/protective-stop/releases/latest):
`pstop_remote-<version>-public-fullflash.bin` (factory image, used now),
`pstop_remote-<version>-public.bin` (app image, for updates later) and
`SHA256SUMS`. Connect the remote by USB (a blank board is already in download
mode) and, using the exact filename you downloaded:

```sh
python3 -m pip install --user esptool      # once; the repo's tools/ venv (uv) also has it, see tools/README.md
sha256sum -c SHA256SUMS --ignore-missing   # expect: two lines ending in OK
ls /dev/ttyACM*                            # expect: /dev/ttyACM0
python3 -m esptool --chip esp32s3 -p /dev/ttyACM0 -b 460800 write_flash 0x0 pstop_remote-<version>-public-fullflash.bin
# expect: … Hash of data verified. … Hard resetting via RTS pin…
```

If esptool cannot connect: hold **BOOT**, tap **RESET**, release BOOT, retry.

This is a factory image: everything on the chip is replaced, there is no
Tailscale key in it, and the admin password is `microlink`. Set it as a
variable for the commands that follow:

```sh
ADMIN_PW=microlink
```

**Provision on a network you control.** Until step 4 is done the remote sits
on your LAN with a public password and no key; anyone on that LAN could paste
their own key first. Do step 4 on the USB tether, or on a LAN with only your
own machines on it (a switch on the desk, not the office network). If several
people provision at once (a workshop), use the USB path or one switch per
person, and provision your own password (Appendix E) before any
remote goes on a shared network for good.

(Updating a remote that already runs this firmware, or one you have already
configured: Appendix C — the factory image would erase its settings.)

## 4. First boot: find the remote and give it the key

Unplug and replug the remote (Ethernet path: connect the Ethernet cable now;
the board is powered by USB or PoE). Within ~10 s the small onboard LED blinks
the **last number of the remote's IP address**, digit by digit — **blue** on
Ethernet, **green** on USB. Find the full address:

| Path | Address |
|---|---|
| Ethernet | Your LAN's subnet plus the blinked digits, e.g. `192.168.1.` + `47`. Missed the blink? Power-cycle the remote and watch again, or look in your router's DHCP table for the lease that appeared when you plugged it in (the remote sends no hostname). |
| USB | `ip neigh show dev esp-pstop0` → `10.42.0.X`; your laptop is `10.42.0.1`. |

```sh
DEV=<that address>
```

Open **`http://$DEV/admin`** (user `admin`, password `microlink`) → **Settings**
→ paste the auth key into **Tailscale Auth Key** → **Save** → **Restart**. Or do
the same from the shell:

```sh
curl -u "admin:$ADMIN_PW" -H 'Content-Type: application/json' \
     -d '{"auth_key":"tskey-auth-…"}' -X POST "http://$DEV/admin/api/settings"   # expect: {"ok":true…}
curl -u "admin:$ADMIN_PW" -X POST "http://$DEV/admin/api/restart"
```

About a minute after the restart:

```sh
tailscale status | grep pstop-       # expect: 100.a.b.c  pstop-01xxxxxx  …
REMOTE_HOST=$(tailscale status | awk '/pstop-01/{print $2; exit}')
REMOTE=$(tailscale ip -4 "$REMOTE_HOST")                      # the remote's tailnet address
REMOTE_ID=$(printf '%d' 0x${REMOTE_HOST#pstop-})              # its device id, decimal, for the ROS node
curl -s "http://$REMOTE/state.json" | grep -o '"ml_state":[0-9]'   # expect: "ml_state":4
```

The LED ring plays a dim purple sweep and settles on solid **white** (no
machine configured). From here on use `$REMOTE`; it works over either path.

Two things to do now in the Tailscale console
(<https://login.tailscale.com/admin/machines>): on the new `pstop-…` machine,
**⋯ → Disable key expiry** (otherwise it drops off the tailnet in 180 days), and
if device approval is on and you did not pre-approve the key, **approve** it.

## 5. Laptop: run the ROS 2 machine node

```sh
source /opt/ros/jazzy/setup.bash
sudo apt install -y ros-jazzy-generate-parameter-library ros-jazzy-diagnostic-updater \
                    ros-jazzy-rclcpp-lifecycle ros-jazzy-rclcpp-components libcurl4-openssl-dev
cd ros2                              # from the repo root (step 2); build from ros2/, not the repo root
colcon build --packages-up-to protective_stop_machine
source install/setup.bash
ros2 run protective_stop_machine machine_bridge_node
```

The defaults are all you need: the node listens on UDP 8890 on every interface
and admits every remote. In a second terminal (source both setup files again):

```sh
ros2 lifecycle get /machine_bridge   # expect: active [3]
```

If `ufw` is on: `sudo ufw allow in on tailscale0 to any port 8890 proto udp`.
Restricting which remotes may bond, or running without ROS: Appendix D.

## 6. Pair the remote to the laptop

The remote initiates the bond; tell it where the machine is, then promote it
from stop-only to operator (a new remote can stop the machine but not arm it):

```sh
curl -X POST "http://$REMOTE/api/pstop_peer?ip=$LAPTOP_TS&port=8890"
# expect: {"ok":true,...}    ring: white -> blue within a few seconds
curl -u "admin:$ADMIN_PW" -X POST "http://$REMOTE/api/role?role=operator"
# expect: {"ok":true,"role":"operator","message":"applied"}
```

Both settings persist on the remote.

## 7. Test it

Third terminal, node topics:

```sh
ros2 topic echo /machine_bridge/remotes       # expect: device_id "01xxxxxx", bond_state 2, stop_only false
ros2 topic echo /machine_bridge/machine_state # expect: status 1, "need_stop (awaiting arming gesture)"
ros2 topic echo /pstop_hb                     # expect: stop: true at ~10 Hz
```

| Do | Expect |
|---|---|
| Press the button, hold ≥ 0.5 s, twist to release | `machine_state` → `status: 0`, `"armed (cleared to run)"`; `/pstop_hb` → `stop: false`; ring **green** |
| Press the button | `status: 1`; `stop: true` within one heartbeat (~200 ms); ring **red** |
| Release (twist) | Stays stopped until the next hold-and-release |
| Pull the network cable while armed (USB or Ethernet) | `status: 1` within ~2 s; remote disappears from `/remotes` |
| Reconnect | Ring blue; hold-and-release to re-arm |

`/pstop_hb` is the signal for the rest of your robot stack: `stop: false` at
10 Hz means cleared to run; anything else, including silence, means stop.

That is the whole setup. If a step did not match, see section 8; everything
else lives in the appendices.

## 8. Troubleshooting

| Symptom | Check |
|---|---|
| esptool cannot connect | Hold BOOT, tap RESET, release BOOT, retry. `lsusb \| grep 303a`: `303a:1001` is download mode (good), `303a:4001` is a running remote. |
| LED does not blink an address | Ethernet: no DHCP lease — cable, switch port, DHCP server. USB: `lsusb \| grep 303a` shows `303a:4001`? `nmcli con show esp-pstop` exists? Re-run `host/setup/install.sh`, replug. |
| Admin page does not load on `$DEV` | Same LAN? A laptop on a different subnet or on WiFi with client isolation cannot reach it. USB: `ip addr show esp-pstop0` must show `10.42.0.1`. |
| LED strobes red | Local link up but no internet: the remote cannot reach Tailscale. Ethernet: LAN has no internet. USB: laptop has no internet or sharing is off (`nmcli con show esp-pstop \| grep ipv4.method` → `shared`). |
| No `pstop-` in `tailscale status` after 2 min | Key wrong, single-use or expired: `curl -u "admin:$ADMIN_PW" http://$DEV/admin/api/status` → `state`. Device approval on and key not pre-approved → approve in the console. |
| `ml_state` stuck below 4 | Same as above (no internet, or key not accepted). |
| Ring stays white after `pstop_peer` | The POST failed; re-run and read the JSON. |
| Ring blue, never green | Role still stop-only (`curl http://$REMOTE/api/role`), node not running, or ufw. `/machine_bridge/remotes` shows `stop_only: true` while the remote announces stop-only. |
| Ring red pulsing slowly | Peer configured but unreachable: node down, wrong `$LAPTOP_TS`, ufw. `tailscale ping $REMOTE` from the laptop. |
| Ring purple | One switch loop open while the other is closed: wiring fault ([`hardware/README.md`](../hardware/README.md)). |

Remote-side counters: `curl -s "http://$REMOTE/state.json" | python3 -m json.tool | grep -E 'pstop_(sent|replies)|ml_state|role'` — `pstop_sent` and `pstop_replies` climbing together means the bond is healthy. More: [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md), [`API.md`](API.md).

---

## Appendix A: WiFi, and switching between uplinks

The remote tries uplinks in order: Ethernet (6 s DHCP wait), USB, WiFi.
Ethernet wins over USB whenever both are connected, including hot-plug on a
running unit; pulling it falls back to USB with one ~5 s Tailscale re-register
that a bonded machine rides through — provided the laptop has run
`host/setup/install.sh` (step 2), which is what gives the USB link an address
and internet. On the Ethernet path that script is optional; run it if you want
the USB cable to be a working fallback rather than power only.

WiFi: add the network under **WiFi Networks** on the admin page (its own
section, above Device Settings), or provision it (Appendix E). A remote that **boots** with neither Ethernet nor USB and cannot
join WiFi within 60 s opens its own access point `microlink-XXYYZZ` (password
`microlink`) with the admin page at `http://192.168.4.1/admin`. This is a
boot-time fallback only: a unit that loses its wired/USB link while running
falls back to the configured WiFi but does not open the access point;
power-cycle it to get there.

The admin page listens on **every** network the remote joins (tether, LAN,
tailnet, WiFi). Before putting a remote on a shared LAN or WiFi, provision
your own password (Appendix E); the default `microlink` is public.

Two remotes on one laptop by USB: only the first gets `esp-pstop0`; see
[`USB_NCM_SETUP.md`](USB_NCM_SETUP.md), which also has Windows/macOS notes.

## Appendix B: Tailscale key settings and fleet isolation

| Setting | Value | Why |
|---|---|---|
| Reusable | on | The firmware re-sends the key on every re-registration. |
| Ephemeral | off | The remote keeps its node identity in flash; ephemeral nodes get deleted. |
| Pre-approved | on, if device approval is enabled | The remote has no browser to approve itself with. |
| Tags | optional | [`TAILSCALE_ISOLATION.md`](TAILSCALE_ISOLATION.md) has a locked-down fleet policy. Not needed for this guide. |

The key is stored in the remote's settings area and survives OTA updates and
app-image flashes (not the factory image, Appendix C). A key saved through the
admin page takes priority over a provisioned one (Appendix E).

## Appendix C: updating, reflashing, and what persists

**Update a running unit** (keeps every setting): send the app image
(`pstop_remote-<version>-public.bin`, downloaded in step 3 or from the new
release) over the network; the remote reboots into it.

```sh
curl -u "admin:$ADMIN_PW" --data-binary @pstop_remote-<version>-public.bin -X POST "http://$REMOTE/admin/api/ota"
```

**Reflash by cable**: a running unit has no serial port. Hold BOOT, tap RESET
(or `curl -u "admin:$ADMIN_PW" -X POST "http://$REMOTE/api/enter_download?confirm=1"`),
then flash. The full-flash image is the factory image (step 3) and erases the
settings; to keep them, use the OTA command above instead.

**Start over**: flash the full-flash image, or
`python3 -m esptool --chip esp32s3 -p /dev/ttyACM0 erase_flash` (then flash the
full-flash image; an erased chip has no firmware). Either wipes the Tailscale
identity, key, peer, role and health counters. The remote does not come back
on the tailnet by itself: repeat step 4 (new key paste), then step 6; it
registers as a new machine.

| Data | Lives in | Survives OTA / app-image flash | Survives full-flash image | Survives `erase_flash` |
|---|---|---|---|---|
| Tailscale auth key, WiFi | NVS if set via `/admin/`, else the provisioned secrets (Appendix E) | yes (NVS wins over provisioned) | no (re-provision, Appendix E) | no |
| Admin password | provisioned secrets (Appendix E), else `microlink` | yes | no (back to `microlink` until re-provisioned) | no (no firmware left) |
| Tailscale node identity | NVS | yes | no (new machine on tailnet) | no |
| Machine peer, self-role | NVS | yes | no | no |
| Lifetime health counters (`/api/health`) | NVS | yes | no | no |
| Admission lists (`allowlist`/`denylist`) | laptop: `pstop_machine.yaml` / `machine.toml` | n/a (not on the remote) | n/a | n/a |

## Appendix D: machine-side options

**Restrict which remotes may bond** (the default admits every remote). Run
this in the terminal where `REMOTE_ID` was set in step 4, or set it again
first; if it is empty the list becomes `[]`, which admits everyone:

```sh
echo "${REMOTE_ID:?set REMOTE_ID first (step 4)}"
cat > pstop_machine.yaml <<EOF
/machine_bridge:
  ros__parameters:
    software:
      allowlist: [$REMOTE_ID]       # only these may bond (empty = everyone)
      denylist: []                  # never these
EOF
ros2 run protective_stop_machine machine_bridge_node --ros-args --params-file pstop_machine.yaml
```

A refused remote shows `REJECTED` in `/machine_bridge/remotes`; fix the list,
then press **Rebond** on the remote (or `POST /api/pstop_peers?slot=0&rebond=1`).

**Demote a remote** back to stop-only at any time: `role=stop_only` on
`/api/role`. An armed machine keeps running, but refuses the next re-arm until
an `operator` remote does STOP → OK.

**Without ROS**: the plain-C runner prints every state change, which the ROS
node does not:

```sh
cd host && make
./machine_app_runner machine.toml   # admits every remote; [policy] allowlist/denylist to restrict
# expect: machine_app_runner listening on 0.0.0.0:8890
#         pstop 0x01XXXXXX -> BOND
#         ARMED by 0x01XXXXXX: STOP held 804 ms (policy minimum 500 ms)
```

Same pairing steps as section 6. Details: [`host/README.md`](../host/README.md).

**No hardware yet**: `cd tools && uv run python pstop_test_remote.py --port 8890`
is a software remote that runs the same arming sequence against the node
([`TESTING.md`](TESTING.md), [`../tools/README.md`](../tools/README.md)).

## Appendix E: provision your own credentials

Your own admin password, WiFi, or a Tailscale key without the admin-page step
go into each remote's encrypted secrets partition; no firmware image contains
them. With the remote flashed (step 3) and in download mode:

```sh
cd tools
cp credentials.env.example credentials.env
$EDITOR credentials.env                      # set TAILSCALE_AUTH_KEY and ADMIN_PASSWORD, leave the rest
uv run python provision_secrets.py provision --port /dev/ttyACM0 secrets.bin
uv run esptool --chip esp32s3 -p /dev/ttyACM0 write-flash 0x1C000 secrets.bin
rm secrets.bin
cd ..                                        # back to the repo root for step 5
```

The first run permanently burns a random key into the chip's eFuse and keeps
a copy in `tools/device_keys/<mac>.bin`; the partition is encrypted to it.
Re-run both commands to change the credentials, or after the full-flash image
or `erase_flash` wiped the partition. Keep `tools/device_keys/` private and
backed up: without a unit's file, its provisioned credentials cannot be
replaced (the admin page still overrides them).
`tools/flash_pstop.sh` does all of this in one go for staged builds.
Then skip the admin-page part of step 4 and set `ADMIN_PW` to your password.

The encryption keeps credentials out of every `.bin`, backup, and flash dump.
It does not stop someone holding the unit from flashing their own firmware and
having the chip decrypt the partition; that takes Secure Boot, which this
project does not enable.

## Appendix F: build the firmware yourself

Needed only to change the firmware. Install ESP-IDF **v5.5**
([guide](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/get-started/linux-macos-setup.html);
older IDF will not build, the USB tether needs a 5.5 fix), then:

```sh
cd firmware
. ~/esp/esp-idf/export.sh                    # wherever you installed IDF 5.5
idf.py build
idf.py -p /dev/ttyACM0 flash                 # keeps the secrets partition
cd ..
```

Do not run `idf.py monitor`: the USB port becomes the network tether a few
seconds into boot and the serial console goes quiet by design.
