# Steam Frame notes

What Roommate Radar builds on: getting into the Frame over the LAN, its cameras, and what didn't work.

## Device

| | |
|-|-|
| Hostname | `frame` (`frame.local` via mDNS) |
| User | `steamos` |
| OS | SteamOS, aarch64, kernel 6.18 |

## Connect

```sh
ssh steamos@frame.local
```

Key auth uses `~/.ssh/id_rsa`. Needs **Settings → System → Developer Mode** on.

## Pairing a key (devkit host pairing)

The Frame runs a devkit service on port 32000 (the thing Valve's SteamOS Devkit
Client talks to). Pairing is one POST: the public key + a `devkit-client:` comment
+ the client's magic phrase. Approve on the headset if it prompts.

```sh
key=$(cut -d' ' -f1-2 ~/.ssh/id_rsa.pub)
curl -X POST -H 'Content-Type: text/plain' --data-binary \
  "$key devkit-client:$(whoami)@$(hostname -s) 900b919520e4cf601998a71eec318fec
" http://frame.local:32000/register
# -> Registered
```

Source: `client/devkit_client/__init__.py` `register()` in
https://gitlab.steamos.cloud/devkit/steamos-devkit

Other endpoints: `GET /login-name` → `steamos`, `GET /properties.json`.

SSH password auth is off (`/etc/ssh/sshd_config.d/10-no-password.conf`: `PasswordAuthentication no`), so
pairing is the only way to add a key. Developer settings → **Set User Password** still works for local/sudo use.

Root: `steamos` has passwordless sudo via `/etc/sudoers.d/zz-steamos-nopasswd`.

## Cameras

| Node | Sensor | Name (XRService) | Native | Notes |
|-|-|-|-|-|
| `/dev/video0` | og01a1b `4-0036` | `slam_right` | 1056×1024 Y8, ~115 fps | outward fisheye |
| `/dev/video3` | og01a1b `4-0060` | `slam_left` | 1056×1024 Y8 | |
| `/dev/video9` | og0ve10 `5-003e` | upper (L or R) | 640×480 Y8 | outward fisheye |
| `/dev/video13` | og0ve10 `5-0060` | upper (L or R) | 640×480 Y8 | |
| — | arcimx616 ×2 (`0-0010`, `0-001a`) | ? ("passthrough cameras") | UYVY 1920×1080 | not streaming |
| `/dev/video99` | v4l2loopback | — | 1920×1080 RGB | compositor mirror (`steamvr-v4l2cam`) |

XRService alternates exposures: ~8.4 ms (tracking/passthrough) and ~0.037 ms (controller LEDs).

### Live raw frames without stopping SteamVR

XRService holds the capture queues exclusively, but as root `rawcam.py` duplicates its fds
(`pidfd_getfd`), asks each queue for its newest completed buffer (`VIDIOC_QUERYBUF`,
read-only), and mmaps that dmabuf. Buffers are NV12 with Y stride 1152 (SLAM) / 640 (upper).

```sh
scripts/stream.sh                            # live 2x2 mosaic in ffplay
scripts/stream.sh --exposure controller      # the LED-tracking exposure instead
ssh steamos@frame.local 'sudo python3 - --frames 1' < scripts/rawcam.py \
  | ffmpeg -f rawvideo -pix_fmt gray -s 2112x1504 -i - snap.png   # one raw snapshot
```

## Dead ends (so we don't retry them)

- OpenXR `/interaction_profiles/valve/frame_controller`: SteamVR rejects suggested bindings for it; suggest
  `valve/index_controller` + `khr/simple_controller` aim poses instead and SteamVR remaps the Frame controllers.
- `IVRTrackedCamera` (OpenVR): HMD reports a camera but `Prop_CameraFrameLayout=0`, `GetCameraFrameSize` fails.
- Reading `/dev/videoN` directly: EBUSY while SteamVR runs. With SteamVR stopped it streams,
  but exposure is ~0 (V4L2 exposes only `analogue_gain`; std OV exposure regs aren't what XRService uses).
- Stopping SteamVR: `steamvr.service` is `Restart=always` and gets restarted by `steam.service`.
  `XRService --snapshotCamerasAndExit` works if SteamVR is masked first (`systemctl --user mask --runtime`).
- `/proc/<pid>/mem` can't read dmabuf mappings (EIO) — hence `pidfd_getfd`.

## Finding it on the network

```sh
dig +short frame.local @224.0.0.251 -p 5353
nmap -p 22,32000 --open <your LAN>/24   # the Frame answers on 22 (ssh) and 32000 (devkit)
```

## Refs

- https://partner.steamgames.com/doc/steamhardware/steamframe/setup
- https://partner.steamgames.com/doc/steamhardware/steamframe/debugging
