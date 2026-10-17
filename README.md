# Steam Frame

Notes and code for poking at my Steam Frame over the LAN.

## Device

| | |
|-|-|
| Hostname | `frame` (`frame.local` via mDNS) |
| IP | frame.local (DHCP, may change) |
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

Fallback: Developer settings → **Set User Password**, then `ssh-copy-id steamos@frame.local`.

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
./stream.sh                                    # live 2x2 mosaic in ffplay
./stream.sh --exposure controller              # the LED-tracking exposure instead
ssh steamos@frame.local 'sudo python3 - --frames 1' < rawcam.py \
  | ffmpeg -f rawvideo -pix_fmt gray -s 2112x1504 -i - snap.png   # one raw snapshot
./mirror.sh                                    # headset view (/dev/video99); black/~1 fps when not worn
```

## yoloxr: YOLO boxes over passthrough

OpenXR scene app (`XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND`, Vulkan) that draws COCO YOLO11n
detections as world-locked 3D boxes; SteamVR keeps doing tracking and passthrough.

```
camserver (root)  ──/dev/shm/steamframe-cams──▶  yoloxr detector thread  ──▶  yoloxr XR thread
newest normal-exposure SLAM L/R frames           rectify both views to          re-anchor boxes with head
+ capture time (CLOCK_MONOTONIC)                 head-aligned pinholes, YOLO     pose at capture time, draw
                                                 (ncnn Vulkan) on the stacked    outlines + labels
                                                 pair, stereo-match, depth=fB/d
```

- Calibration: `/persist/xrservice.json` (factory KB4 intrinsics + extrinsics) and
  `/persist/device_config.json` (`head` and XRService's `cv/cad_from_cal`). `calib.py` chains
  them into `calib.txt` (head_from_cam per SLAM camera). Cam0 = `slam_right`, Cam1 = `slam_left`.
- Camera timestamps are `TIMESTAMP_MONOTONIC`-flagged but run ~210–230 ms ahead of
  `CLOCK_MONOTONIC`; camserver tracks the offset from arrival times. `--latency-ms` tunes the rest.
- The rectified views share the head's orientation, so they're a rectified stereo pair with a
  105.6 mm horizontal baseline.
- A box and its label are shown only while the eye gaze (`XR_EXT_eye_gaze_interaction`) hits the
  box grown by a rectangular margin of `--focus-margin` label heights (default 2), or the label
  itself once shown. An open
  SteamVR dashboard keeps the session at VISIBLE (not FOCUSED) and OpenXR then delivers no gaze, so
  `run.sh` runs `vrcmd --hidedashboard` after launch. Log line `eye gaze tracked in N% of frames`.
- Gaze accuracy check: `run.sh --gaze-cal 1.5` (no detector) shows 9 world-fixed dots 15° apart, 1.5 m away,
  one at a time with a countdown ring. Per dot it logs the median gaze − target offset (head frame, from the
  gaze origin) and its spread, then the mean offset and error; raw samples go to `/tmp/gaze_cal.csv`. It starts
  once gaze has been tracked for 4 s, and at the end shows the dots, where your gaze landed, and a live cursor.

Build & run on the Frame (ncnn built in `~/steamframe/third_party`, see below):

```sh
# from the Mac
COPYFILE_DISABLE=1 tar cf - --no-xattrs --exclude build yoloxr | ssh steamos@frame.local 'cd ~/steamframe && tar xf -'
ssh steamos@frame.local 'cd ~/steamframe/yoloxr && cmake -S . -B build -G Ninja && cmake --build build'
ssh steamos@frame.local '~/steamframe/yoloxr/run.sh --no-xr --dump /tmp/d'   # detector only, debug PGMs
# in the headset, detached (stop: systemctl --user stop yoloxr; logs: journalctl --user -u yoloxr -f)
ssh steamos@frame.local 'systemd-run --user --unit=yoloxr --collect --working-directory=$HOME/steamframe/yoloxr $HOME/steamframe/yoloxr/run.sh'
```

ncnn: `git clone --depth 1 https://github.com/Tencent/ncnn` into `~/steamframe/third_party/ncnn`, then
`cmake -G Ninja -DNCNN_VULKAN=ON -DNCNN_SIMPLEVK=OFF -DNCNN_SYSTEM_GLSLANG=ON
-DGLSLANG_TARGET_DIR=/usr/lib/cmake/glslang -DNCNN_SHARED_LIB=OFF -DNCNN_BUILD_TOOLS=OFF
-DNCNN_BUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=$HOME/steamframe/third_party/ncnn-install`.
(`SIMPLEVK=OFF` because its stub Vulkan types clash with `vulkan.h`.)
Models: `yolo export model=yolo11n.pt format=ncnn imgsz=<2S>,<S>` (ultralytics) → `~/steamframe/models/yolo11n_<S>.ncnn.{param,bin}`,
then `python3 yoloxr/prep_model.py models/yolo11n_<S>.ncnn` → `yolo11n_<S>_gray.ncnn.*`, which yoloxr loads (attention
MatMuls → Vulkan SDPA, since ncnn's MatMul is CPU-only; 1-channel input folded into the first conv).
The export bakes in the input size, so each `--size S` needs its own (default S=576; 480 and 640 also exported).
YOLO11n on the Adreno 750 (Turnip) for the stacked S x 2S input, SteamVR running: 23 ms @480, 31.4 ms @576, 38.5 ms @640
(37 ms @576 before prep_model.py and the conv options).

### Dead ends (so we don't retry them)

- `IVRTrackedCamera` (OpenVR): HMD reports a camera but `Prop_CameraFrameLayout=0`, `GetCameraFrameSize` fails.
- Reading `/dev/videoN` directly: EBUSY while SteamVR runs. With SteamVR stopped it streams,
  but exposure is ~0 (V4L2 exposes only `analogue_gain`; std OV exposure regs aren't what XRService uses).
- Stopping SteamVR: `steamvr.service` is `Restart=always` and gets restarted by `steam.service`.
  `XRService --snapshotCamerasAndExit` works if SteamVR is masked first (`systemctl --user mask --runtime`).
- `/proc/<pid>/mem` can't read dmabuf mappings (EIO) — hence `pidfd_getfd`.
- Hardware H.264 encoder (`v4l2h264enc`, `h264_v4l2m2m`) fails here; scripts use libx264.
- YOLO attention as per-head ncnn `Gemm` (both inputs dynamic): ~21 ms per 64×648×648 product on Turnip. SDPA is ~5 ms.
- ncnn Vulkan options: fp16 arithmetic, winograd, shader local memory, subgroup ops all best at their defaults.
  No fp8 on the Adreno 750 (ncnn reports `fp8-cm=0`). int8 (`ncnn2table`/`ncnn2int8`, all layers still on the GPU):
  50 ms @576 vs 32 ms fp16; every int8 conv is ~1.7x slower (no cooperative matrix on Turnip).

## Finding it if the IP changes

```sh
dig +short frame.local @224.0.0.251 -p 5353
nmap -p 22,32000 --open <your LAN>/24
```

## Refs

- https://partner.steamgames.com/doc/steamhardware/steamframe/setup
- https://partner.steamgames.com/doc/steamhardware/steamframe/debugging
