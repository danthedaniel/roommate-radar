# Arcturus Vision Camera (AV1) notes

The color passthrough add-on for the Steam Frame ("Arcturus Vision Camera", AV1 in Valve's code): what the Frame
does with it, the modes it has, how XRService and the Arcturus app drive it, and where the frames are. From the
running system (kernel, XRService logs, V4L2 queries) and from the strings and disassembly of XRService, SteamVR's
`vrclient.so` and `vrcompositor`, and the Arcturus app (Steam app `ArcturusVisionCamera`). "Inferred" marks what
isn't confirmed on the device. Addresses and offsets are for the builds of 2026-10-26: SteamOS 20261022.6150929 (kernel
6.18), SteamVR 1791331621 (binaries dated 2026-10-06), Mesa 26.3.0-devel (Turnip), ICP firmware 7.0.21. [How to
redo it](#how-this-was-found) after an update is at the end.

## Hardware

- Two Sony IMX616 sensors (32 MP each, RGB Bayer), 64 mm apart, on the front expansion port's two MIPI
  connectors. Kernel driver `arcimx616` (built in): `0-0010` = `passthrough_right` (csiphy5, `/dev/v4l-subdev28`),
  `0-001a` = `passthrough_left` (csiphy3, `/dev/v4l-subdev29`). Bus format reported as `Y10_1X10` 2464×2464 (it's the
  raw Bayer data; the ISP demosaics).
- Per-module factory calibration in the module's EEPROM (`AI_JSON_CAL` file table entry, a JSON rig, read with the
  private `VIDIOC_PRIV_G_EEPROM_DATA` ioctl). XRService loads it into its calibration store; it isn't in `/persist`.
  Its live-refined alignment to the headset is saved to
  `~/.config/openvr/config/cv/xrservice/colorPassthroughCalibrationHistory.json`.
- The headset FPGA switches bitstreams for it (log: "Passthrough connected but FPGA is PASSTHRU - loading VCINT").
- Hot-pluggable. XRService gets a connection event and sets all its cameras up again.
- The sensor subdev implements no `ENUM_FRAME_SIZE`/`ENUM_FRAME_INTERVAL` (ENOTTY). Its only controls are
  `brightness` (changes at runtime, probably exposure), `temperature` (volatile) and `link_frequency` (2.2 GHz).
  Modes are chosen by XRService through the ICP firmware and private ioctls, not through standard V4L2.

## Modes

| | Full res | Half res ("V2H2 binning") |
|-|-|-|
| Per eye | 2464×2464 (~6 MP) | 1232×1232 (~1.5 MP) |
| Frame rate | 60 fps (72 Hz possible) | up to 120 fps (144 Hz mentioned) |

Each mode can run 8-bit or 10-bit ("HDR 10-bit"). These numbers come from Arcturus/press material and the
Arcturus app's UI strings ("Full res", "Half res", "Up to 60 FPS", "Up to 120 FPS", "HDR 10-bit", "8 BIT"). The app
notes that Half res "combines pixels", i.e. it bins. XRService builds its FPS list at runtime from the camera layer
(up to 16 entries), so the binary has no fixed table. XRService logs "Setting camera FPS: 60" at each camera setup.

"Automatic V2H2 binning" (`automaticV2H2BinningControl`) exists, which is presumably binning in low light (inferred).

## XRService settings

XRService reads overrides from `~/.config/openvr/config/cv/xrservice/XRServiceSettings.json` (an invalid key is logged
as "XRServiceSettings has an invalid setting"). Keys under `captureSessionSettings.passthroughCameras`, as the
settings loader reads them:

`enable`, `enableGamma`, `useAutoExposure`, `autoExposureMode` (`"auto"`), `useHDR`, `useAutoSignalBoosting`,
`useAutoWhiteBalance`, `initialExposure`, `initialGain`, `automaticStreamingControl`, `streamingRate`,
`enableLiveAlignmentToHeadset`, `enableLiveStereoRefinement`, `icpPresetName` (an ISP preset, e.g. "Adaptive
(sharpening, temporal denoising, single-pass)"; 19 presets exist), `enableThermalModel`, `enableV2H2Binning`,
`automaticV2H2BinningControl`, `simulateUnpluggedAtBoot`, `use10bitPipeline`.

Related keys elsewhere: `captureSessionSettings.provideIPEOutputAttachment` ("provides UBWC compressed data for
hardware encoding"), `xrSessionSettings.lightSourceFrequencyEstimate` (anti-flicker 50/60 Hz) and
`captureSessionSettings.trackingCameras.*` (`streamingMode`, `slamFrameFPSMode`, ...).

The defaults and value encodings (`streamingRate` in particular) aren't known: the key names come from strings, and
none of these keys has been tried. XRService's own debug panel ("Passthrough FPS", "V2H2 Binning", "Auto V2H2
Binning", "10-bit", "Passthrough HDR", "Passthrough AE Mode", "Auto Signal Boosting", "Auto White Balance",
exposure/gain sliders, "Save") edits and saves these keys.

## What changes when it's attached

XRService gives the add-on the main VFEs and the ICP/IPE (Qualcomm's ISP firmware, `/dev/icp`) and moves everything
else:

| | Without the add-on | With it |
|-|-|-|
| `/dev/video0` / `video3` | slam_right / slam_left: ISP, NV12, Y stride 1152 | passthrough_right / passthrough_left: ISP output |
| `/dev/video9` / `video13` | upper cameras | slam_right / slam_left: raw RDI, GREY 1056×1024 packed |
| `/dev/video6` / `video7` | unused | upper cameras |

XRService logs the mapping (`TrackingCameraInit: index: N. video device: ...`; index 0 = slam_right, 1 =
slam_left) and "ISP disabled for tracking cameras (passthrough using main VFE)". Raw SLAM frames have black level ~4
instead of ~16 and still alternate normal and controller-LED exposures. The passthrough cameras pause and resume with
user presence (standby), independently of the tracking cameras. They stream while the headset is worn, not only in
Room View.

## Passthrough frame format

Seen on `/dev/video0`/`video3` (multiplanar, `G_FMT` says NV12 2464×2464, bytesperline 2464):

- Plane 0 of 12353536 bytes: exactly **TP10 UBWC** (Qualcomm compressed 10-bit) for 2464×2464, i.e. Y meta
  40960 + Y 8200192 (stride 3328) + UV meta 12288 + UV 4100096. It is not decodable as linear data. The bytes look
  compressed, and the "top 2 bits zero" test for linear TP10 fails.
- Plane 1 of 1064960 bytes, its own dmabuf: an extra output (`VIDIOC_PRIV_S_EXTRA_OUTPUT_PLANE_CFG`). Not an image:
  a small header, histogram-like bars, then gridded data. Probably the ISP's 3A statistics.
- 9106944 bytes (exactly 8-bit linear NV12 2464×2464) shows up only for about a second at SteamVR start, before
  XRService finishes setting up passthrough. After that it's 10-bit UBWC: that is **XRService's default**. It stays
  10-bit after a SteamVR restart with the Arcturus app closed and Camera Switch disabled (no process running). The
  "8 BIT"/"10 BIT" choice in the Arcturus app is a camera control for the app's own capture, used only while the app
  owns the cameras. The sensor itself always outputs 10-bit raw (`Y10_1X10`); bit depth is the ISP's output format.
  Which key would make XRService output 8-bit (`use10bitPipeline: false`, `provideIPEOutputAttachment`?) is untested.
  XRService logs
  "Setting up the shared passthrough camera frames without UBWC compression (DRM_FORMAT_MOD_QCOM_COMPRESSED modifier
  not available)" when it can't use UBWC.

The GPU can't decompress it through Vulkan. Turnip (Mesa 26.3 on the Frame) imports the buffer with
`DRM_FORMAT_MOD_QCOM_COMPRESSED` and explicit plane layouts. Imported as a 32bpp image of 832×2464 at offset 0 with
pitch 3328, its computed layout matches the Y plane exactly (8241152 bytes). The copy-out is still garbage for every
32bpp format tried (`A2B10G10R10` UNORM/UINT, `R32_UINT`, `R32_SFLOAT`, `R8G8B8A8_UNORM`, `R16G16_UNORM`): wrong data
in each 64-byte block and many zero words. So UBWC's block encoding depends on the format, and TP10 needs the
hardware's `FMT6_TP10`/`TP10_Y`/`TP10_UV` texture formats (in `a6xx_enums.xml`), which Turnip maps no Vulkan format
to. Turnip also gives a 2-plane format's UV plane 16×8 UBWC blocks only for R8G8 (NV12). P010's UV plane gets 16×4,
which wouldn't fit this buffer's 12288-byte UV metadata anyway. 8-bit NV12 UBWC, on the other hand, Turnip supports
natively (`G8_B8R8_2PLANE_420_UNORM`).

## The Arcturus app

`~/.local/share/Steam/steamapps/common/ArcturusVisionCamera` (`launch.sh` runs `ArcturusVisionCamera`). Built from
Valve's `arcturus-xr` tree, the same one as XRService (source paths in the binaries).

- `libAV1ExternalCameraProvider.so` (single export `PECreateExternalCameraProviderV1`) holds a copy of XRService's
  capture source (`AV1ProviderDeckardCaptureSource`: V4L2, media controller, ICP firmware `CAMERA_ICP.mbn`, udmabuf).
  The app loads it and asks XRService for the cameras: XRService logs "[ExternalCameraOwnership] External camera
  ownership requested", closes its passthrough interfaces and the ICP sessions, and runs the tracking cameras raw
  until "... released". Frames reach the app over `/tmp/av1_external_camera_provider.sock`, and
  `/tmp/passthrough_camera.lock` keeps a second owner out ("passthrough cameras are already owned by another
  provider").
- In-app camera controls (log line "Selected camera controls"): `hdr`, `bitDepth`, `toneMapping`, `transfer`,
  `colorEncoding`, `resolution` (Full/Half res), `fps`, `redLed`, `awb`. They apply only while the app owns the
  cameras. The provider exposes `applyPassthroughCameraBinningOrFormatChange(binning, format)` and a settable
  "provider target passthrough FPS".
- Camera Switch (`camera-switch/`): a GTK app, installed by `camera-switch-install` as a desktop entry plus the systemd
  user unit `arcturus-camera-switch.service` (`WantedBy=steamvr.service`), that turns color passthrough on and off.
- The app reads `cv/xrservice/XRServiceSettings.json` and `steamvr.vrsettings` only to attach them to support reports;
  it doesn't write XRService's settings.

## Shared camera frames (what Room View shows)

XRService (or the Arcturus app, while it owns the cameras) publishes processed frames for the compositor through
SteamVR's internal `vr::IVRCameraPassthroughInternal_001`. This is how a consumer can get frames that are already
ISP-processed and **8-bit**, with per-frame calibration, independent of the V4L2 node layout.

**The shared state** is a 25088-byte (0x6200) POSIX shm segment, `/dev/shm/u<uid>-Shm_<hash>`. Any OpenVR client maps it
when it gets `IVRCameraPassthroughInternal_001`, so it can be found by size in `/proc/self/maps` (it's world-readable).
Layout (vrclient build of 2026-10-06):

| Offset | |
|-|-|
| 0x0000 | flags set by the interface's bool setters (`[0]`, `[1]`), a `{u32, u8}` setting at `[2..6]`, a u32 at `[8]` |
| 0x0014 | block for camera type 0 = tracking (1056×1024) |
| 0x20e8 | block for camera type 1 = color passthrough (2464×2464) |
| 0x41f4 | list of 32-byte entries, count at 0x61fc (empty so far) |

A block: `+0` u32 generation (incremented on each producer setup), `+4` u8 "changed", `+8` u32 slot count N (4),
`+0xc` u32 width, `+0x10` u32 height, `+0x14` u32 flags/format (1 for color), `+0x40` u8 available, then per eye
(stride 0x1040) up to 16 slots of 0x104 bytes:
- `+0` u32 state: 0 = free, 1 = held by a consumer, 2 = the producer's newest frame.
- `+4` 248 bytes of frame info: two doubles (capture times, seconds, same clock as the V4L2 timestamps), then floats
  fx, fy, cx, cy and Kannala-Brandt k1..k4 for this frame, then pose matrices and further parameters (rest not decoded).
- `+0xfc` u64 `SharedTextureHandle_t`. The low 32 bits are vrcompositor's pid: the compositor hosts the resource.

**Interface methods** (`CVRCameraPassthroughInternal` vtable, 17 entries; arguments in the order taken; returns
0 = ok, 1 = bad argument, 2 = no graphics device set, 3 = camera type not available):
- `[0]` `SetGraphicsDevice({u32 type = 2 (Vulkan); VRVulkanDevice_t *})`
- `[1]` producer setup: `(type, desc {u32 count ≤ 16, ...}, per-slot dma-buf sources[0x68 bytes each])`. It imports
  the sources into the shared-resource namespace and writes the block.
- `[2]` `(type, bool *available)`
- `[3]` `(type, eye, u32 *states, capacity)`: every slot's state.
- `[4]` `(type, eye, slot)`: hold that slot (state 1), releasing the eye's other held slots.
- `[5]` `(type, eye, slot, out[248])`: the slot's frame info.
- `[6]` `(type, eye, out)`: info about the eye's spline distortion texture.
- `[7]`/`[8]`, `[11]`/`[12]`: get/set two shared bools; `[9]`/`[10]` get/set the `{u32, u8}` (the setter notifies
  over IPC); `[13]` reads the u32; `[14]` drops the client's stale imports after a producer reset; `[15]` forwards
  to an inner object; `[16]` `(double *time, u64 *count, entries[], bytes)` copies the 32-byte list.

**How SteamVR shares fds** (vrcore, in `vrclient.so` and `vrcompositor`):
- Shared memory: `/dev/shm/u<uid>-Shm_<hash>` (`/u%d-Shm_%x`), plus `/dev/shm/u<uid>-ValveIPCSharedObj-SteamVR`, the
  shared-object manager. XRService has its own request/response segments (`/dev/shm/XR_Client*`, `XR_Server*`).
- Shared resources (images, buffers, semaphores) live in a namespace (`CSharedResourceNamespaceServer`, reached
  over the `SteamVR_Namespace` pipe). A `SharedTextureHandle_t` is `id << 32 | pid of the hosting process`.
- An fd changes hands per request: the owner opens an abstract `SOCK_DGRAM` Unix socket `fd-srv-<ipc handle>` (`%lu`)
  on a thread. The client binds `fd-cl-<...>`, sends one byte, and gets the fd back with `SCM_RIGHTS`. Nothing
  listens until a ref asks for an IPC handle (`/proc/net/unix` showed no `fd-srv-` sockets otherwise).
- `IVRIPCResourceManagerClient_003` (`openvr.h`; vrclient also has 001, 002 and 004) forwards one-to-one to
  `CVRIPCResourceManager` (vtable 0x619148): `[3]` `RefResource` sends message 550
  (`CVRMsg_CompositorRefSharedResource`: handle plus a "want IPC handle" flag) and copies the IPC handle from the
  reply 551; `[8]` `ReceiveSharedFd` does the `fd-cl-`/`fd-srv-` exchange.
- In `vrcompositor` the 550 handler (0x13b6d8) refs the resource (0x13b5d8) and, only when asked, exports it
  (0x10af18: look the handle up, "does not exist" otherwise; 0x10a430: ask the resource for an fd, then share it on
  `fd-srv-`). If the resource has no fd it gives up silently, and the reply carries IPC handle 0.

**Getting the pixels.** The public `IVRIPCResourceManagerClient_003` (`RefResource(handle, &ipcHandle)`, then
`ReceiveSharedFd`) doesn't work for these handles. As a Background app the ref fails ("Failed to ref VRCompositor
resource"; a Background app has no compositor connection). As an Overlay app (also with
`{"is_system_process" : true }`, the startup info Valve's `v4l2cam` passes) it succeeds but returns IPC handle 0: the
compositor imported these frames from XRService and has no fd of its own to export. What does work: the producer keeps
the buffers. XRService holds exactly
8 udmabufs of **9564160 bytes**, i.e. 2 eyes × 4 slots of **8-bit NV12 UBWC** 2464×2464:
- Y meta 40960, Y 2560×2464, UV meta 20480, UV 2560×1248.
- Vulkan import: `VK_FORMAT_G8_B8R8_2PLANE_420_UNORM` with modifier `DRM_FORMAT_MOD_QCOM_COMPRESSED` and explicit plane
  layouts (offset 0, pitch 2560) and (offset 6348800, pitch 2560). Turnip computes exactly 9564160 bytes for that.
- Copied out with `vkCmdCopyImageToBuffer` (planes 0 and 1), they're clean color frames (BT.601 limited range).

So from the driver (vrserver, XRService's parent: `pidfd_getfd`), the color frames are readable today. Tested (with
the headset still):
- **Slot ↔ buffer:** XRService's 8 color buffers sorted by fd number are slots 0–3 of one eye, then slots 0–3 of the
  other. When slot s becomes newest, buffers s and 4+s are the ones rewritten (found by hashing the raw bytes, which
  change with sensor noise even in a static scene). **Eye 0 = `passthrough_right`** (buffers 0–3), eye 1 =
  `passthrough_left` (buffers 4–7). Checked by covering the right camera: Y mean 9 vs 158. That's XRService's usual
  order (camera index 0 = slam_right) and the handle creation order (0x5f…0x62 = eye 0). The buffers (and their fds)
  survive the Arcturus app taking and releasing the cameras.
- **Rate and slots:** a 2-frame ring at 60.0 fps (16.67 ms every frame); slots 2 and 3 stay unused.
- **Clock:** the first double in the frame info runs ~2.11 s ahead of `CLOCK_MONOTONIC`, like the V4L2 camera clock,
  so the same offset tracking applies. The second double is ~1 ms earlier, sometimes one frame earlier. (The V4L2
  timestamps themselves ran ~2.0–2.17 s ahead all day, with or without the add-on, not the ~210 ms noted earlier in
  `packages/roommate-radar/README.md`. camserver's offset tracking absorbs it and logs "camera clock … from
  CLOCK_MONOTONIC" once.)
- **Hand-off to the app:** an fd sent with `SCM_RIGHTS` over a Unix socket in `/dev/shm` reaches a process inside
  the Steam Linux Runtime 4 container (`SteamLinuxRuntime_4-arm64/run`). There the container's Mesa imports it as NV12
  UBWC, and the copy-out is the correct frame.
- **Tracking frames (type 0):** published only while Room View uses the tracking cameras: without the add-on, or while
  the Arcturus app owns the color cameras (then type 1 is unavailable, block `+0x40` = 0). With the color cameras in
  XRService's hands, type 0 isn't published, so the IR cameras still need the V4L2 path. When published they're
  XRService's lowest-numbered 8 buffers of 1769472 bytes (fds 111–118; the other 32 are the V4L2 capture buffers). They
  follow the same slot mapping (s and 4+s), update about 30 times a second, and are linear: Y 1056×1024 with stride 1152
  and an empty UV plane, readable on the CPU. One dump was nearly black (mean 4.9 vs 65), possibly a controller-LED
  exposure.
- **During a game** (Half-Life: Alyx, Room View off): XRService pauses the color cameras as the game comes up
  ("Passthrough cameras paused"). Neither type is published (both `+0x40` = 0), and `S[0]` goes from 1 to 0. Setting
  `S[0]` through `vt[8]` from an overlay client didn't resume them; it read 0 again within a second, so it looks like
  state that something else writes, not a request. The V4L2 tracking cameras keep streaming (60 fps).
  `passthroughCameras.automaticStreamingControl: false` might keep the color cameras running, but that's untested
  and not pursued.

So Roommate Radar stays on the IR (SLAM) cameras: in games the color cameras aren't running, and that's when it's
used. The driver finds the SLAM cameras on either V4L2 layout (`camserver.rs` `LAYOUTS`), with or without the add-on.

## Dead ends and open questions

- `VIDIOC_SUBDEV_ENUM_FRAME_SIZE` on the sensor: ENOTTY.
- Plane 0 read as linear TP10 (stride 3328): garbage, because it's UBWC.
- Plane 0 decompressed on the GPU as any 32bpp Vulkan format (see above): garbage. Plane 1 (1064960 bytes) isn't an
  image: a header, histograms and gridded data, probably the ISP's 3A statistics.
- `use10bitPipeline: false` in `XRServiceSettings.json` (SteamVR restarted): the V4L2 buffers stay 10-bit UBWC
  (12353536 bytes). The key doesn't affect them, if it's read at all.
- Shared frames through `IVRIPCResourceManagerClient` `RefResource`/`ReceiveSharedFd`: no fd (see above).
- `v4l2cam` as a guide to the camera interface: it only carries the interface name (in a built-in list) and mirrors
  the compositor's headset view instead.
- Resuming the color cameras in a game by setting `S[0]` (`vt[8]`): no effect (see above).
- Open: `streamingRate` encoding and defaults; the rest of the 248-byte frame info; whether the shared tracking ring
  carries controller-LED exposures; what `vt[15]` and the 32-byte list are for.

## How this was found

Everything was read-only on the Frame, except the tests named above (each one undone afterwards).

- **Logs first.** XRService's log (`~/.local/share/Steam/logs/XRService-<date>.log`) names cameras, nodes, modes and
  ownership changes. vrserver's log (`vrserver.txt`) has the driver's lines (`roommate_radar: ...`). Each OpenVR client
  gets `vrclient_<exe>.txt`, which is where "Failed to ref VRCompositor resource" showed up.
- **The camera graph:** `media-ctl -p`, `v4l2-ctl --get-fmt-video` and `--get-subdev-fmt` (read-only), and
  `/proc/<XRService>/fd` for which nodes it holds. Buffer contents came from `pidfd_getfd` on XRService's dmabufs
  (root for the probes, or the driver as XRService's parent), `VIDIOC_QUERYBUF` for the newest completed one, then
  `mmap`. `/proc/<pid>/fdinfo/<fd>` gives each dmabuf's `size` and `exp_name` as the same user. The sizes alone
  identified the formats: 12353536 = TP10 UBWC, 9564160 = NV12 UBWC, 1769472 = NV12 with stride 1152, all for these
  dimensions (Venus/Mesa UBWC layout rules: meta then data per plane, 4 KiB aligned).
- **Binaries, no decompiler:** strings, then `llvm-objdump -d` with every `adrp`+`add` pair resolved to the string it
  points at, which gives a code-address → string index. Virtual methods come from RTTI: find the `<len>ClassName`
  typeinfo name, the `R_AARCH64_RELATIVE` relocation that points at it (typeinfo + 8), the one that points at the
  typeinfo (vtable + 8), then the relocations after it are the vtable entries. That's how
  `CVRCameraPassthroughInternal` (0x616b30 in vrclient), `CVRIPCResourceManager` and the `IVRIPCResourceManagerClient_003`
  forwarders were mapped. Callers of a function: search the disassembly for `bl <addr>`. Members used as callbacks
  have no direct references. Ghidra 12.1's headless analyzer runs on the Mac, but its release ships no macOS arm64
  decompiler binary, so it wasn't usable.
- **Settings keys:** XRService's settings loader references every key in a row, so its string index around one known
  key (`useAutoWhiteBalance`) lists them all. The debug panel code shows labels next to keys.
- **Prototypes:** small C programs built in the Steam Runtime SDK image (`registry.gitlab.steamos.cloud/steamrt/
  steamrt4/sdk/arm64`, `gcc ... -lvulkan` / `-lopenvr_api`) and copied to the Frame:
  - a Vulkan dmabuf import with an explicit DRM modifier and plane layouts, then `vkCmdCopyImageToBuffer` to a host
    buffer;
  - an OpenVR client that maps the shared state (`VR_GetGenericInterface("IVRCameraPassthroughInternal_001")`, then
    the 25088-byte `/dev/shm` mapping in `/proc/self/maps`) and calls interface methods through the C++ vtable.

  `~/.local/share/Steam/steamapps/common/SteamLinuxRuntime_4-arm64/run -- ./prog` runs one inside the same container
  as Roommate Radar.
- **Which buffer is which slot:** hash a few KB of every candidate buffer's raw bytes each time the shared state's
  newest slot changes. The rewritten buffer is the one whose hash changed, because sensor noise changes every frame
  even with the headset still. Which eye: cover one camera and compare brightness.
- **Formats by eye:** decode on the Mac with numpy (unpack, BT.601) and look at the image.
