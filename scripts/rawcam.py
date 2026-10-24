"""Read raw tracking-camera frames out of XRService while SteamVR keeps running.

XRService owns the V4L2 capture queues (DMABUF memory backed by udmabuf). As root we
duplicate its fds with pidfd_getfd, VIDIOC_QUERYBUF each queue (read-only) to find the
newest completed buffer, and mmap that dmabuf. Nothing is stopped or reconfigured.

Writes a 2112x1504 GRAY8 mosaic to stdout at --fps:
  [ video0 SLAM 1056x1024 | video3 SLAM 1056x1024 ]
  [ video9 640x480 | video13 640x480 | black      ]
Run on the Frame: sudo python3 rawcam.py [--fps 30] [--exposure normal|controller|any]
"""

import argparse
import ctypes
import fcntl
import mmap
import os
import subprocess
import sys
import time
from collections.abc import Iterator

libc = ctypes.CDLL(None, use_errno=True)
SYS_pidfd_getfd = 438


class Plane(ctypes.Structure):
    _fields_ = [  # pyright: ignore[reportUnannotatedClassAttribute]
        ("bytesused", ctypes.c_uint32),
        ("length", ctypes.c_uint32),
        ("m", ctypes.c_uint64),
        ("data_offset", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 11),
    ]


class Buffer(ctypes.Structure):
    _fields_ = [  # pyright: ignore[reportUnannotatedClassAttribute]
        ("index", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("bytesused", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("field", ctypes.c_uint32),
        ("tv_sec", ctypes.c_int64),
        ("tv_usec", ctypes.c_int64),
        ("timecode", ctypes.c_uint32 * 4),
        ("sequence", ctypes.c_uint32),
        ("memory", ctypes.c_uint32),
        ("m", ctypes.c_uint64),
        ("length", ctypes.c_uint32),
        ("reserved2", ctypes.c_uint32),
        ("request_fd", ctypes.c_int32),
    ]


assert ctypes.sizeof(Buffer) == 88 and ctypes.sizeof(Plane) == 64
VIDIOC_QUERYBUF = (3 << 30) | (88 << 16) | (ord("V") << 8) | 9
DMA_BUF_IOCTL_SYNC = (1 << 30) | (8 << 16) | (ord("b") << 8) | 0
SYNC_START_READ, SYNC_END_READ = 1, 4 | 1
TYPE_MPLANE, FLAG_QUEUED = 9, 0x2

# node -> (width, height, row stride of the NV12 Y plane)
CAMS = {
    "video0": (1056, 1024, 1152),
    "video3": (1056, 1024, 1152),
    "video9": (640, 480, 640),
    "video13": (640, 480, 640),
}
W, H = 2112, 1504


def xrservice_pid() -> int:
    return int(subprocess.check_output(["pgrep", "-f", "/XRService |^XRService"]).split()[0])


class Cam:
    def __init__(self, pidfd: int, pid: int, node: str):
        _, h, stride = CAMS[node]
        self.size: int = stride * h  # bytes in the NV12 Y plane
        fd = next(int(f) for f in os.listdir(f"/proc/{pid}/fd") if os.readlink(f"/proc/{pid}/fd/{f}") == f"/dev/{node}")
        self.vfd: int = self.getfd(pidfd, fd)
        self.pidfd: int = pidfd
        self.maps: dict[int, tuple[int, mmap.mmap]] = {}  # XRService dmabuf fd -> (local fd, mmap)

    @staticmethod
    def getfd(pidfd: int, fd: int) -> int:
        local: int = libc.syscall(SYS_pidfd_getfd, pidfd, fd, 0)
        if local < 0:
            raise OSError(ctypes.get_errno(), "pidfd_getfd")
        return local

    def buffers(self) -> Iterator[tuple[int, int]]:
        """Yield (sequence, dmabuf fd in XRService) for every completed, not-requeued buffer."""
        for i in range(64):
            planes = (Plane * 8)()
            b = Buffer(index=i, type=TYPE_MPLANE, length=8, m=ctypes.addressof(planes))
            try:
                fcntl.ioctl(self.vfd, VIDIOC_QUERYBUF, b)
            except OSError:
                return
            if not b.flags & FLAG_QUEUED and b.tv_sec:
                yield b.sequence, ctypes.c_int32(planes[0].m & 0xFFFFFFFF).value

    def read(self, remote_fd: int) -> bytes:
        if remote_fd not in self.maps:
            local = self.getfd(self.pidfd, remote_fd)
            self.maps[remote_fd] = (local, mmap.mmap(local, self.size, prot=mmap.PROT_READ))
        local, m = self.maps[remote_fd]
        fcntl.ioctl(local, DMA_BUF_IOCTL_SYNC, ctypes.c_uint64(SYNC_START_READ))
        y = bytes(m[: self.size])
        fcntl.ioctl(local, DMA_BUF_IOCTL_SYNC, ctypes.c_uint64(SYNC_END_READ))
        return y

    def latest(self, exposure: str) -> bytes | None:
        # XRService alternates a normal exposure with a ~37us one for controller LEDs;
        # tell them apart by brightness of the two newest frames.
        newest = sorted(self.buffers(), reverse=True)[:2]
        frames = [self.read(fd) for _, fd in newest]
        if exposure == "any" or len(frames) < 2:
            return frames[0] if frames else None
        pick = max if exposure == "normal" else min
        return pick(frames, key=lambda f: sum(f[::4099]) / len(f[::4099]))


def mosaic(frames: list[tuple[str, bytes | None]]) -> bytearray:
    out = bytearray(W * H)
    for (node, f), x0, y0 in zip(frames, (0, 1056, 0, 640), (0, 0, 1024, 1024)):
        if f is None:
            continue
        w, h, stride = CAMS[node]
        for r in range(h):
            out[(y0 + r) * W + x0 : (y0 + r) * W + x0 + w] = f[r * stride : r * stride + w]
    return out


class Args(argparse.Namespace):
    fps: float = 30
    frames: int = 0
    exposure: str = "normal"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fps", type=float, default=30)
    ap.add_argument("--frames", type=int, default=0, help="stop after N frames (0 = forever)")
    ap.add_argument("--exposure", choices=["normal", "controller", "any"], default="normal")
    a = ap.parse_args(namespace=Args())

    pid = xrservice_pid()
    pidfd = os.pidfd_open(pid)
    cams = [(n, Cam(pidfd, pid, n)) for n in CAMS]
    n, t = 0, time.monotonic()
    while not a.frames or n < a.frames:
        sys.stdout.buffer.write(mosaic([(name, c.latest(a.exposure)) for name, c in cams]))
        sys.stdout.buffer.flush()
        n += 1
        t += 1 / a.fps
        time.sleep(max(0, t - time.monotonic()))


if __name__ == "__main__":
    main()
