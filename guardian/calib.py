"""Extract SLAM camera calibration into calib.txt for guardian.

Chain: head <- CAD (device_config "head") <- cal/cam0 frame (device_config cv/cad_from_cal,
computed by XRService) <- cam_i (factory extrinsics in /persist/xrservice.json).
Every frame here is given as plus_x / plus_z axes (+ position) expressed in its parent,
with plus_y = plus_z x plus_x. Camera frames are OpenCV-style (x right, y down, z forward).
"""

import json
import sys
from typing import Any

Vec = list[float]
Mat = list[list[float]]
Pose = tuple[Mat, Vec]  # (R parent_from_child, t)


def cross(a: Vec, b: Vec) -> Vec:
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def frame(d: dict[str, Any], scale: float = 1.0) -> Pose:  # pyright: ignore[reportExplicitAny]
    x: Vec = d["plus_x"]
    z: Vec = d["plus_z"]
    y = cross(z, x)
    R = [[x[i], y[i], z[i]] for i in range(3)]
    return R, [p * scale for p in d["position"]]


def mul(A: Mat, B: Mat) -> Mat:
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mv(A: Mat, v: Vec) -> Vec:
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def T(A: Mat) -> Mat:
    return [list(r) for r in zip(*A)]


def compose(a: Pose, b: Pose) -> Pose:  # a∘b
    return mul(a[0], b[0]), [x + y for x, y in zip(mv(a[0], b[1]), a[1])]


def inv(a: Pose) -> Pose:
    Rt = T(a[0])
    return Rt, [-x for x in mv(Rt, a[1])]


def fmt(vals: list[float]) -> str:
    return " ".join(f"{v:.9g}" for v in vals)


def main() -> None:
    with open("/persist/device_config.json") as f:
        dev = json.load(f)
    with open("/persist/xrservice.json") as f:
        xrs = json.load(f)
    head_from_cad = inv(frame(dev["head"]))
    cad_from_cal = frame(dev["cv"]["cad_from_cal"])
    lines = ["# name fx fy cx cy k1 k2 k3 k4 | R head_from_cam (row-major 9) | t head_cam (m)"]
    for cam in xrs["cameras"]:
        name: str = cam["sourceCamera"]
        if not name.startswith("slam"):
            continue
        k = next(i for i in cam["intrinsics"] if i.get("cameraModel") == "kb")
        head_from_cam = compose(compose(head_from_cad, cad_from_cal), frame(cam["extrinsics"], 1e-3))
        R, t = head_from_cam
        intrinsics: Vec = [k[n] for n in ("fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4")]
        lines.append(f"{name} {fmt(intrinsics + [v for r in R for v in r] + t)}")
        pos = [round(v, 4) for v in t]
        axis = [round(R[i][2], 3) for i in range(3)]
        print(f"{name}: pos_head={pos} optical_axis_head={axis}", file=sys.stderr)
    # eye_to_head: 3x4 [R|t] per eye, used to relate OpenXR VIEW space to the head frame
    for i, e in enumerate(dev["tracking_to_eye_transform"]):
        lines.append(f"eye{i} {fmt([v for r in e['eye_to_head'] for v in r])}")
    text = "\n".join(lines) + "\n"
    if len(sys.argv) < 2:
        sys.stdout.write(text)
    else:
        with open(sys.argv[1], "w") as f:
            f.write(text)


if __name__ == "__main__":
    main()
