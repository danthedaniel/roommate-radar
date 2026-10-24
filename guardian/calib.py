"""Extract SLAM camera calibration into calib.txt for guardian.

Chain: head <- CAD (device_config "head") <- cal/cam0 frame (device_config cv/cad_from_cal,
computed by XRService) <- cam_i (factory extrinsics in /persist/xrservice.json).
Every frame here is given as plus_x / plus_z axes (+ position) expressed in its parent,
with plus_y = plus_z x plus_x. Camera frames are OpenCV-style (x right, y down, z forward).
"""
import json, sys

def cross(a, b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
def frame(d, scale=1.0):  # -> (R parent_from_child, t)
    x, z = d["plus_x"], d["plus_z"]
    y = cross(z, x)
    R = [[x[i], y[i], z[i]] for i in range(3)]
    return R, [p * scale for p in d["position"]]
def mul(A, B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def mv(A, v): return [sum(A[i][k]*v[k] for k in range(3)) for i in range(3)]
def T(A): return [list(r) for r in zip(*A)]
def compose(a, b):  # a∘b
    return mul(a[0], b[0]), [x + y for x, y in zip(mv(a[0], b[1]), a[1])]
def inv(a): Rt = T(a[0]); return Rt, [-x for x in mv(Rt, a[1])]

dev = json.load(open("/persist/device_config.json"))
xrs = json.load(open("/persist/xrservice.json"))
head_from_cad = inv(frame(dev["head"]))
cad_from_cal = frame(dev["cv"]["cad_from_cal"])
out = sys.stdout if len(sys.argv) < 2 else open(sys.argv[1], "w")
print("# name fx fy cx cy k1 k2 k3 k4 | R head_from_cam (row-major 9) | t head_cam (m)", file=out)
for cam in xrs["cameras"]:
    name = cam["sourceCamera"]
    if not name.startswith("slam"): continue
    k = next(i for i in cam["intrinsics"] if i.get("cameraModel") == "kb")
    head_from_cam = compose(compose(head_from_cad, cad_from_cal), frame(cam["extrinsics"], 1e-3))
    R, t = head_from_cam
    vals = [k[n] for n in ("fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4")] + sum(R, []) + t
    print(name, " ".join("%.9g" % v for v in vals), file=out)
    print("%s: pos_head=%s optical_axis_head=%s" % (name, [round(v, 4) for v in t], [round(R[i][2], 3) for i in range(3)]), file=sys.stderr)
# eye_to_head: 3x4 [R|t] per eye, used to relate OpenXR VIEW space to the head frame
for i, e in enumerate(dev["tracking_to_eye_transform"]):
    print("eye%d" % i, " ".join("%.9g" % v for v in sum(e["eye_to_head"], [])), file=out)
