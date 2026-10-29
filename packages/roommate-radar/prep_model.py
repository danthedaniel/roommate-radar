"""Adapt an ultralytics YOLO11 ncnn export for roommate-radar. Usage: prep_model.py models/yolo11n_576.ncnn

Writes <prefix>_rr.ncnn.{param,bin}:
- The C2PSA attention's MatMul/Softmax/MatMul becomes one SDPA layer. ncnn has no Vulkan MatMul, so it ran on the
  CPU with GPU<->CPU round trips in the middle of the net.
- GRAY2RGB and the 1/255 scale are folded into the first conv: the input is the 1-channel 0..255 image.
  (Just the scale for yolo11n-grayscale, which already takes 1 channel.)
- Only the classes roommate-radar tracks (CLASSES, in that order) are scored: each scale's last class conv keeps their
  rows of weights, so the output is [4 + len(CLASSES), N] instead of [4 + 80, N].
"""

import struct
import sys

CLASSES = [0, 15, 16]  # person, cat, dog: COCO's ids, as in tracker.rs

src = sys.argv[1]
dst = src.removesuffix(".ncnn") + "_rr.ncnn"
with open(src + ".param") as f:
    lines = f.read().splitlines()

# q arrives transposed to (heads, seq, dim) and pre-scaled by the BinaryOp before it, hence scale 1.
i = next((n for n, l in enumerate(lines) if l.startswith("MatMul")), None)
assert i is not None, "no MatMul (already converted?)"
mm1, sm, mm2 = (l.split() for l in lines[i : i + 3])
assert mm1[0] == "MatMul" and sm[0] == "Softmax" and mm2[0] == "MatMul" and mm2[-1] == "0=1", (
    "unexpected attention block"
)
q, k, v, out = mm1[4], mm1[5], mm2[4], mm2[6]
lines[i : i + 3] = [
    f"Permute sdpa_k_t 1 1 {k} sdpa_k 0=1",
    f"Permute sdpa_v_t 1 1 {v} sdpa_v 0=1",
    f"SDPA sdpa 3 1 {q} sdpa_k sdpa_v sdpa_out 6=1.0",
    f"Permute sdpa_out_t 1 1 sdpa_out {out} 0=1",
]
nl, nb = map(int, lines[1].split())
lines[1] = f"{nl + 1} {nb + 1}"

# The first conv's weights come first in the .bin: a 4-byte zero tag, then fp32 [out][cin][kh][kw].
i = next(n for n, l in enumerate(lines) if l.startswith("Convolution"))
kv = dict(t.split("=") for t in lines[i].split()[6:])
co, kk = int(kv["0"]), int(kv["1"]) * int(kv.get("11", kv["1"]))
cin = int(kv["6"]) // (co * kk)
assert cin in (1, 3), f"first conv takes {cin} channels"
lines[i] = lines[i].replace(f"6={kv['6']}", f"6={co * kk}")
with open(src + ".bin", "rb") as f:
    b = f.read()
assert b[:4] == bytes(4), "weights aren't fp32"
w = struct.unpack_from(f"<{co * cin * kk}f", b, 4)
folded = [sum(w[(o * cin + c) * kk + j] for c in range(cin)) / 255 for o in range(co) for j in range(kk)]

b = b[:4] + struct.pack(f"<{co * kk}f", *folded) + b[4 + co * cin * kk * 4 :]


def params(line):
    """A layer line's key=value params: after its type, name, input and output counts, inputs and outputs."""
    f = line.split()
    return dict(t.split("=", 1) for t in f[4 + int(f[2]) + int(f[3]) :])


def set_param(line, key, value):
    return " ".join(t if t.split("=")[0] != key or "=" not in t else f"{key}={value}" for t in line.split())


def blobs(line):
    """The layer's weight blobs in the .bin, as (has a 4-byte type tag, float count)."""
    kind, kv = line.split()[0], params(line)
    if kind in ("Convolution", "ConvolutionDepthWise"):
        return [(True, int(kv["6"]))] + ([(False, int(kv["0"]))] if kv.get("5") == "1" else [])
    if kind == "MemoryData":
        return [(False, int(kv.get("0", 1)) * max(int(kv.get("1", 1)), 1) * max(int(kv.get("2", 1)), 1))]
    assert kind in ("BinaryOp Concat Input Interp Permute Pooling Reshape SDPA Sigmoid Slice Softmax Split Swish".split()), (
        f"{kind}: weights unknown"
    )
    return []


# Each scale's class scores: a 1x1 conv to 80 channels, then a Reshape to (N, 80).
nc = 80
producer = {}  # blob name -> the line of the layer that outputs it
for n, l in enumerate(lines[2:], 2):
    f = l.split()
    for out in f[4 + int(f[2]) : 4 + int(f[2]) + int(f[3])]:
        producer[out] = n
heads = [n for n, l in enumerate(lines) if l.startswith("Reshape") and params(l).get("1") == str(nc)]
convs = [producer[lines[n].split()[4]] for n in heads]
assert len(heads) == 3 and all(lines[c].startswith("Convolution ") and params(lines[c])["0"] == str(nc) for c in convs), (
    "unexpected detection head"
)

out = bytearray()
at = 0
for n, l in enumerate(lines[2:], 2):
    for k, (tagged, count) in enumerate(blobs(l)):
        size = (4 if tagged else 0) + 4 * count
        blob = b[at : at + size]
        at += size
        if n in convs:
            data = struct.unpack_from(f"<{count}f", blob, 4 if tagged else 0)
            per = count // nc  # weights per class (cin), or 1 for the bias
            keep = [x for c in CLASSES for x in data[c * per : (c + 1) * per]]
            blob = (blob[:4] if tagged else b"") + struct.pack(f"<{len(keep)}f", *keep)
        out += blob
assert at == len(b), f"walked {at} of {len(b)} bytes: a layer's weights are laid out differently"
for c in convs:
    weights = int(params(lines[c])["6"]) // nc * len(CLASSES)
    lines[c] = set_param(set_param(lines[c], "0", len(CLASSES)), "6", weights)
for n in heads:
    lines[n] = set_param(lines[n], "1", len(CLASSES))

with open(dst + ".param", "w") as f:
    f.write("\n".join(lines) + "\n")
with open(dst + ".bin", "wb") as f:
    f.write(out)
print("wrote", dst)
