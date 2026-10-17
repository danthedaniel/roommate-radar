"""Adapt an ultralytics YOLO11 ncnn export for yoloxr. Usage: prep_model.py models/yolo11n_576.ncnn

Writes <prefix>_gray.ncnn.{param,bin}:
- The C2PSA attention's MatMul/Softmax/MatMul becomes one SDPA layer. ncnn has no Vulkan MatMul, so it ran on the
  CPU with GPU<->CPU round trips in the middle of the net.
- GRAY2RGB and the 1/255 scale are folded into the first conv: the input is the 1-channel 0..255 image.
"""
import struct
import sys

src = sys.argv[1]
dst = src.removesuffix(".ncnn") + "_gray.ncnn"
lines = open(src + ".param").read().splitlines()

# q arrives transposed to (heads, seq, dim) and pre-scaled by the BinaryOp before it, hence scale 1.
i = next((n for n, l in enumerate(lines) if l.startswith("MatMul")), None)
assert i is not None, "no MatMul (already converted?)"
mm1, sm, mm2 = (l.split() for l in lines[i:i + 3])
assert mm1[0] == "MatMul" and sm[0] == "Softmax" and mm2[0] == "MatMul" and mm2[-1] == "0=1", "unexpected attention block"
q, k, v, out = mm1[4], mm1[5], mm2[4], mm2[6]
lines[i:i + 3] = [
    f"Permute sdpa_k_t 1 1 {k} sdpa_k 0=1",
    f"Permute sdpa_v_t 1 1 {v} sdpa_v 0=1",
    f"SDPA sdpa 3 1 {q} sdpa_k sdpa_v sdpa_out 6=1.0",
    f"Permute sdpa_out_t 1 1 sdpa_out {out} 0=1",
]
nl, nb = map(int, lines[1].split())
lines[1] = f"{nl + 1} {nb + 1}"

# The first conv's weights come first in the .bin: a 4-byte zero tag, then fp32 [out][3][kh][kw].
i = next(n for n, l in enumerate(lines) if l.startswith("Convolution"))
kv = dict(t.split("=") for t in lines[i].split()[6:])
co, kk = int(kv["0"]), int(kv["1"]) * int(kv.get("11", kv["1"]))
assert int(kv["6"]) == co * 3 * kk, "first conv doesn't take 3 channels (already converted?)"
lines[i] = lines[i].replace(f"6={kv['6']}", f"6={co * kk}")
b = open(src + ".bin", "rb").read()
assert b[:4] == bytes(4), "weights aren't fp32"
w = struct.unpack_from(f"<{co * 3 * kk}f", b, 4)
folded = [sum(w[(o * 3 + c) * kk + j] for c in range(3)) / 255 for o in range(co) for j in range(kk)]

open(dst + ".param", "w").write("\n".join(lines) + "\n")
open(dst + ".bin", "wb").write(b[:4] + struct.pack(f"<{co * kk}f", *folded) + b[4 + co * 3 * kk * 4:])
print("wrote", dst)
