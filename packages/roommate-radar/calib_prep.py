"""int8 calibration input for ncnn2table, like roommate-radar's YOLO input. Usage: calib_prep.py IMAGES OUT

IMAGES holds calib_images.txt's COCO val2017 images (people, cats, dogs). Each is gray and letterboxed to 640x640 (pad
114, as ultralytics), two stacked per 640x1280 PNG like the app's stereo pair; ncnn2table resizes them to each S x 2S.
Writes OUT/NNNN.png and OUT/list.txt.
"""

import sys
from pathlib import Path

from PIL import Image

S = 640
src, out = Path(sys.argv[1]), Path(sys.argv[2])
names = [line.split()[0] for line in (Path(__file__).parent / "calib_images.txt").read_text().splitlines()]
out.mkdir(parents=True, exist_ok=True)


def letterbox(name: str) -> Image.Image:
    im = Image.open(src / name).convert("L")
    s = min(S / im.width, S / im.height)
    w, h = round(im.width * s), round(im.height * s)
    c = Image.new("L", (S, S), 114)
    c.paste(im.resize((w, h), Image.Resampling.BILINEAR), ((S - w) // 2, (S - h) // 2))
    return c


files: list[str] = []
for n in range(0, len(names) - 1, 2):
    pair = Image.new("L", (S, 2 * S))
    pair.paste(letterbox(names[n]), (0, 0))
    pair.paste(letterbox(names[n + 1]), (0, S))
    f = out / f"{n // 2:04d}.png"
    pair.save(f)
    files.append(f.name)
(out / "list.txt").write_text("\n".join(files) + "\n")
print(f"wrote {len(files)} pairs to {out}")
