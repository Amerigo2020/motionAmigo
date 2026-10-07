"""Assembles the frames written by record.mjs into an optimized GIF.

Usage: uv run --with pillow python web/tools/make_gif.py frames-dir docs/media/demo.gif
"""

import sys
from pathlib import Path

from PIL import Image

src, dst = Path(sys.argv[1]), Path(sys.argv[2])
width = int(sys.argv[3]) if len(sys.argv) > 3 else 800
frames = []
for path in sorted(src.glob("frame_*.png")):
    img = Image.open(path).convert("RGB")
    img = img.resize((width, round(img.height * width / img.width)), Image.LANCZOS)
    frames.append(img)
palette = frames[len(frames) // 2].quantize(colors=128, method=Image.Quantize.MEDIANCUT)
quantized = [f.quantize(palette=palette, dither=Image.Dither.NONE) for f in frames]
dst.parent.mkdir(parents=True, exist_ok=True)
quantized[0].save(dst, save_all=True, append_images=quantized[1:], duration=70, loop=0, optimize=True)
print(f"{len(frames)} frames, {dst.stat().st_size / 1e6:.2f} MB -> {dst}")
