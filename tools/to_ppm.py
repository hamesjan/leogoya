#!/usr/bin/env python3
"""Convert any image (jpg/png/...) to a square binary PPM for the simulator.
Needs Pillow:  python3 -m pip install pillow

    python3 tools/to_ppm.py photo.jpg ref.ppm [size]
    make run REF=ref.ppm
"""
import sys

from PIL import Image

src, dst = sys.argv[1], sys.argv[2]
size = int(sys.argv[3]) if len(sys.argv) > 3 else 512
im = Image.open(src).convert("RGB")
s = min(im.size)  # center-crop to square: the canvas is 38" x 38"
left, top = (im.width - s) // 2, (im.height - s) // 2
im.crop((left, top, left + s, top + s)).resize((size, size), Image.LANCZOS).save(dst, format="PPM")
print("wrote", dst)
