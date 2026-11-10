#!/usr/bin/env python3
"""Convert the simulator's .ppm dumps to .png and build a side-by-side
montage (reference | layer0 | layer1 | ...). Standard library only.

    python3 scripts/preview.py build/sim/out
"""
import os
import struct
import sys
import zlib


def read_ppm(path):
    with open(path, "rb") as f:
        data = f.read()
    parts, i = [], 0
    while len(parts) < 4:
        while data[i:i + 1].isspace():
            i += 1
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        parts.append(data[i:j])
        i = j
    assert parts[0] == b"P6", path
    w, h = int(parts[1]), int(parts[2])
    return w, h, data[i + 1:i + 1 + w * h * 3]


def write_png(path, w, h, rgb):
    raw = b"".join(b"\x00" + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))

    def chunk(tag, body):
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(chunk(b"IEND", b""))


def main(d):
    names = ["reference"] + sorted(n[:-4] for n in os.listdir(d) if n.startswith("sim_layer") and n.endswith(".ppm"))
    imgs = []
    for n in names:
        p = os.path.join(d, n + ".ppm")
        if not os.path.exists(p):
            continue
        w, h, rgb = read_ppm(p)
        write_png(os.path.join(d, n + ".png"), w, h, rgb)
        imgs.append((w, h, rgb))
    if not imgs:
        print("no ppm files in", d)
        return
    h = imgs[0][1]
    gap = 8
    W = sum(w for w, _, _ in imgs) + gap * (len(imgs) - 1)
    rows = []
    for y in range(h):
        row = b""
        for k, (w, _, rgb) in enumerate(imgs):
            row += rgb[y * w * 3:(y + 1) * w * 3]
            if k < len(imgs) - 1:
                row += b"\xff" * (gap * 3)
        rows.append(row)
    write_png(os.path.join(d, "montage.png"), W, h, b"".join(rows))
    print("wrote", os.path.join(d, "montage.png"), "(" + " | ".join(names) + ")")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "build/sim/out")
