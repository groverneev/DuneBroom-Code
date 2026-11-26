#!/usr/bin/env python3
"""Write a printable US-letter checkerboard PDF for camera calibration.

    python3 tools/make_checkerboard.py            -> docs/checkerboard_25mm.pdf

7 x 9 squares of 25 mm (6 x 8 inner corners, what tools/camera_cal.py looks
for). Print at 100 % ("actual size"), then measure a square: pass the real size
to camera_cal.py with --square-mm if the printer scaled it.
"""

from __future__ import annotations

import sys
from pathlib import Path

COLS, ROWS, SQUARE_MM = 7, 9, 25.0
PT_PER_MM = 72 / 25.4
PAGE_W, PAGE_H = 612, 792  # US letter, points


def page_content() -> bytes:
    s = SQUARE_MM * PT_PER_MM
    x0 = (PAGE_W - COLS * s) / 2
    y0 = (PAGE_H - ROWS * s) / 2 - 20
    ops = ["0 g"]
    for r in range(ROWS):
        for c in range(COLS):
            if (r + c) % 2 == 0:
                ops.append(f"{x0 + c * s:.3f} {y0 + r * s:.3f} {s:.3f} {s:.3f} re f")
    text = [
        "DuneBroom camera calibration: 7 x 9 squares, 25 mm each",
        "Print at 100% / Actual size (no 'fit to page'). Tape flat to card.",
    ]
    for i, line in enumerate(text):
        ops.append(f"BT /F1 11 Tf {x0:.1f} {PAGE_H - 40 - i * 15} Td ({line}) Tj ET")
    return "\n".join(ops).encode()


def build_pdf() -> bytes:
    content = page_content()
    objs = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {PAGE_W} {PAGE_H}] "
        f"/Contents 4 0 R /Resources << /Font << /F1 5 0 R >> >> >>".encode(),
        b"<< /Length %d >>\nstream\n" % len(content) + content + b"\nendstream",
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>",
    ]
    out = bytearray(b"%PDF-1.4\n")
    offsets = []
    for i, body in enumerate(objs, 1):
        offsets.append(len(out))
        out += f"{i} 0 obj\n".encode() + body + b"\nendobj\n"
    xref = len(out)
    out += f"xref\n0 {len(objs) + 1}\n0000000000 65535 f \n".encode()
    for off in offsets:
        out += f"{off:010d} 00000 n \n".encode()
    out += f"trailer\n<< /Size {len(objs) + 1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n".encode()
    return bytes(out)


def main() -> None:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else (
        Path(__file__).resolve().parent.parent / "docs" / "checkerboard_25mm.pdf")
    path.write_bytes(build_pdf())
    print(path)


if __name__ == "__main__":
    main()
