"""Makes the printed sheet: sheet.png (A4 at 300 dpi, print at 100%) and sheet_layout.json (every word's ink box).

    python make_sheet.py [path/to/LiberationSans-Regular.ttf]

Font: Liberation Sans Regular (metric-compatible with Arial; SIL Open Font License 1.1), 12 pt, line pitch 18 pt.
Boxes are in mm in the sheet frame: u from the sheet's left edge along the line (reading direction), w from the
sheet's top edge down the page. The simulation draws sheet_tex.png (the strip around the text) on the sheet.
"""

from __future__ import annotations

import json
import pathlib
import sys

from PIL import Image, ImageDraw, ImageFont

HERE = pathlib.Path(__file__).resolve().parent
DPI = 300
MM = DPI / 25.4
A4 = (210.0, 297.0)  # portrait, mm
PT = 25.4 / 72  # mm per point
SIZE_PT, PITCH_PT = 12, 18
LEFT, FIRST_BASE = 25.0, 140.0  # margin and the first baseline, mm from the top
LINES = [
    "Dear Ms Carter,",
    "",
    "Thank you for sending the quarterly figures last week. We have",
    "checked them against our own records and they agree in full.",
    "Please file teh report by Friday so the board can see it.",
    "The signed copy will follow in the post next Monday.",
    "",
    "Kind regards,",
]
TYPO_LINE, TYPO = 4, "teh"
TEX_W = (125.0, 200.0)  # sheet_tex.png covers w 125-200 mm (the letter), full width


def main(font_path: str) -> dict:
    font = ImageFont.truetype(font_path, round(SIZE_PT * PT * MM))
    img = Image.new("L", (round(A4[0] * MM), round(A4[1] * MM)), 255)
    d = ImageDraw.Draw(img)
    words = []
    for i, line in enumerate(LINES):
        base = FIRST_BASE + i * PITCH_PT * PT
        x = LEFT * MM
        for j, word in enumerate(line.split(" ")):
            if not word:
                continue
            if j:
                x += font.getlength(" ")
            d.text((x, base * MM), word, font=font, fill=0, anchor="ls")
            l, t, r, b = d.textbbox((x, base * MM), word, font=font, anchor="ls")
            words.append({"line": i, "word": word, "u0": round(l / MM, 3), "u1": round(r / MM, 3),
                          "w0": round(t / MM, 3), "w1": round(b / MM, 3)})
            x += font.getlength(word)
    img.save(HERE / "sheet.png", dpi=(DPI, DPI))
    top, bot = round(TEX_W[0] * MM), round(TEX_W[1] * MM)
    tex = img.crop((0, top, img.width, bot))
    tex.resize((1240, round(1240 * (bot - top) / img.width))).save(HERE / "sheet_tex.png")
    typo = [k for k, w in enumerate(words) if w["line"] == TYPO_LINE and w["word"] == TYPO]
    assert len(typo) == 1
    k = typo[0]
    layout = {"sheet_mm": A4, "tex_w_mm": TEX_W, "font": "Liberation Sans Regular 12 pt", "words": words,
              "typo": k, "neighbours": [k - 1, k + 1]}
    (HERE / "sheet_layout.json").write_text(json.dumps(layout, indent=1))
    return layout


if __name__ == "__main__":
    lay = main(sys.argv[1] if len(sys.argv) > 1 else "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf")
    for k in (lay["typo"] - 1, lay["typo"], lay["typo"] + 1):
        w = lay["words"][k]
        print(w["word"], f"u {w['u0']:.2f}-{w['u1']:.2f} ({w['u1'] - w['u0']:.2f}) w {w['w0']:.2f}-{w['w1']:.2f} "
              f"({w['w1'] - w['w0']:.2f})")
