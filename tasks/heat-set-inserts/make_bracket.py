"""Writes bracket.stl (needs `pip install manifold3d numpy`) and bracket_drawing.svg (no dependencies).

The bracket: an 80 x 40 x 12 mm PETG plate with four 4.0 mm blind holes, 7.0 mm deep, on its top face at
(+-28, +-10) mm from its centre: the hole CNC Kitchen gives for its M3 x 5.7 mm standard insert (1 mm deeper than
the insert, so an insert pushed past flush has 1 mm to go before it bottoms).
Print flat, top face up, 0.2 mm layers, 4 perimeters, 30 % infill.
"""

import pathlib
import struct

L, W, H = 80.0, 40.0, 12.0
HOLE_D, HOLE_DEPTH = 4.0, 7.0
HOLES = [(sx * 28.0, sy * 10.0) for sx in (-1, 1) for sy in (-1, 1)]
HERE = pathlib.Path(__file__).resolve().parent


def write_stl(path: pathlib.Path) -> None:
    import manifold3d as m3d
    import numpy as np

    plate = m3d.Manifold.cube((L, W, H), center=True).translate((0, 0, H / 2))
    for x, y in HOLES:
        hole = m3d.Manifold.cylinder(HOLE_DEPTH + 1.0, HOLE_D / 2, circular_segments=64).translate(
            (x, y, H - HOLE_DEPTH))
        plate = plate - hole
    mesh = plate.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3]
    tris = np.asarray(mesh.tri_verts)
    with open(path, "wb") as f:
        f.write(b"heat-set-inserts bracket, mm".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            a, b, c = v[t]
            n = np.cross(b - a, c - a)
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))


def write_svg(path: pathlib.Path) -> None:
    s = 6.0  # px per mm
    ox, oy = 60, 60
    top = []
    X = lambda x: ox + (x + L / 2) * s  # noqa: E731
    Y = lambda y: oy + (W / 2 - y) * s  # noqa: E731
    top.append(f'<rect x="{X(-L/2)}" y="{Y(W/2)}" width="{L*s}" height="{W*s}" fill="none" stroke="black"/>')
    for x, y in HOLES:
        top.append(f'<circle cx="{X(x)}" cy="{Y(y)}" r="{HOLE_D/2*s}" fill="none" stroke="black"/>')
        top.append(f'<line x1="{X(x)-12}" y1="{Y(y)}" x2="{X(x)+12}" y2="{Y(y)}" stroke="grey" stroke-dasharray="3,2"/>')
        top.append(f'<line x1="{X(x)}" y1="{Y(y)-12}" x2="{X(x)}" y2="{Y(y)+12}" stroke="grey" stroke-dasharray="3,2"/>')
    t = lambda x, y, txt: f'<text x="{x}" y="{y}" font-size="13" font-family="sans-serif">{txt}</text>'  # noqa: E731
    top += [t(X(-10), Y(W/2) - 12, "80.0"), t(X(L/2) + 8, Y(0), "40.0"),
            t(X(-28) - 20, Y(-W/2) + 22, "56.0 hole pitch"), t(X(L/2) + 8, Y(10) + 4, "20.0 pitch"),
            t(X(-L/2), Y(-W/2) + 44, "4 x  4.0 mm, 7.0 deep (blind), top face; centres at (+-28, +-10) from the centre")]
    # section through two holes
    sy0 = Y(-W/2) + 90
    Z = lambda z: sy0 + (H - z) * s  # noqa: E731
    sec = [f'<rect x="{X(-L/2)}" y="{Z(H)}" width="{L*s}" height="{H*s}" fill="#ddd" stroke="black"/>']
    for x in (-28, 28):
        sec.append(f'<rect x="{X(x - HOLE_D/2)}" y="{Z(H)}" width="{HOLE_D*s}" height="{HOLE_DEPTH*s}" fill="white" '
                   f'stroke="black"/>')
    sec += [t(X(L/2) + 8, Z(H/2), "12.0"), t(X(28) + 16, Z(H - HOLE_DEPTH/2), "7.0"),
            t(X(-L/2), Z(0) + 24, "Section through two holes. Material PETG, all dimensions mm, tolerance +-0.1.")]
    h = int(Z(0) + 50)
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{int(2*ox + L*s + 120)}" height="{h}">'
           f'<rect width="100%" height="100%" fill="white"/>{"".join(top + sec)}</svg>')
    path.write_text(svg)


if __name__ == "__main__":
    write_svg(HERE / "bracket_drawing.svg")
    write_stl(HERE / "bracket.stl")
    print("wrote bracket.stl and bracket_drawing.svg")
