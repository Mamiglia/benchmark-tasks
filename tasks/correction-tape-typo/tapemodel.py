"""The correction-tape rule and the score. Pure Python (no Isaac), unit-tested.

Inputs per physics step, in the sheet frame (mm; u along the printed line in reading direction from the sheet's left
edge, w down the page from its top edge): the centre of the applicator's pressing lip, the lip's normal force on the
sheet (N), its tilt from flat in the stroke plane and sideways (deg), and two unit vectors in the sheet plane: ``lead``
(the way the dispenser's lip lays tape, a sideways pull, projected on the sheet) and ``across`` (the tape's width).

The tape comes off the lip only while the lip is pressed flat on the paper and pulled the way it lays tape:
- force at least ``f_min`` (lighter, and the film does not grip the paper: gaps);
- lip within ``pitch_max`` of flat (the dispenser held at about 45 degrees: 30 to 60), and within ``roll_max``
  sideways (tilted, one edge of the lip lifts and the tape folds);
- moving along ``lead`` (within ``dir_cos`` of it), at any speed: the film feeds out as far as the lip moves.
Where it comes off, it covers the 4.2 mm wide strip swept by the lip (a 0.1 mm grid). Pressed and pushed the wrong
way (against ``lead``) for more than ``back_mm``: the tape bunches up at the lip (latched). A stroke is one
unbroken run of laid tape; lifting (no contact for ``lift_s``) and laying again is a second stroke.
"""

from __future__ import annotations

import json
import math
import pathlib
from dataclasses import dataclass, field

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
LAYOUT = json.loads((HERE / "sheet_layout.json").read_text())
CELL = 0.1  # mm
U0, U1, W0, W1 = 15.0, 195.0, 125.0, 200.0  # the grid: the whole letter


@dataclass(frozen=True)
class Law:
    tape_w: float = 25.4 / 6  # mm, 1/6 in
    f_min: float = 0.8  # N
    pitch_max: float = 15.0  # deg
    roll_max: float = 8.0  # deg
    dir_cos: float = 0.7
    step_mm: float = 0.05  # forward travel that feeds tape
    back_mm: float = 1.0
    lift_s: float = 0.1
    touch_n: float = 0.05  # N: the lip counts as on the paper


@dataclass
class Tape:
    law: Law = field(default_factory=Law)

    def __post_init__(self):
        self.reset()

    def reset(self) -> None:
        self.grid = np.zeros((round((U1 - U0) / CELL), round((W1 - W0) / CELL)), bool)
        self.outside = False  # tape laid off the grid
        self.bunched = False
        self.strokes = 0
        self.laying = False
        self.anchor = None
        self.back = 0.0
        self.off_t = 1e9
        self.last = None
        self.laid_mm = 0.0
        self.stroke_mm = 0.0

    def step(self, dt, p, force, pitch, roll, lead, across) -> str:
        """One physics step. Returns a message when something happens (for the log)."""
        L, msg = self.law, ""
        p = np.asarray(p, float)
        lead = np.asarray(lead, float)
        prev, self.last = self.last, p
        if force < L.touch_n:
            self.off_t += dt
            if self.off_t >= L.lift_s:
                self.laying, self.anchor = False, None
            self.back = 0.0
            return msg
        self.off_t = 0.0
        if prev is None:
            return msg
        pressed = force >= L.f_min and abs(pitch) <= L.pitch_max and abs(roll) <= L.roll_max
        along = float((p - prev) @ lead)
        if pressed and along < 0:
            self.back += -along
            if self.back > L.back_mm and not self.bunched:
                self.bunched, msg = True, "tape bunched: pushed the wrong way"
        elif along > 0:
            self.back = 0.0
        if not pressed:
            self.anchor = None  # the film lets go of the paper here: what follows is a new piece
            return msg
        if self.anchor is None:
            self.anchor = p
            return msg
        d = p - self.anchor  # from where the film is stuck down to the lip
        dist = float(np.linalg.norm(d))
        go = float(d @ lead)
        if go < 0:
            self.anchor = p  # moved back: the film stays where it was laid (bunching is counted above)
        elif go >= L.step_mm and go >= L.dir_cos * dist:
            if not self.laying:
                self.strokes += 1
                self.laying = True
                self.stroke_mm = 0.0
                self.stroke_from = self.anchor
                msg = f"stroke {self.strokes} starts at u {self.anchor[0]:.2f} w {self.anchor[1]:.2f}"
            self._paint(self.anchor, p, np.asarray(across, float))
            self.laid_mm += dist
            self.stroke_mm += dist
            self.anchor = p
        elif dist >= L.step_mm:
            self.anchor = p  # moved sideways: the lip slid over the paper without feeding tape
        return msg

    def _paint(self, a, b, across) -> None:
        h = self.law.tape_w / 2
        n = across / max(np.linalg.norm(across), 1e-9)
        quad = np.array([a - h * n, b - h * n, b + h * n, a + h * n])
        lo, hi = quad.min(0), quad.max(0)
        if lo[0] < U0 or lo[1] < W0 or hi[0] > U1 or hi[1] > W1:
            self.outside = True
        i0, j0 = max(int((lo[0] - U0) / CELL), 0), max(int((lo[1] - W0) / CELL), 0)
        i1 = min(int(math.ceil((hi[0] - U0) / CELL)) + 1, self.grid.shape[0])
        j1 = min(int(math.ceil((hi[1] - W0) / CELL)) + 1, self.grid.shape[1])
        if i1 <= i0 or j1 <= j0:
            return
        uu, ww = np.meshgrid(U0 + (np.arange(i0, i1) + 0.5) * CELL, W0 + (np.arange(j0, j1) + 0.5) * CELL,
                             indexing="ij")
        pts = np.stack([uu, ww], -1)
        inside = np.ones(uu.shape, bool)
        for k in range(4):  # convex quad (either winding)
            e = quad[(k + 1) % 4] - quad[k]
            c = e[0] * (pts[..., 1] - quad[k][1]) - e[1] * (pts[..., 0] - quad[k][0])
            inside &= c >= -1e-9 if _ccw(quad) else c <= 1e-9
        self.grid[i0:i1, j0:j1] |= inside


def _ccw(q) -> bool:
    return sum(q[k][0] * q[(k + 1) % 4][1] - q[(k + 1) % 4][0] * q[k][1] for k in range(4)) > 0


def _box(grid, w):
    i0, i1 = round((w["u0"] - U0) / CELL), round((w["u1"] - U0) / CELL)
    j0, j1 = round((w["w0"] - W0) / CELL), round((w["w1"] - W0) / CELL)
    return grid[i0:i1, j0:j1]


def word_cover(grid, w) -> float:
    """Fraction of the word's ink box under tape."""
    return float(_box(grid, w).mean())


def covered_length(grid, w) -> float:
    """mm of the word's length under tape: columns of its box at least half covered (what a photo shows as a letter
    part hidden)."""
    return float((_box(grid, w).mean(axis=1) >= 0.5).sum() * CELL)


def touched(grid, w) -> bool:
    """Any of the word's box under tape more than 0.2 mm along the line (a visible nick into a letter)."""
    return float((_box(grid, w).any(axis=1)).sum() * CELL) > 0.2


def score_tape(t: Tape) -> dict:
    words, k = LAYOUT["words"], LAYOUT["typo"]
    cover = word_cover(t.grid, words[k])
    others = [w for i, w in enumerate(words) if i != k]
    worst = max(covered_length(t.grid, w) for w in others)
    clean = not t.outside and not any(touched(t.grid, w) for w in others)
    one = t.strokes == 1 and not t.bunched
    if worst > 1.5 or t.outside:
        s = 0.0
    else:
        s = 0.6 * min(1.0, cover / 0.95) + (0.2 if cover >= 0.9 and clean else 0.0) + (0.2 if cover >= 0.9 and one
                                                                                     else 0.0)
    return {"score": round(s, 4), "cover": round(cover, 4), "neighbour_mm": round(worst, 2), "clean": clean,
            "strokes": t.strokes, "bunched": t.bunched, "outside": t.outside, "laid_mm": round(t.laid_mm, 1)}


if __name__ == "__main__":  # quick self-check: one clean stroke over the word
    w = LAYOUT["words"][LAYOUT["typo"]]
    t, wm = Tape(), (w["w0"] + w["w1"]) / 2
    u = w["u0"] - 0.3
    while u < w["u1"] + 0.3:
        t.step(1 / 240, (u, wm), 1.5, 0, 0, (1, 0), (0, 1))
        u += 20 / 240
    print(score_tape(t))
