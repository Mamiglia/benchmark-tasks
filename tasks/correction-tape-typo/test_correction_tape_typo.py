"""Unit tests for the tape rule and the score (no Isaac Sim needed): python -m pytest test_correction_tape_typo.py"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from tapemodel import LAYOUT, Tape, score_tape  # noqa: E402

DT = 1 / 240
WORDS = LAYOUT["words"]
TYPO = WORDS[LAYOUT["typo"]]
WM = (TYPO["w0"] + TYPO["w1"]) / 2


def stroke(t, u_from, u_to, w=WM, force=1.5, pitch=0.0, roll=0.0, v=10.0, lead=(1.0, 0.0)):
    """Drag the lip from u_from to u_to (mm) at v mm/s, then lift it for 0.2 s. ``lead``: the way it lays tape."""
    step = v * DT * (1 if u_to > u_from else -1)
    u = u_from
    while (u_to - u) * step > 0:
        t.step(DT, (u, w), force, pitch, roll, lead, (0.0, 1.0))
        u += step
    for _ in range(48):
        t.step(DT, (u, w), 0.0, pitch, roll, lead, (0.0, 1.0))
    return t


def run(*strokes, **kw):
    t = Tape()
    for a, b in strokes:
        stroke(t, a, b, **kw)
    return score_tape(t)


def test_clean_stroke_scores_one():
    r = run((TYPO["u0"] - 0.5, TYPO["u1"] + 0.3))
    assert r["score"] == 1.0 and r["strokes"] == 1 and r["clean"]


def test_light_touch_lays_nothing():
    r = run((TYPO["u0"] - 0.5, TYPO["u1"] + 0.3), force=0.4)
    assert r["score"] == 0.0 and r["laid_mm"] == 0.0


def test_too_steep_or_tilted_lays_nothing():
    assert run((TYPO["u0"] - 0.5, TYPO["u1"] + 0.3), pitch=25.0)["score"] == 0.0
    assert run((TYPO["u0"] - 0.5, TYPO["u1"] + 0.3), roll=12.0)["score"] == 0.0


def test_wrong_way_push_bunches_and_lays_nothing():
    r = run((TYPO["u1"] + 0.3, TYPO["u0"] - 0.5))  # right to left
    assert r["bunched"] and r["laid_mm"] == 0.0 and r["score"] == 0.0


def test_whole_line_sweep_scores_zero():
    line = [w for w in WORDS if w["line"] == TYPO["line"]]
    r = run((line[0]["u0"] - 1, line[-1]["u1"] + 1))
    assert r["cover"] == 1.0 and r["score"] == 0.0


def test_two_strokes_lose_the_stroke_credit():
    mid = (TYPO["u0"] + TYPO["u1"]) / 2
    r = run((TYPO["u0"] - 0.5, mid), (mid - 0.3, TYPO["u1"] + 0.3))
    assert r["strokes"] == 2 and r["score"] == 0.8


def test_overrun_into_the_next_word():
    # 1 mm into "report" (past the 1.1 mm gap): touched, not over 1.5 mm -> no clean credit
    r = run((TYPO["u0"] - 0.5, TYPO["u1"] + 2.1))
    assert r["clean"] is False and r["score"] == 0.8
    # 3 mm into it -> 0
    assert run((TYPO["u0"] - 0.5, TYPO["u1"] + 4.1))["score"] == 0.0


def test_partial_cover_partial_credit():
    mid = (TYPO["u0"] + TYPO["u1"]) / 2
    r = run((TYPO["u0"] - 0.5, mid))
    assert 0.2 < r["score"] < 0.45 and r["cover"] < 0.9


def test_off_centre_line_loses_cover():
    r = run((TYPO["u0"] - 0.5, TYPO["u1"] + 0.3), w=WM + 1.5)
    assert r["cover"] < 0.8 and r["score"] < 0.5
