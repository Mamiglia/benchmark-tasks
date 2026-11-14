"""Tests for waggle-mix-tubes: the liquid model, the scorer and the demo's vision. No Isaac Sim needed."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task  # noqa: E402
from liquid import TubeLiquid, wall_top_table  # noqa: E402
from mixkit import RackState, TubeState, stage_for  # noqa: E402

DT = 1 / 240


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def run(traj, seconds: float = 6.0) -> TubeLiquid:
    liq = TubeLiquid()
    for i in range(int(seconds / DT)):
        b, r = traj(i * DT)
        liq.step(b, r, DT)
    return liq


def side(amp, hz):
    return lambda t: (np.array([0.0, amp * math.sin(2 * math.pi * hz * t), 0.0]), np.eye(3))


def swing(rad, hz, r=0.28):  # about a pivot r above the bottom (the wrist), like the demo's waggle
    def f(t):
        rot = rot_x(rad * math.sin(2 * math.pi * hz * t))
        return np.array([0.0, 0.0, r]) - rot @ np.array([0.0, 0.0, r]), rot
    return f


# -- liquid model ----------------------------------------------------------------------------------------------------


def test_wall_table_level_and_linear():
    ts, tops = wall_top_table(0.007, 0.026)
    assert tops[0] == pytest.approx(0.026, abs=1e-4)
    t = 1.0  # bottom still covered (R tan phi = 7 mm < 26 mm): plane through the volume's centre
    assert np.interp(t, ts, tops) == pytest.approx(0.026 + 0.007 * t, abs=2e-4)
    assert np.all(np.diff(tops) >= -1e-6)


def test_still_tube_neither_mixes_nor_spills():
    liq = run(lambda t: (np.zeros(3), np.eye(3)))
    assert not liq.spilled and liq.strokes == 0 and liq.contrast == 1.0


def test_gentle_sway_does_not_mix():
    liq = run(side(0.01, 2.0))
    assert not liq.spilled and liq.contrast == 1.0


@pytest.mark.parametrize("traj", [side(0.015, 3.0), swing(0.12, 3.0)])
def test_waggle_mixes_without_spilling(traj):
    liq = run(traj, 3.0)
    assert liq.mixed and not liq.spilled
    assert liq.peak_wall < 0.06  # the liquid stays well below the mouth (98 mm)


def test_vigorous_side_shaking_spills():
    """Shaken sideways near the slosh mode's frequency (about 8 Hz), the liquid climbs the wall to the mouth."""
    liq = run(side(0.01, 8.0), 2.0)
    assert liq.spilled


def test_hard_up_and_down_shaking_spills():
    liq = run(lambda t: (np.array([0.0, 0.0, 0.04 * math.sin(2 * math.pi * 4 * t)]), np.eye(3)), 2.0)
    assert liq.spilled


def test_inverting_spills():
    liq = run(lambda t: (np.zeros(3), rot_x(math.pi * min(1.0, t / 1.5))), 3.0)
    assert liq.spilled


@pytest.mark.parametrize("deg,spills", [(60, False), (80, False), (88, True)])
def test_slow_tilt(deg, spills):
    liq = run(lambda t: (np.zeros(3), rot_x(math.radians(deg) * min(1.0, t / 3))), 5.0)
    assert liq.spilled is spills


def test_spill_latches():
    liq = TubeLiquid()
    for i in range(int(2 / DT)):
        liq.step(np.zeros(3), rot_x(math.pi * min(1.0, i * DT)), DT)
    assert liq.spilled
    for _ in range(int(2 / DT)):
        liq.step(np.zeros(3), np.eye(3), DT)
    assert liq.spilled


def test_stage_mapping():
    assert [stage_for(c) for c in (1.0, 0.7, 0.5, 0.2, 0.1, 0.0)] == [0, 1, 2, 3, 4, 4]


# -- scorer ----------------------------------------------------------------------------------------------------------


def tube(k, mixed=True, spilled=False, z=0.0, tilt=0.0, dx=0.0):
    ax = rot_x(math.radians(tilt))[:, 2]
    bottom = np.array([task.RACK_X + dx, task.HOLES_Y[k], task.FLOOR + 0.001 + z])
    pos = bottom + ax * 0.05
    return TubeState(pos=pos, axis=ax, bottom=bottom, mouth=bottom + ax * 0.1, tilt_deg=tilt, lin_vel=np.zeros(3),
                     ang_vel=np.zeros(3), spilled=spilled, contrast=0.05 if mixed else 1.0, mixed=mixed, strokes=0,
                     wall_mm=26, peak_wall_mm=30)


def rack(tubes, hands_at=None):
    far = np.array([[0.0, 0.5, 1.2]] * 3)
    pads = {"left_robot": far, "right_robot": far}
    if hands_at is not None:
        pads = {"left_robot": np.array([hands_at] * 3), "right_robot": far}
    z = np.zeros(3)
    return {"tube_rack": RackState(pos=z, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=z, ang_vel=z, low=z, high=z,
                                   tubes=tuple(tubes), pads=pads, open_frac={})}


def test_scores():
    assert task.score(rack([tube(0, False), tube(1, False)])) == 0.0  # untouched
    assert task.score(rack([tube(0), tube(1, False)])) == 0.25
    assert task.score(rack([tube(0, z=0.05), tube(1, z=0.05)])) == 0.5  # both mixed, held above the rack
    t0 = tube(0)
    assert task.score(rack([t0, tube(1)], hands_at=t0.mouth)) == 0.5  # racked but still gripped
    assert task.score(rack([tube(0), tube(1)])) == 0.75 and task.success(rack([tube(0), tube(1)]))


def test_spill_is_zero_whatever_else():
    assert task.score(rack([tube(0, spilled=True), tube(1)])) == 0.0


def test_racked_needs_own_hole_and_upright():
    assert not task.racked(tube(0, dx=0.01), 0)
    assert task.racked(tube(0, dx=0.005), 0)
    assert not task.racked(tube(0, tilt=25), 0)
    assert not task.racked(tube(0), 1)  # the other hole
    assert task.racked(tube(1), 1)
    # leaning 15 degrees with the bottom slid under the board, the tube still passing through its hole
    lean = tube(0, tilt=15, dx=0.0)
    off = lean.axis * (task.BOARD_MID - lean.bottom[2]) / lean.axis[2]
    b = lean.bottom - np.r_[off[:2], 0.0]
    assert task.racked(TubeState(**{**lean.__dict__, "bottom": b}), 0)
    assert not task.racked(TubeState(**{**lean.__dict__, "bottom": b + (0, 0.01, 0)}), 0)


# -- demo vision -----------------------------------------------------------------------------------------------------


def test_top_blueness_on_synthetic_image():
    pytest.importorskip("yam_arena")
    import mixdemo
    from mixkit import stage_colors

    vals = []
    for st in range(5):
        lo, up = (np.array(c) * 255 for c in stage_colors(st))
        img = np.zeros((176, 320, 3), np.uint8)
        img[:] = (225, 205, 170)  # the table
        img[60:76, 150:166] = up  # the top face, lit
        img[76:84, 150:166] = lo * 0.6  # the column's far side, in shade
        img[50:60, 130:186] = (40, 140, 75)  # the green rack
        vals.append(mixdemo.top_blueness(img))
    assert vals[0] < 0.15 and all(b > a for a, b in zip(vals, vals[1:]))
    assert mixdemo.top_blueness(np.full((176, 320, 3), (225, 205, 170), np.uint8)) is None


def test_find_rack_on_synthetic_image():
    pytest.importorskip("yam_arena")
    import mixdemo

    cam = mixdemo.TopCamera(320, 176)
    img = np.full((176, 320, 3), 180, np.uint8)
    corners = [cam.project((x, y, mixdemo.TOP + mixdemo.RACK_TOP)) for x in (0.23, 0.29) for y in (-0.12, 0.12)]
    us, vs = [c[0] for c in corners], [c[1] for c in corners]
    img[int(round(min(vs))):int(round(max(vs))) + 1, int(round(min(us))):int(round(max(us))) + 1] = (40, 140, 75)
    (u, v), ul, ur = mixdemo.find_rack(img)
    mid = cam.on_plane((u, v), 2, mixdemo.TOP + mixdemo.RACK_TOP)
    assert mid[0] - mixdemo.RACK_DEPTH / 2 == pytest.approx(0.26, abs=0.004)
    assert abs(mid[1]) < 0.004
