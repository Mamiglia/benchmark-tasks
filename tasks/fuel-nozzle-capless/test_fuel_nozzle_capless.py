"""Unit tests for the latch and score bookkeeping and the geometry (NumPy only; no Isaac Sim)."""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import nozzlemodel as M  # noqa: E402

DT = 1 / 240
HIGH = M.TOP + 0.2


def run(mon, seconds, **kw):
    args = dict(lever=0.0, clip=0.0, depth=-0.2, off_axis=0.0, housing_z=HIGH, vz=0.0, lever_touched=False)
    args.update(kw)
    cmds = []
    for _ in range(round(seconds / DT)):
        cmds += mon.step(0.0, **args)
    return cmds


def full_fill(mon):
    run(mon, 1.0, depth=0.095)
    assert mon.score(False) == 0.15
    run(mon, 0.5, depth=0.095, lever=0.36, lever_touched=True)
    assert run(mon, 0.1, depth=0.095, lever=0.36, clip=0.7, lever_touched=True) == ["latch"]
    run(mon, 3.1, depth=0.095, lever=M.LATCH_HOLD, clip=0.65)
    assert mon.filled and mon.score(False) == 0.5


def test_full_sequence():
    mon = M.Monitor(dt=DT)
    full_fill(mon)
    assert run(mon, 0.2, depth=0.095, lever=0.36, lever_touched=True) == ["unlatch"]
    run(mon, 0.5, depth=0.095, lever=0.0)
    assert mon.released and mon.score(False) == 0.7 and mon.score(True) == 1.0


def test_release_does_not_relatch_until_clip_drops():
    mon = M.Monitor(dt=DT)
    full_fill(mon)
    assert run(mon, 0.5, depth=0.095, lever=0.36, clip=0.6, lever_touched=True) == ["unlatch"]
    assert not mon.latched
    run(mon, 0.2, depth=0.095, lever=0.36, clip=0.1, lever_touched=True)
    assert run(mon, 0.1, depth=0.095, lever=0.36, clip=0.7, lever_touched=True) == ["latch"]


def test_no_insert_no_credit():
    mon = M.Monitor(dt=DT)
    run(mon, 2.0, depth=0.05)
    assert mon.score(True) == 0.0


def test_hand_held_lever_is_not_a_fill():
    mon = M.Monitor(dt=DT)
    run(mon, 1.0, depth=0.095)
    run(mon, 5.0, depth=0.095, lever=0.36, lever_touched=True)
    assert not mon.latched and mon.score(True) == 0.15


def test_latched_but_touched_does_not_count():
    mon = M.Monitor(dt=DT)
    run(mon, 1.0, depth=0.095)
    run(mon, 0.2, depth=0.095, lever=0.36, clip=0.7, lever_touched=True)
    run(mon, 5.0, depth=0.095, lever=M.LATCH_HOLD, clip=0.65, lever_touched=True)
    assert mon.latched and not mon.filled


def test_hold_must_be_continuous():
    mon = M.Monitor(dt=DT)
    run(mon, 1.0, depth=0.095)
    run(mon, 0.2, depth=0.095, lever=0.36, clip=0.7, lever_touched=True)
    run(mon, 2.0, depth=0.095, lever=M.LATCH_HOLD)
    run(mon, 0.1, depth=0.06, lever=M.LATCH_HOLD)  # spout backs out under 8 cm
    run(mon, 2.0, depth=0.095, lever=M.LATCH_HOLD)
    assert not mon.filled


def test_lever_open_outside_filler_fails_for_good():
    mon = M.Monitor(dt=DT)
    run(mon, 0.2, depth=0.0, lever=0.2, lever_touched=True)
    assert mon.failed
    full_fill_mon = mon
    run(full_fill_mon, 1.0, depth=0.095)
    assert full_fill_mon.score(True) == 0.0


def test_lever_open_off_axis_fails():
    mon = M.Monitor(dt=DT)
    run(mon, 0.1, depth=0.09, off_axis=0.05, lever=0.2)
    assert mon.failed


def test_withdraw_while_latched_fails():
    mon = M.Monitor(dt=DT)
    full_fill(mon)
    run(mon, 0.1, depth=0.0, lever=M.LATCH_HOLD)
    assert mon.failed and mon.score(True) == 0.0


def test_drop_fails():
    mon = M.Monitor(dt=DT)
    run(mon, 0.1, vz=-1.0)
    assert mon.failed == "nozzle dropped"
    mon2 = M.Monitor(dt=DT)
    run(mon2, 0.1, housing_z=M.TOP + 0.02)
    assert mon2.failed


def test_release_needs_fill_first():
    mon = M.Monitor(dt=DT)
    run(mon, 1.0, depth=0.095)
    run(mon, 1.0, depth=0.095, lever=0.0)
    assert not mon.released and mon.score(True) == 0.15


def test_geometry():
    pos, r = M.nozzle_pose_in_filler(0.09)
    depth, off = M.filler_depth(pos, r)
    assert abs(depth - 0.09) < 1e-9 and off < 1e-9
    bd, bo = M.boot_depth(M.BOOT_N, M.R_BOOT)
    assert bd > M.HOLSTERED and bo < 1e-9
    pts = M.hose_path(M.HOSE_ANCHOR, M.BOOT_N + M.R_BOOT @ M.SWIVEL)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    assert np.all(np.abs(seg - M.HOSE_SEG) < 0.003) and pts[:, 2].min() >= M.TOP
