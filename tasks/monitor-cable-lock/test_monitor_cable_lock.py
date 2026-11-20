"""Unit tests for the slot and T-bar geometry, the key rules and the score bookkeeping (NumPy only; no Isaac Sim)."""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import lockmodel as M  # noqa: E402

I3 = np.eye(3)
MON0 = np.array([M.BACK_X, M.MON_Y, M.TOP])


def seat(dx=0.0, dy=0.0, dz=0.0, rot=I3, mon_shift=(0.0, 0.0, 0.0)):
    p, _ = M.locked_pose()
    shift = np.asarray(mon_shift, float)
    return M.seat_of(p + [dx, dy, dz] + shift, rot, MON0 + shift, I3, MON0)


def test_start_poses_lie_on_the_desk_in_front_of_the_monitor():
    for seed in range(50):
        p, r = M.start_pose(seed)
        assert abs(p[0] - M.START_XY[0]) <= 0.0201 and abs(p[1] - M.START_XY[1]) <= 0.0201
        assert math.isclose(r[2, 2], 1.0) and p[2] > M.TOP + M.BARREL_D / 2
        yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
        assert -20.0 <= yaw <= 20.0
        assert p[0] + M.BAR_X1 + 0.01 < M.BACK_X - 0.12  # the lock is clear of the stand's base


def test_t_bar_fits_the_slot_unlocked_and_not_turned():
    # unlocked: crossbar and side pins fit the 7 x 3 mm slot with clearance
    assert M.BAR_L < M.SLOT_W and M.BAR_T < M.SLOT_H and 2 * M.PIN_Y1 < M.SLOT_W and M.PIN_Z < M.SLOT_H
    # turned a quarter: the crossbar overlaps the wall by 1.5 mm above and below the slot, so it cannot pass
    assert (M.BAR_L - M.SLOT_H) / 2 >= 0.0015
    # flush, the crossbar is fully behind the wall (free to turn), and the pocket behind has room for it turned
    assert M.BAR_X0 > M.WALL and M.BAR_L < min(M.CAVITY[:2]) and M.BAR_X1 < M.WALL + M.CAVITY[2]


def test_seat_geometry():
    d, off, tilt = seat()
    assert abs(d) < 1e-12 and off < 1e-12 and tilt < 1e-9
    assert M.through(d, off, tilt)
    assert not M.through(*seat(dx=-0.002))  # 2 mm out: the crossbar is in the wall
    assert not M.through(*seat(dy=0.003))  # beside the slot
    assert not M.through(*seat(rot=M.rot_z(12.0)))  # skewed
    # the monitor moved with the lock in it: same seat
    d2, off2, tilt2 = seat(mon_shift=(0.01, -0.02, 0.0))
    assert abs(d2) < 1e-9 and off2 < 1e-9


def test_detent_and_cord():
    assert M.detent_target(0.2) == 0.0 and M.detent_target(1.0) == M.LOCKED_ANGLE
    assert np.allclose(M.cord_force([0.3, 0.0, M.TOP + 0.009]), 0.0)
    f = M.cord_force([0.4, 0.05, M.TOP + 0.13])
    assert math.isclose(np.linalg.norm(f), M.CORD_PULL) and f[2] < 0 and f[1] > 0


def test_monitor_moved():
    p0 = MON0
    assert not M.monitor_moved(p0 + [0.02, 0.0, 0.0], I3, p0)
    assert M.monitor_moved(p0 + [0.025, 0.025, 0.0], I3, p0)
    c, s = math.cos(math.radians(12)), math.sin(math.radians(12))
    assert M.monitor_moved(p0, np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]), p0)
    assert M.monitor_moved(p0 - [0.0, 0.0, 0.2], I3, p0)


def test_scores():
    t = M.Tracker()
    home = seat()
    t.step(*seat(dx=-0.05), moved=False)
    assert t.score(*seat(dx=-0.05), 0.0, False) == 0.0
    t.step(*home, moved=False)
    assert t.score(*home, 0.0, False) == 0.3  # in, not turned
    assert t.score(*home, 90.0, False) == 0.6  # locked, key in
    assert t.score(*home, 90.0, True) == 1.0  # locked, key out, let go
    assert t.score(*home, 90.0, True, clear=False) == 0.8  # ... still held
    assert t.score(*seat(dx=-0.05), 90.0, True) == 0.1  # turned outside the slot after being in
    t.step(*home, moved=True)
    assert t.score(*home, 90.0, True) == 0.0  # monitor shoved: 0 for good
    t.reset()
    assert t.score(*seat(dx=-0.05), 90.0, True) == 0.0  # key turned with the lock in the air: nothing
