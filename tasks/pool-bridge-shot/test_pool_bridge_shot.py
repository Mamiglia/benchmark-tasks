"""Unit tests for the shot rules (no Isaac Sim needed): pytest test_pool_bridge_shot.py"""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from poolkit import CueParams, Rules, ShotMonitor, TableGeom, shot_score  # noqa: E402

G = TableGeom(top=0.7475)
R = 0.019
ON = G.felt + R
CB0, OB0 = np.array([0.25, 0.15, ON]), np.array([0.23, 0.325, ON])
IN_TARGET = np.array([G.x0 + 0.03, G.y1 - 0.03, G.felt - 0.04])
IN_OTHER = np.array([G.x1 - 0.03, G.y1 - 0.03, G.felt - 0.04])


def run(events, rules=Rules()):
    """events: list of (steps, kwargs) fed to ShotMonitor.step; kwargs may set cb/ob positions and contacts."""
    m = ShotMonitor(rules, G, R)
    cb, ob = CB0.copy(), OB0.copy()
    for steps, kw in events:
        cb = kw.pop("cb", cb)
        ob = kw.pop("ob", ob)
        args = dict(cue_cb=0.0, cue_ob=0.0, robot_cb=0.0, robot_ob=0.0, cb_ob=0.0)
        args.update(kw)
        for _ in range(steps):
            m.step({"cue_ball": cb, "object_ball": ob}, **args)
    return m


STRIKE = (2, dict(cue_cb=5.0))  # 8 ms contact
QUIET = (24, {})
HIT = (2, dict(cb_ob=3.0))


def test_clean_pot_scores_one():
    m = run([QUIET, STRIKE, QUIET, HIT, QUIET, (1, dict(ob=IN_TARGET)), QUIET])
    assert m.strikes == 1 and m.hit and m.potted == {"object_ball": "near_left"}
    assert shot_score(m, IN_TARGET, True) == 1.0


def test_other_pocket_and_near_miss_score_half():
    m = run([STRIKE, QUIET, HIT, (1, dict(ob=IN_OTHER))])
    assert shot_score(m, IN_OTHER, True) == 0.5
    near = np.r_[G.pocket_mouth("near_left") + np.array([0.06, -0.06]), ON]  # 8.5 cm from the corner
    m = run([STRIKE, QUIET, HIT, (5, dict(ob=near))])
    assert shot_score(m, near, True) == 0.5
    assert shot_score(m, near, False) == 0.25  # still rolling: not counted yet
    far = np.r_[G.pocket_mouth("near_left") + np.array([0.10, -0.10]), ON]
    assert shot_score(run([STRIKE, QUIET, HIT, (5, dict(ob=far))]), far, True) == 0.25


def test_scratch_caps_at_quarter():
    m = run([STRIKE, QUIET, HIT, (1, dict(ob=IN_TARGET)), (1, dict(cb=IN_OTHER))])
    assert m.potted == {"object_ball": "near_left", "cue_ball": "far_left"}
    assert shot_score(m, IN_TARGET, True) == 0.25


def test_no_hit_or_no_strike_is_zero():
    assert shot_score(run([STRIKE, QUIET]), OB0, True) == 0.0  # missed the object ball
    assert shot_score(run([QUIET, HIT, (1, dict(ob=IN_TARGET))]), IN_TARGET, True) == 0.0  # never struck


def test_double_strike_push_and_fouls_are_zero():
    two = run([STRIKE, (12, {}), STRIKE, HIT, (1, dict(ob=IN_TARGET))])  # 50 ms apart: a second strike
    assert two.strikes == 2 and shot_score(two, IN_TARGET, True) == 0.0
    one = run([STRIKE, (3, {}), STRIKE, HIT, (1, dict(ob=IN_TARGET))])  # 12 ms gap: one contact (bounce)
    assert one.strikes == 1 and shot_score(one, IN_TARGET, True) == 1.0
    push = run([(20, dict(cue_cb=2.0)), HIT, (1, dict(ob=IN_TARGET))])  # 83 ms in contact
    assert push.push and shot_score(push, IN_TARGET, True) == 0.0
    for foul in (dict(robot_ob=1.0), dict(robot_cb=1.0), dict(cue_ob=1.0)):
        m = run([STRIKE, QUIET, (1, foul), HIT, (1, dict(ob=IN_TARGET))])
        assert m.foul and shot_score(m, IN_TARGET, True) == 0.0


def test_off_table_object_ball_and_pocket_holes():
    off = np.array([0.0, 0.0, G.top - 0.3])
    m = run([STRIKE, QUIET, HIT, (1, dict(ob=off))])
    assert "object_ball" in m.off_table and shot_score(m, off, True) == 0.25
    holes = G.pockets()
    assert len(holes) == 6 and all(x0 < x1 and y0 < y1 for x0, x1, y0, y1 in holes.values())
    assert holes["near_left"][3] == G.y1 and holes["near_left"][0] == G.x0
    assert m.where(np.array([0.3, 0.0, ON])) == ""  # on the cloth


def test_geometry_and_cue():
    boxes = G.boxes()
    felt_top = [c[2] + s[2] / 2 for m, c, s in boxes if m == "felt"]
    assert np.allclose(felt_top, G.felt)
    assert all(s[i] > 0 for _, _, s in boxes for i in range(3))
    c = CueParams()
    assert 0.55 < c.com_from_tip < 0.65 and 2 * c.butt_r < 0.094  # the butt fits the 94 mm gripper


if __name__ == "__main__":
    import pytest

    sys.exit(pytest.main([__file__, "-q"]))
