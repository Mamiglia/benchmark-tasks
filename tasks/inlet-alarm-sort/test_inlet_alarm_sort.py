"""Offline tests (no GPU): the bottle's pour model, every scoring rule, and the demo's timeline replayed kinematically
(the bottle rides rigidly in the gripper) for every start level, with the three cheats.

    PYTHONPATH=<yam-isaac-lab-arena> python -m pytest -q
"""

import math
import zlib
from dataclasses import dataclass, field

import numpy as np
import pytest
import task
import tankkit
from yam_arena import ik
from yam_arena.demo import ArmPlan, Waypoint

UP = np.array([0.0, 0.0, 0.0, 1.0])


@dataclass
class S:  # the fields of ObjectState / PartState the task reads
    pos: np.ndarray
    quat_xyzw: np.ndarray = field(default_factory=lambda: UP.copy())
    low: np.ndarray = None
    hinge: float = 0.0

    def __post_init__(self):
        self.pos = np.asarray(self.pos, float)
        if self.low is None:
            self.low = self.pos - (0, 0, task.B_H / 2)


def tank(lever=0.0, pos=(task.TX, task.TY, task.TOP)):
    return S(pos, hinge=lever)


def at_rest():
    return {n: S((x, y, task.TOP + task.B_H / 2)) for n, (x, y) in zip(task.BOTTLE_NAMES, task.BOTTLE_XY)}


def tilted_quat(deg, axis=(1, 0, 0)):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    h = math.radians(deg) / 2
    return np.r_[a * math.sin(h), math.cos(h)]


@pytest.fixture(autouse=True)
def fresh():
    tankkit.SEED = 0
    task._reset_state()


def objects(lever=0.0, **bottles):
    o = {"tank": tank(lever), **at_rest()}
    o.update(bottles)
    return o


def test_capacity_falls_with_tilt():
    assert abs(task.CAP[0] - math.pi * task.B_R ** 2 * task.B_H) < 2e-6
    assert task.capacity(math.radians(60)) > task.FILL  # carried 60 degrees over, a full bottle does not pour
    assert task.capacity(math.radians(80)) < task.FILL
    assert task.capacity(task.POUR_TILT) <= task.EMPTY  # at the demo's tilt it drains to the empty mark
    assert np.all(np.diff(task.CAP) <= 1e-9)


def test_valve_flow():
    assert task.valve_open(0.0) == 1.0 and task.valve_open(task.CLOSED_AT) == 0.0
    assert 0.4 < task.valve_open(math.radians(45)) < 0.5


def trial_seed(s):
    """inspect-robots' derive_seed(--seed, None, epoch 0): the seed the embodiment's reset receives."""
    return zlib.crc32(f"{s}:None:0".encode()) & 0xFFFFFFFF


RUN_SEEDS = {s: trial_seed(s) for s in (0, 4, 10)}


def test_start_levels_by_seed():
    assert [task.start_level(t) for t in RUN_SEEDS.values()] == [0.060, 0.070, 0.020]


def test_standing_bottles_keep_their_liquid_and_the_inlet_fills_the_tank():
    task.advance(objects(), 10.0)
    assert all(v == task.FILL for v in task._STATE["bottles"].values())
    assert abs(task.level() - (0.020 + 0.010)) < 1e-9
    assert task.score(objects()) == 0.0 + 0.0  # nothing emptied, below the line, open: 0


def pour_over_tank(name, deg=115, steps=200):
    b = S((task.TX, task.TY + 0.02, task.TOP + 0.33), tilted_quat(deg))
    for _ in range(steps):
        task.advance(objects(**{name: b}), 0.05)


def test_pouring_over_the_tank_raises_the_level():
    pour_over_tank("bottle_1")
    assert task._STATE["bottles"]["bottle_1"] <= task.EMPTY
    assert task._STATE["spilled"] == 0.0 and task.emptied() == 1
    poured = task._STATE["poured"]["bottle_1"]
    assert abs(task.level() - (0.020 + 0.010 + poured / task.AREA)) < 1e-9


def test_pouring_beside_the_tank_is_a_spill():
    b = S((task.TX - 0.2, 0.25, task.TOP + 0.33), tilted_quat(115))
    task.advance(objects(bottle_1=b), 0.5)
    assert task._STATE["spilled"] > task.SPILL_LIMIT
    assert task.score(objects(lever=math.pi / 2)) == 0.0


def test_knocked_over_bottle_spills():
    b = S((0.40, 0.18, task.TOP + task.B_R), tilted_quat(90))
    for _ in range(40):
        task.advance(objects(bottle_1=b), 0.05)
    assert task._STATE["spilled"] > task.SPILL_LIMIT and task.score(objects()) == 0.0


def test_a_bottle_dropped_into_the_tank_scores_zero():
    b = S((task.TX, task.TY, task.TOP + 0.12))
    task.advance(objects(bottle_1=b), 0.05)
    assert task._STATE["in_tank"] and task.score(objects(lever=math.pi / 2)) == 0.0


def test_full_score_needs_four_bottles_line_and_shut_valve():
    for n in task.BOTTLE_NAMES:
        pour_over_tank(n, steps=120)
    task._STATE["volume"] = 0.135 * task.AREA
    assert task.score(objects(lever=0.0)) == 0.0  # at the line, inlet still running
    assert task.success(objects(lever=math.radians(88)))
    task._STATE["volume"] = 0.125 * task.AREA  # shut early: short of the line
    assert task.score(objects(lever=math.radians(88))) == pytest.approx(0.6)


def test_overflow_latches():
    task._STATE["volume"] = 0.188 * task.AREA
    task.advance(objects(), 30.0)
    assert task._STATE["overflow"]
    task._STATE["volume"] = 0.15 * task.AREA  # even if it were somehow lower again
    assert task.score(objects(lever=math.pi / 2)) == 0.0


def test_moved_tank_scores_zero():
    o = objects(lever=math.pi / 2)
    o["tank"] = tank(math.pi / 2, pos=(task.TX + 0.03, task.TY, task.TOP))
    task.advance(o, 0.05)
    assert task.score(o) == 0.0


def test_lamp_follows_the_level():
    task._STATE["volume"] = 0.129 * task.AREA
    task.advance(objects(), 1.5)  # +1.5 mm
    assert task._STATE["lamp"] and task._STATE["alarm_t"] is not None


# -- the demo replayed kinematically ------------------------------------------------------------------------------

SIDE_OFFSET = np.array([0.0, task.GRIP_Z - task.B_H / 2, 0.0])  # bottle centre in the TCP frame (y_g points down)
HELD = {"grip", "lift", "over the tank", "down to pour", "tip", "pour", "drain", "upright", "up", "back over the table", "to the bin", "into the bin"}


def plan_of(wps, base):
    return ArmPlan([Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps], base)


def replay(start_seed=0, cheat="", close_s=4.5, end=120.0):
    """The demo's left-arm plan with the bottle carried rigidly from the grip to the set-down, and the valve shut
    ``close_s`` after the lamp has been on for 3 frames (the right arm's turn), as the Controller would."""
    tankkit.SEED = start_seed
    task._reset_state()
    left = plan_of(task.left_plan(), (0.0, 0.305, 0.75))
    dt, t, idx = 0.05, 0.0, 0
    rest = at_rest()
    held, seen, shut_at = None, 0, (0.0 if cheat == "first" else None)
    lead = 6.0 if cheat == "first" else 0.0
    cycle = sum(w[0] for w in task._pour(0, 0))
    while t < end:
        tl = t - lead
        pos, rot, finger, label = left.target(max(0.0, tl))
        idx = min(3, int(max(0.0, tl) // cycle))
        name = task.BOTTLE_NAMES[idx]
        if tl > left.duration:
            held = None
        elif label in HELD:
            held = name
            centre = pos + rot @ SIDE_OFFSET
            r_b = rot @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)  # bottle z = -y_g
            rest[name] = S(centre, np.asarray(tankkit.matrix_to_quat(r_b)))
        elif held == name:
            held = None
        lever = math.pi / 2 if (shut_at is not None and t >= shut_at) else 0.0
        o = {"tank": tank(lever), **rest}
        task.advance(o, dt)
        seen = seen + 1 if task._STATE["lamp"] else 0
        if shut_at is None and ((cheat == "" and seen >= 3) or (cheat == "timed" and t >= 45.0)):
            shut_at = t + close_s
        t += dt
    return task.score(o), task._STATE


@pytest.mark.parametrize("seed", range(8))
def test_demo_reaches_full_score_from_every_start_level(seed):
    s, st = replay(seed)
    assert s == 1.0, (seed, st)
    assert st["alarm_t"] is not None and st["spilled"] == 0.0


def test_cheats_fail():
    seeds = RUN_SEEDS.values()
    ignore = [replay(s, "ignore")[0] for s in seeds]
    first = [replay(s, "first")[0] for s in seeds]
    timed = [replay(s, "timed")[0] for s in seeds]
    assert ignore == [0.0, 0.0, 0.0]
    assert max(first) <= 0.6 + 1e-9 and min(first) < 1.0
    assert min(timed) < 1.0


def test_demo_poses_are_reachable():
    for wps, base in ((task.left_plan(), (0.0, 0.305, 0.75)), (task.right_close(), (0.0, -0.305, 0.75))):
        for d, p, r, f, label in wps:
            q, pe, re = ik.solve(np.asarray(p, float), ik.matrix_to_quat_xyzw(r), np.array([0, 1, 1, -0.6, 0, 0.0]),
                                 base_pos=np.array(base), iters=300)
            assert pe < 0.003 and re < math.radians(2), (label, p, pe, math.degrees(re))
