"""Offline tests: the cut rule, every scoring outcome on handcrafted states, and the demo's reach. No GPU needed."""

import math

import numpy as np
import pytest
import task
from cordkit import CordState, Cut, Cutter, edge_force, nearer_joint

TOP = task.TOP
LEFT, RIGHT = "left_robot", "right_robot"


def state(cuts=(), touched=None, on_table=False, knife=None, knife_vel=0.0, tag=None, tcp=None):
    knife = np.array(task.KNIFE if knife is None else knife, float)
    tag = np.array(task.TAG0 if tag is None else tag, float)
    tcp = tcp or {LEFT: np.array([0.0, 0.3, TOP + 0.3]), RIGHT: np.array([0.0, -0.3, TOP + 0.3])}
    return {"cord": CordState(pos=tag, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                              low=tag, high=tag, seg_pos=np.zeros((task.N_SEG, 3)), tag_pos=tag, knife_pos=knife,
                              knife_R=np.eye(3), knife_vel=np.r_[knife_vel, 0, 0, 0, 0, 0],
                              blade_pos=knife + task.BLADE_C, tcp=tcp, cuts=tuple(cuts), touched=touched,
                              blade_on_table=on_table)}


def fresh():
    task._reset_state()


def cut(joint=5, hand=0.10, held=0.005):
    return Cut(joint, np.array([task.ANCHOR[0], task.ANCHOR[1], task.CUT_Z]), hand, 3.0, held)


LIFTED = (task.KNIFE[0], task.KNIFE[1], task.KNIFE[2] + 0.08)
HELD = {LEFT: np.array(task.TAG0), RIGHT: np.array(LIFTED) + (0, 0.035, 0.009)}


def test_untouched_scene_scores_zero():
    fresh()
    assert task.score(state()) == 0.0


def test_partial_credit_ladder():
    fresh()
    assert task.score(state(knife=LIFTED)) == 0.25                     # knife lifted, tag not held
    assert task.score(state(knife=LIFTED, tcp=HELD)) == 0.5             # both hands engaged
    assert task.score(state()) == 0.5                                   # latched: knife back down, uncut
    fresh()
    assert task.score(state(knife=LIFTED, tcp=HELD, cuts=[cut()])) == 0.75   # cut, knife still in hand
    away = {LEFT: np.array(task.TAG0), RIGHT: np.array([0.2, -0.2, TOP + 0.15])}
    assert task.score(state(cuts=[cut()], tcp=away)) == 1.0             # cut, knife flat on the table, let go
    assert task.success(state(cuts=[cut()], tcp=away))
    assert task.score(state(cuts=[cut()], tcp=away, knife_vel=0.05)) == 0.75  # still sliding


def test_unsafe_or_destructive_outcomes_score_zero():
    away = {LEFT: np.array(task.TAG0), RIGHT: np.array([0.2, -0.2, TOP + 0.15])}
    for kwargs in [dict(cuts=[cut(hand=0.04)]),                         # cut 4 cm from the holding gripper
                   dict(cuts=[cut(joint=2)]),                           # cut 2 cm from the anchor
                   dict(cuts=[cut(held=0.10)]),                         # other gripper not holding the cord
                   dict(cuts=[cut(), cut(joint=11, hand=0.03)]),        # a second cut near the hand
                   dict(cuts=[cut()], touched="left_robot/tip_left"),   # blade touched a gripper
                   dict(cuts=[cut()], on_table=True),                   # blade touched the table
                   dict(cuts=[cut()], knife=(0.2, -0.5, TOP - 0.5)),    # knife off the table
                   dict(cuts=[cut()], tag=(0.3, 0.6, TOP - 0.4))]:      # tag piece off the table
        fresh()
        assert task.score(state(tcp=away, **kwargs)) == 0.0, kwargs


def test_edge_force_counts_only_pressure_on_the_edge():
    R = np.eye(3)  # knife frame = world at the start: edge faces +x
    assert edge_force([-5.0, 0, 0], R) == pytest.approx(5.0)            # cord pushes back on the edge
    assert edge_force([5.0, 0, 0], R) < 0                               # spine
    assert edge_force([0, 0, 5.0], R) == 0.0 and edge_force([0, 5.0, 0], R) == 0.0  # flat / tip
    yawed = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])              # turned 90 degrees: edge faces +y
    assert edge_force([0, -5.0, 0], yawed) == pytest.approx(5.0)


def test_cutter_needs_a_sustained_press_and_latches_touches():
    c = Cutter(task.RIG)
    press = np.zeros(task.N_SEG)
    press[5] = task.CUT_N + 0.1
    assert all(c.step(press, {}) == [] for _ in range(task.CUT_HOLD - 1))
    assert c.step(press, {}) == [5]                                     # 24 physics steps (0.1 s) in a row
    c.reset()
    spike = press.copy()
    for f in (spike, np.zeros(task.N_SEG), spike, spike):              # a gap resets the count
        assert c.step(f, {}) == []
    weak = np.full(task.N_SEG, task.CUT_N - 0.1)
    assert all(c.step(weak, {}) == [] for _ in range(50))
    c.step(np.zeros(task.N_SEG), {"right_robot/tip_left": 0.01})
    assert c.touched is None
    c.step(np.zeros(task.N_SEG), {"left_robot/tip_right": 0.5})
    assert c.touched == "left_robot/tip_right"


def test_the_cut_joint_is_the_segment_end_nearer_the_edge():
    seg_p, R = np.array([0.3, -0.06, 0.1]), np.eye(3)
    assert nearer_joint(4, seg_p, R, (0.3, -0.07, 0.104), (0, 1, 0), 0.01) == 4    # edge at the top end
    assert nearer_joint(4, seg_p, R, (0.3, -0.07, 0.096), (0, 1, 0), 0.01) == 5    # bottom end


def test_slack_cord_cannot_reach_the_cut_force():
    """The cord starts lying on the table. Pushed sideways, it slides: it resists only with friction (mu = 1) on the
    cord and tag beyond the blade, far under the cut force. Pressed down onto the table it can be cut, as in reality;
    the holding rule scores that 0 (test_unsafe_or_destructive_outcomes_score_zero)."""
    w = (task.N_SEG * task.RIG.seg_mass + task.RIG.tag_mass) * 9.81
    assert task.RIG.friction * w < 0.2 * task.CUT_N


def test_the_cord_starts_on_the_table_and_held_upright_cuts_safely():
    seg = np.array([task.RIG.seg_centre(k) for k in range(task.N_SEG)])
    assert np.allclose(seg[:, 2], task.ANCHOR[2]) and task.ANCHOR[2] - task.RIG.radius > TOP + task.PLATE[2]
    assert task.TAG0[2] - task.TAG[0] / 2 == pytest.approx(TOP)            # tag block lying flat on the table
    assert np.allclose(task.RIG.quat_wxyz, [math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0])  # z -> +x
    above, below = task.TAUT[2] - task.GRIP_DZ - task.CUT_Z, task.CUT_Z - task.ANCHOR[2]
    assert above > task.HAND_M + 0.03                                      # cut 10 cm below the holding pads
    assert below / task.SEG >= task.ANCHOR_JOINTS + 3                     # 9 cm above the knot
    assert task.CUT_Z - task.BLADE[0] > TOP + 0.05                         # knife held well above the table


def test_held_cord_needs_tension_far_above_the_cut_force():
    """Both ends fixed, the edge 6 cm past the cord: the cord would have to stretch 3.5 cm, so the press is limited by
    the arms, not the cord. At the cut force the tension is a few newtons."""
    above, below = task.TAUT[2] - task.GRIP_DZ - task.CUT_Z, task.CUT_Z - task.ANCHOR[2]
    stretch = math.hypot(above, task.PUSH) + math.hypot(below, task.PUSH) - (above + below)
    assert stretch > 0.02
    tension = task.CUT_N / (task.PUSH / math.hypot(above, task.PUSH) + task.PUSH / math.hypot(below, task.PUSH))
    assert tension < 10.0


def test_the_demo_is_reachable():
    from yam_arena import station
    from yam_arena.demo import ArmPlan

    for one_hand in (False, True):
        for arm, wps in task.plan(one_hand).items():
            from yam_arena.demo import Waypoint

            p = ArmPlan([Waypoint(d, tuple(x), f, lab, r) for d, x, r, f, lab in wps], station.BASES[arm])
            q = np.array([p.action7(t)[0] for t in np.arange(0, p.duration, 0.05)])  # worst_ik_* are lazy
            assert p.worst_ik_mm < 2.0 and p.worst_ik_deg < 1.0, (arm, p.worst_ik_mm)
            assert np.all(np.abs(q[:, 5]) < math.radians(120))
            assert p.duration < 1200 / 20


def test_blade_at_rest_is_clear_of_the_table():
    from cordkit import blade_low_z
    z = blade_low_z(np.add(task.KNIFE, task.BLADE_C), np.eye(3), task.BLADE)
    assert z > TOP + 0.001                                              # the knife lying flat does not latch
    tilted = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.0]])              # blade on its long edge
    assert blade_low_z((0, 0, TOP + 0.009), tilted, task.BLADE) < TOP + 0.001
