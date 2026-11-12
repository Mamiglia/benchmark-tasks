"""Offline tests: the buckling rule, every scoring rule on handcrafted trajectories, the seeds, and the demo's reach.
No GPU needed."""

import math

import grip_demo
import numpy as np
import task
from cupkit import N_PANELS, CupMonitor, CupState, check_pairs, side_forces

P, R, TOP = task.PARAMS, task.RULES, task.TOP
LEFT, RIGHT = "left_robot", "right_robot"
UP = np.array([0.0, 0.0, 0.0, 1.0])
FAR = {LEFT: np.array([0.0, 0.3, TOP + 0.3]), RIGHT: np.array([0.0, -0.3, TOP + 0.3])}


def squeeze(mm):
    """Flat pads pressing the wall in by ``mm`` at panels 2 and 6 (facing +y and -y): their neighbours give by
    mm / sin45 along their own axes. The force on each side is 5 k mm."""
    d = np.zeros(N_PANELS)
    d[2] = d[6] = mm / 1000
    d[[1, 3, 5, 7]] = mm / 1000 / math.sin(math.pi / 4)
    return d


def test_side_force_of_a_flat_pad():
    f = side_forces(squeeze(2.0), P)
    assert abs(f[2] - 5 * P.wall_k * 0.002) < 1e-9 and f[2] == f.max() and abs(f[6] - f[2]) < 1e-9


def mon(fill=0.2):
    return CupMonitor(fill, P, R)


def held_at(m, xy, base_z, mm=3.0, arm=LEFT, grip_dz=0.03):
    pos = np.array([xy[0], xy[1], base_z])
    tcps = dict(FAR, **{arm: pos + (0, 0, grip_dz)})
    m.control(pos, UP, squeeze(mm), tcps)


def free_at(m, xy, base_z, quat=UP):
    m.control(np.array([xy[0], xy[1], base_z]), quat, np.zeros(N_PANELS), FAR)


def state_of(m, xy, base_z, quat=UP, tcps=FAR):
    pos = np.array([xy[0], xy[1], base_z])
    return CupState(pos=pos, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos,
                    collapsed=m.collapsed, touched=m.touched, dropped=m.dropped, slipped=m.slipped, tipped=m.tipped,
                    fell=m.fell, doubled=m.doubled, lifted=m.lifted, placed=m.placed, holder=m.holder,
                    on_tray=m.on_tray(pos))


def carry(m, start, slot, mm=3.0, steps=10, lift=0.045):
    """Grip at the start, lift, carry to the slot, lower onto the tray, let go."""
    for k in range(steps):
        held_at(m, start, TOP + lift * k / steps, mm)
    for k in range(steps + 1):
        s = k / steps
        held_at(m, (start[0] + s * (slot[0] - start[0]), start[1] + s * (slot[1] - start[1])), TOP + lift, mm)
    held_at(m, slot, R.tray_z + 0.004, mm)
    free_at(m, slot, R.tray_z)


def test_wall_buckles_only_past_its_force_for_three_steps():
    m = mon(0.045)
    limit_mm = P.crush_n(0.045) / (5 * P.wall_k) * 1000  # 2.4 N at 1,000 N/m: 2.4 mm
    for _ in range(50):
        m.physics(squeeze(limit_mm - 0.1), 0.0)
    assert not m.collapsed
    m.physics(squeeze(limit_mm + 0.5), 0.0)
    m.physics(squeeze(limit_mm + 0.5), 0.0)
    m.physics(squeeze(0.0), 0.0)
    assert not m.collapsed
    for _ in range(3):
        buckled = m.physics(squeeze(limit_mm + 0.5), 0.0)
    assert m.collapsed and buckled[[1, 2, 3, 5, 6, 7]].all() and not buckled[0] and not buckled[4]
    m.physics(squeeze(0.0), 0.0)
    assert m.collapsed  # latched


def test_full_gripper_force_buckles_every_cup():
    assert 5 * P.wall_k * P.travel_m > P.crush_n(task.FILL_BANDS[-1][1])  # the wall buckles before its hard stop
    for fill in (0.02, 0.2, 0.4):
        m = mon(fill)
        force_mm = 50.0 / (5 * P.wall_k) * 1000  # the gripper's 50 N on the wall
        for _ in range(3):
            m.physics(squeeze(min(force_mm, P.travel_m * 1000)), 0.0)
        assert m.collapsed, fill


def test_no_single_grip_force_lifts_every_cup_of_any_seed():
    mu = P.grip_mu
    for seed in range(200):
        fills = task.fills_for(seed)
        assert len(set(fills)) == 4 and all(0.02 <= f <= 0.4 for f in fills)
        need = max(9.81 * (f + P.body_mass + N_PANELS * P.panel_mass) / (2 * mu) for f in fills)
        crush = min(P.crush_n(f) for f in fills)
        assert need > crush + 0.5, (seed, need, crush)
        # and every single cup can be lifted intact by the demo: its carry grip is at most 1.2 x a grip that is at
        # most 1.4 x 1.15 times the lift-off grip (the load-plateau rule), which stays under the buckling force
        for f in fills:
            assert P.crush_n(f) > max(1.2 * 1.4 * 1.15 * 9.81 * (f + 0.026) / (2 * mu), 0.8 * 1.4), (seed, f)


def test_seed_zero_is_the_default_and_reproducible():
    assert task.SEED == 0 and task.FILLS == task.fills_for(0) == task.fills_for(0)
    assert task.fills_for(1) != task.fills_for(0)


def test_touching_the_base_or_seal_fails():
    m = mon()
    m.physics(np.zeros(N_PANELS), 1.0)
    m.physics(np.zeros(N_PANELS), 1.0)
    assert not m.touched
    m.physics(np.zeros(N_PANELS), 1.0)
    assert m.touched and m.failed


def test_lifted_carried_and_placed_scores():
    m = mon()
    start, slot = task.CUP_XY[0], task.SLOTS["cup_lf"]
    carry(m, start, slot)
    assert m.lifted and m.placed and not m.failed
    assert task.cup_ok(state_of(m, slot, R.tray_z))


def test_still_held_at_the_end_does_not_count():
    m = mon()
    start, slot = task.CUP_XY[0], task.SLOTS["cup_lf"]
    carry(m, start, slot)
    held_at(m, slot, R.tray_z)
    tcps = dict(FAR, **{LEFT: np.array([slot[0], slot[1], R.tray_z + 0.03])})
    assert not task.cup_ok(state_of(m, slot, R.tray_z, tcps=tcps))


def test_pushed_onto_the_tray_never_counts():
    m = mon()
    start, slot = task.CUP_XY[0], task.SLOTS["cup_lf"]
    for k in range(21):  # slid along the table, then over the rim onto the floor
        s = k / 20
        z = TOP if s < 0.6 else R.tray_z
        free_at(m, (start[0] + s * (slot[0] - start[0]), start[1] + s * (slot[1] - start[1])), z)
    assert not m.lifted and not task.cup_ok(state_of(m, slot, R.tray_z))


def test_set_down_on_the_table_then_pushed_onto_the_tray_does_not_count():
    m = mon()
    start, slot = task.CUP_XY[0], task.SLOTS["cup_lf"]
    for k in range(10):
        held_at(m, start, TOP + 0.03 * k / 9)
    held_at(m, start, TOP)
    free_at(m, start, TOP)
    assert m.lifted and not m.placed
    free_at(m, slot, R.tray_z)  # pushed there
    assert not task.cup_ok(state_of(m, slot, R.tray_z))


def test_off_the_support_without_a_wall_grip_fails():
    """A drop of more than 1 cm, a scoop from underneath, a carry on the closed fingers or the seal: the cup is off
    its support and no wall grip holds it."""
    m = mon()
    free_at(m, task.CUP_XY[0], TOP + 0.008)
    assert not m.dropped
    free_at(m, task.CUP_XY[0], TOP + 0.012)
    assert m.dropped and m.failed
    m = mon()
    pos = np.array([*task.CUP_XY[0], TOP + 0.03])
    m.control(pos, UP, squeeze(0.2), dict(FAR, left_robot=pos + (0, 0, 0.03)))  # light touch, not a grip
    assert m.dropped


def test_a_grip_on_the_lid_level_or_below_the_wall_is_not_a_wall_grip():
    m = mon()
    pos = np.array([*task.CUP_XY[0], TOP + 0.03])
    m.control(pos, UP, squeeze(3), dict(FAR, left_robot=pos + (0, 0, P.height - 0.005)))
    assert m.holder is None and m.dropped


def test_slip_of_more_than_a_centimetre_in_the_grip_fails():
    m = mon()
    xy = task.CUP_XY[0]
    held_at(m, xy, TOP)
    for k in range(10):  # hand rises 1.5 cm, the cup stays on the table: the pads slide up the wall
        held_at(m, xy, TOP, grip_dz=0.03 + 0.015 * k / 9)
    assert m.slipped and m.failed
    m = mon()
    for k in range(10):
        held_at(m, xy, TOP, grip_dz=0.03 + 0.008 * k / 9)
    assert not m.slipped


def test_tipped_or_off_the_table_fails():
    m = mon()
    q = [math.sin(math.radians(20)), 0.0, 0.0, math.cos(math.radians(20))]  # 40 degrees
    free_at(m, task.CUP_XY[0], TOP, quat=np.array(q))
    assert m.tipped
    m = mon()
    free_at(m, (0.6, 0.0), TOP - 0.06)
    assert m.fell


def test_two_cups_in_one_gripper_fail_both():
    a, b = mon(), mon()
    for m, xy in ((a, (0.30, 0.20)), (b, (0.30, 0.27))):
        pos = np.array([*xy, TOP + 0.03])
        m.control(pos, UP, squeeze(3), dict(FAR, left_robot=np.array([0.30, 0.235, TOP + 0.06])))
    check_pairs([a, b])
    assert a.doubled and b.doubled


def test_score_counts_cups():
    mons = {n: mon() for n in task.CUP_NAMES}
    states = {}
    for i, n in enumerate(task.CUP_NAMES):
        if i < 3:
            carry(mons[n], task.CUP_XY[i], task.SLOTS[n])
            states[n] = state_of(mons[n], task.SLOTS[n], R.tray_z)
        else:
            states[n] = state_of(mons[n], task.CUP_XY[i], TOP)
    assert task.score(states) == 0.75 and not task.success(states)
    carry(mons["cup_rn"], task.CUP_XY[3], task.SLOTS["cup_rn"])
    states["cup_rn"] = state_of(mons["cup_rn"], task.SLOTS["cup_rn"], R.tray_z)
    assert task.score(states) == 1.0 and task.success(states)


def test_slots_fit_inside_the_tray():
    for x, y in task.SLOTS.values():
        assert mon().on_tray(np.array([x, y, R.tray_z]))


class _Obs:
    def __init__(self, state):
        self.state = {"joint_pos": state}
        self.images = {}


def test_demo_finishes_within_the_run_limit_and_reaches_every_target():
    """Dry run of the controller against an ideal arm and a gripper that meets a 7 cm cup (wall 600 N/m per side):
    every command is reached by IK within 1 mm and both arms finish within 1,200 steps."""
    from yam_arena import ik, station

    ctl = task.demo_controller(dict(station.BASES), 0.05)
    k_s = 5000 * 5 * P.wall_k / (5000 + 5 * P.wall_k)
    worst = 0.0
    state = np.r_[ctl.arms["left"].action7(), ctl.arms["right"].action7()]
    for step in range(1200):
        meas = state.copy()
        for i in (6, 13):  # the fingers stop at the cup and lag by force / drive stiffness
            opening = state[i] * grip_demo.OPENING
            force = k_s * max(0.0, (0.070 - opening) / 2)
            meas[i] = state[i] + force / grip_demo.K_GRIP
        action, done = ctl.act(_Obs(meas))
        state = np.asarray(action, float)
        for side, arm in ctl.arms.items():
            worst = max(worst, float(np.linalg.norm(ik.tcp_pose(arm.q, arm.base)[0] - arm.pos)))
        if done:
            break
    assert done, "demo did not finish"
    assert worst < 0.001, worst
