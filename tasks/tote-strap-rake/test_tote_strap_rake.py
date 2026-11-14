"""Scoring, geometry checks and demo vision for tote-strap-rake. No Isaac Sim needed.

    pytest tasks/tote-strap-rake/test_tote_strap_rake.py     (in the yam_arena environment)
"""

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import totedemo  # noqa: E402
import totekit as K  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

T = load(HERE / "task.py").module
I3 = np.eye(3)
FAR = {"left_robot": np.full((3, 3), 5.0), "right_robot": np.full((3, 3), 5.0)}
OPEN = {"left_robot": 1.0, "right_robot": 1.0}
RAKE0 = K.start_poses()["rake"][0]


def rot_x(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def state(tote=(0.20, -0.06), tote_z=None, R=I3, caught=False, rake=RAKE0, pads=None, open_frac=None, v=0.0,
          rake_v=0.0, **latches):
    """A rig state; by default the tote stands on the table in front of the shelf, released, the ring free."""
    z = K.TOP + K.TOTE_H / 2 if tote_z is None else tote_z
    p = np.array([tote[0], tote[1], z])
    c = K.tote_corners(p, R)
    return {"tote_rig": K.ToteState(
        pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.array([v, 0, 0]), ang_vel=np.zeros(3), low=c.min(0),
        high=c.max(0), tote_R=R, corners=c, ring_pos=np.zeros(3), ring_caught=caught, rake_pos=np.asarray(rake),
        rake_pts=K.rake_world_points(rake, I3), rake_moved=0.0, rake_vel=rake_v,
        pads=FAR if pads is None else pads, open_frac=OPEN if open_frac is None else open_frac, **latches)}


# -- scoring ---------------------------------------------------------------------------------------------------------


def test_freed_down_and_released_succeeds():
    assert T.score(state()) == 0.75 and T.success(state())


def test_strap_still_caught_scores_zero_wherever_the_tote_is():
    assert T.score(state(caught=True)) == 0.0  # a short careful carry that left the rake in place
    assert T.score(state(caught=True, tote=K.TOTE_XY, tote_z=K.DECK_TOP + K.TOTE_H / 2)) == 0.0


def test_any_latched_violation_scores_zero_even_if_the_end_looks_right():
    for flag in ("rake_off", "dragged", "tote_dropped", "tote_floor"):
        assert T.score(state(**{flag: True})) == 0.0, flag


def test_held_or_moving_on_the_table_is_half():
    near = {"left_robot": np.tile([0.20, -0.06 + K.TOTE_L / 2 + K.LEDGE_OUT + 0.02, K.TOP + 0.09], (3, 1)),
            "right_robot": np.full((3, 3), 5.0)}
    assert T.score(state(pads=near)) == 0.5
    assert T.score(state(open_frac={"left_robot": 1.0, "right_robot": 0.5})) == 0.5
    assert T.score(state(v=0.05)) == 0.5
    assert not T.success(state(pads=near))


def test_freed_but_not_on_the_table_is_a_quarter():
    assert T.score(state(tote=K.TOTE_XY, tote_z=K.DECK_TOP + K.TOTE_H / 2)) == 0.25  # still on the shelf
    assert T.score(state(tote_z=K.TOP + K.TOTE_H / 2 + 0.03)) == 0.25  # held 3 cm above the table


def test_table_check_needs_height_footprint_and_upright():
    assert T.score(state(tote_z=K.TOP + K.TOTE_H / 2 + 0.015)) == 0.25  # bottom 1.5 cm above the table
    assert T.score(state(tote=(0.40, -0.06))) == 0.25  # partly under the shelf
    assert T.score(state(tote=(0.20, -0.62))) == 0.25  # hanging off the table's right edge
    assert T.score(state(R=rot_x(8.0), tote_z=K.TOP + K.TOTE_H / 2 + 0.02)) == 0.25  # tilted 8 degrees
    x0 = K.DECK_X[0] - K.TOTE_W / 2 - 0.001
    assert T.score(state(tote=(x0, -0.06))) == 0.75  # just in front of the shelf


def test_rake_must_rest_on_the_deck():
    assert T.score(state(rake=RAKE0 + (0, 0, 0.05))) == 0.0  # held up in the air
    assert T.score(state(rake_v=0.05)) == 0.0  # still moving
    assert T.score(state(rake=RAKE0 + (-0.20, 0, 0))) == 0.0  # its head 2 cm past the front edge


def test_rake_edges_all_four_sides():
    pts = K.rake_world_points(RAKE0, I3)
    assert not K.rake_past_edge(pts)
    lo, hi = pts.min(0), pts.max(0)
    for d in ((K.DECK_X[0] - lo[0] - 0.011, 0), (K.DECK_X[1] - hi[0] + 0.011, 0), (0, K.DECK_Y[0] - lo[1] - 0.011),
              (0, K.DECK_Y[1] - hi[1] + 0.011)):
        assert K.rake_past_edge(pts + (*d, 0)), d
    assert not K.rake_past_edge(pts + (0, K.DECK_Y[1] - hi[1] + 0.009, 0))  # 9 mm over the left edge: still on
    assert K.rake_past_edge(pts - (0, 0, 0.011))  # sunk below the deck top: fallen


# -- geometry ---------------------------------------------------------------------------------------------------------


def test_ring_on_tine_detection():
    poses = K.start_poses()
    ring, rake = poses["ring"][0], poses["rake"][0]
    assert K.ring_on_tine(ring, I3, rake, I3)
    assert not K.ring_on_tine(ring + (0, 0, K.TINE_H + 0.01), I3, rake, I3)  # lifted over the tip
    assert not K.ring_on_tine(ring + (0, 0.03, 0), I3, rake, I3)  # beside the tines
    assert K.ring_on_tine(ring + (0.01, 0, 0.03), rot_x(30.0), rake, I3)  # tilted, still round it
    assert K.ring_on_tine(ring + (0.03, 0, 0), I3, rake, I3)  # slid onto the next tine


def test_strap_fits_between_anchor_and_ring():
    poses = K.start_poses()
    assert K.N_LINKS * K.LINK_L > poses["_gap"] > K.N_LINKS * K.LINK_L - 0.05
    for s in ((0.02, 0.03), (-0.02, -0.03)):  # the seed shift moves everything together
        p = K.start_poses(s)
        assert np.allclose(p["ring"][0] - poses["ring"][0], (*s, 0))


def test_seed_shift_is_bounded_and_reproducible():
    shifts = np.array([K.shift_for_seed(s) for s in range(200)])
    assert (np.abs(shifts[:, 0]) <= K.SHIFT_X).all() and (np.abs(shifts[:, 1]) <= K.SHIFT_Y).all()
    assert np.allclose(K.shift_for_seed(7), K.shift_for_seed(7)) and not np.allclose(K.shift_for_seed(7), 0)


def test_rig_starts_on_the_deck_and_the_tote_inside_it():
    c = K.tote_corners(K.start_poses()["tote"][0], I3)
    assert (c[:, 0] > K.DECK_X[0]).all() and (c[:, 0] < K.DECK_X[1]).all()
    assert abs(c[:4, 2].min() - K.DECK_TOP) < 0.001


# -- demo vision ------------------------------------------------------------------------------------------------------


def test_camera_rays_round_trip():
    cam = totedemo.TopCamera(320, 176)
    for p in ((0.46, -0.06, totedemo.RIM), (0.56, 0.16, totedemo.STRAP_Z)):
        u, v = cam.project(p)
        back = cam.on_height(np.array([u]), np.array([v]), p[2])[0]
        assert np.allclose(back, p, atol=1e-6)


def test_find_rig_reads_a_synthetic_shift():
    """Paint the tote's rim plane and the strap as blobs, shifted, and check the shift comes back."""
    cam = totedemo.TopCamera(320, 176)

    def image(dx, dy):
        img = np.full((176, 320, 3), 200, np.uint8)
        for x in np.linspace(-0.08, 0.08, 30):
            for y in np.linspace(-0.12, 0.12, 40):
                u, v = cam.project((0.46 + dx + x, -0.06 + dy + y, totedemo.RIM))
                img[int(round(v)), int(round(u))] = (30, 80, 180)
        for t in np.linspace(0, 1, 40):
            u, v = cam.project((0.55 + dx + 0.01 * t, 0.04 + dy + 0.12 * t, totedemo.STRAP_Z))
            img[int(round(v)), int(round(u))] = (245, 120, 15)
        return img

    a, b = totedemo.find_rig(cam, image(0, 0)), totedemo.find_rig(cam, image(0.02, -0.03))
    assert np.allclose(b[0] - a[0], (0.02, -0.03), atol=0.004)
    assert np.allclose(b[1] - a[1], (0.02, -0.03), atol=0.006)


def test_grip_frames_are_rotations_with_vertical_finger_axis():
    for p in (0.0, 50.0):
        R = totedemo.grip(p)
        assert np.allclose(R.T @ R, I3, atol=1e-9) and np.isclose(np.linalg.det(R), 1.0)
        assert abs(R[1, 0]) < 1e-9  # the fingers close in the vertical plane through the approach axis
