"""Tests for forearm-on-wedge: scoring outcomes, the line and wedge geometry, and the demo's colour masks and camera
model. Run in the yam_arena environment; no simulator or GPU needed."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task  # noqa: E402
import wedgedemo  # noqa: E402
from wedgekit import (Latches, WardState, body_start, line_start, hand_contact,  # noqa: E402
                      on_cannula_site, signed_angle_x, wedge_height)

S = task.RIG
BED = task.BED


def rot_x(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def state(on_wedge=True, latch=None, robot_n=0.0, pads=None, trapped=False, wedge_xy=None, speed=0.0):
    """A synthetic end state: the forearm lying on the wedge's slope (or flat on the bed)."""
    el = np.array(S.elbow) + (0, 0, BED)
    lf = float(np.linalg.norm(np.subtract(S.wrist, S.elbow)))
    wxy = wedge_xy if wedge_xy is not None else (S.elbow[0], S.elbow[1] - S.wedge_size[1] / 2)
    wp = np.array([*wxy, BED])
    if on_wedge:
        lo, hi = S.wedge_h
        slope = math.degrees(math.atan((hi - lo) / S.wedge_size[1]))
        fr = rot_x(-slope)  # forearm -y axis tilted up
        el = el.copy()
        # the elbow sits over the low end; the underside on the top surface
        el[2] = BED + lo + S.r_forearm / math.cos(math.radians(slope)) + 0.003
        el[1] = wp[1] + S.wedge_size[1] / 2 - 0.005
    else:
        fr = np.eye(3)
    wrist = el + fr @ np.array([0, -lf, 0])
    hr = fr @ rot_x(10)  # hand drooping 10 deg
    line = np.array([[0.22, -0.40 + 0.01 * i, BED + 0.002] for i in range(S.n_seg)])
    if trapped:
        line[5] = [wp[0], wp[1], BED + 0.002]
    far = {a: np.full((3, 3), 5.0) for a in ("left_robot", "right_robot")}
    return {"ward": WardState(pos=(el + wrist) / 2, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                              ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3),
                              bodies={"forearm": (el, fr), "hand": (wrist, hr), "upper": (el, np.eye(3)),
                                      "wedge": (wp, np.eye(3))},
                              elbow=el, wrist=wrist, wrist_deg=signed_angle_x(fr, hr), line=line,
                              robot_arm_n=robot_n, wedge_arm_n=5.0 if on_wedge else 0.0,
                              pads=pads or far, open_frac={}, forearm_speed=speed,
                              latches=latch or Latches())}


def test_success_state():
    st = state()
    assert task.on_wedge(st["ward"]) >= 0.6
    assert st["ward"].wrist[2] - BED >= 0.08
    assert task.score(st) == 0.75 and task.success(st)


@pytest.mark.parametrize("flag", ["site_touched", "hand_pressed", "line_pulled", "wrist_bent", "too_fast", "squeezed"])
def test_any_latch_is_zero(flag):
    lat = Latches()
    setattr(lat, flag, True)
    assert task.score(state(latch=lat)) == 0.0


HUB = np.subtract(S.hub, S.wrist)  # the cannula hub in the hand's frame (at rest)
TOP = S.hand_size[2] - S.wrist[2]  # the back of the hand's top face, hand frame
PALM = -S.wrist[2]  # the palm's underside, hand frame


def test_site_is_the_dressing_and_hub_only():
    for dy in (-0.03, 0.0, 0.03):  # the dressing (6 cm long, centred on the hub), on the back of the hand
        assert on_cannula_site(S, (HUB[0], HUB[1] + dy, TOP))
    assert on_cannula_site(S, HUB)  # the hub itself
    assert not on_cannula_site(S, (HUB[0], HUB[1], PALM))  # palm, under the hub
    assert not on_cannula_site(S, (0.0, -0.15, PALM))  # palm under the fingers
    assert not on_cannula_site(S, (0.0, -0.15, TOP))  # back of the fingers
    assert not on_cannula_site(S, (S.hand_size[0] / 2, -0.06, PALM + 0.01))  # side of the hand, low
    assert hand_contact(S, [HUB, (0.0, -0.15, PALM)], [-1.0, -2.0]) == (1.0, 2.0)  # PhysX's signed normal forces


def contact_latches(contacts, steps=480):
    """The kit's hand check on the same synthetic contacts held for ``steps`` physics steps: [(hand-frame point,
    normal force N)]."""
    site, other = hand_contact(S, [p for p, _ in contacts], [f for _, f in contacts])
    return Latches(site_touched=site > S.touch_n, hand_pressed=other > S.support_n and steps >= 24)


def test_gentle_palm_support_still_succeeds():
    """A gentle hand under the palm, away from the cannula (here 6 N in all, the hand's weight and a little more,
    held for 2 s) latches nothing: the trial still scores 0.75 at the end, 1.0 after the 2 s hold."""
    lat = contact_latches([((0.0, HUB[1], PALM), 3.0), ((0.0, -0.15, PALM), 3.0)])
    assert not lat.any()
    assert task.score(state(latch=lat)) == 0.75


def test_contact_on_dressing_fails():
    lat = contact_latches([((HUB[0], HUB[1] + 0.02, TOP), 1.0)], steps=1)
    assert lat.site_touched and task.score(state(latch=lat)) == 0.0


def test_firm_grip_on_hand_fails():
    lat = contact_latches([((S.hand_size[0] / 2, -0.12, PALM + 0.015), 18.0), ((-S.hand_size[0] / 2, -0.12, PALM + 0.015), 18.0)])
    assert lat.hand_pressed and task.score(state(latch=lat)) == 0.0


def test_trapped_line_caps_at_half():
    st = state(trapped=True)
    assert task.line_trapped(st["ward"])
    assert task.score(st) == 0.5


def test_held_or_moving_is_half():
    st = state(robot_n=3.0)
    assert task.score(st) == 0.5
    near = {a: np.tile(state()["ward"].pos, (3, 1)) for a in ("left_robot", "right_robot")}
    assert task.score(state(pads=near)) == 0.5
    assert task.score(state(speed=0.05)) == 0.5


def test_wedge_under_but_not_resting_is_0_3():
    st = state(on_wedge=False)
    assert task.over_wedge(st["ward"]) >= 0.6
    assert task.score(st) == 0.3


def test_nothing_done_is_zero():
    st = state(on_wedge=False, wedge_xy=S.wedge_pos)
    assert task.score(st) == 0.0


def test_line_chain_is_continuous_and_starts_at_hub():
    segs = line_start(S)
    assert len(segs) == S.n_seg
    hub = np.array(S.hub) + (0, 0, BED)
    c0, r0 = segs[0]
    assert np.allclose(c0 - r0[:, 2] * S.seg_len / 2, hub, atol=1e-9)
    for (c, r), (c2, r2) in zip(segs, segs[1:]):
        assert np.allclose(c + r[:, 2] * S.seg_len / 2, c2 - r2[:, 2] * S.seg_len / 2, atol=1e-9)
    # at the start the loop lies in the wedge's landing zone (beside the forearm, inside its footprint)
    xs = np.array([c[0] for c, _ in segs])
    assert np.any(np.abs(xs - S.elbow[0]) < S.wedge_size[0] / 2)
    assert set(body_start(S)) >= {"upper", "forearm", "hand", "wedge"}


def test_wedge_height():
    assert wedge_height(S, S.wedge_size[1] / 2) == pytest.approx(S.wedge_h[0])
    assert wedge_height(S, -S.wedge_size[1] / 2) == pytest.approx(S.wedge_h[1])


def test_masks_on_rendered_colours():
    """The demo's masks pick each part's colour as the top camera renders it (sampled from a rendered frame) and
    nothing else in the scene."""
    parts = {"line": (212, 108, 97), "clamp": (233, 225, 51), "skin": (228, 203, 191), "wedge": (41, 98, 216)}
    others = {"sheet": (226, 227, 228), "gown": (158, 213, 187), "pyjama": (159, 198, 221), "table": (160, 136, 93),
              "table, lit": (219, 198, 169), "robot": (225, 226, 227), "gripper": (30, 30, 32)}
    for name, c in parts.items():
        m = wedgedemo.masks(np.full((2, 2, 3), c, np.uint8))
        assert m[name].all(), name
        assert not any(m[k].any() for k in m if k != name), name
    for name, c in others.items():
        assert not any(v.any() for v in wedgedemo.masks(np.full((2, 2, 3), c, np.uint8)).values()), name


def test_camera_round_trip():
    cam = wedgedemo.TopCamera(320, 176)
    p = np.array([0.33, -0.05, BED + 0.05])
    u, v = cam.project(p)
    mask = np.zeros((176, 320), bool)
    mask[int(round(v)), int(round(u))] = True
    got = cam.on_z(mask, BED + 0.05)[0]
    assert np.linalg.norm(got - p[:2]) < 0.01
