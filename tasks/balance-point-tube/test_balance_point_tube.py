"""Scoring, mass model, demo vision and demo search for balance-point-tube. No Isaac Sim needed.

    pytest tasks/balance-point-tube/test_balance_point_tube.py     (in the yam_arena environment)
"""

import math
import pathlib
import sys
from types import SimpleNamespace

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import tubedemo  # noqa: E402
import tubekit  # noqa: E402
from tubekit import TubeState, lowest_point, mass_props, slug_for_seed  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

TASK = load(HERE / "task.py")
T = TASK.module
S = T.RIG
FAR = {"left_robot": np.full((3, 3), 5.0), "right_robot": np.full((3, 3), 5.0)}
OPEN = {"left_robot": 1.0, "right_robot": 1.0}


def axis_for(tilt_deg):
    a = math.radians(tilt_deg)
    return np.array([0.0, math.cos(a), math.sin(a)])


def tube(centre=None, tilt=0.0, hook_n=9.9, robot_n=0.0, pads=None, open_frac=None, v=0.0, lifted=True,
         regrounded=False, dropped=False):
    """A tube state; by default resting level in the hook, released."""
    axis = axis_for(tilt)
    if centre is None:
        centre = np.array([T.HOOK_XY[0], T.HOOK_XY[1] - 0.02, T.TOP + T.REST_AXIS])
    centre = np.asarray(centre, float)
    return {"tube_rig": TubeState(
        pos=centre, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.array([v, 0, 0]), ang_vel=np.zeros(3),
        low=centre - 0.5, high=centre + 0.5, axis=axis, tilt_deg=tubekit.tilt_deg(axis),
        lowest=lowest_point(centre, axis, T.L, T.R), hook_n=hook_n, robot_n=robot_n, lifted=lifted,
        regrounded=regrounded, dropped=dropped, pads=FAR if pads is None else pads,
        open_frac=OPEN if open_frac is None else open_frac)}


# -- scoring ---------------------------------------------------------------------------------------------------------


def test_resting_level_released_and_still_succeeds():
    assert T.score(tube()) == 0.75 and T.success(tube())


def test_any_violation_scores_zero_even_if_it_ends_on_the_hook():
    assert T.score(tube(regrounded=True)) == 0.0
    assert T.score(tube(dropped=True)) == 0.0


def test_held_or_touched_on_the_hook_is_partial():
    near = {"left_robot": np.full((3, 3), 5.0),
            "right_robot": np.tile([T.HOOK_XY[0], T.HOOK_XY[1], T.TOP + T.REST_AXIS + T.R + 0.03], (3, 1))}
    assert T.score(tube(pads=near)) == 0.5  # pads 3 cm from the surface
    assert T.score(tube(robot_n=0.5)) == 0.5  # an arm link leaning on it
    assert T.score(tube(open_frac={"left_robot": 1.0, "right_robot": 0.5})) == 0.5
    assert T.score(tube(v=0.05)) == 0.5  # still rocking
    assert not T.success(tube(pads=near))


def test_not_level_or_not_in_the_groove():
    assert T.score(tube(tilt=4.0)) == 0.25
    off = np.array([T.HOOK_XY[0] + 0.03, T.HOOK_XY[1], T.TOP + T.REST_AXIS])
    assert T.score(tube(centre=off)) == 0.25  # resting on the lip, not in the cradle
    beyond = np.array([T.HOOK_XY[0], T.HOOK_XY[1] + 0.49, T.TOP + T.REST_AXIS])
    assert T.score(tube(centre=beyond)) == 0.25  # the hook is not between the plugs
    assert T.score(tube(hook_n=0.0)) == 0.25  # held at the right place, but the hook carries nothing


def test_on_the_table_scores_nothing_and_lifting_alone_a_quarter():
    flat = np.array([*T.TUBE_XY, T.TOP + T.R])
    assert T.score(tube(centre=flat, hook_n=0.0, lifted=False)) == 0.0
    aloft = np.array([0.3, -0.1, T.TOP + 0.15])
    assert T.score(tube(centre=aloft, hook_n=0.0, robot_n=5.0)) == 0.25


def test_the_hook_cannot_be_reached_without_lifting():
    """A level tube in the cradle has its lowest point 8.5 cm up, so it cannot be pushed or rolled there along the
    table; and the kit latches 'lifted' as soon as the hook carries the tube off the table."""
    rest_low = lowest_point(np.array([0, 0, T.TOP + T.REST_AXIS]), axis_for(0.0), T.L, T.R)
    assert rest_low > T.TOP + S.lift_m + 0.02


# -- mass model and seeds -----------------------------------------------------------------------------------------


def test_offsets_vary_with_seed_and_stay_inside_the_tube_and_payload():
    offsets = []
    for seed in range(200):
        y = slug_for_seed(S, seed)
        d, inertia = mass_props(S, y)
        assert abs(y) + S.slug_len / 2 < S.length / 2 - S.plug_len
        assert S.offset_range[0] - 1e-9 <= abs(d) <= S.offset_range[1] + 1e-9
        assert np.all(inertia > 0)
        offsets.append(d)
    assert min(offsets) < -0.1 and max(offsets) > 0.1
    assert S.total_mass < 1.1  # kg; the YAM is rated for about 1.5 kg (i2rt's spec, not in the embodiment code)


def test_the_hook_is_shorter_than_the_balance_tolerance_of_a_middle_grip():
    """The balance point is at least 6 cm from the middle and the cradle reaches 2 cm either side of where the tube
    is hung, so a policy that sets the tube down by its middle (or any fixed point) cannot balance it."""
    assert T.CRADLE_LEN / 2 < S.offset_range[0] / 2


def _aabb(box):
    c, size = np.asarray(box[0]), np.asarray(box[1])
    th = np.radians(box[3]) if len(box) > 3 else 0.0
    ext = np.array([abs(np.cos(th)) * size[0] + abs(np.sin(th)) * size[2], size[1],
                    abs(np.sin(th)) * size[0] + abs(np.cos(th)) * size[2]]) / 2
    return c - ext, c + ext


def test_the_v_faces_touch_the_seated_tube():
    """Each 45-degree face's top surface passes through its contact point with the seated tube, 2.5 cm from the axis."""
    axis = np.array([T.HOOK_XY[0], T.TOP + T.REST_AXIS])
    for box in T.HOOK_BOXES[-2:]:
        th = np.radians(box[3])
        n = np.array([np.sin(th), np.cos(th)])  # the face's normal in (x, z)
        top_centre = np.array([box[0][0], box[0][2]]) + n * box[1][2] / 2
        dist = (axis - top_centre) @ n  # distance from the face's plane to the axis
        assert abs(dist - T.R) < 1e-9


def test_the_hook_frame_clears_the_tube_and_the_way_in():
    """The tube's start, and the tube slid in at the demo's entry height (with 1.5 cm of sag), touch no part of the
    hook frame (bounding boxes, conservative); the way in passes under the overhead arm."""
    def hits(cx, z_axis, y_span):
        for box in T.HOOK_BOXES:
            lo, hi = _aabb(box)
            if (lo[0] < cx + T.R and hi[0] > cx - T.R and lo[1] < y_span[1] and hi[1] > y_span[0]
                    and lo[2] < z_axis + T.R and hi[2] > z_axis - T.R):
                return True
        return False

    y0 = T.TUBE_XY[1]
    span = (y0 - 0.5, y0 + 0.5)
    assert not hits(T.TUBE_XY[0], T.TOP + T.R, span)
    entry_axis = T.TOP + 0.175 - 0.008 - 0.015
    for x in np.linspace(T.HOOK_XY[0] - 0.12, T.HOOK_XY[0], 25):
        assert not hits(x, entry_axis, span), x
    assert entry_axis + 0.015 + T.R < T.TOP + T.ARM_Z  # under the overhead arm, even without sag
    # the demo rolls the tube back (if it crept beyond PUSH_ABOVE_X) before the final grip: the gripper body (8 cm out
    # from the tube's axis), and the pushing hands, then stay clear of the hook's lip
    from tubedemo import Controller
    lip_front = _aabb(T.HOOK_BOXES[-2])[0][0]
    assert Controller.PUSH_ABOVE_X + 0.08 < lip_front
    assert Controller.PUSH_ABOVE_X + 0.055 + 0.03 < lip_front


# -- demo vision ---------------------------------------------------------------------------------------------------


def render(cam, red, blue, w=320, h=176, r_px=3):
    img = np.full((h, w, 3), (205, 180, 140), np.uint8)  # oak
    vv, uu = np.mgrid[0:h, 0:w]
    for p, col in ((red, (200, 25, 25)), (blue, (25, 60, 200))):
        u, v = cam.project(p)
        img[(uu - u) ** 2 + (vv - v) ** 2 <= r_px**2] = col
    return img


def test_camera_projection_round_trips():
    cam = tubedemo.TopCamera(320, 176)
    p = np.array([0.2, 0.4, T.TOP + 0.03])
    uv = cam.project(p)
    assert 0 < uv[0] < 320 and 0 < uv[1] < 176
    assert np.allclose(cam.on_plane(uv, 2, p[2]), p, atol=1e-6)


def test_vision_locates_the_tube_and_reads_its_tilt():
    cam = tubedemo.TopCamera(320, 176)
    x, yc = T.TUBE_XY
    half = T.L / 2 - S.plug_len / 2
    red, blue = np.array([x, yc + half, T.TOP + T.R]), np.array([x, yc - half, T.TOP + T.R])
    seen = tubedemo.tube_on_table(cam, render(cam, red, blue), T.R)
    assert np.linalg.norm(seen[0] - red) < 0.01 and np.linalg.norm(seen[1] - blue) < 0.01
    for tilt in (-2.0, -0.5, 0.5, 2.0):
        a = axis_for(tilt)
        c = np.array([x, yc, T.TOP + 0.06])
        got = tubedemo.tilt_in_plane(cam, render(cam, c + a * half, c - a * half), x)
        assert abs(got - tilt) < 0.35 and np.sign(got) == np.sign(tilt)


# -- demo search (toy world: the camera readings come from a simple pivot model) ------------------------------------


def run_demo(com_from_middle, dt=0.25, x=None):
    """Run the controller against a toy world and return (controller, the grip y it carried the tube from)."""
    x0, yc = T.TUBE_XY
    x = x0 if x is None else x
    ctl = tubedemo.Controller(dict(tubedemo.station.BASES), dt, SimpleNamespace(
        R=T.R, REST_AXIS=T.REST_AXIS, HOOK_XY=T.HOOK_XY))
    half = T.L / 2 - S.plug_len / 2
    ctl._see_table = lambda obs: (np.array([x, yc + half, T.TOP + T.R]), np.array([x, yc - half, T.TOP + T.R]), 0.0, 0.0)

    def see_tilt(obs):
        if not ctl.lifted_now:
            return 0.0
        off = (yc + com_from_middle) - ctl.grip_y  # balance point minus grip (m)
        return float(np.clip(-off * 30.0, -2.3, 2.3))  # pivot: the heavy end goes down, about 0.3 deg per cm

    ctl._see_tilt = see_tilt
    ctl._see_tcp = lambda obs: np.array([[T.HOOK_XY[0], T.HOOK_XY[1] + 0.11, T.TOP + 0.15],
                                         [T.HOOK_XY[0], T.HOOK_XY[1] - 0.11, T.TOP + 0.15]])
    img = np.zeros((176, 320, 3), np.uint8)
    t = 0.0
    while True:
        obs = SimpleNamespace(state_time=t, images={"top_cam": img})
        left, right, done = ctl.act(obs)
        for q in (left, right):
            assert np.all(q[:6] >= tubedemo.ik.LOWER - 1e-6) and np.all(q[:6] <= tubedemo.ik.UPPER + 1e-6)
        if done is not None:
            last = left, right
            for _ in range(3):  # after the end it holds the final pose (grippers open), it does not replay a segment
                t += dt
                held = ctl.act(SimpleNamespace(state_time=t, images={"top_cam": img}))
                assert np.allclose(held[0], last[0]) and np.allclose(held[1], last[1])
            assert left[6] == 1.0 and right[6] == 1.0
            break
        t += dt
        assert t < 120.0, "the demo does not finish within the run limit"
    carried = [m for m in ctl.log if m.startswith("balance point at")]
    return ctl, float(carried[0].split("at ")[1].split(" cm")[0]) / 100, t


@pytest.mark.parametrize("seed, x", [(0, None), (1, None), (4, 0.24)])
def test_the_demo_search_finds_the_balance_point_within_the_hook(seed, x):
    """x 0.24: the tube has crept toward the hook, so the demo rolls it back first (the toy keeps reporting x, so the
    rest of the plan is checked with the tube still there: a stricter reach check)."""
    d, _ = mass_props(S, slug_for_seed(S, seed))
    ctl, found, t = run_demo(d, x=x)
    assert abs(found - d) < T.CRADLE_LEN / 2 - 0.01
    worst = max([ctl.worst_ik_mm] + [p.worst_ik_mm for p in ctl.plans.values()])
    assert worst < 3.0, f"IK misses a target by {worst:.1f} mm"
    assert t / 0.05 < 2400  # control steps at 20 Hz, under the run limit
