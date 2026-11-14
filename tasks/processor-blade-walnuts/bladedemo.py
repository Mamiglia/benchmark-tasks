"""Closed-loop scripted demo for processor-blade-walnuts, played by ``yam_scripted`` (see ``bladekit._patch_scripted``).

It sees only what the robot sees: the top camera's RGB image and the arms' measured joint positions. It never reads
the simulator (no object poses).

1. Both arms turn aside, out of the top camera's view. The camera finds the drive adapter's black stem, the blue
   mixing bowl and the green cutting board by colour; with the camera's calibration each blob's centroid is a ray,
   cut at the height where that object stands (the stem's top, the mixing bowl's mid-height, the board's top).
   The stem is on the bowl's centre post, so it also locates the work bowl and its handle.
2. The left arm grips the stem (approach tilted 20 degrees about the finger axis, so the arm reaches high enough)
   and pulls it straight up along the post's axis, 11.5 cm, slowly, then carries the blade to the board and stands
   it on its hub there. It checks with forward kinematics of its measured joints that the hand really rose.
3. The right arm takes the bowl's handle (horizontal approach along the bowl's side, fingers either side of the
   upright grip bar, one of them in the opening between bar and wall), lifts the bowl high over the mixing bowl and
   turns its wrist 120 degrees: the bowl swings down under the handle and empties. It shakes, turns back and sets
   the bowl down where it was.

``DEMO_CHEAT`` plays contrasts for the README: ``tip`` (pour with the blade still in), ``shallow`` (blade still in,
tip only to 105 degrees and shake, hoping the nuts leave before the blade), ``angled`` (pull the stem 20 degrees off
the post's axis).
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation
from yam_arena.urdf import axis_angle_matrix

from bladekit import rot_x

ARMS = ("left", "right")
TOP = station.TABLE_TOP
# The black blob is the stem plus the adapter's cap and sleeve seen inside the bowl: its median pixel lies on the
# axis about 7 cm above the table (measured once on the station camera; the stem's top is at 10.5 cm).
STEM_BLOB_Z = 0.07


# -- vision ----------------------------------------------------------------------------------------------------------


def masks(img: np.ndarray) -> dict[str, np.ndarray]:
    rgb = img.astype(np.float32)
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    return {"bowl": (b > 150) & (b > r + 22) & (b < 1.45 * r),
            "stem": (r < 45) & (g < 45) & (b < 50),
            "mix": (b > 90) & (b > 1.5 * r) & (b > 1.2 * g),
            "board": (g > 80) & (g > 1.35 * r) & (g > 1.25 * b)}


@dataclass
class TopCamera:
    """The station's calibrated top camera (OpenGL convention: looks along -z, +y up), for a resized image."""

    width: int
    height: int
    cam: object = field(default_factory=lambda: station.top_camera("calibrated"))

    def ray(self, u: float, v: float) -> tuple[np.ndarray, np.ndarray]:
        from yam_arena.urdf import quat_xyzw_to_matrix

        lens = self.cam.lens
        su, sv = lens.width / self.width, lens.height / self.height
        uf, vf = (u + 0.5) * su - 0.5, (v + 0.5) * sv - 0.5
        d = quat_xyzw_to_matrix(np.asarray(self.cam.rot_xyzw)) @ np.array(
            [(uf - lens.cx) / lens.fx, -(vf - lens.cy) / lens.fy, -1.0])
        return np.asarray(self.cam.pos, float), d / np.linalg.norm(d)

    def project(self, p) -> tuple[float, float]:
        from yam_arena.urdf import quat_xyzw_to_matrix

        lens = self.cam.lens
        c = quat_xyzw_to_matrix(np.asarray(self.cam.rot_xyzw)).T @ (np.asarray(p, float) - self.cam.pos)
        uf, vf = lens.cx + lens.fx * c[0] / -c[2], lens.cy - lens.fy * c[1] / -c[2]
        su, sv = lens.width / self.width, lens.height / self.height
        return (uf + 0.5) / su - 0.5, (vf + 0.5) / sv - 0.5

    def at_height(self, uv, z: float) -> np.ndarray:
        o, d = self.ray(*uv)
        return o + d * (z - o[2]) / d[2]

    def table_xy(self, shape) -> tuple[np.ndarray, np.ndarray]:
        """World x, y where each pixel's ray meets the table top (for a region of interest)."""
        h, w = shape
        v, u = np.mgrid[0:h, 0:w]
        from yam_arena.urdf import quat_xyzw_to_matrix

        lens = self.cam.lens
        su, sv = lens.width / w, lens.height / h
        uf, vf = (u + 0.5) * su - 0.5, (v + 0.5) * sv - 0.5
        d = np.stack([(uf - lens.cx) / lens.fx, -(vf - lens.cy) / lens.fy, -np.ones_like(uf, dtype=float)], -1)
        d = d @ quat_xyzw_to_matrix(np.asarray(self.cam.rot_xyzw)).T
        o = np.asarray(self.cam.pos, float)
        t = (TOP - o[2]) / np.minimum(d[..., 2], -1e-6)
        return o[0] + t * d[..., 0], o[1] + t * d[..., 1]


def find(cam: TopCamera, img: np.ndarray, heights: dict, min_px: int = 4) -> dict:
    """World positions of the stem, the mixing bowl and the board, from colour blobs in the work area. The stem is
    the black blob inside the clear work bowl (within 18 px of the bowl's pixels' median), so the parked arms' black
    links elsewhere in the image do not count."""
    x, y = cam.table_xy(img.shape[:2])
    area = (x > 0.12) & (x < 0.70) & (y > -0.40) & (y < 0.50)
    ms = masks(img)
    v, u = np.nonzero(ms.pop("bowl") & area)
    if len(u) >= min_px:
        vv, uu = np.mgrid[0:img.shape[0], 0:img.shape[1]]
        area_stem = (uu - np.median(u)) ** 2 + (vv - np.median(v)) ** 2 < 18**2
    else:
        area_stem = np.zeros(img.shape[:2], bool)
    out = {}
    for name, m in ms.items():
        v, u = np.nonzero(m & (area_stem if name == "stem" else area))
        out[name] = (cam.at_height((float(np.median(u)), float(np.median(v))), heights[name]) if len(u) >= min_px
                     else None)
    return out


# -- motion ----------------------------------------------------------------------------------------------------------


def frame(z, x) -> np.ndarray:
    """Gripper rotation with approach axis ``z`` and finger-closing axis ``x`` (world)."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.asarray(x, float) - z * np.dot(x, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


class JointMove:
    """A smooth joint-space move to ``q`` (same interface as ``ArmPlan``)."""

    def __init__(self, duration: float, q, q_start, finger):
        self.duration, self.q_goal, self.q_start = duration, np.asarray(q, float), np.asarray(q_start, float)
        self.q = self.q_start.copy()
        self.waypoints = [Waypoint(duration, (0.0, 0.0, 0.0), finger)]
        self.worst_ik_mm = 0.0

    def action7(self, t: float):
        self.q = self.q_start + ik.smoothstep(min(t / self.duration, 1.0)) * (self.q_goal - self.q_start)
        return np.r_[self.q, self.waypoints[0].finger / FINGER_OPEN], "joint move"


# Parked: the ready pose turned 110 degrees outward at the base, out of the top camera's view.
PARK_Q = {"left": np.array([1.9, *READY[1:]]), "right": np.array([-1.9, *READY[1:]])}


@dataclass
class Segment:
    """Waypoints per arm (an arm left out holds its last target), played together; then ``collect`` frames."""

    moves: dict  # arm -> list of Waypoint; a float: park the arm in that many s; ("ready", s): back to READY
    collect: object = None
    min_s: float = 0.0


class Controller:
    """Plays :meth:`script` segment by segment; returns (left 7-vector, right 7-vector, finished time or None)."""

    STEM_TILT = math.radians(20)   # left grip: straight down, turned 20 degrees about the finger axis
    PULL = 0.115                   # straight up along the post: 8.05 cm clears the post, 11.5 cm the bowl's rim
    PULL_S = 4.0
    LIFT_Z = 0.355                 # handle grip height while pouring: the swinging bowl clears the mixing bowl's rim
    POUR_DY = 0.04                 # the lip ends 8.3 cm to -y of the handle, so the handle is 4 cm +y of the target
    POUR_DEG = 120.0

    def __init__(self, bases, dt, task):
        self.task, self.dt = task, dt
        self.bases = {a: np.asarray(bases[a], float) for a in ARMS}
        self.q = {a: np.array(READY, float) for a in ARMS}
        self.finger = dict.fromkeys(ARMS, FINGER_OPEN)
        self.plans: dict = {}
        self.t0 = self.dur = 0.0
        self.collect, self.samples = None, []
        self.finished_at = None
        self.cam = None
        self.cheat = os.environ.get("DEMO_CHEAT", "")
        self.script_it = self.script()

    def say(self, msg: str) -> None:
        print(f"[bladedemo] {msg}", flush=True)

    def _start(self, seg: Segment) -> None:
        for arm, plan in self.plans.items():
            self.q[arm], self.finger[arm] = plan.q.copy(), plan.waypoints[-1].finger
            if getattr(plan, "worst_ik_mm", 0) > 5:
                self.say(f"{arm}: IK off by {plan.worst_ik_mm:.0f} mm in the last segment")
        self.plans = {}
        for a, w in seg.moves.items():
            if isinstance(w, float):
                self.plans[a] = JointMove(w, PARK_Q[a], self.q[a], self.finger[a])
            elif isinstance(w, tuple):
                self.plans[a] = JointMove(w[1], READY, self.q[a], self.finger[a])
            else:
                self.plans[a] = ArmPlan(list(w), self.bases[a], q_start=self.q[a], finger_start=self.finger[a])
        self.dur = max([p.duration for p in self.plans.values()] + [seg.min_s])
        self.collect, self.samples = seg.collect, []

    def act(self, obs):
        t = float(obs.state_time)
        if self.cam is None:
            h, w = obs.images["top_cam"].shape[:2]
            self.cam = TopCamera(w, h)
        if self.collect is not None:
            self.samples.append(self.collect(obs))
        while self.finished_at is None and t + self.dt >= self.t0 + self.dur:
            self.t0 += self.dur
            try:
                self._start(next(self.script_it))
            except StopIteration:
                self.t0 -= self.dur
                self.finished_at = t
                self.collect = None
        out = []
        for arm in ARMS:
            if arm in self.plans:
                a7, _ = self.plans[arm].action7(min(t + self.dt - self.t0, self.plans[arm].duration))
            else:
                a7 = np.r_[self.q[arm], self.finger[arm] / FINGER_OPEN]
            out.append(a7)
        return out[0], out[1], self.finished_at

    def _tcp(self, obs, arm: str) -> np.ndarray:
        joints = np.asarray(next(iter(obs.state.values())), float).reshape(-1)
        q = joints[:6] if arm == "left" else joints[7:13]
        return ik.tcp_pose(q, self.bases[arm])[0]

    # -- the script ----------------------------------------------------------------------------------------------

    def _locate(self):
        T = self.task
        heights = {"stem": TOP + STEM_BLOB_Z, "mix": TOP + T.MIX_H / 2, "board": TOP + T.BOARD_SIZE[2]}
        yield Segment({}, collect=lambda o: find(self.cam, o.images["top_cam"], heights), min_s=0.5)
        got = {}
        for name in heights:
            seen = [s[name] for s in self.samples if s[name] is not None]
            if not seen:
                raise RuntimeError(f"the top camera does not see the {name}")
            got[name] = np.median(seen, axis=0)
            self.say(f"{name} at x {got[name][0]:.3f} y {got[name][1]:+.3f}")
        return got

    def script(self):
        yield Segment(dict.fromkeys(ARMS, 2.0))  # turn both arms aside
        seen = yield from self._locate()
        stem, mix, board = seen["stem"], seen["mix"], seen["board"]
        if self.cheat == "shallow":  # contrast: leave the blade in, tip only to 105 degrees and shake
            self.POUR_DEG = 105.0
        if self.cheat not in ("tip", "shallow"):
            yield from self._lift_blade(stem[:2], board[:2])
        yield from self._pour(stem[:2], mix[:2])
        yield Segment(dict.fromkeys(ARMS, 2.0))

    def _lift_blade(self, c, board):
        T = self.task
        R = axis_angle_matrix(np.array([0.0, -1.0, 0.0]), self.STEM_TILT) @ grip_rotation(0.0)
        grip = np.array([c[0], c[1], TOP + T.STEM_TOP - 0.010])
        up = np.array([0.0, 0.0, 1.0])
        if self.cheat == "angled":  # contrast: pull 20 degrees off the post's axis (toward the board)
            up = np.array([0.0, math.sin(math.radians(20)), math.cos(math.radians(20))])
        top = grip + self.PULL * up
        place = np.array([board[0] + 0.02, board[1] - 0.03, TOP + T.BOARD_SIZE[2] + 0.004 + (T.STEM_TOP - 0.010)
                          - T.HUB_Z0])
        yield Segment({"left": ("ready", 1.5)})
        yield Segment({"left": [Waypoint(1.5, tuple(grip + (0, 0, 0.07)), FINGER_OPEN, "over the stem", R),
                                Waypoint(1.5, tuple(grip), FINGER_OPEN, "down to the stem", R),
                                Waypoint(0.8, tuple(grip), FINGER_CLOSED, "grip the stem", R),
                                Waypoint(0.4, tuple(grip), FINGER_CLOSED, "settle", R),
                                Waypoint(self.PULL_S, tuple(top), FINGER_CLOSED, "pull straight up the post", R)]})
        yield Segment({}, collect=lambda o: self._tcp(o, "left"), min_s=0.3)
        rose = float(np.median([s[2] for s in self.samples])) - grip[2]
        self.say(f"the hand rose {rose * 100:.1f} cm along the post")
        yield Segment({"left": [Waypoint(2.0, (place[0], place[1], top[2]), FINGER_CLOSED, "carry to the board", R),
                                Waypoint(1.5, tuple(place), FINGER_CLOSED, "stand it on the board", R),
                                Waypoint(0.6, tuple(place), FINGER_OPEN, "let go", R),
                                Waypoint(1.0, (place[0], place[1], place[2] + 0.08), FINGER_OPEN, "up", R)]})
        yield Segment({"left": 2.0})

    def _pour(self, c, mix):
        """Right hand: approach the handle's grip bar horizontally along +x, fingers either side of it (one in the
        opening between bar and wall), grip, lift and carry the bowl so the handle is 3.5 cm above where the lip will
        be over the mixing bowl, turn the wrist 120 degrees (the bowl swings down under the handle and empties toward
        +y), shake, turn back, set the bowl down where it was."""
        T = self.task
        R0 = frame((1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
        half = -0.025  # finger target: pads 5 cm apart, so the inner finger clears the wall
        g0 = np.array([c[0], c[1] - T.HANDLE_BAR_R, TOP + T.HANDLE_BAR_Z])
        g1 = np.array([mix[0], mix[1] + self.POUR_DY, TOP + self.LIFT_Z])
        pre = g0 - (0.04, 0.0, 0.0)
        up = np.array([g0[0], g0[1], g1[2]])

        def turn(deg):
            return rot_x(-math.radians(deg)) @ R0

        steps = [Waypoint(2.0, tuple(pre + (0, 0, 0.08)), half, "above, behind the handle", R0),
                 Waypoint(1.0, tuple(pre), half, "behind the handle", R0),
                 Waypoint(1.5, tuple(g0), half, "fingers either side of the grip bar", R0),
                 Waypoint(0.8, tuple(g0), FINGER_CLOSED, "grip the handle", R0),
                 Waypoint(2.0, tuple(up), FINGER_CLOSED, "lift", R0),
                 Waypoint(2.0, tuple(g1), FINGER_CLOSED, "over the mixing bowl", R0)]
        for deg in (40.0, 80.0, self.POUR_DEG):
            steps.append(Waypoint(1.5, tuple(g1), FINGER_CLOSED, f"turn {deg:.0f}", turn(deg)))
        for _ in range(4):
            steps += [Waypoint(0.35, tuple(g1), FINGER_CLOSED, "shake", turn(self.POUR_DEG - 12.0)),
                      Waypoint(0.35, tuple(g1), FINGER_CLOSED, "shake", turn(self.POUR_DEG))]
        steps.append(Waypoint(1.5, tuple(g1), FINGER_CLOSED, "let it drain", turn(self.POUR_DEG)))
        for deg in (80.0, 40.0, 0.0):
            steps.append(Waypoint(1.2, tuple(g1), FINGER_CLOSED, f"turn back {deg:.0f}", turn(deg)))
        steps += [Waypoint(2.0, tuple(up), FINGER_CLOSED, "back over its place", R0),
                  Waypoint(2.0, tuple(g0 + (0, 0, 0.004)), FINGER_CLOSED, "set it down", R0),
                  Waypoint(0.6, tuple(g0 + (0, 0, 0.004)), half, "let go", R0),
                  Waypoint(1.2, tuple(pre), half, "back out", R0)]
        yield Segment({"right": ("ready", 1.5)})
        yield Segment({"right": steps})
        yield Segment({"right": 2.0})
