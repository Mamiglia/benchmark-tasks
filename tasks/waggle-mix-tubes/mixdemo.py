"""Closed-loop scripted demo for waggle-mix-tubes, played by ``yam_scripted`` (see ``mixkit._patch_scripted``).

It sees only what the robot sees: the top and wrist cameras' RGB images and the arms' measured joint positions. It
never reads the simulator (no tube pose, no liquid state).

1. Both arms turn outward at the base, out of the top camera's view of the rack.
2. **Locate.** The rack is the green blob in the image. Its far top edge is the blob's top row; with the camera's
   calibration (``yam_arena.station.top_camera``), that row's middle and its two ends are rays that meet the plane
   of the rack's top face. The holes are 3 cm in front of that edge, 8 cm either side of its middle (the rack's
   drawing). The tubes stand in the holes; their liquid is hidden under the top board.
3. **Lift.** Each arm comes down over its tube, measures its grip position (forward kinematics of the measured
   joints), aims off by its lag, and grips the tube from above, 8 mm below the mouth (the only part above the rack;
   this is also where a person holds a tube to mix it). It lifts it straight up out of its hole.
4. **Waggle.** With the tube hanging upright below the gripper, the wrist (joints 4 and 5 together) swings it side
   to side about the wrist, in the plane of the fingers' closing direction, 3 times a second, +-0.20 rad (+-11
   degrees), ramped in and out over 0.4 s so that the strokes start and stop without a jerk. The tube's bottom
   moves about +-4 cm. This swings the liquid's surface past 30 degrees each stroke while it stays far below the
   mouth; the tube is never tipped far, inverted or jerked up and down.
5. **Look.** Before the first burst and after each 2 s burst it holds still and looks at the tube with the
   gripping arm's wrist camera, which looks down along the tube from beside the gripper and sees the top face of the
   liquid. It measures how blue that face is (:func:`top_blueness`): water is nearly white, and the face turns blue
   as the copper(II) sulfate below is mixed up into it. It waggles again until the face has turned visibly blue (0.15
   above its start) and its blueness rises by less than 0.02 over a burst (the colour no longer changes), at most 6
   bursts.
6. **Rack.** When it grips a tube in the rack, the closing fingers centre the tube between them, so the measured
   grip position (forward kinematics of the measured joints) is where the tube stood. It carries the tube back over
   that point, lowers it until its bottom is 8 mm above the top board, measures the grip again and aims off by the
   arm's lag, lowers it until its bottom is 3 mm above the base board, lets go, rises and parks.

Both arms work at the same time, each on its own tube.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import ArmPlan, Waypoint

ARMS = ("left", "right")
TOP = station.TABLE_TOP
TUBE_LEN, BASE_T = 0.100, 0.012
FLOOR = TOP + BASE_T
LOWER, UPPER = 0.013, 0.013  # liquid layers
GRIP_BELOW_MOUTH = 0.008  # TCP this far below the mouth
WAGGLE_HZ, WAGGLE_RAD, BURST_S, RAMP_S = 3.0, 0.20, 2.0, 0.4
MAX_BURSTS = 6
PLATEAU = 0.02  # mixed once the top's blueness rises by less than this over a burst (it no longer changes) ...
RISEN = 0.15  # ... after it has risen at least this much from the start (the top has visibly turned blue)
RACK_TOP, RACK_DEPTH, HOLE_Y = 0.070, 0.06, 0.08  # the rack (README): top face height, depth, hole spacing


# -- vision ----------------------------------------------------------------------------------------------------------


def green_mask(img: np.ndarray) -> np.ndarray:
    """The rack's green plastic."""
    rgb = img.astype(np.float32)
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    return (g > 50) & (g > 1.4 * r) & (g > 1.2 * b)


def find_rack(img: np.ndarray, min_px: int = 30):
    """(u, v) of the middle of the rack's far top edge, and the u of its left and right ends on that edge, or None."""
    m = green_mask(img)
    v, u = np.nonzero(m)
    if len(u) < min_px:
        return None
    v0 = int(v.min())
    row = np.nonzero(m[v0:v0 + 3].any(axis=0))[0]
    return (float(row.mean()), float(v0)), float(row.min()), float(row.max())


@dataclass
class TopCamera:
    """The station's calibrated top camera (OpenGL convention: looks along -z, +y up), for a resized image."""

    width: int
    height: int
    cam: object = field(default_factory=lambda: station.top_camera("calibrated"))

    def ray(self, u: float, v: float):
        from yam_arena.urdf import quat_xyzw_to_matrix

        lens = self.cam.lens
        su, sv = lens.width / self.width, lens.height / self.height
        uf, vf = (u + 0.5) * su - 0.5, (v + 0.5) * sv - 0.5  # cv2.resize pixel centres
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

    def on_plane(self, uv, axis: int, value: float):
        if uv is None:
            return None
        o, d = self.ray(*uv)
        if abs(d[axis]) < 1e-9:
            return None
        return o + d * (value - o[axis]) / d[axis]


WRIST_ROI = (slice(50, 110), slice(130, 186))  # rows, columns of a 320 x 176 wrist image: the gripped tube's liquid


def top_blueness(img: np.ndarray, min_px: int = 12) -> float | None:
    """How blue the top of the liquid looks to the wrist camera of the arm holding the tube upright. The camera looks
    down along the tube from beside the gripper and sees the top face of the upper layer as the brightest part of the
    liquid. Liquid pixels are the bluish ones (blue at least red and green; the beige table, the green rack and the
    dark gripper are not); the top face is the brightest quarter of them. Blueness is (B - R) / B, which does not
    change with shading: about 0.1 for water, rising as the copper(II) sulfate below mixes up into it. None if too
    few liquid pixels are seen."""
    roi = img[WRIST_ROI[0], WRIST_ROI[1]].astype(np.float32).reshape(-1, 3)
    r, g, b = roi.T
    roi = roi[(b >= r) & (b >= g) & (b > 60)]
    if len(roi) < min_px:
        return None
    lum = roi.sum(axis=1)
    face = roi[lum >= np.percentile(lum, 75)]
    return float(np.median((face[:, 2] - face[:, 0]) / np.maximum(face[:, 2], 1.0)))


# -- motion ----------------------------------------------------------------------------------------------------------


class JointMove:
    """A smooth joint-space move to ``q`` (same interface as ``ArmPlan``)."""

    def __init__(self, duration: float, q, q_start, finger: float = FINGER_OPEN):
        self.duration, self.q_goal, self.q_start = duration, np.asarray(q, float), np.asarray(q_start, float)
        self.q = self.q_start.copy()
        self.waypoints = [Waypoint(duration, (0.0, 0.0, 0.0), finger)]
        self.worst_ik_mm = 0.0

    def action7(self, t: float):
        self.q = self.q_start + ik.smoothstep(min(t / self.duration, 1.0)) * (self.q_goal - self.q_start)
        return np.r_[self.q, self.waypoints[0].finger / FINGER_OPEN], "move"


class Waggle:
    """Swing the wrist (joints 4 and 5 together, ``SWING``) sinusoidally about the arm's current pose, ramped in and
    out."""

    def __init__(self, q_start, finger: float, swing, phase: float = 0.0, amp: float = WAGGLE_RAD):
        self.q_start = np.asarray(q_start, float)
        self.q = self.q_start.copy()
        self.duration, self.phase, self.amp, self.swing = BURST_S, phase, amp, np.asarray(swing, float)
        self.waypoints = [Waypoint(BURST_S, (0.0, 0.0, 0.0), finger)]
        self.worst_ik_mm = 0.0

    def action7(self, t: float):
        ramp = min(1.0, t / RAMP_S, max(0.0, (self.duration - t) / RAMP_S))
        ramp = ik.smoothstep(ramp)
        q = self.q_start.copy()
        q += self.swing * ramp * self.amp * math.sin(2 * math.pi * WAGGLE_HZ * t + self.phase)
        self.q = q if t < self.duration else self.q_start.copy()
        return np.r_[q, self.waypoints[0].finger / FINGER_OPEN], "waggle"


FLIP = np.array([0.0, 0.0, 0.0, 2.6, 0.0, 0.0])  # 'invert' contrast: the wrist pitch turns the tube over
UP_DOWN = np.array([0.0, 1.0, -1.0, 0.0, 0.0, 0.0])  # 'shake' contrast: shoulder and elbow move the tube up and down
PARK_Q = {"left": np.array([1.2, *READY[1:]]), "right": np.array([-1.2, *READY[1:]])}
YAW = {"left": 0.0, "right": 0.0}  # fingers close along y (side to side), which centres the tube across them
# The waggle turns joints 4 and 5 together so that the tube swings side to side (in the y-z plane, about an axis
# along x, across the fingers' closing direction): the pads' grip then resists the swing as a couple.
SWING = {"left": np.array([0.0, 0.0, 0.0, 0.6, 0.8, 0.0]), "right": np.array([0.0, 0.0, 0.0, 0.6, -0.8, 0.0])}


@dataclass
class Segment:
    moves: dict  # arm -> list of Waypoint | float (park) | ("ready", s) | ("waggle", phase) | ("turn", s, dq)
    collect: object = None
    min_s: float = 0.0


class Controller:
    HOLD = {"left": (0.22, 0.14), "right": (0.22, -0.14)}  # where each tube is waggled (TCP x, y)
    HOLD_Z = 0.19  # TCP height above the table while waggling (a straight-down grip reaches up to about 0.2 m)

    def __init__(self, bases, dt):
        self.dt = dt
        self.bases = {a: np.asarray(bases[a], float) for a in ARMS}
        self.q = {a: np.array(READY, float) for a in ARMS}
        self.finger = dict.fromkeys(ARMS, FINGER_OPEN)
        self.plans: dict = {}
        self.t0 = self.dur = 0.0
        self.collect, self.samples = None, []
        self.finished_at = None
        self.cam = None
        self.cheat = os.environ.get("DEMO_CHEAT", "")  # README contrasts: "invert", "shake", "nomix"
        self.script_it = self.script()

    def say(self, msg: str) -> None:
        print(f"[mixdemo] {msg}", flush=True)

    def _start(self, seg: Segment) -> None:
        for arm, plan in self.plans.items():
            self.q[arm], self.finger[arm] = plan.q.copy(), plan.waypoints[-1].finger
        plans = {}
        for a, w in seg.moves.items():
            if isinstance(w, float):
                plans[a] = JointMove(w, PARK_Q[a], self.q[a], self.finger[a])
            elif isinstance(w, tuple) and w[0] == "ready":
                plans[a] = JointMove(w[1], READY, self.q[a], self.finger[a])
            elif isinstance(w, tuple) and w[0] == "shake":
                plans[a] = Waggle(self.q[a], self.finger[a], UP_DOWN, 0.0, 0.12)
            elif isinstance(w, tuple) and w[0] == "waggle":
                plans[a] = Waggle(self.q[a], self.finger[a], SWING[a], w[1], *w[2:])
            elif isinstance(w, tuple) and w[0] == "turn":  # ("turn", duration, joint offsets from here)
                plans[a] = JointMove(w[1], self.q[a] + w[2], self.q[a], self.finger[a])
            else:
                plans[a] = ArmPlan(list(w), self.bases[a], q_start=self.q[a], finger_start=self.finger[a])
        self.plans = plans
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

    # perception
    def _joints(self, obs, arm):
        j = np.asarray(next(iter(obs.state.values())), float).reshape(-1)
        return j[:6] if arm == "left" else j[7:13]

    def _see_tcp(self, obs):
        """Each arm's tool point from its measured joints (forward kinematics)."""
        return {a: ik.tcp_pose(self._joints(obs, a), self.bases[a])[0] for a in ARMS}

    def _see_rack(self, obs):
        """The holes' centres from the rack's far top edge (its middle and its two ends), on the plane of the top
        board's top face; the holes sit 3 cm in front of that edge, 8 cm either side of the middle."""
        found = find_rack(obs.images["top_cam"])
        if found is None:
            return None
        (u, v), ul, ur = found
        z = TOP + RACK_TOP
        mid = self.cam.on_plane((u, v), 2, z)
        ends = [self.cam.on_plane((x, v), 2, z) for x in (ul, ur)]
        if mid is None or any(e is None for e in ends):
            return None
        cy = (ends[0][1] + ends[1][1]) / 2
        cx = mid[0] - RACK_DEPTH / 2
        return {"left": np.array([cx, cy + HOLE_Y]), "right": np.array([cx, cy - HOLE_Y])}

    def _see_layers(self, obs):
        return {a: top_blueness(obs.images[f"{a}_cam"]) for a in ARMS}

    def _median(self, arm):
        c = [s[arm] for s in self.samples if s[arm] is not None]
        return round(float(np.median(c)), 3) if c else None

    def _aim(self, want):
        """The arms lag their commands by a few millimetres: measure the grips (forward kinematics of the measured
        joints) and return targets aimed off by the error."""
        yield Segment({}, collect=self._see_tcp, min_s=0.4)
        aim = {}
        for a in ARMS:
            got = np.median([s[a] for s in self.samples], axis=0)[:2]
            aim[a] = 2 * np.asarray(want[a]) - got
            self.say(f"{a} grip {np.round((got - want[a]) * 1000, 1)} mm off: aim off by that")
        return aim

    # the script
    def script(self):
        yield Segment(dict.fromkeys(ARMS, 2.0))  # park out of the camera's view
        yield Segment({}, collect=self._see_rack, min_s=0.4)
        seen = [s for s in self.samples if s is not None]
        if not seen:
            raise RuntimeError("the rack is not visible")
        pos = {a: np.median([s[a] for s in seen], axis=0) for a in ARMS}
        for a in ARMS:
            self.say(f"{a} hole at x {pos[a][0]:.3f} y {pos[a][1]:+.3f}")
        mouth = FLOOR + TUBE_LEN
        gz = mouth - GRIP_BELOW_MOUTH
        up = gz + 0.075  # the tube's bottom 1.3 cm above the rack's top board
        yield Segment(dict.fromkeys(ARMS, ("ready", 1.5)))
        yield Segment({a: [Waypoint(1.5, (*pos[a], gz + 0.06), FINGER_OPEN, "over the tube", YAW[a]),
                           Waypoint(1.0, (*pos[a], gz), FINGER_OPEN, "down", YAW[a])] for a in ARMS})
        aim = yield from self._aim(pos)
        yield Segment({a: [Waypoint(0.5, (*aim[a], gz), FINGER_OPEN, "aim", YAW[a]),
                           Waypoint(0.6, (*aim[a], gz), FINGER_CLOSED, "grip near the mouth", YAW[a])]
                       for a in ARMS})
        # Where the tube stood: the fingers have centred it between them, so the measured grip position (forward
        # kinematics of the measured joints) is the tube's place in its hole. It goes back exactly there.
        yield Segment({}, collect=self._see_tcp, min_s=0.4)
        home = {a: np.median([s[a] for s in self.samples], axis=0)[:2] for a in ARMS}
        for a in ARMS:
            self.say(f"{a} tube gripped at x {home[a][0]:.3f} y {home[a][1]:+.3f}")
        yield Segment({a: [Waypoint(1.5, (*pos[a], up), FINGER_CLOSED, "lift out of the rack", YAW[a]),
                           Waypoint(1.2, (*self.HOLD[a], TOP + self.HOLD_Z), FINGER_CLOSED, "to the mixing spot",
                                    YAW[a]),
                           Waypoint(0.5, (*self.HOLD[a], TOP + self.HOLD_Z), FINGER_CLOSED, "settle", YAW[a])]
                       for a in ARMS})
        if self.cheat == "invert":  # contrast: mix by turning the tube over, as for a stoppered tube
            for _ in range(3):
                yield Segment({a: ("turn", 0.8, FLIP) for a in ARMS})
                yield Segment({a: ("turn", 0.8, -FLIP) for a in ARMS})
        elif self.cheat == "shake":  # contrast: shake it hard up and down (shoulder and elbow, strokes of about 10 cm)
            yield Segment({a: ("shake",) for a in ARMS})
        elif self.cheat != "nomix":
            yield Segment({}, collect=self._see_layers, min_s=0.3)
            last = {a: self._median(a) for a in ARMS}
            start = dict(last)
            self.say(f"before mixing: top blueness {last}")
            todo = set(ARMS)
            for n in range(MAX_BURSTS):
                yield Segment({a: ("waggle", 0.0 if a == "left" else math.pi) for a in todo})
                yield Segment({}, min_s=0.4)
                yield Segment({}, collect=self._see_layers, min_s=0.3)
                for a in sorted(todo):
                    c = self._median(a)
                    self.say(f"burst {n + 1}: {a} tube, top blueness {c}")
                    if (c is not None and last[a] is not None and start[a] is not None and c - start[a] >= RISEN
                            and c - last[a] < PLATEAU):
                        todo.discard(a)  # the colour no longer changes: mixed
                    last[a] = c
                if not todo:
                    break
            if todo:
                self.say(f"still changing after {MAX_BURSTS} bursts: {sorted(todo)}")
        seat = FLOOR + 0.003 + TUBE_LEN - GRIP_BELOW_MOUTH  # TCP height with the bottom 3 mm above the base board
        above = TOP + RACK_TOP + 0.008 + TUBE_LEN - GRIP_BELOW_MOUTH  # the bottom 8 mm above the top board
        yield Segment({a: [Waypoint(1.5, (*home[a], up), FINGER_CLOSED, "back over the hole", YAW[a]),
                           Waypoint(1.0, (*home[a], above), FINGER_CLOSED, "down to just above the hole", YAW[a])]
                       for a in ARMS})
        aim = yield from self._aim(home)
        yield Segment({a: [Waypoint(0.5, (*aim[a], above), FINGER_CLOSED, "aim", YAW[a]),
                           Waypoint(1.5, (*aim[a], seat), FINGER_CLOSED, "lower it in", YAW[a]),
                           Waypoint(0.6, (*aim[a], seat), FINGER_OPEN, "let go", YAW[a]),
                           Waypoint(1.0, (*aim[a], seat + 0.08), FINGER_OPEN, "straight up", YAW[a])]
                       for a in ARMS})
        yield Segment(dict.fromkeys(ARMS, 2.0))
