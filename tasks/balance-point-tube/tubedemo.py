"""Closed-loop scripted demo for balance-point-tube, played by ``yam_scripted`` (see ``tubekit._patch_scripted``).

It sees only what the robot sees: the top camera's RGB image and the arms' measured joint positions (the
observation's ``joint_pos``). It does not use joint efforts (the observation does not carry them) and it never reads the
simulator: not the tube's pose, not the slug, not the centre of mass.

Vision. The tube's ends carry a red and a blue plug. Each frame, :func:`find_caps` takes the centroid of the red and
of the blue pixels in the top camera image. With the camera's calibration (``yam_arena.station.top_camera``: pose and
intrinsics, as a real station has them), each centroid is a ray from the camera. Lying on the table, the plugs are at
the tube's axis height, so the rays meet that horizontal plane at the plugs: that locates the tube. Lifted, the tube
hangs in the vertical plane through its axis (x = the tube's x), so the ray to the plug at the far end meets that
plane at the plug. The grip point comes from forward kinematics of the gripping arm's measured joints. The slope from
the grip to the far plug is the tube's tilt. (The near plug is often hidden by the gripping arm's elbow.)

Search. A pinch grip on the tube is a pivot: lifted off its balance point, the tube tilts toward its heavy end in the
compliant grip, and the low end touches down on the table first. Grip at the middle of the possible range, read the
tilt once on the table (the zero), lift about 3 cm, read it again, set the tube down, and keep the half of the range
toward the low end, until the range is 1.5 cm wide. Then grip with both hands, 11 cm either side of it (the hook goes
between them), lift, slide the tube into the hook from the front, correct the aim by the measured grip position, lower it until it
almost seats, open, and rise straight up.
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


# -- vision ----------------------------------------------------------------------------------------------------------


def find_caps(img: np.ndarray, min_px: int = 6):
    """Pixel centroids (u, v) of the red and the blue plug, or None for one not seen."""
    rgb = img.astype(np.float32)
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    masks = {"red": (r > 90) & (r > 2.0 * g) & (r > 2.0 * b), "blue": (b > 90) & (b > 1.8 * r) & (b > 1.3 * g)}
    out = {}
    for name, m in masks.items():
        v, u = np.nonzero(m)
        out[name] = (float(u.mean()), float(v.mean())) if len(u) >= min_px else None
    return out


@dataclass
class TopCamera:
    """The station's calibrated top camera (OpenGL convention: looks along -z, +y up), for a resized image."""

    width: int
    height: int
    cam: object = field(default_factory=lambda: station.top_camera("calibrated"))

    def ray(self, u: float, v: float) -> tuple[np.ndarray, np.ndarray]:
        """(origin, unit direction) in the world of the ray through pixel (u, v) of the resized image."""
        from yam_arena.urdf import quat_xyzw_to_matrix

        lens = self.cam.lens
        su, sv = lens.width / self.width, lens.height / self.height
        uf, vf = (u + 0.5) * su - 0.5, (v + 0.5) * sv - 0.5  # cv2.resize pixel centres
        d = quat_xyzw_to_matrix(np.asarray(self.cam.rot_xyzw)) @ np.array(
            [(uf - lens.cx) / lens.fx, -(vf - lens.cy) / lens.fy, -1.0])
        return np.asarray(self.cam.pos, float), d / np.linalg.norm(d)

    def project(self, p) -> tuple[float, float]:
        """Pixel (u, v) of the resized image where world point ``p`` appears (for tests)."""
        from yam_arena.urdf import quat_xyzw_to_matrix

        lens = self.cam.lens
        c = quat_xyzw_to_matrix(np.asarray(self.cam.rot_xyzw)).T @ (np.asarray(p, float) - self.cam.pos)
        uf, vf = lens.cx + lens.fx * c[0] / -c[2], lens.cy - lens.fy * c[1] / -c[2]
        su, sv = lens.width / self.width, lens.height / self.height
        return (uf + 0.5) / su - 0.5, (vf + 0.5) / sv - 0.5

    def on_plane(self, uv, axis: int, value: float) -> np.ndarray | None:
        """Where the ray through ``uv`` meets the plane world[axis] = value."""
        if uv is None:
            return None
        o, d = self.ray(*uv)
        if abs(d[axis]) < 1e-9:
            return None
        return o + d * (value - o[axis]) / d[axis]


def tube_on_table(cam: TopCamera, img, radius: float):
    """(red plug, blue plug) positions with the tube lying on the table, or None."""
    caps = find_caps(img)
    red, blue = (cam.on_plane(caps[c], 2, TOP + radius) for c in ("red", "blue"))
    return None if red is None or blue is None else (red, blue)


def tilt_in_plane(cam: TopCamera, img, x: float) -> float | None:
    """Tilt (deg) of the tube hanging in the plane world x = ``x``: positive when the red (+y) end is higher."""
    caps = find_caps(img)
    red, blue = (cam.on_plane(caps[c], 0, x) for c in ("red", "blue"))
    if red is None or blue is None:
        return None
    return math.degrees(math.atan2(red[2] - blue[2], red[1] - blue[1]))


# -- motion ----------------------------------------------------------------------------------------------------------


def grip_yaw(arm: str, pos, base) -> float:
    """Of the two equivalent straight-down grips that close the fingers along x (yaw +-90 deg), the one IK reaches
    with the wrist roll (joint6) furthest inside its limits."""
    best = None
    for yaw in (math.pi / 2, -math.pi / 2):
        from yam_arena.demo import grip_rotation

        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(grip_rotation(yaw)), np.array(READY),
                             base_pos=base, iters=80)
        cost = pe * 100 + re + abs(q[5]) * 1e-3
        if best is None or cost < best[0]:
            best = (cost, yaw)
    return best[1]


class JointMove:
    """A smooth joint-space move to ``q`` with the gripper open (same interface as ``ArmPlan``)."""

    def __init__(self, duration: float, q, q_start):
        self.duration, self.q_goal, self.q_start = duration, np.asarray(q, float), np.asarray(q_start, float)
        self.q = self.q_start.copy()
        self.waypoints = [Waypoint(duration, (0.0, 0.0, 0.0), FINGER_OPEN)]
        self.worst_ik_mm = 0.0

    def action7(self, t: float):
        self.q = self.q_start + ik.smoothstep(min(t / self.duration, 1.0)) * (self.q_goal - self.q_start)
        return np.r_[self.q, 1.0], "park"


# Parked: the ready pose turned 70 degrees outward at the base, so the arm stays out of the top camera's view of the
# tube's ends.
PARK_Q = {"left": np.array([1.2, *READY[1:]]), "right": np.array([-1.2, *READY[1:]])}


@dataclass
class Segment:
    """Waypoints per arm (an arm left out holds its last target), played together; then ``collect`` frames."""

    moves: dict  # arm -> list of Waypoint; a duration (s): park the arm; ("ready", duration): back to the ready pose
    collect: object = None  # called with every observation during the segment
    min_s: float = 0.0


class Controller:
    """Plays :meth:`script` segment by segment; returns (left 7-vector, right 7-vector, finished time or None)."""

    GRIP_ABOVE_AXIS = 0.008  # TCP over the tube's axis: pad tips 5 mm below it
    HOVER = 0.10  # above the table
    TEST_LIFT = 0.05  # commanded; the loaded arm sags, so the tube rises about 3 cm
    RANGE = 0.40  # the grip search runs over +-40 cm of the tube's middle (its plugs sit at +-48.5 cm)
    WIDTH = 0.015  # ... until the range is this narrow
    PUSH_ABOVE_X = 0.20  # before the final grip, roll the tube back toward the arms if it lies beyond this x ...
    PUSH_TO_X = 0.17  # ... to here
    HAND_SPAN = 0.11  # final grip: both hands, 11 cm either side of the balance point

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
        self.log: list[str] = []
        self.worst_ik_mm = 0.0
        self.cheat = os.environ.get("DEMO_CHEAT", "")  # contrasts for the README: "middle" or "hold"
        self.script_it = self.script()

    # plumbing
    def say(self, msg: str) -> None:
        self.log.append(msg)
        print(f"[tubedemo] {msg}", flush=True)

    def _start(self, seg: Segment) -> None:
        for arm, plan in self.plans.items():  # the last segment's end becomes the start
            self.worst_ik_mm = max(self.worst_ik_mm, plan.worst_ik_mm)
            self.q[arm], self.finger[arm] = plan.q.copy(), plan.waypoints[-1].finger
        self.plans = {a: JointMove(w, PARK_Q[a], self.q[a]) if isinstance(w, float) else
                      JointMove(w[1], READY, self.q[a]) if isinstance(w, tuple) else
                      ArmPlan(list(w), self.bases[a], q_start=self.q[a], finger_start=self.finger[a])
                      for a, w in seg.moves.items()}
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
            except StopIteration:  # the script is done: keep holding the end of the last segment
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

    # perception during a segment
    def _see_table(self, obs):
        """Lying on the table: each plug's position, and how high the plane-x reading puts it above its true height
        (the axis height), which is subtracted from later readings."""
        img = obs.images["top_cam"]
        seen = tube_on_table(self.cam, img, self.task.R)
        if seen is None:
            return None
        caps = find_caps(img)
        bias = [self.cam.on_plane(caps[c], 0, p[0])[2] - (TOP + self.task.R) for c, p in zip(("red", "blue"), seen)]
        return seen[0], seen[1], bias[0], bias[1]

    def _see_tilt(self, obs):
        """Lifted: tilt (deg, + with the red end higher) of the line from the grip to the plug at the far end. The grip
        point comes from the arm's measured joint angles (forward kinematics); the plug from the camera, on the plane
        x = the tube's x. The near plug is often hidden behind the gripping arm's elbow."""
        far = "blue" if self.grip_arm == "left" else "red"
        p = self.cam.on_plane(find_caps(obs.images["top_cam"])[far], 0, self.x_plane)
        if p is None:
            return None
        joints = np.asarray(next(iter(obs.state.values())), float).reshape(-1)
        q = joints[:6] if self.grip_arm == "left" else joints[7:13]
        g = ik.tcp_pose(q, self.bases[self.grip_arm])[0] - (0.0, 0.0, self.GRIP_ABOVE_AXIS)
        z = p[2] - self.bias[far]
        if far == "red":
            return math.degrees(math.atan2(z - g[2], p[1] - g[1]))
        return math.degrees(math.atan2(g[2] - z, g[1] - p[1]))

    def _locate(self):
        """Yield a short look; return (x, y) of the tube's middle on the table."""
        yield Segment({}, collect=self._see_table, min_s=0.4)
        seen = [s for s in self.samples if s is not None]
        if not seen:
            raise RuntimeError("the tube's plugs are not visible")
        red = np.median([s[0] for s in seen], axis=0)
        blue = np.median([s[1] for s in seen], axis=0)
        self.bias = {"red": float(np.median([s[2] for s in seen])), "blue": float(np.median([s[3] for s in seen]))}
        mid = (red + blue) / 2
        self.x_plane = mid[0]
        self.say(f"tube on the table: middle x {mid[0]:.3f} y {mid[1]:+.3f}, plugs {np.linalg.norm(red - blue):.3f} "
                 f"m apart")
        return mid[0], mid[1]

    # the script
    def script(self):
        yield Segment(dict.fromkeys(ARMS, 2.0))  # park both arms out of the camera's view
        x, yc = yield from self._locate()
        lo, hi = -self.RANGE, self.RANGE
        if self.cheat == "middle":  # contrast: no search, carry it by its middle
            lo = hi = 0.0
        n = 0
        while hi - lo > self.WIDTH:
            n += 1
            s = (lo + hi) / 2
            tilt = yield from self._test(x, yc + s)
            if tilt > 0:  # the red (+y) end rose: the heavy side is toward -y
                hi = s
            else:
                lo = s
            self.say(f"test {n}: grip at {s * 100:+.1f} cm from the middle: tilt {tilt:+.2f} deg, "
                     f"balance point in [{lo * 100:+.1f}, {hi * 100:+.1f}] cm")
            x, yc = yield from self._locate()
        s = (lo + hi) / 2
        self.say(f"balance point at {s * 100:+.1f} cm from the middle: hang it there")
        if x > self.PUSH_ABOVE_X:  # the test lifts rolled it toward the hook: roll it back before gripping it there
            yield from self._roll_back(x, yc)
            x, yc = yield from self._locate()
        yield from self._place(x, yc + s)

    def _read_tilt(self):
        yield Segment({}, collect=self._see_tilt, min_s=0.5)
        tilts = [s for s in self.samples if s is not None]
        if len(tilts) < 2:
            raise RuntimeError(f"the far plug was seen in only {len(tilts)} frames")
        return float(np.median(tilts))

    def _test(self, x, y):
        t = self.task
        arm = "left" if y >= 0 else "right"
        self.grip_y, self.grip_arm = y, arm
        other = "right" if arm == "left" else "left"
        yaw = grip_yaw(arm, (x, y, TOP + t.R + self.GRIP_ABOVE_AXIS), self.bases[arm])
        gz = TOP + t.R + self.GRIP_ABOVE_AXIS
        hz = TOP + self.HOVER
        yield Segment({arm: [Waypoint(1.5, (x, y, hz), FINGER_OPEN, "over the grip point", yaw),
                             Waypoint(1.0, (x, y, gz), FINGER_OPEN, "down", yaw),
                             Waypoint(0.6, (x, y, gz), FINGER_CLOSED, "grip", yaw)],
                       other: 1.5})
        self.lifted_now = False
        flat = yield from self._read_tilt()  # still on the table: the reading of a level tube, with this grip
        yield Segment({arm: [Waypoint(1.0, (x, y, gz + self.TEST_LIFT), FINGER_CLOSED, "test lift", yaw),
                             Waypoint(0.6, (x, y, gz + self.TEST_LIFT), FINGER_CLOSED, "settle", yaw)]})
        self.lifted_now = True
        lifted = yield from self._read_tilt()
        tilt = lifted - flat
        self.say(f"  reading on the table {flat:+.2f} deg, lifted {lifted:+.2f} deg")
        yield Segment({arm: [Waypoint(1.0, (x, y, gz), FINGER_CLOSED, "set down", yaw),
                             Waypoint(0.5, (x, y, gz), FINGER_OPEN, "let go", yaw),
                             Waypoint(0.8, (x, y, hz), FINGER_OPEN, "up", yaw)]})
        yield Segment({arm: 1.5})  # park again: the arm hides a plug from the camera
        return tilt

    def _roll_back(self, x, yc):
        """Both hands, fingers closed, come down behind the tube (beyond it) 25 cm either side of its middle and roll it
        back toward the arms to PUSH_TO_X; the round tube rolls rather than slides."""
        behind, to = x + 0.055, self.PUSH_TO_X + 0.055
        z = TOP + 0.035
        side = {"left": 0.25, "right": -0.25}
        yaw = {"left": -math.pi / 2, "right": math.pi / 2}
        yield Segment(dict.fromkeys(ARMS, ("ready", 1.5)))
        yield Segment({a: [Waypoint(1.5, (behind, yc + side[a], TOP + self.HOVER), FINGER_CLOSED, "behind the tube", yaw[a]),
                           Waypoint(1.0, (behind, yc + side[a], z), FINGER_CLOSED, "down", yaw[a]),
                           Waypoint(2.0, (to, yc + side[a], z), FINGER_CLOSED, "roll it back", yaw[a]),
                           Waypoint(1.0, (to + 0.03, yc + side[a], TOP + self.HOVER), FINGER_OPEN, "up", yaw[a])]
                       for a in ARMS})
        yield Segment(dict.fromkeys(ARMS, 1.5))  # park

    def _see_tcp(self, obs):
        """Tool points of both arms from their measured joints (forward kinematics), as a (2, 3) array."""
        joints = np.asarray(next(iter(obs.state.values())), float).reshape(-1)
        return np.array([ik.tcp_pose(joints[:6], self.bases["left"])[0],
                         ik.tcp_pose(joints[7:13], self.bases["right"])[0]])

    def _place(self, x, y):
        """Grip with both hands, HAND_SPAN either side of the balance point (the hook goes between them; the tube's
        weight is shared equally, so it stays level in the grips), lift the tube a little, bring it in front of the
        hook, raise it above the lip, slide it in over the cradle, correct the aim by the measured grip positions,
        lower it until it almost seats, let go with both hands, rise and back away."""
        t = self.task
        hx, hy = t.HOOK_XY
        a = self.HAND_SPAN
        gz = TOP + t.R + self.GRIP_ABOVE_AXIS
        low = TOP + 0.05
        entry = TOP + 0.175  # the arms sag under the load: the tube's bottom passes about 1.5 cm over the lip
        seat = TOP + t.REST_AXIS + self.GRIP_ABOVE_AXIS + 0.002  # let go 2 mm above where the tube seats
        front = hx - 0.12
        side = {"left": a, "right": -a}
        yaw = {"left": -math.pi / 2, "right": math.pi / 2}

        def both(steps):
            return {arm: [Waypoint(d, (px, py + side[arm], pz), f, label, yaw[arm]) for d, (px, py, pz), f, label in steps]
                    for arm in ARMS}

        yield Segment(dict.fromkeys(ARMS, ("ready", 1.5)))  # from parked, through the ready pose (a good IK start)
        yield Segment(both([(1.5, (x, y, TOP + self.HOVER), FINGER_OPEN, "over the grip points"),
                            (1.0, (x, y, gz), FINGER_OPEN, "down"),
                            (0.6, (x, y, gz), FINGER_CLOSED, "grip with both hands"),
                            (1.0, (x, y, low), FINGER_CLOSED, "lift"),
                            (1.5, (front, hy, low), FINGER_CLOSED, "in front of the hook"),
                            (1.2, (front, hy, entry), FINGER_CLOSED, "raise above the lip"),
                            (2.0, (hx, hy, entry), FINGER_CLOSED, "slide in over the cradle"),
                            (0.8, (hx, hy, entry), FINGER_CLOSED, "settle")]))
        # The loaded arms lag their commands: correct the aim by the measured grip positions (proprioception).
        yield Segment({}, collect=self._see_tcp, min_s=0.4)
        tcp = np.median(self.samples, axis=0).mean(axis=0)  # midpoint of the two grips
        ax, ay = hx - 0.005 + (hx - tcp[0]), hy + (hy - tcp[1])  # 5 mm short: the V seat pulls it in; past it is the shank
        self.say(f"over the cradle the grips' midpoint is {(tcp[0] - hx) * 100:+.1f}, {(tcp[1] - hy) * 100:+.1f} cm "
                 f"off: aim there")
        steps = [(1.0, (ax, ay, entry), FINGER_CLOSED, "aim"),
                 (1.5, (ax, ay, seat), FINGER_CLOSED, "lower onto the hook"),
                 (0.5, (ax, ay, seat), FINGER_CLOSED, "settle")]
        if self.cheat == "hold":  # contrast: keep holding it on the hook
            steps.append((3.0, (ax, ay, seat), FINGER_CLOSED, "hold it there"))
        else:
            steps += [(0.8, (ax, ay, seat), FINGER_OPEN, "let go"),
                      (1.5, (ax, ay, seat + 0.05), FINGER_OPEN, "straight up, clear of the tube"),
                      (1.5, (front, ay, seat + 0.05), FINGER_OPEN, "back away")]
        yield Segment(both(steps))
