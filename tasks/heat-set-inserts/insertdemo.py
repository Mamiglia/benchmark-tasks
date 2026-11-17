"""Closed-loop scripted demo for heat-set-inserts, played by ``yam_scripted`` (see ``insertkit._patch_scripted``).

It sees only the observation: the arms' measured joint positions (and the camera images, unused). It never reads the
simulator: not the inserts' depth, not contacts. It knows the fixed layout (the bracket's place and the stand's).

Motion (right arm). A grip across the iron's handle, 150 mm from the tip, approached along the iron's axis while it
rests in the stand; the iron drawn out along the holder tube, lifted, turned upright and carried over each insert.

Feel. The measured joints give the tool point by forward kinematics, so the height of the iron's shoulder (the face
that presses on the insert) is known without looking. Per insert: lower the shoulder onto the insert and command it 3
mm lower, which presses with a few newtons while the insert heats; then lower the command at 1 mm/s, which the soft
plastic follows at about 1.5 + 1 / k N, and stop when the measured shoulder is 0.3 mm above the bracket's face.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import ArmPlan, Waypoint

ARMS = ("left", "right")


def _env(name, default):
    return float(os.environ.get(name, default))


def grip_along(axis) -> np.ndarray:
    """TCP rotation whose approach (+z) is ``axis`` (in the x-z plane), fingers closing along world y."""
    z = np.asarray(axis, float) / np.linalg.norm(axis)
    x = np.array([0.0, -1.0, 0.0])
    return np.column_stack([x, np.cross(z, x), z])


class JointMove:
    def __init__(self, duration: float, q, q_start, finger):
        self.duration, self.q_goal, self.q_start = duration, np.asarray(q, float), np.asarray(q_start, float)
        self.q = self.q_start.copy()
        self.waypoints = [Waypoint(duration, (0.0, 0.0, 0.0), finger)]
        self.worst_ik_mm = 0.0

    def action7(self, t: float):
        self.q = self.q_start + ik.smoothstep(min(t / self.duration, 1.0)) * (self.q_goal - self.q_start)
        return np.r_[self.q, self.waypoints[0].finger / FINGER_OPEN], "joint move"


class Plan(ArmPlan):
    """ArmPlan whose IK retries from a few seeds when the warm start does not converge."""

    SEEDS = (READY, [0.5, 2.1, 2.2, -1.6, 0.0, 0.5], [0.2, 1.0, 1.0, -0.8, 0.1, 0.2], [0.3, 1.5, 1.5, -1.0, 0.0, 0.3])

    def action7(self, t: float):
        pos, rot_m, finger, label = self.target(t)
        quat = ik.matrix_to_quat_xyzw(rot_m)
        q, pe, re = ik.solve(pos, quat, self.q, base_pos=self.base_pos, iters=60)
        if pe > 0.002 or re > 0.03:
            for s in self.SEEDS:
                q2, pe2, re2 = ik.solve(pos, quat, np.array(s, float), base_pos=self.base_pos, iters=300)
                if pe2 + 0.05 * re2 < pe + 0.05 * re:
                    q, pe, re = q2, pe2, re2
        self.q = q
        self.worst_ik_mm = max(self.worst_ik_mm, pe * 1000)
        return np.r_[q, float(np.clip(finger / FINGER_OPEN, 0.0, 1.0))], label


PARK_Q = {"left": np.array([1.2, *READY[1:]]), "right": np.array([-1.2, *READY[1:]])}


@dataclass
class Segment:
    moves: dict  # arm -> list of Waypoint (yaw is a rotation matrix); a float: park the arm (s)
    collect: object = None
    min_s: float = 0.0


class Controller:
    GRIP_S = 0.150  # grip point: this far from the pin's end along the iron
    FEED = 0.001  # m/s, once hot
    PRELOAD = 0.003  # m: command below the insert's top while it heats
    HEAT_S = 3.0
    STOP_ABOVE = 0.0006  # stop when the measured shoulder is this far above the face (it reads about 0.4 mm high under load)

    def __init__(self, bases, dt, rig):
        self.rig, self.dt = rig, dt
        self.bases = {a: np.asarray(bases[a], float) for a in ARMS}
        self.q = {a: np.array(READY, float) for a in ARMS}
        self.finger = dict.fromkeys(ARMS, FINGER_OPEN)
        self.plans: dict = {}
        self.t0 = self.dur = 0.0
        self.collect, self.samples = None, []
        self.finished_at = None
        self.cheat = os.environ.get("DEMO_CHEAT", "")  # contrasts for the README: "hard", "tilt", "keep"
        self.feed = 0.012 if self.cheat == "hard" else self.FEED
        self.preload = 0.012 if self.cheat == "hard" else self.PRELOAD
        self.lean = math.radians(8.0 if self.cheat == "tilt" else 0.0)
        # calibration knobs: where the iron sits in the grip, across and along (m)
        self.grip_dx = _env("DEMO_GRIP_DX", 0.0)
        self.grip_ds = _env("DEMO_GRIP_DS", 0.0)
        self.script_it = self.script()

    def say(self, msg: str) -> None:
        print(f"[insertdemo] {msg}", flush=True)

    def _start(self, seg: Segment) -> None:
        for arm, plan in self.plans.items():
            self.q[arm], self.finger[arm] = plan.q.copy(), plan.waypoints[-1].finger
        self.plans = {a: JointMove(w, PARK_Q[a], self.q[a], self.finger[a]) if isinstance(w, float) else
                      Plan(list(w), self.bases[a], q_start=self.q[a], finger_start=self.finger[a])
                      for a, w in seg.moves.items()}
        self.dur = max([p.duration for p in self.plans.values()] + [seg.min_s])
        self.collect, self.samples = seg.collect, []

    def act(self, obs):
        t = float(obs.state_time)
        self.obs = obs
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

    # observations
    def _joints(self, obs, arm):
        j = np.asarray(next(iter(obs.state.values())), float).reshape(-1)
        return j[:6] if arm == "left" else j[7:13]

    def _shoulder_z(self, obs):
        """The iron's shoulder height, from the right arm's measured joints and the grip."""
        p, R = ik.tcp_pose(self._joints(obs, "right"), self.bases["right"])
        return float((p + R @ self.shoulder_in_tcp)[2])

    # geometry
    def _tcp_for_tip(self, tip, axis_down, R):
        """TCP putting the iron's pin end at ``tip`` with the iron pointing along ``axis_down`` (tip direction)."""
        a = np.asarray(axis_down, float) / np.linalg.norm(axis_down)
        return np.asarray(tip, float) - (self.GRIP_S + self.grip_ds) * a - R @ np.array([self.grip_dx, 0.0, 0.0])

    def _tip(self, obs):
        p, R = ik.tcp_pose(self._joints(obs, "right"), self.bases["right"])
        return p + R @ self.tip_in_tcp

    def _step(self, x, y, z, down, R, dur, label):
        """One short move of the tip to (commanded xy, z); then correct the commanded xy by the measured error."""
        tcp = self._tcp_for_tip((*self.cxy, z), down, R)
        yield Segment({"right": [Waypoint(dur, tuple(tcp), FINGER_CLOSED, label, R)]}, collect=self._tip)
        self.tip_now = self.samples[-1] if self.samples else self._tip(self.obs)
        err = np.array([x, y]) - self.tip_now[:2]
        self.cxy = self.cxy + 0.6 * np.clip(err, -0.003, 0.003)

    def script(self):
        r = self.rig
        # The iron at rest in the holder (part of the known layout, as seen on the settled scene): its tip 39 mm above
        # the table, leaning across the 30 mm tube, 38.5 degrees below the horizontal.
        lean = math.radians(_env("DEMO_REST_DEG", 38.5))
        a = np.array([math.cos(lean), 0.0, -math.sin(lean)])
        Rs = grip_along(a)
        tip_rest = np.array([r.tube_bottom[0] + 0.0089, r.tube_bottom[1], r.table_z + 0.0391])
        g = self._tcp_for_tip(tip_rest, a, Rs)
        W = Waypoint
        right = [W(2.0, tuple(g - 0.08 * a + (0, 0, 0.03)), FINGER_OPEN, "behind the grip", Rs),
                 W(1.2, tuple(g - 0.06 * a), FINGER_OPEN, "line up with the iron", Rs),
                 W(1.2, tuple(g), FINGER_OPEN, "slide the fingers over the grip", Rs),
                 W(0.8, tuple(g), FINGER_CLOSED, "grip the handle", Rs),
                 W(2.0, tuple(g - 0.12 * a), FINGER_CLOSED, "draw the iron out of the holder", Rs),
                 W(1.0, tuple(g - 0.12 * a + (0, 0, 0.04)), FINGER_CLOSED, "lift", Rs)]
        yield Segment({"left": 2.0, "right": right})
        down = np.array([math.sin(self.lean), 0.0, -math.cos(self.lean)])
        Rd = grip_along(down)
        # the shoulder (pressing face) in the TCP frame, as gripped
        self.shoulder_in_tcp = Rd.T @ (down * (self.GRIP_S + self.grip_ds - r.pin[1]))
        face = r.face_z
        top0 = face + r.start_proud
        lifted = g - 0.12 * a + (0, 0, 0.04)
        upright = lifted + np.array([-0.06, 0.04, 0.03])
        yield Segment({"right": [W(2.5, tuple(upright), FINGER_CLOSED, "turn the iron upright", Rd)]})
        self.tip_in_tcp = Rd.T @ (down * (self.GRIP_S + self.grip_ds))
        step = 0.1
        for i in range(len(r.holes)):
            x, y = r.hole_xy(i)
            over = self._tcp_for_tip((x, y, top0 - r.pin[1] + 0.015), down, Rd)
            yield Segment({"right": [W(2.0 if i == 0 else 1.2, tuple(over), FINGER_CLOSED, f"over insert {i}", Rd)]},
                          min_s=2.5 if i == 0 else 1.2)
            # Every step below is 0.1 s, then the measured tip (forward kinematics) corrects the command sideways:
            # under load the arm's joints lag their targets by a millimetre or two, more than the pin's capture.
            self.cxy = np.array([x, y], float)
            hist = []
            z = top0 - r.pin[1] + 0.015
            while z > top0 - r.pin[1] + 0.0015:  # down to 1.5 mm over the insert at 15 mm/s
                z -= 0.0015
                yield from self._step(x, y, z, down, Rd, step, "down to the insert")
            t_press = 0.0
            z_goal = top0 - self.preload - r.pin[1]
            while t_press < self.HEAT_S:  # onto it, then a few newtons while it heats
                z = max(z - 0.001, z_goal)
                yield from self._step(x, y, z, down, Rd, step, "press while it heats")
                t_press += step
            for _ in range(250):  # feed down at FEED until the measured shoulder is STOP_ABOVE over the face
                z -= self.feed * step
                yield from self._step(x, y, z, down, Rd, step, "feed")
                sz = self.tip_now[2] + r.pin[1]
                hist.append(sz)
                if sz <= face + self.STOP_ABOVE:
                    break
                if len(hist) > 40 and hist[-40] - sz < 0.0002:  # no progress in 4 s: give up on this one
                    self.say(f"insert {i}: stalled")
                    break
            self.say(f"insert {i}: shoulder {(sz - face) * 1000:+.2f} mm over the face, tip off its axis "
                     f"{np.round((self.tip_now[:2] - (x, y)) * 1000, 2)} mm, command {(z + r.pin[1] - face) * 1000:+.2f} mm")
            up = self._tcp_for_tip((*self.cxy, face + 0.02), down, Rd)
            yield Segment({"right": [W(0.6, tuple(up), FINGER_CLOSED, "lift off", Rd)]})
        if self.cheat == "keep":
            return
        # Back into the holder along the tube's own axis (steeper than the iron rests), tip 12 mm short of the closed
        # end, then let go: the iron settles leaning across the tube as it started.
        at = r.tube_axis
        Rt = grip_along(at)
        tip_in = r.tube_bottom - 0.012 * at
        gt = self._tcp_for_tip(tip_in, at, Rt)
        back = gt - 0.13 * at + (0, 0, 0.03)
        yield Segment({"right": [W(2.5, tuple(back), FINGER_CLOSED, "over the holder", Rt),
                                 W(1.0, tuple(gt - 0.10 * at), FINGER_CLOSED, "tip at the mouth", Rt),
                                 W(2.0, tuple(gt), FINGER_CLOSED, "slide the iron into the holder", Rt),
                                 W(0.6, tuple(gt), FINGER_OPEN, "let go", Rt),
                                 W(1.2, tuple(gt - 0.08 * at), FINGER_OPEN, "back off along the iron", Rt),
                                 W(1.0, tuple(gt - 0.08 * at + (0, 0, 0.08)), FINGER_OPEN, "up", Rt)]})
        yield Segment({"right": 2.0}, min_s=1.5)
