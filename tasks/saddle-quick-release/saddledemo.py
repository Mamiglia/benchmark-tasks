"""saddledemo: the closed-loop scripted demo that yam_scripted runs for this task. NumPy only.

It reads the joint state the task's physics hook keeps (post rise and turn, lever angle), as scripted demos may.
Right hand: pinches the saddle's right edge (top and bottom of the shell) and holds it. Left hand: pinches the
quick-release lever's paddle across its width and swings it open past the cam; the post is now loose and would sink.
Right hand: raises the saddle along the seat tube to 40 mm and turns it straight, servoing the measured rise and
turn, and holds it there. Left hand: pushes the lever shut against the cam's ~30 N, through its dead point, lets go.
Right hand lets go. Both arms back away.

``DEMO_CHEAT`` plays a loophole instead: ``naive`` (raises the saddle without touching the lever), ``short`` (closes
the lever only to 20 degrees, short of its dead point, and lets go).
"""

from __future__ import annotations

import math
import os

import numpy as np
from inspect_robots_yam.packing import pack
from qrmodel import A, PT, TWIST0_DEG, Y, lever_tangent, paddle, post_point, rot_axis

from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

OPENING = 2 * abs(FINGER_OPEN)
DEBUG = bool(os.environ.get("SADDLE_DEBUG"))
TOP = 0.7475
EDGE = PT + np.array([-0.015, -0.050, 0.02])  # right hand: on the saddle's right edge, mid-shell
EDGE_APPROACH = np.array([0.4, 1.0, 0.0]) / np.linalg.norm([0.4, 1.0, 0.0])
GOAL_RISE, GOAL_TURN = 0.040, 0.0
OPEN_DEG = 105.0


def rot(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - z * np.dot(close, z)
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], axis=1)


def g_of(opening: float) -> float:
    return float(np.clip(opening / OPENING, 0.0, 1.0))


class Arm:
    def __init__(self, side, base, ctl):
        self.side, self.base, self.ctl = side, np.asarray(base, float), ctl
        self.q = np.array(READY, float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.g = 1.0
        self.done = False

    def move(self, to, duration, rot=None, g=None):
        start, g0, r0 = self.pos.copy(), self.g, self.rot
        r1 = self.rot if rot is None else self.pick_roll(rot)
        n = max(1, round(duration / self.ctl.dt))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.pos = start + s * (np.asarray(to, float) - start)
            self.rot = ik.slerp_matrix(r0, r1, s)
            if g is not None:
                self.g = g0 + s * (g - g0)
            yield

    def pick_roll(self, r):
        flip = r @ np.diag([-1.0, -1.0, 1.0])
        return r if np.trace(self.rot.T @ r) >= np.trace(self.rot.T @ flip) else flip

    def wait(self, duration):
        for _ in range(max(1, round(duration / self.ctl.dt))):
            yield

    def until(self, cond, timeout=8.0):
        n = 0
        while not cond() and n * self.ctl.dt < timeout:
            n += 1
            yield

    def note(self, msg):
        if DEBUG:
            h = self.ctl.sim.get("hands", {}).get(self.side)
            extra = "" if h is None else (f" | cmd-tcp {np.round((self.pos - np.asarray(h[0])) * 1000, 1)} "
                                          f"open {h[2] * 1000:.1f}")
            print(f"[sdemo] t {self.ctl.t:6.2f} {self.side}: {msg}{extra}", flush=True)

    def action7(self):
        target = ik.matrix_to_quat_xyzw(self.rot)
        q, e, er = ik.solve(self.pos, target, self.q, base_pos=self.base, iters=60)
        if e + 0.05 * er > 0.004:
            for _ in range(6):
                q2, e2, er2 = ik.solve(self.pos, target, self.q + self.ctl.rng.normal(0.0, 0.3, 6),
                                       base_pos=self.base, iters=150)
                if e2 + 0.05 * er2 < e + 0.05 * er - 0.001 and np.abs(q2 - self.q).max() < 0.6:
                    q, e, er = q2, e2, er2
        self.q = q
        return np.r_[q, self.g]


class Controller:
    def __init__(self, bases, dt, sim, cheat=""):
        self.dt, self.t, self.cheat, self.sim = float(dt), 0.0, cheat, sim
        self.flags: dict = {}
        self.rng = np.random.default_rng(0)
        self.left = Arm("left", bases["left"], self)
        self.right = Arm("right", bases["right"], self)
        self.left.script = self.left_script()
        self.right.script = self.right_script()
        self.end_t = None

    def slide(self) -> float:
        return float(self.sim.get("slide", 0.0))

    def turn(self) -> float:
        return float(self.sim.get("turn", 0.0))

    def theta(self) -> float:
        return float(self.sim.get("theta", 0.0))

    # -- right hand: the saddle --
    def edge_pose(self, slide, turn):
        r = rot_axis(A, turn)
        return post_point(EDGE, slide, turn), rot(r @ EDGE_APPROACH, r @ A)

    def right_script(self):
        a = self.right
        p, r = self.edge_pose(self.slide(), self.turn())
        yield from a.move(p - 0.07 * (r[:, 2]), 3.0, rot=r, g=g_of(0.050))
        yield from a.move(p, 1.2)
        a.note("at the edge")
        yield from a.move(p, 0.6, g=g_of(0.022))  # shell is 30 mm thick: ~8 mm squeeze
        a.note(f"holding the saddle: rise {self.slide() * 1000:.1f} turn {math.degrees(self.turn()):.1f}")
        self.flags["holding"] = True
        if self.cheat != "naive":
            yield from a.until(lambda: self.flags.get("open"), timeout=20.0)
        s0, t0 = 0.0, math.radians(TWIST0_DEG)  # where the hand was put: the post has sagged below it since
        bias = np.zeros(2)
        steps = round(3.0 / self.dt)
        k = 0
        while True:
            k += 1
            s = ik.smoothstep(min(1.0, k / steps))
            goal = np.array([s0 + s * (GOAL_RISE - s0), t0 + s * (GOAL_TURN - t0)])
            meas = np.array([self.slide(), self.turn()])
            if k > 1:
                bias += np.array([0.1, 0.2]) * (goal - meas)
                bias = np.clip(bias, [-0.03, -0.25], [0.08, 0.25])
            cmd = goal + bias
            a.pos, r = self.edge_pose(*cmd)
            a.rot = a.pick_roll(r)
            err = goal - meas
            if DEBUG and k % 20 == 0:
                tcp = np.asarray(self.sim["hands"]["right"][0])
                a.note(f"servo goal {np.round(goal * [1000, 57.3], 1)} meas {np.round(meas * [1000, 57.3], 1)} "
                       f"bias {np.round(bias * [1000, 57.3], 1)} cmd-tcp {np.round((a.pos - tcp) * 1000, 1)} "
                       f"opening {self.sim['hands']['right'][2] * 1000:.1f}")
            if k >= steps and abs(err[0]) < 0.0015 and abs(err[1]) < math.radians(1.0):
                self.flags["positioned"] = True
            if self.flags.get("closed") or (self.cheat == "naive" and k > steps + 40):
                break
            if k > steps + 400:
                a.note("gave up waiting")
                break
            yield
        a.note(f"release: rise {self.slide() * 1000:.1f} turn {math.degrees(self.turn()):.2f}")
        yield from a.move(a.pos, 0.6, g=g_of(0.050))
        yield from a.move(a.pos - 0.08 * a.rot[:, 2], 1.2)
        yield from a.move(np.array([0.15, -0.28, TOP + 0.30]), 1.5)
        a.done = True
        while True:
            yield

    # -- left hand: the lever --
    def lever_pose(self, theta):
        return paddle(theta) + np.array([0.0, 0.004, 0.0]), rot(-Y, lever_tangent(theta))

    def servo_lever(self, goal_deg_fn, steps, done_fn, timeout_steps):
        a = self.left
        bias = 0.0
        k = 0
        while True:
            k += 1
            goal = math.radians(goal_deg_fn(min(1.0, k / steps)))
            if k > 1:
                bias = float(np.clip(bias + 0.25 * (goal - self.theta()), -math.radians(25), math.radians(25)))
            a.pos, r = self.lever_pose(goal + bias)
            a.rot = a.pick_roll(r)
            if (k >= steps and done_fn()) or k > steps + timeout_steps:
                return
            yield

    def left_script(self):
        a = self.left
        if self.cheat == "naive":
            yield from a.move(np.array([0.15, 0.28, TOP + 0.30]), 2.0)
            a.done = True
            while True:
                yield
        yield from a.until(lambda: self.flags.get("holding"), timeout=20.0)
        p, r = self.lever_pose(self.theta())
        yield from a.move(p + np.array([0.0, 0.07, 0.0]), 2.0, rot=r, g=g_of(0.030))
        yield from a.move(p, 1.2)
        yield from a.move(p, 0.5, g=g_of(0.010))
        a.note(f"on the lever {math.degrees(self.theta()):.1f}")
        th0 = math.degrees(self.theta())
        yield from self.servo_lever(lambda s: th0 + ik.smoothstep(s) * (OPEN_DEG - th0), round(2.0 / self.dt),
                                    lambda: math.degrees(self.theta()) > 90, 60)
        a.note(f"lever open {math.degrees(self.theta()):.1f}")
        self.flags["open"] = True
        yield from self.servo_lever(lambda s: OPEN_DEG, 1, lambda: self.flags.get("positioned"), round(25 / self.dt))
        a.note(f"saddle positioned; closing (rise {self.slide() * 1000:.1f}, turn {math.degrees(self.turn()):.2f})")
        end = 20.0 if self.cheat == "short" else -3.0
        yield from self.servo_lever(lambda s: OPEN_DEG + ik.smoothstep(s) * (end - OPEN_DEG), round(2.5 / self.dt),
                                    lambda: (math.degrees(self.theta()) < 4.0) or self.cheat == "short",
                                    round(4 / self.dt))
        yield from a.wait(0.4)
        a.note(f"lever closed {math.degrees(self.theta()):.1f}")
        yield from a.move(a.pos, 0.4, g=g_of(0.030))
        yield from a.move(a.pos + np.array([0.0, 0.08, 0.0]), 1.0)
        self.flags["closed"] = True
        yield from a.move(np.array([0.15, 0.28, TOP + 0.30]), 1.5)
        a.done = True
        while True:
            yield

    def act(self, observation):
        if "hands" not in self.sim or not self.sim["hands"]:
            return pack(np.r_[READY, 1.0], np.r_[READY, 1.0]), False
        next(self.left.script)
        next(self.right.script)
        self.t += self.dt
        if self.left.done and self.right.done and self.end_t is None:
            self.end_t = self.t
        done = self.end_t is not None and self.t > self.end_t + 2.0
        return pack(self.left.action7(), self.right.action7()), done
