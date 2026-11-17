"""binderdemo: the closed-loop scripted demo that yam_scripted runs for this task. NumPy only.

It reads the state the task's hooks keep (lever and arch angles, the stack's pose), as scripted demos may.
Left hand: pinches the lever's finger plate across its width and throws the lever up past its centre; the rings open.
Both hands: pinch the stack's punched strip at its two overhanging ends, lift it off the reams, carry it over the
file, line both holes up over both posts (servoing on the measured hole positions), lower it 8 mm onto the posts and
let go; the sheets slide down the posts. Left hand: pinches the open lever again and swings it back to 15 degrees,
past its centre, and lets go; the spring snaps both rings shut. Both arms back away.

``DEMO_CHEAT`` plays a loophole instead: ``naive`` (drops the stack onto the shut rings without touching the lever),
``open`` (files the stack but leaves the rings open), ``force`` (pushes the rings' moving halves open by hand, not
with the lever, then files the stack and pushes the halves shut).
"""

from __future__ import annotations

import math
import os

import numpy as np
from bindermodel import (ARCH_PTS, HOLES, LEVER_GRIP, PIVOT, POST_TOP, POST_X, POST_Y, TOP, arch_point, lever_grip,
                         lever_point, rot_y)
from inspect_robots_yam.packing import pack

from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

OPENING = 2 * abs(FINGER_OPEN)
DEBUG = bool(os.environ.get("BINDER_DEBUG"))
Z = np.array([0.0, 0.0, 1.0])
Y = np.array([0.0, 1.0, 0.0])
X = np.array([1.0, 0.0, 0.0])
PUSH_AP = np.array([0.5, 0.0, -0.866])  # cheat: pushing a ring half with closed fingertips
GRIP_Y = 0.137  # stack: tool points this far from the stack's centre line, on its punched strip
OPEN_TO = math.radians(65.0)
PITCH = math.radians(12.0)  # stack: approach tilted down 12 degrees, keeping the wrist off its 90-degree limit
CLOSE_TO = math.radians(15.0)
INSERT = 0.008  # posts into the holes before letting go


def rot(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - z * np.dot(close, z)
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], axis=1)


def g_of(opening: float) -> float:
    return float(np.clip(opening / OPENING, 0.0, 1.0))


def lever_approach(theta: float) -> np.ndarray:
    """Along the lever from beyond its tip, tilted 25 degrees further down when the lever lies flat."""
    phi = theta + math.radians(25.0) * max(0.0, 1.0 - theta / math.radians(115.0))
    return np.array([math.cos(phi), 0.0, -math.sin(phi)])


class Arm:
    def __init__(self, side, base, ctl):
        self.side, self.base, self.ctl = side, np.asarray(base, float), ctl
        self.q = np.array(READY, float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.g = 1.0
        self.done = False

    def move(self, to, duration, rot=None, g=None, exact=False):
        start, g0, r0 = self.pos.copy(), self.g, self.rot
        r1 = self.rot if rot is None else (rot if exact else self.pick_roll(rot))
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

    def park(self):
        yield from self.move(self.pos + np.array([0.0, 0.0, 0.04]), 1.0)
        self.done = True
        while True:
            yield

    def note(self, msg):
        if DEBUG:
            print(f"[bdemo] t {self.ctl.t:6.2f} {self.side}: {msg}", flush=True)

    def action7(self):
        target = ik.matrix_to_quat_xyzw(self.rot)
        q, e, er = ik.solve(self.pos, target, self.q, base_pos=self.base, iters=60)
        if e + 0.05 * er > 0.004:
            for k in range(10):
                seed = (np.array(READY, float) if k == 0 else self.q) + self.ctl.rng.normal(0.0, 0.3 if k < 5 else 0.8, 6)
                q2, e2, er2 = ik.solve(self.pos, target, seed, base_pos=self.base, iters=150)
                lost = e + 0.05 * er > 0.03  # the local solution is badly off: accept a jump to another branch
                if e2 + 0.05 * er2 < e + 0.05 * er - 0.001 and (lost or np.abs(q2 - self.q).max() < 0.6):
                    q, e, er = q2, e2, er2
            if np.abs(q - self.q).max() > 0.6:
                self.note(f"IK branch change {np.round(q - self.q, 2).tolist()}")
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

    def lever(self) -> float:
        return float(self.sim.get("lever", 0.0))

    def stack(self):
        return np.asarray(self.sim["stack_pos"], float), np.asarray(self.sim["stack_R"], float)

    def holes(self) -> np.ndarray:
        pos, R = self.stack()
        return np.array([pos + R @ np.array(h) for h in HOLES])

    # -- the lever (left hand) --
    def lever_pose(self, theta):
        return lever_grip(theta), rot(lever_approach(theta), Y)

    def servo_lever(self, th0, th1, duration, done_fn, timeout):
        a, bias, k, steps = self.left, 0.0, 0, max(1, round(duration / self.dt))
        while True:
            k += 1
            goal = th0 + ik.smoothstep(min(1.0, k / steps)) * (th1 - th0)
            if k > 1:
                bias = float(np.clip(bias + 0.2 * (goal - self.lever()), -math.radians(20), math.radians(20)))
            a.pos, a.rot = self.lever_pose(goal + bias)
            a.rot = a.pick_roll(a.rot)
            if (k >= steps and done_fn()) or k > steps + round(timeout / self.dt):
                return
            yield

    def grab_lever(self):
        a = self.left
        p, r = self.lever_pose(self.lever())
        yield from a.move(p - 0.07 * r[:, 2] + np.array([0.0, 0.0, 0.03]), 2.0, rot=r, g=g_of(0.032))
        yield from a.move(p - 0.07 * r[:, 2], 0.8)
        yield from a.move(p, 1.0)
        yield from a.move(p, 0.5, g=g_of(0.008))
        a.note(f"on the lever at {math.degrees(self.lever()):.1f}")

    def let_go_lever(self):
        a = self.left
        yield from a.move(a.pos, 0.4, g=g_of(0.032))
        yield from a.move(a.pos - 0.07 * a.rot[:, 2], 1.0)

    def push_arch(self, to_deg):
        """Cheat: push the rings' moving halves by hand (left tool point at the +y ring's top bend)."""
        a = self.left
        y = POST_Y[0]
        th = math.radians(to_deg)
        start = arch_point(ARCH_PTS[2], 0.0, y) + np.array([-0.012, 0.0, 0.0])
        r = rot(PUSH_AP, X)
        yield from a.move(start - 0.05 * r[:, 2], 2.0, rot=r, g=0.0, exact=True)
        yield from a.move(start, 1.0)
        for k in range(1, 61):
            ang = th * k / 60
            yield from a.move(arch_point(ARCH_PTS[2], ang, y) + rot_y(ang) @ np.array([-0.012, 0.0, 0.0]), 0.05)
        yield from a.move(a.pos - 0.05 * a.rot[:, 2], 1.0)

    def left_script(self):
        a = self.left
        if self.cheat == "force":
            yield from self.push_arch(45.0)
        elif self.cheat != "naive":
            yield from self.grab_lever()
            # past its centre (40 degrees) the spring throws the lever the rest of the way open
            yield from self.servo_lever(self.lever(), OPEN_TO, 2.0, lambda: self.lever() > math.radians(60), 2.0)
            a.note(f"lever open {math.degrees(self.lever()):.1f}")
            yield from self.let_go_lever()
        self.flags["opened"] = True
        yield from self.carry(a, +1)
        if self.cheat == "force":
            yield from a.move(np.array([0.30, 0.16, TOP + 0.15]), 1.0)
            yield from self.push_arch(-45.0)
        elif self.cheat not in ("naive", "open"):
            yield from a.move(np.array([0.36, 0.20, TOP + 0.15]), 1.2)
            yield from self.grab_lever()
            yield from self.servo_lever(self.lever(), CLOSE_TO, 2.0, lambda: self.lever() < math.radians(25), 2.0)
            a.note(f"lever let go at {math.degrees(self.lever()):.1f}")
            yield from self.let_go_lever()
            yield from a.wait(0.5)
            a.note(f"lever now {math.degrees(self.lever()):.1f}")
        yield from a.park()

    def right_script(self):
        a = self.right
        yield from a.until(lambda: self.flags.get("opened"), timeout=30.0)
        yield from self.carry(a, -1)
        yield from a.park()

    # -- the stack (both hands; each runs this with its own side) --
    def grip_frame(self, side):
        return rot(np.array([0.0, -side * math.cos(PITCH), -math.sin(PITCH)]), side * Z)

    def carry(self, a, side):
        pos, R = self.stack()
        local = np.array([0.0, side * GRIP_Y, 0.0])
        g0 = pos + R @ local
        r = self.grip_frame(side)
        out = np.array([0.0, side * 0.07, 0.0])
        yield from a.move(g0 + out + np.array([0.0, 0.0, 0.04]), 2.5, rot=r, g=g_of(0.022), exact=True)
        yield from a.move(g0 + out, 1.0)
        yield from a.move(g0, 1.0)
        yield from a.move(g0, 0.6, g=0.0 if side == 1 else g_of(0.004))
        self.flags[f"held{side}"] = True
        yield from a.until(lambda: self.flags.get("held1") and self.flags.get("held-1"), timeout=10.0)
        a.note("holding the stack")
        lift = g0 + np.array([0.0, 0.0, 0.03])
        yield from a.move(lift, 1.0)
        # carry the holes over the posts, 8 mm above their tips (both hands move by the same planned shift)
        posts = np.array([[POST_X, POST_Y[0]], [POST_X, POST_Y[1]]])
        hz0 = float(np.mean(self.holes()[:, 2]))
        shift = np.r_[np.mean(posts - self.holes()[:, :2], axis=0), POST_TOP + 0.008 - hz0]
        yield from a.move(lift + shift, 4.0)
        base = a.pos.copy()
        self.flags.setdefault("corr", {1: np.zeros(3), -1: np.zeros(3)})
        # servo both holes onto the post axes (left hand measures, each hand gets its own correction, so the stack
        # shifts and turns), lowering whenever both holes are within 1 mm of their posts
        for k in range(round(10.0 / self.dt)):
            if side == 1:
                h = self.holes()
                e = posts - h[:, :2]  # per hole (x, y) error
                hz = float(np.mean(h[:, 2]))
                mean = e.mean(axis=0)
                turn = (e[0, 0] - e[1, 0]) / (POST_Y[0] - POST_Y[1])  # x error per metre of y
                corr = self.flags["corr"]
                for sd in (1, -1):
                    want = np.r_[mean[0] + turn * sd * GRIP_Y, mean[1]]
                    corr[sd][:2] = np.clip(corr[sd][:2] + 0.06 * np.clip(want, -0.004, 0.004), -0.015, 0.015)
                    if max(np.linalg.norm(e, axis=1)) < 0.001:
                        corr[sd][2] -= 0.0004
                if DEBUG and k % 10 == 0:
                    hands = self.sim.get("hands", {})
                    lag = {sd: np.round((arm.pos - np.asarray(hands[sd][0])) * 1000, 1).tolist()
                           for sd, arm in (("left", self.left), ("right", self.right)) if sd in hands}
                    a.note(f"align err {np.round(e * 1000, 2).tolist()} mm, holes {(hz - POST_TOP) * 1000:.1f} mm "
                           f"from tip, corr {np.round(corr[1] * 1000, 1)} {np.round(corr[-1] * 1000, 1)} cmd-tcp {lag}")
                hist = self.flags.setdefault("hz", [])
                hist.append(hz)
                aligned = max(np.linalg.norm(e, axis=1)) < 0.0007
                stalled = corr[1][2] < -0.006 and len(hist) > 20 and hist[-20] - hz < 0.001
                if hz < POST_TOP - INSERT or (aligned and stalled):
                    a.note(f"lowered to {(hz - POST_TOP) * 1000:.1f} mm from the tips, err {np.round(e * 1000, 2).tolist()}")
                    self.flags["down"] = True
            a.pos = base + self.flags["corr"][side]
            if self.flags.get("down"):
                break
            yield
        a.note(f"let go: thr {self.sim.get('n_threaded')}")
        self.flags[f"placed{side}"] = True
        yield from a.until(lambda: self.flags.get("placed1") and self.flags.get("placed-1"), timeout=5.0)
        yield from a.move(a.pos, 0.5, g=g_of(0.022))
        yield from a.move(a.pos + np.array([0.0, side * 0.07, 0.0]), 1.0)
        yield from a.move(a.pos + np.array([0.0, 0.0, 0.06]), 0.8)
        yield from a.wait(1.0)
        a.note(f"stack released: thr {self.sim.get('n_threaded')} hinges {np.round(np.degrees(self.sim['hinges']), 0)}")

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
