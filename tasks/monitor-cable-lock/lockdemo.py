"""lockdemo: the closed-loop scripted demo that yam_scripted runs for this task. NumPy only.

It reads the simulator state the task's step hook keeps (the lock's and key's poses, the T-bar's angle), as scripted
demos may. The right hand takes the lock head off the desk from above and behind, across its barrel, lifts it and
brings it square to the slot with the T-bar's crossbar level, lines it up from the measured pose and pushes it flat
onto the monitor's back. Keeping it pressed home, it waits while the left hand comes in from behind along the lock's
axis, pinches the key's bow, turns it a quarter turn and pulls the key out. The left hand lays the key on the desk; the
right hand stops pressing, opens and backs away; both go home.

``DEMO_CHEAT`` plays something else instead: ``onehand`` (the right hand inserts the head and lets go before the key
is turned, so nothing holds it home), ``early`` (the key is turned while the head is still in the air, then the head
is pushed at the slot), ``keyin`` (locks but leaves the key in), ``tug`` (the full demo, but before letting go the right
hand pulls the locked head 2 cm straight back, as the real trial's tug test does).
"""

from __future__ import annotations

import os

import numpy as np
from inspect_robots_yam.packing import pack

import lockmodel as M
from yam_arena import ik
from yam_arena.constants import READY
from yam_arena.urdf import axis_angle_matrix

DEBUG = bool(os.environ.get("LOCK_DEBUG"))
HEAD_APPROACH = np.array([0.55, 0.0, -0.835])  # head frame: the right hand comes from above and behind the barrel
KEY_OPEN, HEAD_OPEN = 0.30, 0.45


def rot(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - z * np.dot(close, z)
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], axis=1)


def rotvec(r: np.ndarray) -> np.ndarray:
    c = np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)
    a = np.arccos(c)
    if a < 1e-9:
        return np.zeros(3)
    return a / (2 * np.sin(a)) * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])


def exp_rot(v) -> np.ndarray:
    a = np.linalg.norm(v)
    return np.eye(3) if a < 1e-12 else axis_angle_matrix(np.asarray(v) / a, a)


class Arm:
    def __init__(self, side, base, ctl):
        self.side, self.base, self.ctl = side, np.asarray(base, float), ctl
        self.q = np.array(READY, float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.g = 1.0
        self._key = None

    def action7(self):
        key = np.r_[self.pos, self.rot.ravel()]
        if self._key is not None and np.abs(key - self._key).max() < 1e-5:
            return np.r_[self.q, self.g]
        self._key = key
        q, e, er = ik.solve(self.pos, ik.matrix_to_quat_xyzw(self.rot), self.q, base_pos=self.base, iters=40,
                            tol=3e-4)
        if e + 0.05 * er > 0.004 and DEBUG:
            print(f"[ldemo] t {self.ctl.t:6.2f} {self.side} ik error {e * 1000:.1f} mm {er:.3f} rad", flush=True)
        self.q = q
        return np.r_[q, self.g]


class Controller:
    def __init__(self, bases, dt, sim, cheat=""):
        self.dt, self.t, self.cheat, self.sim = float(dt), 0.0, cheat, sim
        self.arms = {"left": Arm("left", bases["left"], self), "right": Arm("right", bases["right"], self)}
        self.script = self.run()
        self.done = False
        self.end_t = None
        self.grasp = None  # (tool point, hand rotation) in the head frame while the right hand holds it
        self.pp, self.pr = None, None  # planned head pose
        self.bias_p, self.bias_r = np.zeros(3), np.zeros(3)
        self.correct = False
        self.bias_lim = np.array([0.01, 0.01, 0.01])
        self.gain = 0.15

    def note(self, msg):
        if DEBUG:
            print(f"[ldemo] t {self.t:6.2f} {msg}", flush=True)

    def head(self):
        p = self.sim["states"]["lock"]
        return p.pos, p.rotation()

    def key(self):
        p = self.sim["states"]["key"]
        return p.pos, p.rotation()

    def follow(self):
        if self.grasp is None or self.pp is None:
            return
        if self.correct:
            pos, r = self.head()
            step = np.clip(self.gain * (self.pp - pos), -0.002, 0.002)
            self.bias_p = np.clip(self.bias_p + step, -0.03, self.bias_lim)
            self.bias_r = np.clip(self.bias_r + np.clip(0.15 * rotvec(self.pr @ r.T), -0.01, 0.01), -0.3, 0.3)
        rc = exp_rot(self.bias_r)
        r_cmd, p_cmd = rc @ self.pr, self.pp + self.bias_p
        a = self.arms["right"]
        a.pos, a.rot = p_cmd + r_cmd @ self.grasp[0], r_cmd @ self.grasp[1]

    def wait(self, s):
        for _ in range(max(1, round(s / self.dt))):
            yield

    def until(self, cond, timeout):
        n = 0
        while not cond() and n * self.dt < timeout:
            n += 1
            yield

    def move(self, side, pos, r, s, g=None):
        a = self.arms[side]
        p0, r0, g0 = a.pos.copy(), a.rot.copy(), a.g
        n = max(1, round(s / self.dt))
        for k in range(1, n + 1):
            f = ik.smoothstep(k / n)
            a.pos = p0 + f * (np.asarray(pos) - p0)
            a.rot = ik.slerp_matrix(r0, r, f)
            if g is not None:
                a.g = g0 + f * (g - g0)
            yield

    def grip(self, side, g, s):
        a = self.arms[side]
        g0 = a.g
        n = max(1, round(s / self.dt))
        for k in range(1, n + 1):
            a.g = g0 + ik.smoothstep(k / n) * (g - g0)
            yield

    def carry(self, pos, r, s):
        p0, r0 = self.pp.copy(), self.pr.copy()
        n = max(1, round(s / self.dt))
        for k in range(1, n + 1):
            f = ik.smoothstep(k / n)
            self.pp = p0 + f * (np.asarray(pos) - p0)
            self.pr = ik.slerp_matrix(r0, r, f)
            yield

    # -- the job --
    def take_head(self):
        pos, r = self.head()
        level = np.array([r[0, 0], r[1, 0], 0.0])
        level /= np.linalg.norm(level)
        rl = np.stack([level, np.cross([0.0, 0.0, 1.0], level), [0.0, 0.0, 1.0]], axis=1)  # head frame, level
        grip = pos + r @ np.array([M.HEAD_GRIP_X, 0.0, 0.0]) + [0.0, 0.0, 0.002]
        hand = rot(rl @ HEAD_APPROACH, -rl[:, 1])
        yield from self.move("right", grip - 0.08 * hand[:, 2], hand, 3.0, g=HEAD_OPEN)
        yield from self.move("right", grip, hand, 1.5)
        yield from self.grip("right", 0.0, 0.8)
        yield from self.wait(0.4)
        pos, r = self.head()
        a = self.arms["right"]
        self.grasp = (r.T @ (a.pos - pos), r.T @ a.rot)
        self.pp, self.pr = pos.copy(), r.copy()
        self.note(f"head taken, grasp {np.round(self.grasp[0] * 1000, 1)} mm")
        yield from self.carry(pos + [0.0, 0.0, 0.07], r, 1.5)

    def to_slot(self):
        seat, rs = M.locked_pose()
        yield from self.carry(seat + [-0.04, 0.0, 0.0], rs, 3.0)
        self.correct, self.bias_lim, self.gain = True, np.array([0.03, 0.03, 0.03]), 0.15
        yield from self.line_up()

    def line_up(self):
        seat, rs = M.locked_pose()
        yield from self.carry(seat + [-0.006, 0.0, 0.0], rs, 1.0)
        yield from self.until(lambda: self.sim["off"] < 0.0003 and abs(self.sim["depth"] + 0.006) < 0.001
                              and self.sim["tilt"] < 1.0, 5.0)
        self.note(f"lined up: depth {self.sim['depth'] * 1000:.2f} off {self.sim['off'] * 1000:.2f} "
                  f"tilt {self.sim['tilt']:.2f} roll {np.degrees(rotvec(self.head()[1]))}")

    def push_home(self):
        """Push the head flat onto the monitor, pressing about 6 mm past the face (a few newtons); if the T-bar stops
        on the wall, back off 6 mm, line up again and push again (up to four tries)."""
        seat, rs = M.locked_pose()
        for attempt in range(4):
            self.bias_lim, self.gain = np.array([0.006, 0.03, 0.03]), 0.15
            yield from self.carry(seat - [0.0045, 0.0, 0.0], rs, 1.0)  # the crossbar's tip 0.5 mm off the wall
            yield from self.until(lambda: self.sim["off"] < 0.00025, 3.0)
            yield from self.carry(seat + [0.003, 0.0, 0.0], rs, 2.0)
            yield from self.until(lambda: self.sim["depth"] > -0.0003, 2.0)
            if self.sim["depth"] > -0.0003:
                break
            self.note(f"stopped on the wall: depth {self.sim['depth'] * 1000:.2f} off {self.sim['off'] * 1000:.2f}")
            self.bias_lim, self.gain = np.array([0.03, 0.03, 0.03]), 0.15
            yield from self.line_up()
        yield from self.wait(0.5)
        self.note(f"home: depth {self.sim['depth'] * 1000:.2f} off {self.sim['off'] * 1000:.2f} "
                  f"bias {np.round(self.bias_p * 1000, 1)} mm")

    def turn_key(self):
        kp, kr = self.key()
        grip = kp + kr @ np.array([M.KEY_GRIP_X, 0.0, 0.0])
        hand = rot(kr[:, 0], kr[:, 2])
        yield from self.move("left", grip - 0.10 * hand[:, 2] + [0.0, 0.0, 0.03], hand, 2.5, g=KEY_OPEN)
        yield from self.move("left", grip - 0.10 * hand[:, 2], hand, 1.0)
        yield from self.move("left", grip, hand, 1.5)
        yield from self.grip("left", 0.0, 0.8)
        yield from self.wait(0.3)
        a = self.arms["left"]
        r0, axis = a.rot.copy(), hand[:, 2]
        n = round(2.5 / self.dt)
        for k in range(1, n + 1):  # a quarter turn about the key's axis, a little past the stop
            a.rot = axis_angle_matrix(axis, 1.70 * ik.smoothstep(k / n)) @ r0
            yield
        yield from self.wait(0.5)
        self.note(f"key turned: angle {self.sim['angle']:.1f} depth {self.sim['depth'] * 1000:.2f}")

    def pull_key(self):
        a = self.arms["left"]
        yield from self.move("left", a.pos - 0.035 * a.rot[:, 2], a.rot, 1.5)
        yield from self.wait(0.3)
        self.note(f"key out: {not self.sim['key_in']}")
        yield from self.move("left", a.pos + [-0.10, 0.08, 0.0], a.rot, 1.5)
        yield from self.move("left", np.r_[a.pos[:2], M.TOP + 0.05], a.rot, 1.0)
        yield from self.grip("left", 0.5, 0.6)  # lay the key on the desk
        yield from self.move("left", a.pos + [-0.05, 0.0, 0.05], a.rot, 1.0)

    def let_go(self):
        self.correct = False
        b0 = self.bias_p.copy()
        n = round(1.0 / self.dt)
        for k in range(1, n + 1):  # stop pressing
            self.bias_p = b0 * (1 - ik.smoothstep(k / n))
            yield
        self.grasp = None
        a = self.arms["right"]
        yield from self.grip("right", HEAD_OPEN, 0.6)
        yield from self.move("right", a.pos - 0.06 * a.rot[:, 2], a.rot, 1.2)

    def home(self, side, s=2.0):
        p, r = ik.tcp_pose(np.array(READY, float), self.arms[side].base)
        yield from self.move(side, p, r, s)

    def tug(self):
        """Still holding the locked head, pull it 2 cm straight back (the arm is compliant: a firm pull), then ease off."""
        self.correct = False
        seat, rs = M.locked_pose()
        yield from self.carry(seat - [0.02, 0.0, 0.0], rs, 1.5)
        yield from self.wait(1.0)
        self.note(f"tugged: depth {self.sim['depth'] * 1000:.2f} angle {self.sim['angle']:.1f}")
        yield from self.carry(seat, rs, 1.0)

    def run(self):
        while not self.sim.get("states"):
            yield
        yield from self.take_head()
        if self.cheat == "early":
            yield from self.carry(self.pp + [0.04, 0.05, 0.03], self.pr, 2.0)
            yield from self.turn_key()
            yield from self.move("left", self.arms["left"].pos - [0.06, 0.0, 0.0], self.arms["left"].rot, 1.0,
                                 g=KEY_OPEN)
            yield from self.home("left")
        yield from self.to_slot()
        yield from self.push_home()
        if self.cheat == "onehand":
            yield from self.let_go()
            yield from self.home("right")
        yield from self.turn_key()
        if self.cheat == "keyin":
            yield from self.move("left", self.arms["left"].pos, self.arms["left"].rot, 0.5, g=KEY_OPEN)
            yield from self.move("left", self.arms["left"].pos - [0.08, 0.0, 0.0], self.arms["left"].rot, 1.0)
        else:
            yield from self.pull_key()
        yield from self.home("left")
        if self.grasp is not None:
            if self.cheat == "tug":
                yield from self.tug()
            yield from self.let_go()
            yield from self.home("right")
        self.done = True
        while True:
            yield

    def act(self, observation):
        if not self.sim.get("states"):
            return pack(np.r_[READY, 1.0], np.r_[READY, 1.0]), False
        next(self.script)
        self.follow()
        self.t += self.dt
        if self.done and self.end_t is None:
            self.end_t = self.t
        done = self.end_t is not None and self.t > self.end_t + 2.0
        return pack(self.arms["left"].action7(), self.arms["right"].action7()), done
