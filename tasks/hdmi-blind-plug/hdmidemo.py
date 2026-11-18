"""hdmidemo: the closed-loop scripted demo that yam_scripted runs for this task. NumPy only.

It reads the simulator state the task's step hook keeps (the plug's pose), as scripted demos may. The right hand takes
the plug off the desk from above, across its 21 mm width, lifts it and pitches its wrist up so the plug points up and
the fingers point at the monitor. It brings the plug under the screen's lower edge, its tip below the panel, to just
under the HDMI port, lines the tip up with the port from the measured pose, and pushes it straight up until the
overmold meets the port face. Then it opens, drops down clear of the plug, pulls back out from under the screen and
goes home. The left arm stays in its ready pose.

``DEMO_CHEAT`` plays something else instead: ``naive`` (no lining up: drives the plug open loop to where the port
should be from the planned grasp, and lets go), ``partial`` (pushes the shell only 4.5 mm in, just past where the
springs take hold, and lets go), ``hold`` (seats the plug but keeps holding it to the end).
"""

from __future__ import annotations

import os

import numpy as np
from inspect_robots_yam.packing import pack

import hdmimodel as M
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.urdf import axis_angle_matrix

OPENING = 2 * abs(FINGER_OPEN)
DEBUG = bool(os.environ.get("PLUG_DEBUG"))
TCP_Z = M.TOP + 0.014  # tool point height for the pick: pad tips 1 mm above the desk


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
            print(f"[hdemo] t {self.ctl.t:6.2f} {self.side} ik error {e * 1000:.1f} mm {er:.3f} rad", flush=True)
        self.q = q
        return np.r_[q, self.g]


class Controller:
    def __init__(self, bases, dt, sim, cheat=""):
        self.dt, self.t, self.cheat, self.sim = float(dt), 0.0, cheat, sim
        self.arms = {"left": Arm("left", bases["left"], self), "right": Arm("right", bases["right"], self)}
        self.script = self.run()
        self.done = False
        self.end_t = None
        self.grasp = None  # (tool point, hand rotation) in the plug frame while the right hand holds it
        self.pp, self.pr = None, None  # planned plug pose
        self.bias_p, self.bias_r = np.zeros(3), np.zeros(3)
        self.correct = False
        self.lift = 0.0  # rad
        self.bias_lim = 0.03  # m; raised while pushing, so the stiff-but-compliant arm can press harder

    def note(self, msg):
        if DEBUG:
            print(f"[hdemo] t {self.t:6.2f} {msg}", flush=True)

    def plug(self):
        p = self.sim["states"]["plug"]
        return p.pos, p.rotation()

    def follow(self):
        if self.grasp is None or self.pp is None:
            return
        if self.correct:
            pos, r = self.plug()
            step = np.clip(0.15 * (self.pp - pos), -0.002, 0.002)
            self.bias_p = np.clip(self.bias_p + step, -self.bias_lim, self.bias_lim)
            self.bias_r = np.clip(self.bias_r + np.clip(0.15 * rotvec(self.pr @ r.T), -0.01, 0.01), -0.3, 0.3)
        rc = exp_rot(self.bias_r)
        r_cmd, p_cmd = rc @ self.pr, self.pp + self.bias_p
        a = self.arms["right"]
        a.pos, a.rot = p_cmd + r_cmd @ self.grasp[0], r_cmd @ self.grasp[1]
        if self.lift:  # pitch the hand up about the tool point: the wrist presses the plug up
            a.rot = M.rot_y(-np.degrees(self.lift)) @ a.rot

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

    def grip(self, g, s):
        a = self.arms["right"]
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
    def take_plug(self):
        pos, r = self.plug()
        grip = pos + r @ np.array([M.GRIP_X, 0.0, 0.0])
        grip[2] = TCP_Z
        hand = rot((0.0, 0.0, -1.0), -r[:, 1])
        yield from self.move("right", grip + [0.0, 0.0, 0.08], hand, 3.0, g=0.45)
        yield from self.move("right", grip, hand, 1.5)
        yield from self.grip(0.0, 0.8)
        yield from self.wait(0.4)
        pos, r = self.plug()
        a = self.arms["right"]
        if self.cheat == "naive":  # the grasp as planned, not as measured
            plan = np.array([M.GRIP_X, 0.0, TCP_Z - pos[2]])
            self.grasp = (plan, r.T @ hand)
        else:
            self.grasp = (r.T @ (a.pos - pos), r.T @ a.rot)
        self.pp, self.pr = pos.copy(), r.copy()
        self.note(f"plug taken, grasp {np.round(self.grasp[0] * 1000, 1)} mm")
        yield from self.carry(pos + [0.0, 0.0, 0.08], r, 1.5)

    def under_port(self):
        stage, rp = M.plugged_pose(-0.030)
        yield from self.carry(stage + [-0.15, 0.0, 0.0], rp, 3.0)  # pitched up, in front of the screen, tip low
        yield from self.carry(stage, rp, 2.5)  # slide in under the panel's lower edge
        below, _ = M.plugged_pose(-0.004)
        if self.cheat == "naive":
            return
        yield from self.carry(below, rp, 1.0)
        self.correct = True
        yield from self.until(lambda: self.sim["off"] < 0.0004 and abs(self.sim["depth"] + 0.004) < 0.0015
                              and self.sim["tilt"] < 1.0, 5.0)
        self.note(f"lined up: depth {self.sim['depth'] * 1000:.1f} off {self.sim['off'] * 1000:.2f} "
                  f"tilt {self.sim['tilt']:.2f}")

    def push(self, depth, s=2.5):
        full, rp = M.plugged_pose(depth + 0.003)
        self.bias_lim = 0.15
        yield from self.carry(full, rp, s)
        for k in range(1, 31):  # then press harder with the wrist
            self.lift = 0.5 * k / 30
            if self.sim["depth"] > depth - 0.0007:
                break
            yield
        yield from self.until(lambda: self.sim["depth"] > depth - 0.0007, 6.0)
        yield from self.wait(0.3)
        self.note(f"pushed: depth {self.sim['depth'] * 1000:.1f} off {self.sim['off'] * 1000:.2f} "
                  f"bias {np.round(self.bias_p * 1000, 1)} mm")

    def unload(self, s=1.0):
        """Seated: stop pressing (the springs hold the plug), so the hand does not spring off it when it opens."""
        self.correct = False
        b0, l0 = self.bias_p.copy(), self.lift
        n = max(1, round(s / self.dt))
        for k in range(1, n + 1):
            f = ik.smoothstep(k / n)
            self.bias_p = b0 + f * (np.array([b0[0], b0[1], 0.0]) - b0)
            self.lift = l0 * (1 - f)
            yield

    def let_go(self):
        yield from self.unload()
        self.correct = False
        self.grasp = None
        a = self.arms["right"]
        yield from self.grip(0.45, 0.6)
        self.lift = 0.0
        tcp = self.sim["hands"]["right"][0]  # where the hand really is (the command was pressed past it)
        yield from self.move("right", tcp - [0.0, 0.0, 0.04], a.rot, 1.0)
        yield from self.move("right", a.pos - [0.17, 0.0, 0.0], a.rot, 1.5)

    def run(self):
        while not self.sim.get("states"):
            yield
        yield from self.take_plug()
        yield from self.under_port()
        if self.cheat == "partial":
            yield from self.push(0.0045)
        else:
            yield from self.push(M.SHELL_LEN)
        if self.cheat == "hold":
            yield from self.wait(3.0)
            self.done = True
            while True:
                yield
        yield from self.let_go()
        home = ik.tcp_pose(np.array(READY, float), self.arms["right"].base)
        yield from self.move("right", home[0], home[1], 2.0)
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
