"""plugdemo: the closed-loop scripted demo that yam_scripted runs for this task. NumPy only.

It reads the simulator state the task's step hook keeps (plug pose, lid angle), as scripted demos may. The left hand
pinches the lid's thumb tab from the left and swings the lid up to 115 degrees, keeping it there. The right hand takes
the plug from the V-block from above, lifts it, turns it about its axis until the key is at 12 o'clock, brings it onto
the socket axis in front of the mouth, lines it up from the measured pose, and pushes it home against the pins. Then
the left hand lets go of the tab (the lid's spring drops it onto the plug, its catch into the notch behind the ridge),
and both hands open and draw back.

``DEMO_CHEAT`` plays a loophole instead: ``naive`` (one hand: pushes the plug at the shut lid, never lifts the lid),
``holdlid`` (seats the plug but keeps the lid held open to the end), ``early`` (lets the lid go before the plug is in:
the lid shuts on the mouth, then the plug is pushed at it).
"""

from __future__ import annotations

import os

import numpy as np
from inspect_robots_yam.packing import pack

import plugmodel as M
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.urdf import axis_angle_matrix

OPENING = 2 * abs(FINGER_OPEN)
DEBUG = bool(os.environ.get("PLUG_DEBUG"))
GRIP_LOCAL = np.array([-0.145, 0.0, 0.0])  # right hand's tool point on the plug body (plug frame)
TAB_LOCAL = np.array([-0.0125, 0.0, -0.073])  # thumb tab centre (lid frame)
LID_UP = np.radians(115.0)


def rot(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - z * np.dot(close, z)
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], axis=1)


def g_of(opening: float) -> float:
    return float(np.clip(opening / OPENING, 0.0, 1.0))


def rotvec(r: np.ndarray) -> np.ndarray:
    c = np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)
    a = np.arccos(c)
    if a < 1e-9:
        return np.zeros(3)
    return a / (2 * np.sin(a)) * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])


def exp_rot(v) -> np.ndarray:
    a = np.linalg.norm(v)
    return np.eye(3) if a < 1e-12 else axis_angle_matrix(np.asarray(v) / a, a)


A20 = np.radians(20.0)
PICK_ROT = rot((np.sin(A20), 0.0, -np.cos(A20)), (0.0, -1.0, 0.0))  # right hand: from above, leaning forward 20 deg
TAB_ROT0 = rot((0.0, -1.0, 0.0), (0.0, 0.0, -1.0))  # left hand: from the left, fingers closing across the tab


def tab_pose(lid: float) -> tuple[np.ndarray, np.ndarray]:
    r = M.rot_y(lid)
    return M.HINGE + r @ TAB_LOCAL, r @ TAB_ROT0


class Arm:
    def __init__(self, side, base, ctl):
        self.side, self.base, self.ctl = side, np.asarray(base, float), ctl
        self.q = np.array(READY, float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.g = 1.0

    def action7(self):
        key = np.r_[self.pos, self.rot.ravel()]
        if getattr(self, "_key", None) is not None and np.abs(key - self._key).max() < 1e-5:
            return np.r_[self.q, self.g]
        self._key = key
        q, e, er = ik.solve(self.pos, ik.matrix_to_quat_xyzw(self.rot), self.q, base_pos=self.base, iters=40,
                            tol=3e-4)
        if e + 0.05 * er > 0.004 and DEBUG:
            print(f"[pdemo] t {self.ctl.t:6.2f} {self.side} ik error {e * 1000:.1f} mm {er:.3f} rad", flush=True)
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
        self.lid_plan = None  # planned lid angle while the left hand holds the tab

    def note(self, msg):
        if DEBUG:
            print(f"[pdemo] t {self.t:6.2f} {msg}", flush=True)

    def plug(self):
        p = self.sim["states"]["plug"]
        return p.pos, p.rotation()

    def follow(self):
        if self.grasp is not None and self.pp is not None:
            if self.correct:
                pos, r = self.plug()
                self.bias_p = np.clip(self.bias_p + np.clip(0.15 * (self.pp - pos), -0.003, 0.003), -0.08, 0.08)
                self.bias_r = np.clip(self.bias_r + np.clip(0.15 * rotvec(self.pr @ r.T), -0.01, 0.01), -0.5, 0.5)
            rc = exp_rot(self.bias_r)
            r_cmd, p_cmd = rc @ self.pr, self.pp + self.bias_p
            a = self.arms["right"]
            a.pos, a.rot = p_cmd + r_cmd @ self.grasp[0], r_cmd @ self.grasp[1]
        if self.lid_plan is not None:
            a = self.arms["left"]
            a.pos, a.rot = tab_pose(self.lid_plan)

    def wait(self, s):
        for _ in range(max(1, round(s / self.dt))):
            yield

    def both(self, *gens):
        gens = list(gens)
        while gens:
            for g in list(gens):
                try:
                    next(g)
                except StopIteration:
                    gens.remove(g)
            if gens:
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

    def swing(self, lid, s):
        l0 = self.lid_plan
        n = max(1, round(s / self.dt))
        for k in range(1, n + 1):
            self.lid_plan = l0 + ik.smoothstep(k / n) * (lid - l0)
            yield

    # -- the job --
    def take_plug(self):
        pos, r = self.plug()
        grip = pos + r @ GRIP_LOCAL
        up = -PICK_ROT[:, 2]
        yield from self.move("right", grip + 0.09 * up, PICK_ROT, 3.0, g=1.0)
        yield from self.move("right", grip + np.array([0.0, 0.0, 0.004]), PICK_ROT, 1.5)
        yield from self.grip("right", 0.0, 0.8)
        yield from self.wait(0.4)
        pos, r = self.plug()
        a = self.arms["right"]
        self.grasp = (r.T @ (a.pos - pos), r.T @ a.rot)
        self.pp, self.pr = pos.copy(), r.copy()
        self.bias_p[:], self.bias_r[:] = 0.0, 0.0
        self.note(f"plug taken, roll {self.sim['roll']:.1f}")
        yield from self.carry(pos + [0.0, 0.0, 0.07], r, 1.5)

    def lift_lid(self):
        p, r = tab_pose(0.0)
        yield from self.move("left", p + np.array([0.0, 0.09, 0.0]), r, 3.0, g=g_of(0.030))
        yield from self.move("left", p, r, 1.5)
        yield from self.grip("left", 0.0, 0.8)
        self.lid_plan = 0.0
        yield from self.swing(LID_UP, 2.5)
        self.note(f"lid up {np.degrees(self.sim['lid']):.1f}")

    def present(self, depth):
        p, _ = M.coupled_pose(depth)
        yield from self.carry(p + np.array([-0.04, -0.03, 0.04]), np.eye(3), 3.0)
        yield from self.carry(p, np.eye(3), 1.5)
        self.correct = True
        yield from self.until(lambda: self.sim["off"] < 0.0015 and abs(self.sim["depth"] - depth) < 0.003
                              and abs(self.sim["roll"]) < 2.0, 5.0)
        self.note(f"aligned depth {self.sim['depth'] * 1000:.1f} off {self.sim['off'] * 1000:.1f} roll "
                  f"{self.sim['roll']:.1f}")

    def push_home(self):
        full, _ = M.coupled_pose(M.DEPTH_FULL + 0.004)
        yield from self.carry(full, np.eye(3), 3.0)
        yield from self.until(lambda: self.sim["depth"] > M.DEPTH_FULL - 0.0015, 4.0)
        yield from self.wait(0.3)
        self.note(f"home: depth {self.sim['depth'] * 1000:.1f} off {self.sim['off'] * 1000:.1f}")

    def release_lid(self):
        self.lid_plan = None
        a = self.arms["left"]
        yield from self.grip("left", 1.0, 0.4)
        yield from self.move("left", a.pos + np.array([0.0, 0.10, 0.03]), a.rot, 1.2)

    def release_plug(self):
        self.correct = False
        self.grasp = None
        a = self.arms["right"]
        yield from self.grip("right", 1.0, 0.6)
        yield from self.move("right", a.pos - 0.10 * a.rot[:, 2], a.rot, 1.2)

    def run(self):
        while not self.sim.get("states"):
            yield
        if self.cheat == "naive":
            yield from self.take_plug()
            yield from self.present(-0.04)
            yield from self.push_home()
            yield from self.release_plug()
        else:
            yield from self.both(self.take_plug(), self.lift_lid())
            if self.cheat == "early":
                yield from self.release_lid()
                yield from self.present(-0.04)
                yield from self.push_home()
                yield from self.release_plug()
            else:
                yield from self.present(-0.04)
                yield from self.push_home()
                if self.cheat == "holdlid":
                    yield from self.release_plug()
                    yield from self.wait(2.0)
                else:
                    yield from self.release_lid()
                    yield from self.wait(0.8)
                    self.note(f"lid down {np.degrees(self.sim['lid']):.1f} (caught below "
                              f"{np.degrees(M.LID_CAUGHT_MAX):.1f})")
                    yield from self.release_plug()
        home = {s: ik.tcp_pose(np.array(READY, float), self.arms[s].base) for s in ("left", "right")}
        yield from self.both(*(self.move(s, home[s][0], home[s][1], 2.0)
                               for s in ("left", "right") if not (self.cheat == "holdlid" and s == "left")))
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
        if DEBUG and round(self.t / self.dt) % 10 == 0 and self.correct:
            pos, r = self.plug()
            print(f"[pdemo] t {self.t:6.2f} bias_p {np.round(self.bias_p * 1000, 1)} bias_r {np.round(self.bias_r, 3)} "
                  f"perr {np.round((self.pp - pos) * 1000, 1)}", flush=True)
        return pack(self.arms["left"].action7(), self.arms["right"].action7()), done
