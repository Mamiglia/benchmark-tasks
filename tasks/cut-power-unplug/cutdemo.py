"""cutdemo: the closed-loop scripted demo that yam_scripted runs for this task. NumPy only.

What it reads from the simulator (a scripted demo's stand-in for the cameras): the position of the plug at the end
of the fan's cord (the task's step hook keeps every plug's and cord's pose; the demo takes the fan cord's plug, which
is what following that cord from the fan to the strip finds), the strip's pose and the plug's lift above its outlet.

1. The hand on the strip's centre side of the fan plug comes down at the free outlet beside it, pads either side
   of the strip's 50 mm width, and grips the strip there.
2. The other hand comes down on the fan plug's body (pads on its 24 mm sides, above the strip face), grips it and
   draws it straight up (hand commanded 85 mm up), past the 16 mm the blades reach into the outlet.
3. It carries the plug forward off the strip, sets it down upright on the desk, lets go and moves away; then the
   holding hand lets go of the strip and moves away.

``DEMO_CHEAT`` plays something else instead, to check the scorer: ``cut`` (one hand takes the scissors at the pivot
and pushes their blades onto the fan's cord), ``cord`` (the strip is held as above, but the other hand grips the fan's
cord where it comes down off the plug and pulls it up), ``wrong`` (pulls the neighbouring plug first, then the fan's as
above), ``onehand`` (pulls the fan plug without holding the strip).
"""

from __future__ import annotations

import math
import os

import numpy as np
from inspect_robots_yam.packing import pack

import cordmodel as M
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

OPENING = 2 * abs(FINGER_OPEN)
DEBUG = bool(os.environ.get("UNPLUG_DEBUG"))
PAD_TIP = 0.013
SERVO_GAIN, SERVO_MAX = 0.15, 0.04
TOP = M.TOP


def rot(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - z * np.dot(close, z)
    x /= -np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], axis=1)


FLIP = np.diag([-1.0, -1.0, 1.0])
DOWN_Y = rot((0, 0, -1), (0, 1, 0))
TILT = math.radians(20.0)


def down(side, close):
    """Pointing down, leaned 20 degrees so the wrist stays over its own arm's side, pads closing along ``close``."""
    s = -1.0 if side == "left" else 1.0
    return rot((0.0, s * math.sin(TILT), -math.cos(TILT)), close)


def g_of(opening: float) -> float:
    return float(np.clip(opening / OPENING, 0.0, 1.0))


class Arm:
    def __init__(self, side, base, ctl):
        self.side, self.base, self.ctl = side, np.asarray(base, float), ctl
        self.q = np.array(READY, float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.g = 1.0
        self.done = False
        self.bias = np.zeros(3)
        self.servo = True

    def measured(self) -> np.ndarray:
        return ik.tcp_pose(self.ctl.joints[self.side][:6], self.base)[0]

    def best_rot(self, at, r):
        """Of a grip and the same grip turned half a turn about the approach, the one this arm reaches better."""
        errs = []
        for cand in (r, r @ FLIP):
            q, e, er = ik.solve(at, ik.matrix_to_quat_xyzw(cand), self.q, base_pos=self.base, iters=150)
            errs.append(e + 0.05 * er + 0.02 * float(np.abs(q - np.array(READY)).max()))
        return r if errs[0] <= errs[1] else r @ FLIP

    def move(self, to, duration, rot=None, g=None):
        start, g0, r0 = self.pos.copy(), self.g, self.rot
        r1 = self.rot if rot is None else rot
        n = max(1, round(duration / self.ctl.dt))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.pos = start + s * (np.asarray(to, float) - start)
            self.rot = ik.slerp_matrix(r0, r1, s)
            if g is not None:
                self.g = g0 + s * (g - g0)
            yield

    def wait(self, duration):
        for _ in range(max(1, round(duration / self.ctl.dt))):
            yield

    def until(self, cond, timeout=10.0):
        n = 0
        while not cond() and n * self.ctl.dt < timeout:
            n += 1
            yield

    def note(self, msg):
        if DEBUG:
            print(f"[udemo] t {self.ctl.t:6.2f} {self.side}: {msg}", flush=True)

    def park(self):
        yield from self.move(self.base + np.array([0.22, 0.0, 0.20]), 2.0)
        self.done = True
        while True:
            yield

    def action7(self):
        if self.ctl.joints is not None and self.servo:
            self.bias = np.clip(self.bias + SERVO_GAIN * (self.pos - self.measured()), -SERVO_MAX, SERVO_MAX)
        goal = self.pos + self.bias
        target = ik.matrix_to_quat_xyzw(self.rot)
        q, e, er = ik.solve(goal, target, self.q, base_pos=self.base, iters=60)
        if e + 0.05 * er > 0.004:
            for _ in range(6):
                q2, e2, er2 = ik.solve(goal, target, self.q + self.ctl.rng.normal(0.0, 0.3, 6),
                                       base_pos=self.base, iters=150)
                if e2 + 0.05 * er2 < e + 0.05 * er - 0.001 and np.abs(q2 - self.q).max() < 0.6:
                    q, e, er = q2, e2, er2
        if DEBUG and e > 0.003 and round(self.ctl.t / self.ctl.dt) % 10 == 0:
            self.note(f"IK miss {e * 1000:.1f} mm {math.degrees(er):.1f} deg at {np.round(self.pos, 4)}")
        self.q = q
        return np.r_[q, self.g]


class Controller:
    def __init__(self, bases, dt, sim, lay, cheat=""):
        self.dt, self.t, self.cheat, self.sim, self.lay = float(dt), 0.0, cheat, sim, lay
        self.flags: dict = {}
        self.rng = np.random.default_rng(0)
        self.arms = {s: Arm(s, bases[s], self) for s in ("left", "right")}
        self.joints = None
        self.end_t = None
        self.scripts = None

    # -- what the robot sees --
    def plug(self, dev):
        """(body centre, lift above its outlet) of a device's plug, as last seen."""
        p = self.sim.get("plugs", {}).get(dev)
        if p is None:
            o = M.plug_origin(self.lay[dev])
            return o + (0, 0, M.PLUG_H / 2), 0.0
        return np.asarray(p[2], float), float(p[0])

    def plan(self):
        c, _ = self.plug("fan")
        y_f = c[1]
        hold_y = M.OUTLET_Y[M.empty_outlet(self.lay)]  # the free outlet beside the fan plug (seen from above)
        holder = "right" if hold_y < y_f else "left"
        puller = "left" if holder == "right" else "right"
        if self.cheat == "onehand":
            holder = None
        scripts = {puller: self.pull_script(self.arms[puller])}
        if holder:
            scripts[holder] = self.hold_script(self.arms[holder], hold_y)
        else:
            other = "left" if puller == "right" else "right"
            scripts[other] = self.arms[other].park()
            self.flags["held"] = True
        if self.cheat == "cut":
            scripts = {"left": self.cut_script(self.arms["left"]), "right": self.arms["right"].park()}
        return scripts

    # -- the holding hand --
    def hold_script(self, a, hold_y):
        yield from a.wait(0.2)

        def spot():
            sp, sr = self.sim["strip"]  # where the strip lies (a camera gives this)
            return np.asarray(sp, float) + np.asarray(sr) @ np.array([0.0, hold_y - M.STRIP_C[1], 0.0]) + \
                np.array([0.0, 0.0, TOP + 0.010 + PAD_TIP - sp[2]])  # pad tips 10 mm above the desk

        p = spot()
        r = a.best_rot(p, down(a.side, (1, 0, 0)))
        yield from a.move(p + (0, 0, 0.10), 2.0, rot=r, g=g_of(0.078))
        yield from a.wait(0.3)
        p = spot()
        yield from a.move(p + (0, 0, 0.05), 0.8)
        yield from a.wait(0.4)
        p = spot()
        yield from a.move(p, 1.5)
        yield from a.move(p, 0.6, g=g_of(0.030))  # squeeze the 50 mm strip
        yield from a.wait(0.3)
        a.note(f"holding the strip at y {hold_y:.4f}")
        self.flags["held"] = True
        yield from a.until(lambda: self.flags.get("placed"), timeout=40.0)
        yield from a.move(a.pos, 0.6, g=g_of(0.085))
        yield from a.move(a.pos + (0, 0, 0.10), 1.5)
        yield from a.park()

    # -- the pulling hand --
    def take_and_pull(self, a, dev, lift=0.085):
        c, _ = self.plug(dev)
        p = c + np.array([0.0, 0.0, 0.002])  # pads on the body's sides, tips 4 mm above the strip face
        r = a.best_rot(p, down(a.side, (0, 1, 0)))
        yield from a.move(p + (0, 0, 0.09), 2.5, rot=r, g=g_of(0.052))
        yield from a.until(lambda: self.flags.get("held"), timeout=20.0)  # the strip is held before the plug is gripped
        for h in (0.05, 0.0):  # look again on the way down (the strip may have shifted)
            yield from a.wait(0.4)
            p = self.plug(dev)[0] + np.array([0.0, 0.0, 0.002])
            yield from a.move(p + (0, 0, h), 0.9)
        yield from a.move(p, 0.6, g=g_of(0.0))
        yield from a.wait(0.4)
        a.note(f"holding the {dev} plug")
        n = round(2.5 / self.dt)
        p0 = a.pos.copy()
        for k in range(1, n + 1):  # straight up, slowly
            a.pos = p0 + (0, 0, lift * ik.smoothstep(k / n))
            yield
        yield from a.until(lambda: self.plug(dev)[1] >= M.OUT + 0.005, timeout=3.0)
        a.note(f"{dev} plug up {self.plug(dev)[1] * 1000:.1f} mm")

    def put_down(self, a, dev):
        c, _ = self.plug(dev)
        ahead = np.array([M.STRIP_C[0] - M.STRIP_SIZE[0] / 2 - M.PLUG_W / 2 - 0.012, c[1], 0.0])
        yield from a.move(np.array([ahead[0], c[1], a.pos[2] + 0.01]), 1.5)
        yield from a.move(np.array([ahead[0], c[1], TOP + M.PLUG_H / 2 + 0.004]), 1.5)
        yield from a.move(a.pos, 0.6, g=g_of(0.052))
        yield from a.move(a.pos + (0, 0, 0.08), 1.2)

    def pull_script(self, a):
        yield from a.wait(0.2)
        if self.cheat == "wrong":
            c, _ = self.plug("fan")
            side = -1.0 if a.side == "right" else 1.0  # the neighbour on the far side from the holding hand
            wrong = min((d for d in M.DEVICES[1:]), key=lambda d: abs(self.plug(d)[0][1] - (c[1] + side * M.PITCH)))
            yield from self.take_and_pull(a, wrong)
            yield from self.put_down(a, wrong)
        if self.cheat == "cord":
            yield from self.cord_pull(a)
        else:
            yield from self.take_and_pull(a, "fan")
            yield from self.put_down(a, "fan")
        self.flags["placed"] = True
        yield from a.park()

    def cord_pull(self, a):
        pts = self.sim["cords"]["fan"]
        q = pts[2]  # where the cord comes down off the plug's top, just behind the strip
        d = pts[3] - pts[1]
        r = a.best_rot(q, down(a.side, np.cross(d, (0, 0, 1))))
        yield from a.move(q + (0, 0, 0.08), 2.5, rot=r, g=g_of(0.030))
        yield from a.until(lambda: self.flags.get("held"), timeout=20.0)
        yield from a.move(q + (0, 0, 0.004), 1.5)
        yield from a.move(a.pos, 0.6, g=g_of(0.0))
        a.note(f"holding the fan cord, opening {self.joints[a.side][6] if self.joints else 0}")
        yield from a.move(a.pos + (0, 0, 0.10), 2.5)  # pull it up by the cord
        yield from a.wait(1.0)
        a.note(f"fan plug up {self.plug('fan')[1] * 1000:.1f} mm")
        yield from a.move(a.pos, 0.6, g=g_of(0.060))
        yield from a.move(a.pos + (0, 0, 0.05), 1.0)

    # -- cheat: the literal cut --
    def cut_script(self, a):
        yield from a.wait(0.2)
        sp, sr = self.sim["scissors"]
        p = sp + np.array([0.0, 0.0, PAD_TIP - 0.002])
        r = a.best_rot(p, down(a.side, (0, 1, 0)))
        yield from a.move(p + (0, 0, 0.08), 2.5, rot=r, g=g_of(0.050))
        yield from a.move(p, 1.5)
        yield from a.move(p, 0.6, g=g_of(0.0))
        a.note("holding the scissors at the pivot")
        yield from a.move(p + (0, 0, 0.01), 0.8)
        yield from a.move(p + (0.045, 0.0, 0.004), 3.0)  # blades onto the cord
        yield from a.wait(0.5)
        yield from a.move(a.pos, 0.6, g=g_of(0.050))
        yield from a.move(a.pos + (0, 0, 0.08), 1.0)
        yield from a.park()

    def act(self, observation):
        from inspect_robots_yam.packing import STATE_KEY

        state = np.asarray(observation.state[STATE_KEY], float).reshape(-1)
        self.joints = {"left": state[:7], "right": state[7:14]}
        if self.scripts is None and self.sim.get("plugs"):
            self.scripts = self.plan()
        if self.scripts is not None:
            for s in ("left", "right"):
                next(self.scripts[s])
        self.t += self.dt
        if all(a.done for a in self.arms.values()) and self.end_t is None:
            self.end_t = self.t
        done = self.end_t is not None and self.t > self.end_t + 2.0
        return pack(self.arms["left"].action7(), self.arms["right"].action7()), done
