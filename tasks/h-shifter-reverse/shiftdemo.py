"""shiftdemo: the closed-loop scripted demo that yam_scripted plays for this task. NumPy only.

The right hand grips the knob from behind and above (leaning 30 degrees back from the lever), fingers closing sideways
across it. Before reverse, at neutral, it turns upright along the lever so that the push-down goes straight down it. From then on the hand's target is the
knob's place for a wanted gate position (u, v, push-down), with the gripper tilted with the lever. Every control step
it reads the lever's joint state (as scripted demos may; a person feels it) and adds an integral correction for the
arm's compliance against the springs. It follows the gate: it moves sideways only once the lever is back within 2 mm
of the neutral channel, and fore-aft only once it is lined up with the wanted slot within 2 mm.

1 (left plane, forward), 2 (straight back through neutral), 3 (to the middle plane, forward), then back to neutral,
push the knob down (to its 6 mm stop), across to the right plane, back into reverse. Each gear is held until the shifter reads it
engaged for 1.2 s. The hand stays on the knob, still pushing down, to the end.

``DEMO_CHEAT`` plays a contrast run instead: ``nopress`` (reverse without pushing the knob down: the box reads 6),
``late`` (into the 6/R slot first, then the push-down), ``diagonal`` (each change in a straight line from gear to gear,
cutting across the gate as an untrained push would).
"""

from __future__ import annotations

import math
import os

import numpy as np
from inspect_robots_yam.packing import pack

import shiftmodel as M
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import grip_rotation

OPENING = 2 * abs(FINGER_OPEN)
DEBUG = bool(os.environ.get("SHIFT_DEBUG"))
GRIP_UP = 0.006  # tool point this far above the knob centre, along the lever
SQUEEZE = 0.036  # finger opening commanded on the 45 mm knob
OVER = 0.004  # commanded past the end of a slot (into its stop)
PUSH = 0.010  # commanded push-down (past the 6 mm stop, so the push keeps a margin while the lever moves)
ALIGN = 0.002
TILT = math.radians(30)  # the hand leans in from behind, as a driver's does; straight down is past the wrist's range
GRIP_DOWN = grip_rotation(0.0)  # straight down along the lever, fingers closing sideways


def hand_rot(tilt: float) -> np.ndarray:
    """Hand orientation relative to the lever, leaning ``tilt`` back from straight down along it."""
    return np.array([[math.cos(tilt), 0, -math.sin(tilt)], [0, 1, 0],
                     [math.sin(tilt), 0, math.cos(tilt)]]) @ GRIP_DOWN


def g_of(opening: float) -> float:
    return float(np.clip(opening / OPENING, 0.0, 1.0))


class Controller:
    def __init__(self, bases, dt, sim, cheat=""):
        import task

        self.task, self.sim, self.cheat = task, sim, cheat
        self.dt, self.t = float(dt), 0.0
        self.base = np.asarray(bases["right"], float)
        self.q = np.array(READY, float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.g = 1.0
        self.left = np.r_[READY, 1.0]
        self.bias = np.zeros(3)
        self.goal = np.zeros(3)  # wanted (u, v, press)
        self.tilt = TILT
        self.servo = False
        self.rng = np.random.default_rng(0)
        self.script = self.run()
        self.end_t = None

    # -- state
    def meas(self) -> np.ndarray:
        return np.array([self.sim.get("u", 0.0), self.sim.get("v", 0.0), self.sim.get("press", 0.0)])

    def note(self, msg):
        if DEBUG:
            m = self.meas() * 1000
            h = self.sim.get("hands", {}).get("right")
            d = "" if h is None else f" cmd-tcp {np.round((self.pos - np.asarray(h[0])) * 1000, 1)} open {h[2] * 1000:.1f}"
            print(f"[sdemo] t {self.t:6.2f} {msg} | u {m[0]:.1f} v {m[1]:.1f} press {m[2]:.1f} gear "
                  f"{self.sim.get('gear')} done {self.sim['seq'].done}{d}", flush=True)

    def knob_pose(self, u, v, press):
        a, b = M.angles_for(u, v)
        p, r = self.task.knob_world(a, b, press)
        return p + r[:, 2] * GRIP_UP, r @ hand_rot(self.tilt)

    # -- motions
    def move(self, to, duration, rot=None, g=None):
        start, g0, r0 = self.pos.copy(), self.g, self.rot
        r1 = self.rot if rot is None else rot
        n = max(1, round(duration / self.dt))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.pos = start + s * (np.asarray(to, float) - start)
            self.rot = ik.slerp_matrix(r0, r1, s)
            if g is not None:
                self.g = g0 + s * (g - g0)
            yield

    def glide(self, du=None, dv=None, dp=None, duration=0.8):
        """Move the wanted gate position to (du, dv, dp) (None: keep), smoothly."""
        start = self.goal.copy()
        end = start.copy()
        for i, x in enumerate((du, dv, dp)):
            if x is not None:
                end[i] = x
        n = max(1, round(duration / self.dt))
        for k in range(1, n + 1):
            self.goal = start + ik.smoothstep(k / n) * (end - start)
            yield

    def until(self, cond, timeout=6.0):
        n = 0
        while not cond() and n * self.dt < timeout:
            n += 1
            yield
        if not cond():
            self.note("timed out")

    def wait(self, duration):
        for _ in range(max(1, round(duration / self.dt))):
            yield

    def hold_gear(self, gear, hold=1.2):
        n = 0
        while n * self.dt < hold:
            n = n + 1 if self.sim.get("gear") == gear else 0
            yield
        self.note(f"held {gear}")

    def upright(self, duration=1.0):
        """At neutral, turn the hand to straight down along the lever, so the push-down goes along it."""
        t0 = self.tilt
        n = max(1, round(duration / self.dt))
        for k in range(1, n + 1):
            self.tilt = t0 * (1.0 - ik.smoothstep(k / n))
            yield

    def to_neutral(self):
        yield from self.glide(du=0.0, duration=0.7)
        yield from self.until(lambda: abs(self.meas()[0]) < M.C - ALIGN)

    def to_plane(self, v):
        yield from self.glide(dv=v, duration=0.7)
        yield from self.until(lambda: abs(self.meas()[1] - v) < ALIGN)

    def into(self, sign, gear, duration=0.9):
        yield from self.glide(du=sign * (M.U + OVER), duration=duration)
        yield from self.hold_gear(gear)

    def shift(self, gear):
        plane, sign = {"1": (M.P, 1), "2": (M.P, -1), "3": (0.0, 1), "R": (-M.P, -1)}[gear]
        if self.cheat == "diagonal":
            yield from self.glide(du=sign * (M.U + OVER), dv=plane, duration=1.4)
            yield from self.until(lambda: self.sim.get("gear") == gear, timeout=3.0)
            yield from self.hold_gear(gear)
            return
        if abs(self.meas()[1] - plane) > ALIGN:
            yield from self.to_neutral()
            yield from self.to_plane(plane)
        yield from self.into(sign, gear)

    def run(self):
        while not self.sim.get("hands"):
            yield
        p, r = self.knob_pose(0.0, 0.0, 0.0)
        yield from self.move(p - 0.08 * r[:, 2], 2.5, rot=r, g=g_of(0.075))
        yield from self.move(p, 1.2)
        yield from self.move(p, 0.6, g=g_of(SQUEEZE))
        yield from self.wait(0.2)
        self.note("on the knob")
        self.servo = True
        for gear in ("1", "2", "3"):
            yield from self.shift(gear)
        if self.cheat in ("nopress", "late"):
            yield from self.to_neutral()
            yield from self.to_plane(-M.P)
            yield from self.glide(du=-(M.U + OVER), duration=0.9)
            yield from self.wait(1.5)
            if self.cheat == "late":
                yield from self.glide(dp=-PUSH, duration=0.5)
                yield from self.wait(1.5)
        elif self.cheat == "diagonal":
            yield from self.glide(dp=-PUSH, duration=0.5)
            yield from self.shift("R")
        else:
            yield from self.to_neutral()
            yield from self.glide(dv=0.0, duration=0.4)
            yield from self.upright()
            yield from self.glide(dp=-PUSH, duration=0.5)
            yield from self.until(lambda: -self.meas()[2] > M.PRESS_R + 0.0003, timeout=8.0)
            self.note("knob down")
            yield from self.to_plane(-M.P)
            yield from self.into(-1, "R", duration=1.6)
        # the hand stays on the knob, still pushing it down, so the shifter reads R whether or not its lockout
        # holds the knob down by itself
        yield from self.wait(1.0)
        self.note("done")
        self.end_t = self.t
        while True:
            yield

    def servo_step(self):
        meas = self.meas()
        err = self.goal - meas
        if abs(self.goal[0]) > M.U and abs(meas[0]) > M.U - 0.001:  # at a slot's end stop: nothing to integrate
            err[0] = 0.0
        self.bias = np.clip(self.bias + np.array([0.12, 0.12, 0.15]) * err, [-0.02, -0.012, -0.025],
                            [0.02, 0.012, 0.003])
        cmd = self.goal + self.bias
        # never command the hand across a wall the gate will stop it at: keep the command inside the gate's reach
        cmd[1] = float(np.clip(cmd[1], -(M.P + M.C + 0.006), M.P + M.C + 0.006))
        self.pos, self.rot = self.knob_pose(*cmd)
        self.k = getattr(self, "k", 0) + 1
        if self.k % 20 == 0:
            self.note(f"goal {np.round(self.goal * 1000, 1)} bias {np.round(self.bias * 1000, 1)}")

    def act(self, observation):
        next(self.script)
        if self.servo:
            self.servo_step()
        self.t += self.dt
        target = ik.matrix_to_quat_xyzw(self.rot)
        q, e, er = ik.solve(self.pos, target, self.q, base_pos=self.base, iters=60)
        if e + 0.05 * er > 0.004:
            for _ in range(6):
                q2, e2, er2 = ik.solve(self.pos, target, self.q + self.rng.normal(0.0, 0.3, 6), base_pos=self.base,
                                       iters=150)
                if e2 + 0.05 * er2 < e + 0.05 * er - 0.001 and np.abs(q2 - self.q).max() < 0.6:
                    q, e, er = q2, e2, er2
        self.q = q
        done = self.end_t is not None and self.t > self.end_t + 1.5
        return pack(self.left, np.r_[q, self.g]), done
