"""nozzledemo: the closed-loop scripted demo that yam_scripted runs for this task. NumPy only.

It reads the simulator state the task's physics hook keeps (nozzle pose, lever and clip angles, latch state), as
scripted demos may. Two-handed carry: the left hand holds the handle tube from the left with its lower finger in the
gap above the lever (the lever is not touched), the right hand holds the valve housing from the right. The nozzle is
drawn out of its boot, turned level and pushed down the filler bore past the sprung flap, the planned nozzle pose
being corrected for the arms' sag from the measured one. In the filler the left hand moves its lower finger under the
lever and squeezes it fully; the right hand pushes the hold-open clip's tab back from under the guard; the left hand opens and both hold
off for 3.5 s. Then the left hand squeezes the lever past the clip's notch (the clip drops) and lets it return, both
hands take the carry grips again, and the nozzle goes back down the boot.

``DEMO_CHEAT`` plays a loophole instead: ``hold`` (squeezes the lever by hand for 4 s, never sets the clip, then
holsters), ``shallow`` (pushes the spout only 3 cm in, then squeezes and clips the lever there).
"""

from __future__ import annotations

import os

import numpy as np
from inspect_robots_yam.packing import pack

import nozzlemodel as M
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.urdf import axis_angle_matrix

OPENING = 2 * abs(FINGER_OPEN)
DEBUG = bool(os.environ.get("FUEL_DEBUG"))
GAP_OPEN = 0.045  # fingers around the handle tube only, lower finger above the lever


def rot(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - z * np.dot(close, z)
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], axis=1)


def g_of(opening: float) -> float:
    return float(np.clip(opening / OPENING, 0.0, 1.0))


def rotvec(r: np.ndarray) -> np.ndarray:
    """Rotation vector of r."""
    c = np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)
    a = np.arccos(c)
    if a < 1e-9:
        return np.zeros(3)
    return a / (2 * np.sin(a)) * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])


def exp_rot(v) -> np.ndarray:
    a = np.linalg.norm(v)
    return np.eye(3) if a < 1e-12 else axis_angle_matrix(np.asarray(v) / a, a)


# grips in the nozzle frame: (tool point, gripper rotation)
LEFT_APP = np.array([np.sin(np.radians(30)), -np.cos(np.radians(30)), 0.0])  # from the nozzle's left, 30 deg forward
RIGHT_APP = np.array([np.sin(np.radians(15)), np.cos(np.radians(15)), 0.0])  # from its right, 15 deg forward
LEFT_ROT = rot(LEFT_APP, (0, 0, 1))  # fingers closing along the nozzle's z
RIGHT_ROT = rot(RIGHT_APP, (0, 0, 1))
CLIP_ROT = rot((0, 1, 0), (1, 0, 0))  # right hand, square to the nozzle, fingers closed along its x, for the clip's tab
LOUT, ROUT = -0.07 * LEFT_APP, -0.07 * RIGHT_APP  # stand-off before a grip


class Arm:
    def __init__(self, side, base, ctl):
        self.side, self.base, self.ctl = side, np.asarray(base, float), ctl
        self.q = np.array(READY, float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.g = 1.0

    def action7(self):
        key = np.r_[self.pos, self.rot.ravel()]
        if getattr(self, "_key", None) is not None and np.abs(key - self._key).max() < 1e-5:
            return np.r_[self.q, self.g]  # same target as last step: same joints (IK is the demo's slow part)
        self._key = key
        target = ik.matrix_to_quat_xyzw(self.rot)
        q, e, er = ik.solve(self.pos, target, self.q, base_pos=self.base, iters=40, tol=3e-4)
        if e + 0.05 * er > 0.004 and DEBUG:
            print(f"[fdemo] t {self.ctl.t:6.2f} {self.side} ik error {e * 1000:.1f} mm {er:.3f} rad", flush=True)
        if e + 0.05 * er > 0.03:
            for _ in range(2):
                q2, e2, er2 = ik.solve(self.pos, target, self.q + self.ctl.rng.normal(0.0, 0.3, 6),
                                       base_pos=self.base, iters=150)
                if e2 + 0.05 * er2 < e + 0.05 * er - 0.001 and np.abs(q2 - self.q).max() < 0.6:
                    q, e, er = q2, e2, er2
        self.q = q
        return np.r_[q, self.g]


class Controller:
    def __init__(self, bases, dt, sim, mon, cheat=""):
        self.dt, self.t, self.cheat, self.sim, self.mon = float(dt), 0.0, cheat, sim, mon
        self.rng = np.random.default_rng(0)
        self.arms = {"left": Arm("left", bases["left"], self), "right": Arm("right", bases["right"], self)}
        self.script = self.run()
        self.done = False
        self.end_t = None
        # planned nozzle pose and the sag correction applied to it
        self.np_, self.nr = M.BOOT_N.copy(), M.R_BOOT.copy()
        self.bias_p, self.bias_r = np.zeros(3), np.zeros(3)
        self.grips = {"left": None, "right": None}  # (tool point in N, rotation in N) while a hand follows the nozzle
        self.correct = False

    # -- helpers --
    def note(self, msg):
        if DEBUG:
            print(f"[fdemo] t {self.t:6.2f} {msg}", flush=True)

    def nozzle(self):
        n = self.sim["states"]["nozzle"]
        return n.pos, n.rotation()

    def follow(self):
        """Hands that hold the nozzle track the planned pose plus the sag correction."""
        if self.correct:
            pos, r = self.nozzle()
            # integral correction, rate-limited so a stuck spout does not wind it up
            self.bias_p = np.clip(self.bias_p + np.clip(0.10 * (self.np_ - pos), -0.002, 0.002), -0.12, 0.12)
            self.bias_r = np.clip(self.bias_r + np.clip(0.10 * rotvec(self.nr @ r.T), -0.01, 0.01), -0.40, 0.40)
        rc = exp_rot(self.bias_r)
        r_cmd, p_cmd = rc @ self.nr, self.np_ + self.bias_p
        for side, gr in self.grips.items():
            if gr is not None:
                a = self.arms[side]
                a.pos, a.rot = p_cmd + r_cmd @ gr[0], r_cmd @ gr[1]

    def step(self):
        self.follow()
        self.t += self.dt

    def wait(self, s):
        for _ in range(max(1, round(s / self.dt))):
            yield

    def carry(self, pos, r, s):
        """Move the planned nozzle pose to (pos, r) over s seconds."""
        p0, r0 = self.np_.copy(), self.nr.copy()
        n = max(1, round(s / self.dt))
        for k in range(1, n + 1):
            f = ik.smoothstep(k / n)
            self.np_ = p0 + f * (np.asarray(pos) - p0)
            self.nr = ik.slerp_matrix(r0, r, f)
            yield

    def hand(self, side, point, grip_rot, s, g=None, follow=False):
        """Move one hand to a pose given in the nozzle frame (of the measured nozzle) over s seconds."""
        a = self.arms[side]
        self.grips[side] = None
        p0, r0, g0 = a.pos.copy(), a.rot.copy(), a.g
        n = max(1, round(s / self.dt))
        for k in range(1, n + 1):
            f = ik.smoothstep(k / n)
            pos, r = self.nozzle()
            a.pos = p0 + f * (pos + r @ np.asarray(point) - p0)
            a.rot = ik.slerp_matrix(r0, r @ grip_rot, f)
            if g is not None:
                a.g = g0 + f * (g - g0)
            yield
        if follow:  # keep this grip on the planned nozzle pose from here on
            pos, r = self.nozzle()
            rc = exp_rot(self.bias_r)
            r_cmd, p_cmd = rc @ self.nr, self.np_ + self.bias_p
            self.grips[side] = (r_cmd.T @ (a.pos - p_cmd), r_cmd.T @ a.rot)

    def grip(self, side, g, s):
        a = self.arms[side]
        g0 = a.g
        n = max(1, round(s / self.dt))
        for k in range(1, n + 1):
            a.g = g0 + ik.smoothstep(k / n) * (g - g0)
            yield

    def away(self, side, pos, s):
        a = self.arms[side]
        self.grips[side] = None
        p0 = a.pos.copy()
        n = max(1, round(s / self.dt))
        for k in range(1, n + 1):
            a.pos = p0 + ik.smoothstep(k / n) * (np.asarray(pos) - p0)
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

    # -- the job --
    def take(self, p_left=M.HANDLE_CARRY, g_left=GAP_OPEN):
        """Both carry grips from the side: approach, close, then follow the planned pose."""
        ly, ry = LOUT, ROUT
        yield from self.both(self.hand("left", p_left + ly, LEFT_ROT, 2.5, g=g_of(g_left)),
                             self.hand("right", M.HOUSING_GRIP + ry, RIGHT_ROT, 2.5, g=1.0))
        yield from self.both(self.hand("left", p_left, LEFT_ROT, 1.2), self.hand("right", M.HOUSING_GRIP, RIGHT_ROT, 1.2))
        yield from self.both(self.grip("left", 0.0, 0.6), self.grip("right", 0.0, 0.6))
        yield from self.wait(0.3)
        pos, r = self.nozzle()
        self.np_, self.nr = pos.copy(), r.copy()
        self.bias_p[:], self.bias_r[:] = 0.0, 0.0
        yield from self.both(self.hand("left", p_left, LEFT_ROT, 0.05, follow=True),
                             self.hand("right", M.HOUSING_GRIP, RIGHT_ROT, 0.05, follow=True))
        self.correct = True

    def let_go(self, side, out, g=1.0):
        a = self.arms[side]
        gr = self.grips[side]
        self.grips[side] = None
        yield from self.grip(side, g, 0.5)
        pos, r = self.nozzle()
        yield from self.away(side, a.pos + r @ out, 1.0)
        del gr

    def squeeze(self):
        """Left hand: lower finger out of the gap, under the lever, then close (lever fully up)."""
        pos, r = self.nozzle()
        yield from self.grip("left", g_of(GAP_OPEN + 0.006), 0.3)
        yield from self.hand("left", M.HANDLE_CARRY + LOUT, LEFT_ROT, 1.0)
        yield from self.hand("left", M.HANDLE_SQUEEZE + LOUT, LEFT_ROT, 0.8, g=1.0)
        yield from self.hand("left", M.HANDLE_SQUEEZE, LEFT_ROT, 1.0)
        yield from self.grip("left", 0.0, 1.0)

    def set_clip(self):
        """Right hand (closed) in front of the clip's tab, under the guard, then pushes it back."""
        start = M.TAB_PUSH + np.array([0.016, -0.07, 0.0])
        yield from self.hand("right", start, CLIP_ROT, 1.5, g=0.0)
        yield from self.hand("right", start + [0, 0.07, 0], CLIP_ROT, 1.0)
        yield from self.hand("right", start + [-0.042, 0.07, 0], CLIP_ROT, 1.5)
        yield from self.until(lambda: self.mon.latched, 1.5)
        self.note(f"latched {self.mon.latched} clip {self.sim['clip']:.2f} lever {self.sim['lever']:.2f}")
        yield from self.hand("right", start + [0.0, 0.07, 0], CLIP_ROT, 0.6)
        yield from self.hand("right", start, CLIP_ROT, 0.8)

    def run(self):
        while not self.sim.get("states"):
            yield
        d = M.R_F[:, 0]
        yield from self.take()
        self.note("carry grips on")
        out = M.BOOT_N - 0.21 * M.BOOT_AXIS
        yield from self.carry(out, M.R_BOOT, 2.5)
        yield from self.carry(out + [-0.03, 0.0, 0.06], M.R_BOOT, 1.0)
        pre, r_in = M.nozzle_pose_in_filler(-0.06)
        yield from self.carry(pre - 0.04 * d + [0, 0, 0.03], r_in, 3.5)
        yield from self.carry(pre, r_in, 1.5)
        yield from self.until(lambda: self.sim["off"] < 0.0015 and abs(self.sim["depth"] + 0.06) < 0.004, 6.0)
        self.note(f"aligned: depth {self.sim['depth'] * 1000:.1f} off {self.sim['off'] * 1000:.1f} mm")
        full, _ = M.nozzle_pose_in_filler(0.03 if self.cheat == "shallow" else 0.095)
        yield from self.carry(full, r_in, 4.0)
        yield from self.wait(0.8)
        self.note(f"in: depth {self.sim['depth'] * 1000:.1f} mm flap {self.sim['flap']:.2f}")
        self.grips["left"] = None
        if self.cheat == "hold":
            yield from self.squeeze()
            yield from self.wait(4.0)
            yield from self.grip("left", 1.0, 0.8)
        else:
            yield from self.squeeze()
            self.note(f"squeezed lever {self.sim['lever']:.2f}")
            self.grips["right"] = None
            yield from self.let_go("right", ROUT)
            yield from self.set_clip()
            yield from self.grip("left", g_of(0.088), 0.8)  # hands off the lever: the clip holds it
            yield from self.wait(3.6)
            self.note(f"hold {self.mon.held_s:.2f} s filled {self.mon.filled}")
            yield from self.grip("left", 0.0, 0.8)  # squeeze past the notch: the clip drops
            yield from self.until(lambda: not self.mon.latched, 1.5)
            yield from self.grip("left", g_of(0.088), 0.8)
            yield from self.until(lambda: self.sim["lever"] < M.REST, 2.0)
            self.note(f"released {self.mon.released} lever {self.sim['lever']:.2f}")
        # carry grips again: left lower finger back above the lever
        yield from self.hand("left", M.HANDLE_SQUEEZE + LOUT, LEFT_ROT, 1.0, g=1.0)
        self.correct = False
        yield from self.take()
        yield from self.carry(self.np_ - 0.13 * d, self.nr, 5.0)
        yield from self.carry(self.np_ + [-0.05, 0.0, 0.05], self.nr, 1.2)
        yield from self.carry(out + [-0.03, 0.0, 0.06], M.R_BOOT, 3.5)
        yield from self.carry(out, M.R_BOOT, 1.0)
        yield from self.carry(M.BOOT_N + 0.004 * M.BOOT_AXIS, M.R_BOOT, 3.0)
        yield from self.wait(0.5)
        self.correct = False
        self.note(f"boot depth {self.sim['bdepth'] * 1000:.1f} mm")
        yield from self.both(self.let_go("right", 1.2 * ROUT), self.let_go("left", 1.2 * LOUT, g=g_of(GAP_OPEN + 0.008)))
        yield from self.both(self.away("left", self.arms["left"].pos + [-0.06, 0.04, 0.06], 1.5),
                             self.away("right", self.arms["right"].pos + [-0.06, -0.04, 0.06], 1.5))
        self.done = True
        while True:
            yield

    def act(self, observation):
        if not self.sim.get("states"):
            return pack(np.r_[READY, 1.0], np.r_[READY, 1.0]), False
        import time

        t0 = time.perf_counter()
        next(self.script)
        self.step()
        if self.done and self.end_t is None:
            self.end_t = self.t
        done = self.end_t is not None and self.t > self.end_t + 2.0
        out = pack(self.arms["left"].action7(), self.arms["right"].action7()), done
        self.wall = getattr(self, "wall", 0.0) + time.perf_counter() - t0
        if DEBUG and round(self.t / self.dt) % 10 == 0:
            extra = ""
            if self.correct:
                pos, r = self.nozzle()
                tip = M.R_F.T @ (M.spout_tip(pos, r) - M.LIP)
                extra = (f" bias_p {np.round(self.bias_p * 1000, 1)} bias_r {np.round(self.bias_r, 3)} perr "
                         f"{np.round((self.np_ - pos) * 1000, 1)} rerr {np.round(rotvec(self.nr @ r.T), 3)} tipF "
                         f"{np.round(tip * 1000, 1)}")
            print(f"[fdemo] t {self.t:6.2f} demo wall {self.wall:.1f} s{extra}", flush=True)
        return out
