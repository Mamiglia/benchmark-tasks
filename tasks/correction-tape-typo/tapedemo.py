"""Closed-loop scripted demo for correction-tape-typo, played by ``yam_scripted`` (see ``tapekit._patch_scripted``).

It sees the observation: the arms' measured joint positions (forward kinematics gives the tool point). It reads the
simulator once: the dispenser's pose right after lifting it, to know how it sits in the fingers (a person looks at
the tool in their hand). It never reads the contact or the tape. It knows the fixed layout (the README's setup).

The right arm does the stroke; the left arm turns out of the top camera's view.

1. Grip the dispenser across its 33 mm width, from above and behind (pitched 45 degrees), 55 mm back from the lip,
   and lift it.
2. Tilt it 45 degrees, lip down and away from the arm (the case's width along the line, the way a sideways-pull
   dispenser is held), and hold the lip 3 mm above the paper, ``START_MM`` before the word's first letter, centred
   on its height.
3. From the hover on, correct the tool point in x and y from the measured joints (forward kinematics, the measured
   wrist tilt and the in-hand pose give where the lip is); lower the lip to ``PRESS_MM`` below the paper (the
   arm's joints are springs: the lip rests on the paper with about 2 N).
4. One stroke along the line (left to right as printed) at ``SPEED`` to ``END_MM`` past the word's last letter;
   hold there while the feedback brings the lagging lip on to the end (forward only); then lift straight off from
   where the lip is (the tape parts at the lip) and turn away.
"""

from __future__ import annotations

import math
import os

import numpy as np
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY

from tapemodel import LAYOUT

ARMS = ("left", "right")


def _env(name, default):
    return float(os.environ.get(name, default))


# finger command when holding: pads 26 mm apart on the 33 mm case, so the finger springs squeeze it (about 17 N)
# instead of the full 50 N close
GRIP_F = _env("DEMO_GRIP", -0.013)


def Ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


DOWN_Y = yam_demo.grip_rotation(0.0)  # straight down, fingers closing along world y
BODY_REST = np.diag([-1.0, -1.0, 1.0])  # dispenser lying flat, lip toward +x
GRIP_B = np.array([0.055, 0.0, 0.013])  # the TCP in the dispenser's frame when gripped as planned
GRIP_REST = Ry(-math.pi / 4) @ DOWN_Y  # the grip on the lying dispenser: pitched 45 deg, approach down and forward


class Move:
    """Cartesian legs for one arm: (duration, position, rotation, finger, profile). Profile 'smooth' (smoothstep) or
    'line' (constant speed with ``ramp`` s ramps). IK each control step, warm-started."""

    SEEDS = (READY, [0.0, 1.5, 1.2, -0.6, 0.0, 0.0], [-0.3, 1.0, 0.8, -0.3, 0.0, 1.5])

    def __init__(self, base, q_start, finger_start, legs, ramp=0.2, offset=None):
        self.base, self.q = np.asarray(base, float), np.asarray(q_start, float).copy()
        self.offset = np.zeros(3) if offset is None else np.asarray(offset, float).copy()
        p0, R0 = ik.tcp_pose(self.q, self.base)
        p0 = p0 - self.offset  # the start, as a target (the offset is added on top of every target)
        self.legs, self.ramp = [], ramp
        for dur, p, R, finger, prof in legs:
            self.legs.append((dur, np.asarray(p0, float), R0, finger_start, np.asarray(p, float), R, finger, prof))
            p0, R0, finger_start = np.asarray(p, float), R, finger
        self.duration = sum(leg[0] for leg in self.legs)
        self.finger = finger_start

    def _s(self, u: float, dur: float, prof: str) -> float:
        if prof == "smooth":
            return ik.smoothstep(u)
        r = min(self.ramp / dur, 0.5)
        vmax = 1.0 / (1.0 - r)
        if u < r:
            return 0.5 * vmax * u * u / r
        if u > 1 - r:
            return 1.0 - 0.5 * vmax * (1 - u) ** 2 / r
        return 0.5 * vmax * r + vmax * (u - r)

    def target(self, t: float):
        for dur, p0, R0, f0, p1, R1, f1, prof in self.legs:
            if t <= dur:
                s = self._s(min(max(t / dur, 0.0), 1.0), dur, prof)
                return p0 + s * (p1 - p0), ik.slerp_matrix(R0, R1, ik.smoothstep(min(t / dur, 1.0))), f0 + (
                    f1 - f0) * min(t / max(dur * 0.8, 1e-6), 1.0)
            t -= dur
        _, _, _, _, p1, R1, f1, _ = self.legs[-1]
        return p1, R1, f1

    def action7(self, t: float):
        pos, R, finger = self.target(t)
        pos = pos + self.offset
        quat = ik.matrix_to_quat_xyzw(R)
        q, pe, re = ik.solve(pos, quat, self.q, base_pos=self.base, iters=60)
        if pe > 0.002 or re > 0.03:
            for seed in self.SEEDS:
                q2, pe2, re2 = ik.solve(pos, quat, np.array(seed, float), base_pos=self.base, iters=300)
                if pe2 + 0.05 * re2 < pe + 0.05 * re:
                    q, pe, re = q2, pe2, re2
        self.q = q
        return np.r_[q, float(np.clip(finger / FINGER_OPEN, 0.0, 1.0))]


class Joint:
    def __init__(self, duration, q_goal, q_start, finger):
        self.duration, self.q_goal, self.q0 = duration, np.asarray(q_goal, float), np.asarray(q_start, float)
        self.q, self.finger = self.q0.copy(), finger

    def action7(self, t: float):
        self.q = self.q0 + ik.smoothstep(min(t / self.duration, 1.0)) * (self.q_goal - self.q0)
        return np.r_[self.q, self.finger / FINGER_OPEN]


PARK_Q = {"left": np.array([1.2, *READY[1:]]), "right": np.array([-1.2, *READY[1:]])}


class Controller:
    TILT = _env("DEMO_TILT", 45.0)  # deg, the case to the paper
    SPEED = _env("DEMO_SPEED", 0.003)  # m/s along the line
    PRESS_MM = _env("DEMO_PRESS", 4.0)  # command the lip this far below the paper
    START_MM = _env("DEMO_START", 0.2)  # put the lip down this far before the word's first letter
    FB_GAIN = _env("DEMO_FB", 0.35)  # across the line
    FB_GAIN_Y = _env("DEMO_FB_Y", 0.1)  # along it
    END_MM = _env("DEMO_END", 0.2)  # and stop this far past its last

    def __init__(self, bases, dt, rig):
        self.rig, self.dt = rig, dt
        self.bases = {a: np.asarray(bases[a], float) for a in ARMS}
        self.q = {a: np.array(READY, float) for a in ARMS}
        self.finger = dict.fromkeys(ARMS, FINGER_OPEN)
        self.plans: dict = {}
        self.t0, self.dur = -1.0, 0.0
        self.finished_at = None
        self.collect, self.samples = None, []
        self.fb, self.off, self.y_mode = False, np.zeros(3), "track"  # xy feedback on the tool point
        self.grip_b, self.rel = GRIP_B.copy(), GRIP_REST.T @ BODY_REST  # the grip, measured after lifting
        # contrasts for the README: "sweep" (one stroke along the whole line), "light" (lip only touching), "push"
        # (pushed the wrong way, right to left), "two" (two strokes)
        self.cheat = os.environ.get("DEMO_CHEAT", "")
        if self.cheat == "light":
            self.PRESS_MM = -0.3
        self.script_it = self.script()

    def say(self, msg: str) -> None:
        print(f"[tapedemo] {msg}", flush=True)

    # -- playing -------------------------------------------------------------------------------------------------

    def _start(self, seg) -> None:
        moves, min_s, collect = seg
        for arm, plan in self.plans.items():
            self.q[arm], self.finger[arm] = plan.q.copy(), plan.finger
        self.plans = {}
        for a, m in moves.items():
            if isinstance(m, float):
                self.plans[a] = Joint(m, PARK_Q[a], self.q[a], self.finger[a])
            else:
                self.plans[a] = Move(self.bases[a], self.q[a], self.finger[a], m,
                                     offset=self.off if a == "right" and self.fb else None)
        self.dur = max([p.duration for p in self.plans.values()] + [min_s])
        self.collect, self.samples = collect, []

    def act(self, obs):
        t = float(obs.state_time)
        if self.collect is not None:
            self.samples.append(self.collect(obs))
        while self.finished_at is None and t >= self.t0 + self.dur - 1e-6:
            try:
                seg = next(self.script_it)
            except StopIteration:
                self.finished_at = t
                self.collect = None
                break
            self.t0 = t
            self._start(seg)
        plan = self.plans.get("right")
        if self.fb and isinstance(plan, Move):
            # integral feedback in x and y: push the command by the measured error (the pressed arm drifts and lags)
            want = plan.target(min(t - self.t0, plan.duration))[0]
            err = want - self._tool_point(obs)
            dy = self.FB_GAIN_Y * err[1]
            if self.y_mode == "freeze":  # during the stroke: an integrator on a moving target winds up and overshoots
                dy = 0.0
            elif self.y_mode == "forward":  # at the end: only ever push on along the line (-y), never back
                dy = min(dy, 0.0)
            self.off = np.clip(self.off + np.r_[self.FB_GAIN * err[0], dy, 0.0], -0.005, 0.005)
            plan.offset = self.off
            if os.environ.get("TAPE_DEBUG"):
                self.say(f"t {t:6.2f} tool point err {np.round(err * 1000, 2)} mm, offset {np.round(self.off * 1000, 2)} mm")
        out = []
        for arm in ARMS:
            if arm in self.plans:
                out.append(self.plans[arm].action7(min(t + self.dt - self.t0, self.plans[arm].duration)))
            else:
                out.append(np.r_[self.q[arm], self.finger[arm] / FINGER_OPEN])
        return out[0], out[1], self.finished_at

    def _tool_point(self, obs):
        """The measured TCP corrected for the wrist's measured tilt: where the TCP would be if the dispenser were held
        at its planned angle with its lip where it really is (forward kinematics of the joints and the in-hand pose).
        Under the pressing force the wrist tilts a little, and 48 mm above the lip that moves the lip by a millimetre."""
        j = np.asarray(next(iter(obs.state.values())), float).reshape(-1)
        tcp, Rg = ik.tcp_pose(j[7:13], self.bases["right"])
        lip = tcp - Rg @ self.rel @ self.grip_b
        return lip + self.Rb_plan @ self.grip_b

    def _tcp(self, obs, arm="right"):
        j = np.asarray(next(iter(obs.state.values())), float).reshape(-1)
        return ik.tcp_pose(j[:6] if arm == "left" else j[7:13], self.bases[arm])[0]

    # -- geometry ------------------------------------------------------------------------------------------------

    def use_rotations(self, tilt_deg: float):
        """Gripper and dispenser rotations with the case at ``tilt_deg`` to the paper, lip down and forward."""
        Rb = Ry(math.radians(tilt_deg)) @ BODY_REST
        return Rb @ self.rel.T, Rb

    def tcp_for_lip(self, lip, Rb):
        return np.asarray(lip, float) + Rb @ self.grip_b

    def _look_in_hand(self, obs):
        """Where the dispenser sits in the fingers (the only simulator read: the dispenser's pose once it is lifted,
        what a person sees by looking at the tool in their hand). Returns the TCP in the dispenser's frame and the
        dispenser's rotation in the gripper's frame."""
        import tapekit

        lip, Rb = tapekit._RT[self.rig.name]._pose()
        j = np.asarray(next(iter(obs.state.values())), float).reshape(-1)
        tcp, Rg = ik.tcp_pose(j[7:13], self.bases["right"])
        return Rb.T @ (tcp - lip), Rg.T @ Rb

    def lip_at(self, u_mm, w_mm, dz_mm):
        xy = self.rig.sheet_to_env(u_mm, w_mm)
        return np.array([xy[0], xy[1], self.rig.paper_top + dz_mm / 1000])

    # -- the script ----------------------------------------------------------------------------------------------

    def script(self):
        r = self.rig
        yield ({"left": 2.0}, 0.0, None)
        tx, ty = r.tape_xy
        lip0 = np.array([tx, ty, r.table_z - r.case_z0 + 0.0003])
        g = self.tcp_for_lip(lip0, BODY_REST)
        back = -0.04 * GRIP_REST[:, 2]  # 4 cm back along the approach
        yield ({"right": [(2.0, g + back, GRIP_REST, FINGER_OPEN, "smooth")]}, 0.0, None)
        yield ({}, 0.4, self._tcp)  # the arm sags and lags: measure, aim off by it
        off = g + back - np.median(self.samples, axis=0)
        self.say(f"above the dispenser: TCP off {np.round(-off * 1000, 2)} mm")
        g = g + off
        yield ({"right": [(1.2, g, GRIP_REST, FINGER_OPEN, "smooth")]}, 0.0, None)
        yield ({"right": [(0.8, g, GRIP_REST, GRIP_F, "smooth")]}, 0.0, None)
        yield ({"right": [(1.2, g + (0, 0, 0.06), GRIP_REST, GRIP_F, "smooth")]}, 0.0, None)
        yield ({}, 0.3, self._look_in_hand)
        self.grip_b = np.median([s[0] for s in self.samples], axis=0)
        self.rel = self.samples[-1][1]
        self.say(f"in hand: TCP at {np.round(self.grip_b * 1000, 1)} mm in the dispenser's frame (planned "
                 f"{np.round(GRIP_B * 1000, 1)})")

        word = LAYOUT["words"][LAYOUT["typo"]]
        wm = (word["w0"] + word["w1"]) / 2
        u_a, u_b = word["u0"] - self.START_MM, word["u1"] + self.END_MM
        if self.cheat == "sweep":
            line = [w for w in LAYOUT["words"] if w["line"] == word["line"]]
            u_a, u_b = line[0]["u0"] - 1.0, line[-1]["u1"] + 1.0
        push = self.cheat == "push"
        Rg, Rb = self.use_rotations(self.TILT)
        if push:
            u_a, u_b = u_b, u_a  # the same hold, pushed the wrong way (right to left)
        above = self.lip_at(u_a, wm, 3.0)
        yield ({"right": [(1.6, self.tcp_for_lip(above + (0, 0, 0.03), Rb), Rg, GRIP_F, "smooth"),
                          (1.0, self.tcp_for_lip(above, Rb), Rg, GRIP_F, "smooth")]}, 0.0, None)
        yield ({}, 0.4, self._tcp)
        aim = self.tcp_for_lip(above, Rb) - np.median(self.samples, axis=0)
        aim[:2] = 0.0  # x, y: the feedback below holds them; z: the measured sag
        self.say(f"hover: TCP off {np.round(-aim * 1000, 2)} mm in z; xy feedback on from here")

        def at(u, dz):
            return self.tcp_for_lip(self.lip_at(u, wm, dz), Rb) + aim

        self.fb, self.Rb_plan = True, Rb
        yield ({"right": [(0.8, at(u_a, 3.0), Rg, GRIP_F, "smooth")]}, 0.0, None)
        down = -self.PRESS_MM
        yield ({"right": [(0.8, at(u_a, down), Rg, GRIP_F, "smooth")]}, 0.0, None)
        yield ({"right": [(0.8, at(u_a, down), Rg, GRIP_F, "smooth")]}, 0.0, None)
        if self.cheat == "two":
            u_m = (u_a + u_b) / 2
            yield ({"right": [(abs(u_m - u_a) / 1000 / self.SPEED + 0.2, at(u_m, down), Rg, GRIP_F, "line"),
                              (0.4, at(u_m, down), Rg, GRIP_F, "smooth"),
                              (0.4, at(u_m, 4.0), Rg, GRIP_F, "smooth"),
                              (0.4, at(u_m - 0.3, down), Rg, GRIP_F, "smooth")]}, 0.0, None)
            u_a = u_m - 0.3
        dur = abs(u_b - u_a) / 1000 / self.SPEED + 0.2
        self.y_mode = "freeze"
        yield ({"right": [(dur, at(u_b, down), Rg, GRIP_F, "line")]}, 0.0, None)
        # the pressed arm lags behind the command along the line: hold there and let the feedback bring the tool point
        # on to the planned end (forward only)
        self.y_mode = "forward"
        yield ({"right": [(1.5, at(u_b, down), Rg, GRIP_F, "smooth")]}, 0.0, self._tcp)
        self.fb = False
        # lift straight up from where the tool point is (not where it is commanded: the pressed lip lags the command,
        # and would spring on along the line as it comes up)
        here = np.median(self.samples[-4:], axis=0)
        yield ({"right": [(0.3, here, Rg, GRIP_F, "smooth"),
                          (0.3, here + (0, 0, 0.008 - down / 1000), Rg, GRIP_F, "smooth"),
                          (0.8, here + (0, 0, 0.060), Rg, GRIP_F, "smooth")]}, 0.0, None)
        yield ({"right": 2.0}, 0.0, None)  # out of the top camera's view of the sheet, dispenser still held
        yield ({}, 1.0, None)
