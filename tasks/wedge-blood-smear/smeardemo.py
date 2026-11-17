"""Closed-loop scripted demo for wedge-blood-smear, played by ``yam_scripted`` (see ``smearkit._patch_scripted``).

It sees only the observation: the arms' measured joint positions (forward kinematics gives the tool point). It never
reads the simulator: not the contact, not the drop, not the film. It knows the fixed layout (the README's setup).

The right arm does the smear; the left arm turns out of the top camera's view.

1. Grip the spreader across its thickness, 8 mm below its top end, from straight above, and lift it out of the stand.
2. Pitch it so that it leans back over the drop at ``ANGLE`` (35 degrees) to the base slide, its lower edge across the
   slide 6 mm in front of the drop, 4 mm above the glass.
3. Feel the glass: the edge goes down slowly; once the measured tool point stops following the command (the arm's
   joints are springs, so the extra lag is the edge pressing on the glass with a fraction of a newton), it holds that
   command depth plus ``PRESS_M``.
4. Back the edge into the drop at 5 mm/s, to 1 mm in front of its centre, and hold ``WICK_S`` (1.5 s) while the drop
   runs out along the edge.
5. One stroke: the edge goes forward along the slide at ``SPEED`` (45 mm/s, constant, with 0.1 s ramps), at the same
   height and angle, for 44 mm, past the point where the blood runs out (about 38 mm); then it lifts clear.
"""

from __future__ import annotations

import math
import os

import numpy as np
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY

ARMS = ("left", "right")


def _env(name, default):
    return float(os.environ.get(name, default))


def Ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


DOWN_X = yam_demo.grip_rotation(math.pi / 2)  # straight down, fingers closing along world x


def stroke_rotation(angle_deg: float) -> np.ndarray:
    """The grip pitched so that the held spreader leans back (top toward -x) at ``angle_deg`` to the table."""
    return Ry(-math.radians(90.0 - angle_deg)) @ DOWN_X


class Move:
    """Cartesian legs for one arm: (duration, position, rotation, finger, profile). Profile 'smooth' (smoothstep) or
    'line' (constant speed with ``ramp`` s ramps). IK each control step, warm-started."""

    SEEDS = (READY, [0.0, 1.5, 1.2, -0.6, 0.0, 0.0], [-0.3, 1.0, 0.8, -0.3, 0.0, 1.5])

    def __init__(self, base, q_start, finger_start, legs, ramp=0.1):
        self.base, self.q = np.asarray(base, float), np.asarray(q_start, float).copy()
        p0, R0 = ik.tcp_pose(self.q, self.base)
        self.legs, self.ramp = [], ramp
        for dur, p, R, finger, prof in legs:
            self.legs.append((dur, np.asarray(p0, float), R0, finger_start, np.asarray(p, float), R, finger, prof))
            p0, R0, finger_start = np.asarray(p, float), R, finger
        self.duration = sum(leg[0] for leg in self.legs)
        self.finger = finger_start

    def _s(self, u: float, dur: float, prof: str) -> float:
        if prof == "smooth":
            return ik.smoothstep(u)
        r = min(self.ramp / dur, 0.5)  # trapezoid in velocity, normalised
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
    ANGLE = _env("DEMO_ANGLE", 35.0)  # spreader to slide, deg
    SPEED = _env("DEMO_SPEED", 0.045)  # m/s
    WICK_S = _env("DEMO_WICK_S", 1.5)
    PRESS_M = _env("DEMO_PRESS", 0.0008)  # command this far below where the glass was felt
    GRIP_BELOW_TOP = 0.008
    FEEL_M = 0.0004
    STROKE_M = 0.044

    def __init__(self, bases, dt, rig):
        self.rig, self.dt = rig, dt
        self.bases = {a: np.asarray(bases[a], float) for a in ARMS}
        self.q = {a: np.array(READY, float) for a in ARMS}
        self.finger = dict.fromkeys(ARMS, FINGER_OPEN)
        self.plans: dict = {}
        self.t0, self.dur = -1.0, 0.0
        self.finished_at = None
        self.obs = None
        self.collect, self.samples = None, []
        # contrasts for the README: "nowait" (push at once, no wicking), "plough" (come down behind the drop and push
        # through it), "steep" (55 degrees, 90 mm/s), "slow" (15 mm/s), "lift" (lift off half way and finish the stroke)
        self.cheat = os.environ.get("DEMO_CHEAT", "")
        if self.cheat == "steep":
            self.ANGLE, self.SPEED = 55.0, 0.090
        elif self.cheat == "slow":
            self.SPEED = 0.015
        self.edge_len = rig.slide[0] - self.GRIP_BELOW_TOP  # TCP to the lower edge, along the spreader
        self.script_it = self.script()

    def say(self, msg: str) -> None:
        print(f"[smeardemo] {msg}", flush=True)

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
                self.plans[a] = Move(self.bases[a], self.q[a], self.finger[a], m)
        self.dur = max([p.duration for p in self.plans.values()] + [min_s])
        self.collect, self.samples = collect, []

    def act(self, obs):
        t = float(obs.state_time)
        self.obs = obs
        if self.collect is not None:
            self.samples.append(self.collect(obs))
        # a segment ends once its last action is out; the next starts now (the observation clock is not on a grid
        # with the segments' durations)
        while self.finished_at is None and t >= self.t0 + self.dur - 1e-6:
            try:
                seg = next(self.script_it)
            except StopIteration:
                self.finished_at = t
                self.collect = None
                break
            self.t0 = t
            self._start(seg)
        out = []
        for arm in ARMS:
            if arm in self.plans:
                out.append(self.plans[arm].action7(min(t + self.dt - self.t0, self.plans[arm].duration)))
            else:
                out.append(np.r_[self.q[arm], self.finger[arm] / FINGER_OPEN])
        return out[0], out[1], self.finished_at

    # -- observing -----------------------------------------------------------------------------------------------

    def _joints(self, obs, arm):
        j = np.asarray(next(iter(obs.state.values())), float).reshape(-1)
        return j[:6] if arm == "left" else j[7:13]

    def _tcp(self, obs, arm="right"):
        return ik.tcp_pose(self._joints(obs, arm), self.bases[arm])[0]

    # -- geometry ------------------------------------------------------------------------------------------------

    def tcp_for_edge(self, edge, R):
        """The TCP for the spreader's lower edge at ``edge`` (the approach axis runs from the TCP down the slide)."""
        return np.asarray(edge, float) - R[:, 2] * self.edge_len

    def leg(self, dur, edge, R, finger=FINGER_CLOSED, prof="smooth"):
        return (dur, self.tcp_for_edge(edge, R), R, finger, prof)

    def step(self, dur, edge, R, collect=True, prof="smooth"):
        yield ({"right": [self.leg(dur, edge, R, prof=prof)]}, 0.0, self._tcp if collect else None)

    # -- the script ----------------------------------------------------------------------------------------------

    def script(self):
        r = self.rig
        yield ({"left": 2.0}, 0.0, None)
        sx, sy = r.stand_xy
        top = r.spreader_bottom + 0.0002 + r.slide[0]
        gz = top - self.GRIP_BELOW_TOP
        yield ({"right": [(2.0, (sx, sy, gz + 0.06), DOWN_X, FINGER_OPEN, "smooth"),
                          (1.2, (sx, sy, gz), DOWN_X, FINGER_OPEN, "smooth")]}, 0.0, None)
        yield ({"right": [(0.8, (sx, sy, gz), DOWN_X, FINGER_CLOSED, "smooth")]}, 0.0, None)
        yield ({"right": [(1.2, (sx, sy, gz + 0.08), DOWN_X, FINGER_CLOSED, "smooth")]}, 0.0, None)

        R = stroke_rotation(self.ANGLE)
        dx, dy = r.drop_xy
        zt = r.slide_top
        ahead = -0.006 if self.cheat == "plough" else 0.006  # where the edge comes down, from the drop's centre
        above = np.array([dx + ahead, dy, zt + 0.004])
        yield ({"right": [(1.5, self.tcp_for_edge(above + (0, 0, 0.04), R), R, FINGER_CLOSED, "smooth"),
                          (1.0, self.tcp_for_edge(above, R), R, FINGER_CLOSED, "smooth")]}, 0.0, None)
        # the arm lags its commands: measure the TCP and aim off by the error (x, y), note the height lag
        yield ({}, 0.4, self._tcp)
        got = np.median(self.samples, axis=0)
        want = self.tcp_for_edge(above, R)
        off = want - got
        self.say(f"hover: TCP off {np.round(-off * 1000, 2)} mm; aiming off by that")
        aim = np.array([off[0], off[1], 0.0])
        lag0 = off[2]
        z = zt + 0.004
        felt = False
        for _ in range(80):  # feel the glass: 0.25 mm per 0.05 s
            z -= 0.00025
            yield from self.step(0.05, np.array([dx + ahead, dy, z]) + aim, R)
            lag = (self.tcp_for_edge((0, 0, z), R)[2] + aim[2]) - self.samples[-1][2] - lag0
            if lag >= self.FEEL_M:
                felt = True
                break
        self.say(f"glass {'felt' if felt else 'NOT felt'} at edge command {(z - zt) * 1000:+.2f} mm")
        z -= self.PRESS_M
        yield from self.step(0.3, np.array([dx + ahead, dy, z]) + aim, R)
        if self.cheat == "plough":  # push straight through the drop from behind
            x_from = dx + ahead
        else:
            x_from = dx + 0.001
            yield from self.step(abs(ahead - 0.001) / 0.005, np.array([x_from, dy, z]) + aim, R, collect=False)
            if self.cheat != "nowait":
                yield from self.step(self.WICK_S, np.array([x_from, dy, z]) + aim, R, collect=False)
        x_to = x_from + self.STROKE_M
        dur = self.STROKE_M / self.SPEED + 0.1
        if self.cheat == "lift":
            mid = x_from + 0.012
            yield from self.step((mid - x_from) / self.SPEED, np.array([mid, dy, z]) + aim, R, collect=False,
                                 prof="line")
            yield from self.step(0.3, np.array([mid + 0.004, dy, z + 0.004]) + aim, R, collect=False)
            yield from self.step(0.3, np.array([mid + 0.008, dy, z]) + aim, R, collect=False)
            x_from = mid + 0.008
            dur = (x_to - x_from) / self.SPEED + 0.1
        yield from self.step(dur, np.array([x_to, dy, z]) + aim, R, collect=False, prof="line")
        yield from self.step(0.8, np.array([x_to + 0.01, dy, z + 0.03]) + aim, R, collect=False)
        yield ({}, 1.5, None)
