"""Scripted demo for drip-emitter-punch, played by ``yam_scripted`` (see ``dripkit._patch_scripted``).

Right arm, gripper pointing straight down, fingers closing across the line for the punch, along it for the dripper. It reads the mark's place and the hole it
made from the simulator (the review rules allow a scripted demo to read simulator state); it never moves an object
itself: every motion goes through the arm.

Take the punch out of its block hole by the handle , put its point on the mark at
the top of the tube, and push straight down: the command is lowered 2.5 mm per 0.1 s, so the arm's joint stiffness
pushes harder and harder, until the wall gives; lift, stand the punch back in its block hole and let go. Then take the
dripper by its body, put its barb on the hole, push straight down the same way until it pops in, let go, lift clear.
Before each push it reads where the point (or barb) really is and corrects for how the part sits in the fingers.

Contrasts for the README (``DEMO_CHEAT``): ``nopunch`` pushes the dripper onto the mark without punching; ``tilt``
punches with the tool leaning 18 degrees; ``light`` punches with a third of the push; ``twice`` punches a second hole
15 mm along before seating the dripper.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation

ARMS = ("left", "right")


def _env(name, default):
    return float(os.environ.get(name, default))


class JointMove:
    def __init__(self, duration: float, q, q_start, finger):
        self.duration, self.q_goal, self.q_start = duration, np.asarray(q, float), np.asarray(q_start, float)
        self.q = self.q_start.copy()
        self.waypoints = [Waypoint(duration, (0.0, 0.0, 0.0), finger)]

    def action7(self, t: float):
        self.q = self.q_start + ik.smoothstep(min(t / self.duration, 1.0)) * (self.q_goal - self.q_start)
        return np.r_[self.q, self.waypoints[0].finger / FINGER_OPEN], "joint move"


class Plan(ArmPlan):
    """ArmPlan whose IK retries from a few seeds when the warm start does not converge."""

    SEEDS = (READY, [0.5, 2.1, 2.2, -1.6, 0.0, 0.5], [0.2, 1.0, 1.0, -0.8, 0.1, 0.2], [-0.3, 1.5, 1.5, -1.0, 0.0, -0.3])

    def action7(self, t: float):
        pos, rot_m, finger, label = self.target(t)
        quat = ik.matrix_to_quat_xyzw(rot_m)
        q, pe, re = ik.solve(pos, quat, self.q, base_pos=self.base_pos, iters=60)
        if pe > 0.002 or re > 0.03:
            for s in self.SEEDS:
                q2, pe2, re2 = ik.solve(pos, quat, np.array(s, float), base_pos=self.base_pos, iters=300)
                if pe2 + 0.05 * re2 < pe + 0.05 * re:
                    q, pe, re = q2, pe2, re2
        self.q = q
        return np.r_[q, float(np.clip(finger / FINGER_OPEN, 0.0, 1.0))], label


HALF = 0.55 * FINGER_OPEN  # pads about 52 mm apart: clears the neighbouring tool in the block
PARK_Q = {"left": np.array([1.2, *READY[1:]]), "right": np.array([-1.2, *READY[1:]])}


@dataclass
class Segment:
    moves: dict  # arm -> list of Waypoint (yaw is a rotation matrix); a float: park the arm (s)
    min_s: float = 0.0


def tilt_rotation(deg: float) -> np.ndarray:
    """The straight-down grip, fingers closing across the line (along x), leaned ``deg`` about world y."""
    a = math.radians(deg)
    Ry = np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])
    return Ry @ grip_rotation(math.pi / 2)


def upright(a_up, gain: float) -> np.ndarray:
    """The rotation turning the measured tool axis ``a_up`` a fraction ``gain`` of the way to vertical."""
    a = np.asarray(a_up, float) / np.linalg.norm(a_up)
    k = np.cross(a, [0.0, 0.0, 1.0])
    sn = np.linalg.norm(k)
    if sn < 1e-6:
        return np.eye(3)
    k /= sn
    th = gain * math.atan2(sn, a[2])
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


class Controller:
    GRIP_P = 0.050  # TCP this far up the punch from its point (short lever: less twist in the fingers)
    GRIP_E = 0.023  # TCP this far up the dripper from its barb's tip

    def __init__(self, bases, dt, rig):
        self.rig, self.dt = rig, dt
        self.bases = {a: np.asarray(bases[a], float) for a in ARMS}
        self.q = {a: np.array(READY, float) for a in ARMS}
        self.finger = dict.fromkeys(ARMS, FINGER_OPEN)
        self.plans: dict = {}
        self.t0 = self.dur = 0.0
        self.finished_at = None
        self.cheat = os.environ.get("DEMO_CHEAT", "")
        self.press_max = _env("DEMO_PRESS", 0.030 if self.cheat == "light" else 0.10)
        self.press_rate = 0.025  # m/s of command
        self.lean = 18.0 if self.cheat == "tilt" else 0.0
        self.script_it = self.script()

    def say(self, msg: str) -> None:
        print(f"[dripdemo] {msg}", flush=True)

    def _start(self, seg: Segment) -> None:
        for arm, plan in self.plans.items():
            self.q[arm], self.finger[arm] = plan.q.copy(), plan.waypoints[-1].finger
        self.plans = {a: JointMove(w, PARK_Q[a], self.q[a], self.finger[a]) if isinstance(w, float) else
                      Plan(list(w), self.bases[a], q_start=self.q[a], finger_start=self.finger[a])
                      for a, w in seg.moves.items()}
        self.dur = max([p.duration for p in self.plans.values()] + [seg.min_s])

    def act(self, obs):
        t = float(obs.state_time)
        while self.finished_at is None and t + self.dt >= self.t0 + self.dur:
            self.t0 += self.dur
            try:
                self._start(next(self.script_it))
            except StopIteration:
                self.t0 -= self.dur
                self.finished_at = t
        out = []
        for arm in ARMS:
            if arm in self.plans:
                a7, _ = self.plans[arm].action7(min(t + self.dt - self.t0, self.plans[arm].duration))
            else:
                a7 = np.r_[self.q[arm], self.finger[arm] / FINGER_OPEN]
            out.append(a7)
        return out[0], out[1], self.finished_at

    def _rt(self):
        import dripkit

        return dripkit.runtime(self.rig.name)

    def reach(self, goal, R, finger):
        """Correct the command until the measured TCP is at ``goal`` (the arm sags a few mm under its own weight)."""
        goal = np.asarray(goal, float)
        cmd = goal.copy()
        rt = self._rt()
        for _ in range(3 if rt is not None else 0):
            cmd = cmd + (goal - rt._tcp())
            yield Segment({"right": [Waypoint(0.4, tuple(cmd), finger, "settle", R)]})
        if rt is not None:
            self.say(f"tcp {np.round((rt._tcp() - goal) * 1000, 1)} mm off its goal")
        return cmd

    def align(self, body, goal, cmd, R, finger, square=True):
        """Bring the held part's tip (``body``: "punch" or "emitter") to ``goal``, the part upright when ``square``,
        correcting the command from the measured pose (the part sits in the fingers a little off and tilted).
        Returns the command, its rotation and the tip's offset from the tool point in the gripper's frame."""
        rt = self._rt()
        cmd, R = np.asarray(cmd, float), np.asarray(R, float)
        v = R.T @ (goal - cmd)
        for _ in range(8 if rt is not None else 0):
            p, Ra = rt._tcp_pose()
            tip, a_up = rt._tool(getattr(rt, body))
            v = Ra.T @ (tip - p)
            if np.linalg.norm(goal - tip) < 0.0004 and (not square or a_up[2] > math.cos(math.radians(1.0))):
                break
            R2 = upright(a_up, 1.0) @ R if square else R
            cmd = cmd + (goal - tip) - (R2 - R) @ v
            R = R2
            yield Segment({"right": [Waypoint(0.5, tuple(cmd), finger, "line up", R)]})
        if rt is not None:
            tip, a_up = rt._tool(getattr(rt, body))
            self.say(f"{body} tip {np.round((tip - goal) * 1000, 1)} mm off, "
                     f"{math.degrees(math.acos(min(1.0, a_up[2]))):.1f} deg from upright")
        return cmd, R, v

    def script(self):
        r = self.rig
        W = Waypoint
        Rd = grip_rotation(0.0)
        up = np.array([0.0, 0.0, 1.0])
        top = r.line_z + r.line_r
        rt = self._rt()
        mark_y = rt.line.mark_y if rt is not None else r.mark_y
        # the punch, by its handle
        p0 = r.punch_rest()
        g = p0 + self.GRIP_P * up
        Rx = tilt_rotation(0.0)  # the punch is held with the fingers closing along x: the arm's give tilts it in x-z
        yield Segment({"left": 2.0, "right": [
            W(2.0, tuple(g + (0, 0, 0.06)), HALF, "over the punch", Rx),
            W(1.2, tuple(g), HALF, "down to its handle", Rx)]})
        cmd = yield from self.reach(g, Rx, HALF)
        yield Segment({"right": [
            W(0.6, tuple(cmd), FINGER_CLOSED, "grip the handle", Rx),
            W(1.2, tuple(cmd + (0, 0, 0.05)), FINGER_CLOSED, "draw it out of the block", Rx)]})
        cmd, Rc = cmd + (0, 0, 0.05), Rx
        if self.cheat != "nopunch":
            spots = [mark_y] + ([mark_y + 0.015] if self.cheat == "twice" else [])
            for y in spots:
                tip = np.array([r.line_x, y, top])
                Rc = tilt_rotation(self.lean)
                cmd = tip + 0.02 * up + Rc @ np.array([0.0, 0.0, -self.GRIP_P])
                yield Segment({"right": [W(1.8, tuple(cmd), FINGER_CLOSED, "over the mark", Rc)]})
                cmd, Rc, v = yield from self.align("punch", tip + 0.004 * up, cmd, Rc, FINGER_CLOSED, not self.lean)
                yield Segment({"right": [W(0.6, tuple(cmd - 0.005 * up), FINGER_CLOSED, "point on the mark", Rc)]})
                d, made = 0.005, False
                while d < self.press_max and not made:
                    d += self.press_rate * 0.1
                    yield Segment({"right": [W(0.1, tuple(cmd - d * up), FINGER_CLOSED, "push the punch through", Rc)]})
                    if rt is not None:
                        made = any(abs(h.p[1] - y) < 0.004 for h in rt.line.holes)
                        got, a_up = rt._tool(rt.punch)
                        cmd[:2] += 0.4 * (tip[:2] - got[:2])  # keep the point on the crest as the arm gives
                        if not self.lean:  # and the tool upright
                            R2 = upright(a_up, 0.5) @ Rc
                            cmd = cmd - (R2 - Rc) @ v
                            Rc = R2
                self.say(f"punch at y {y:+.4f}: {'hole made' if made else 'NO hole'}, command {d * 1000:.0f} mm down")
                cmd = cmd + 0.03 * up
                yield Segment({"right": [W(1.0, tuple(cmd), FINGER_CLOSED, "lift the punch", Rc)]})
        # back into its block hole, upright
        cmd = p0 + 0.03 * up + self.GRIP_P * up
        yield Segment({"right": [W(2.0, tuple(cmd), FINGER_CLOSED, "over the block hole", Rx)]})
        cmd, Rc, v = yield from self.align("punch", p0 + 0.020 * up, cmd, Rx, FINGER_CLOSED, True)
        yield Segment({"right": [
            W(1.2, tuple(cmd - 0.019 * up), FINGER_CLOSED, "point into the hole", Rc),
            W(0.6, tuple(cmd - 0.019 * up), FINGER_OPEN, "let go", Rc),
            W(1.0, tuple(cmd + 0.05 * up), FINGER_OPEN, "up", Rc)]})
        # the dripper, by its body
        e0 = r.emitter_rest()
        ge = e0 + self.GRIP_E * up
        yield Segment({"right": [
            W(1.5, tuple(ge + (0, 0, 0.05)), HALF, "over the dripper", Rx),
            W(1.0, tuple(ge), HALF, "down to it", Rx)]})
        cmd = yield from self.reach(ge, Rx, HALF)
        yield Segment({"right": [
            W(0.6, tuple(cmd), FINGER_CLOSED, "grip it", Rx),
            W(1.0, tuple(cmd + (0, 0, 0.04)), FINGER_CLOSED, "lift it out", Rx)]})
        holes = rt.line.holes if rt is not None else []
        target = np.array(holes[0].p) if holes else np.array([r.line_x, mark_y, top])
        self.say(f"dripper to {np.round(target, 4)} ({len(holes)} holes)")
        cmd = target + (0.02 + self.GRIP_E) * up
        yield Segment({"right": [W(2.0, tuple(cmd), FINGER_CLOSED, "barb over the hole", Rx)]})
        cmd, Rc, v = yield from self.align("emitter", target + 0.004 * up, cmd, Rx, FINGER_CLOSED, True)
        cmd, Rc, v = yield from self.align("emitter", target + 0.0008 * up, cmd - 0.0032 * up, Rc, FINGER_CLOSED, True)
        for attempt in range(5):  # feel for the hole: set the barb down, and if it landed off the hole, lift and shift
            yield Segment({"right": [W(0.4, tuple(cmd - 0.002 * up), FINGER_CLOSED, "barb on the hole", Rc)]})
            if rt is None:
                break
            got, _ = rt._tool(rt.emitter)
            err = target[:2] - got[:2]
            if np.linalg.norm(err) < 0.0008:
                break
            self.say(f"barb landed {np.round(-err * 1000, 1)} mm off the hole: lift and shift")
            cmd = cmd + 0.002 * up
            yield Segment({"right": [W(0.3, tuple(cmd), FINGER_CLOSED, "lift a little", Rc)]})
            cmd[:2] += err
            yield Segment({"right": [W(0.3, tuple(cmd), FINGER_CLOSED, "shift", Rc)]})
            cmd = cmd - 0.002 * up
        d = 0.002
        while d < 0.06 and not (rt is not None and rt.line.seated is not None):
            d += 0.02 * 0.1
            yield Segment({"right": [W(0.1, tuple(cmd - d * up), FINGER_CLOSED, "push it in", Rc)]})
            if rt is not None and rt.line.seated is None:
                got, a_up = rt._tool(rt.emitter)
                cmd[:2] += 0.4 * (target[:2] - got[:2])
                R2 = upright(a_up, 0.5) @ Rc
                cmd = cmd - (R2 - Rc) @ v
                Rc = R2
        self.say(f"dripper pushed to {d * 1000:.0f} mm down")
        yield Segment({"right": [
            W(0.5, tuple(cmd - 0.002 * up), FINGER_CLOSED, "ease off", Rc),
            W(0.6, tuple(cmd - 0.002 * up), FINGER_OPEN, "let go", Rc),
            W(1.0, tuple(target + 0.08 * up), FINGER_OPEN, "lift clear", Rx)]})
        seated = rt is not None and rt.line.seated is not None
        self.say(f"dripper {'seated' if seated else 'NOT seated'}")
        yield Segment({"right": 2.0}, min_s=1.5)
