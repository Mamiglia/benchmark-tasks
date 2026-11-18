"""Closed-loop scripted demo for table-football-pass, played by ``yam_scripted`` (see ``tfkit._patch_scripted``).

1. **Grip the handles.** Each arm brings its gripper level, pointing along its rod, 7 cm short of the handle; moves
   in along the rod and closes on the rubber handle. Rolling the wrist (joint 6) turns the rod; moving the hand
   along the rod slides it (joint targets from IK on a grid of slide positions).
2. **Line up.** The left arm slides the 5-bar so its nearest figure is behind the ball. The right arm slides the
   3-bar so its middle figure is on the ball's lane and lifts its figures forward (feet up) so the ball can pass
   under them.
3. **Pass.** The left wrist swings the 5-bar back, then snaps it forward: the figure's foot strikes the ball toward
   the 3-bar, through the gap between two blue 5-bar figures.
4. **Trap (front pin).** Every control step the right arm reads the ball's position and velocity (simulator state;
   a player watches the ball) and keeps its middle figure on the ball's lane. When the ball is about to be under the
   raised foot, it drops the foot onto the ball: the ball is pinned against the playfield in front of the rod.
5. **Shoot.** After the ball has been still for 1.3 s, the right wrist rolls the figure back off the ball, then snaps
   it forward: the foot strikes the ball into the goal past the blue 2-bar and goalie.

``DEMO_CHEAT`` (contrast runs): ``direct`` shoots for the goal from the 5-bar (the 3-bar stays out of the way);
``hand`` reaches over the near wall for the ball with the right gripper; ``through`` passes and keeps the 3-bar
lifted (it swings without coming down), so the ball rolls on under it.
"""

from __future__ import annotations

import math
import os

import numpy as np
from yam_arena import ik
from yam_arena.constants import READY

import tfkit
import tfmodel as M

ARMS = ("left", "right")
ROD = {"left": 4, "right": 6}
GRIP = float(os.environ.get("TF_GRIP", 0.18))  # normalised finger command on the 28 mm handle
PRE = 0.07
INITS = {"left": [-0.97, 1.82, 0.3, 1.52, -0.97, 0.0], "right": [0.97, 1.82, 0.3, 1.52, 0.97, 0.0]}
KI = 3.0  # 1/s: wrist-roll integral on the rod's angle error (the handle slips a little in the grip)
SLOW = ("lift", "side", "sback", "behind")
S_GRID = np.round(np.arange(-0.075, 0.0751, 0.005), 4)
P = {k: float(os.environ.get(k, v)) for k, v in dict(
    TF_BACK=30.0, TF_KICK=70.0, TF_RAISE=64.0, TF_PIN_D=0.045, TF_PIN_PRESS=8.0, TF_LEAD=0.18, TF_SBACK=12.0,
    TF_SKICK=75.0, TF_SIDE=0.040, TF_LIFT=3.0).items()}


def grip_rot() -> np.ndarray:
    """Gripper level, approach (z) along +x (the rod), fingers closing horizontally, gripper y down."""
    z = np.array([1.0, 0.0, 0.0])
    x = np.array([0.0, 1.0, 0.0])
    y = np.cross(z, x)
    R = np.c_[x, y, z]
    return R if R[2, 1] < 0 else R @ np.diag([-1.0, -1.0, 1.0])


def touch_angle(d: float) -> float:
    """Forward tilt (rad) at which a figure's foot first touches a ball resting d metres in front of the rod."""
    ball = np.array([M.X_C, M.rod_y(6) - d, M.FLOOR_Z + M.BALL_R])

    def touching(phi):
        return M.figure_gap(6, ball, 0.0, -phi) <= 0

    hi = math.radians(89)
    if touching(hi):
        return hi
    lo = hi
    while lo > 0 and not touching(lo):
        lo -= math.radians(1)
    if lo <= 0:
        return 0.0
    for _ in range(30):
        mid = (lo + hi) / 2
        if touching(mid):
            lo = mid
        else:
            hi = mid
    return lo


class Controller:
    def __init__(self, bases, dt):
        self.dt = dt
        self.bases = {a: np.asarray(bases[a], float) for a in ARMS}
        self.cheat = os.environ.get("DEMO_CHEAT", "")
        self.R = grip_rot()
        self.quat = ik.matrix_to_quat_xyzw(self.R)
        self.plan = {a: self._approach(a) for a in ARMS}
        self.grid = {a: self._grid(a) for a in ARMS}
        self.t_grip = None
        self.phase = {"left": "align", "right": "align"}
        self.t_phase = dict.fromkeys(ARMS, 0.0)
        self.s_cmd = dict.fromkeys(ARMS, 0.0)
        self.phi_cmd = dict.fromkeys(ARMS, 0.0)
        self.integ = dict.fromkeys(ARMS, 0.0)
        self.k = 0
        self.finished_at = None
        self.t_still = None
        self.pin_phi = touch_angle(P["TF_PIN_D"])
        self.say(f"pin at {P['TF_PIN_D'] * 1000:.0f} mm: touch angle {math.degrees(self.pin_phi):.1f} deg; params {P}")

    def say(self, msg):
        print(f"[tfdemo] {msg}", flush=True)

    def _grip_point(self, arm, s=0.0):
        return np.array([M.HANDLE_X + s, M.rod_y(ROD[arm]), M.ROD_Z])

    def _approach(self, arm):
        """Joint waypoints: READY -> 7 cm short of the handle on its axis -> on the handle (straight in)."""
        p = self._grip_point(arm)
        q, pe, re = ik.solve(p - [PRE + 0.03, 0, -0.10], self.quat, q_init=INITS[arm], base_pos=self.bases[arm],
                             iters=400)
        self.above = getattr(self, "above", {})
        self.above[arm] = q
        q, pe, re = ik.solve(p - [PRE, 0, 0], self.quat, q_init=q, base_pos=self.bases[arm], iters=400)
        qs = [q]
        for f in np.linspace(1, 0, 9)[1:]:
            q, pe, re = ik.solve(p - [f * PRE, 0, 0], self.quat, q_init=q, base_pos=self.bases[arm], iters=150)
            qs.append(q)
        self.say(f"{arm}: grip at {np.round(p, 3)}, IK {pe * 1000:.1f} mm {math.degrees(re):.1f} deg")
        return qs

    def _grid(self, arm):
        q0 = self.plan[arm][-1]
        out = {}
        for sgn in (1, -1):
            q = q0.copy()
            for s in S_GRID[::sgn]:
                if sgn * s < 0:
                    continue
                q, pe, _ = ik.solve(self._grip_point(arm, s), self.quat, q_init=q, base_pos=self.bases[arm], iters=150)
                out[float(s)] = q.copy()
        worst = max(np.linalg.norm(ik.tcp_pose(q, self.bases[arm])[0] - self._grip_point(arm, s))
                    for s, q in out.items())
        self.say(f"{arm}: slide grid {len(out)} points, worst IK {worst * 1000:.1f} mm")
        return np.array([out[float(s)] for s in S_GRID])

    def _q(self, arm, s, phi):
        """Joints holding the handle at slide s (relative to where it was gripped) with forward tilt phi."""
        i = np.interp(s, S_GRID, np.arange(len(S_GRID)))
        a = int(min(math.floor(i), len(S_GRID) - 2))
        q = self.grid[arm][a] + (i - a) * (self.grid[arm][a + 1] - self.grid[arm][a])
        q = q.copy()
        q[5] = self.plan[arm][-1][5] - phi  # wrist roll +a turns the rod +a about +x; forward tilt is -theta
        return q

    def _go(self, arm, phase, t):
        self.phase[arm], self.t_phase[arm] = phase, t
        self.say(f"t {t:6.2f} {arm} -> {phase}  ball {np.round(tfkit.SNAP.ball * 1000, 1)} "
                 f"v {np.round(tfkit.SNAP.ball_vel, 2)} stage {tfkit.SNAP.stage}")

    @staticmethod
    def _ramp(a, b, t, T):
        return a + (b - a) * ik.smoothstep(min(max(t / T, 0.0), 1.0))

    def _left(self, t, snap):
        ph, tp = self.phase["left"], t - self.t_phase["left"]
        s0 = snap.rods[4][0]
        if ph == "align":
            if not hasattr(self, "s5_goal"):
                bx = snap.ball[0]
                cands = [bx - (M.X_C + fx) for fx in M.FIG_X[4]]
                cands = [c for c in cands if abs(c) < M.TRAVEL[4] - 0.003]
                self.s5_goal = min(cands, key=abs)
                self.s5_from = self.s_cmd["left"]
                self.say(f"5-bar slide to {self.s5_goal * 1000:.1f} mm")
            self.s_cmd["left"] = self._ramp(self.s5_from, self.s5_goal + (self.s_cmd["left"] - s0) * 0, tp, 1.2)
            if tp > 1.6 and (self.phase["right"] in ("wait", "track") or self.cheat == "direct"):
                self._go("left", "back", t)
        elif ph == "back":
            self.phi_cmd["left"] = self._ramp(0.0, -math.radians(P["TF_BACK"]), tp, 0.5)
            if tp > 0.8:
                self._go("left", "kick", t)
        elif ph == "kick":
            self.phi_cmd["left"] = math.radians(P["TF_KICK"])
            if tp > 0.5:
                self._go("left", "rest", t)
        elif ph == "rest":
            self.phi_cmd["left"] = self._ramp(math.radians(P["TF_KICK"]), 0.0, tp, 1.0)

    def _right(self, t, snap):
        ph, tp = self.phase["right"], t - self.t_phase["right"]
        ball, v = snap.ball, snap.ball_vel
        s_lane = float(np.clip(ball[0] - M.X_C, -M.TRAVEL[6] + 0.003, M.TRAVEL[6] - 0.003))
        if self.cheat == "direct":
            self.phi_cmd["right"] = self._ramp(0.0, math.radians(80), tp, 1.0)
            return
        if ph == "align":
            self.s_cmd["right"] = self._ramp(0.0, s_lane, tp, 1.2)
            self.phi_cmd["right"] = self._ramp(0.0, math.radians(P["TF_RAISE"]), tp, 1.2)
            if tp > 1.4:
                self._go("right", "wait", t)
        elif ph == "wait":
            self.phi_cmd["right"] = math.radians(P["TF_RAISE"])
            if v[1] < -0.05 and ball[1] < M.OPP5_Y + 0.02:
                self._go("right", "track", t)
        elif ph == "track":
            # keep the middle figure on the lane the ball will cross the 3-bar at
            ty = max((ball[1] - M.THREE_Y) / min(v[1], -0.05), 0.0)
            self.s_cmd["right"] = float(np.clip(ball[0] + v[0] * ty - M.X_C, -M.TRAVEL[6] + 0.003,
                                                M.TRAVEL[6] - 0.003))
            if self.cheat == "through":
                if ball[1] < M.THREE_Y + 0.06:
                    self._go("right", "skick", t)
                return
            d_now = M.THREE_Y - ball[1]
            if d_now + (-v[1]) * P["TF_LEAD"] >= P["TF_PIN_D"]:
                self._go("right", "pin", t)
        elif ph == "pin":
            self.phi_cmd["right"] = self.pin_phi - math.radians(P["TF_PIN_PRESS"])
            if snap.stage == "trapped":
                self.t_still = self.t_still or t
            if self.t_still is not None and t - self.t_still > 0.3:
                self._go("right", "lift", t)
        elif ph == "lift":
            # ease the foot up off the ball's top (it stays where it was stopped)
            self.phi_cmd["right"] = self._ramp(self.pin_phi - math.radians(P["TF_PIN_PRESS"]),
                                               self.pin_phi + math.radians(P["TF_LIFT"]), tp, 0.25)
            if tp > 0.3:
                self._go("right", "side", t)
        elif ph == "side":
            # step the figure sideways off the ball (toward the middle of its travel)
            if not hasattr(self, "side_from"):
                self.side_from = self.s_cmd["right"]
                self.side_to = self.side_from - math.copysign(P["TF_SIDE"], self.side_from or 1.0)
                self.ball_x = float(ball[0])
            self.s_cmd["right"] = self._ramp(self.side_from, self.side_to, tp, 0.35)
            if tp > 0.4:
                self._go("right", "sback", t)
        elif ph == "sback":
            # swing the foot down beside the ball to behind the rod line
            self.phi_cmd["right"] = self._ramp(self.pin_phi + math.radians(P["TF_LIFT"]),
                                               -math.radians(P["TF_SBACK"]), tp, 0.35)
            if tp > 0.45:
                self._go("right", "behind", t)
        elif ph == "behind":
            # slide back so the figure is behind the ball, on its lane
            lane = float(np.clip(ball[0] - M.X_C, -M.TRAVEL[6] + 0.003, M.TRAVEL[6] - 0.003))
            self.s_cmd["right"] = self._ramp(self.side_to, lane, tp, 0.35)
            if tp > 0.45:
                self._go("right", "skick", t)
        elif ph == "skick":
            self.phi_cmd["right"] = math.radians(P["TF_SKICK"])
            if tp > 0.8:
                self._go("right", "done", t)
        elif ph == "done":
            self.phi_cmd["right"] = self._ramp(math.radians(P["TF_SKICK"]), 0.0, tp, 1.0)
            if tp > 1.5 and self.finished_at is None:
                self.finished_at = t

    def act(self, obs):
        t = float(obs.state_time)
        snap = tfkit.SNAP
        out = {}
        T_UP, T_PRE, T_IN, T_CLOSE = 2.0, 3.0, 1.5, 0.8
        if t < T_PRE + T_IN + T_CLOSE + 0.3:
            for arm in ARMS:
                qs = self.plan[arm]
                grip = 1.0
                if t < T_UP:
                    q = np.asarray(READY) + ik.smoothstep(t / T_UP) * (self.above[arm] - np.asarray(READY))
                elif t < T_PRE:
                    q = self.above[arm] + ik.smoothstep((t - T_UP) / (T_PRE - T_UP)) * (qs[0] - self.above[arm])
                elif t < T_PRE + T_IN:
                    f = (t - T_PRE) / T_IN * (len(qs) - 1)
                    i = min(int(f), len(qs) - 2)
                    q = qs[i] + (f - i) * (qs[i + 1] - qs[i])
                else:
                    q = qs[-1].copy()
                    grip = max(GRIP, 1.0 - (t - T_PRE - T_IN) / T_CLOSE * (1.0 - GRIP))
                out[arm] = (q, grip)
            if self.cheat == "hand":
                out["left"] = (np.asarray(READY, float), 1.0)
        elif self.cheat == "hand":
            out = self._hand(t, snap)
        else:
            if self.t_grip is None:
                self.t_grip = t
                for a in ARMS:
                    self.t_phase[a] = t
                self.s_at_grip = {a: snap.rods[ROD[a]][0] for a in ARMS}
                self.say(f"gripped at {t:.2f} s, rods {snap.rods}")
            self._left(t, snap)
            self._right(t, snap)
            for arm in ARMS:
                phi = -snap.rods[ROD[arm]][1]
                err = self.phi_cmd[arm] - phi
                if self.phase[arm] in SLOW and abs(err) < math.radians(25):
                    self.integ[arm] = float(np.clip(self.integ[arm] + KI * err * self.dt, -0.25, 0.25))
                out[arm] = (self._q(arm, self.s_cmd[arm] - self.s_at_grip[arm], self.phi_cmd[arm] + self.integ[arm]),
                            GRIP)
            self.k += 1
            if self.k % 4 == 0:
                r = snap.rods
                self.say(f"t {t:6.2f} ball {np.round(snap.ball * 1000, 1)} v {np.round(snap.ball_vel, 2)} "
                         f"5: {r[4][0] * 1000:5.1f}/{self.s_cmd['left'] * 1000:5.1f} mm {-math.degrees(r[4][1]):6.1f}/"
                         f"{math.degrees(self.phi_cmd['left']):6.1f}  3: {r[6][0] * 1000:5.1f}/"
                         f"{self.s_cmd['right'] * 1000:5.1f} mm {-math.degrees(r[6][1]):6.1f}/"
                         f"{math.degrees(self.phi_cmd['right']):6.1f} deg  {snap.stage}")
            if self.cheat == "direct" and self.phase["left"] == "rest" and t - self.t_phase["left"] > 3.0:
                self.finished_at = self.finished_at or t
        return (np.r_[out["left"][0], out["left"][1]], np.r_[out["right"][0], out["right"][1]], self.finished_at)

    def _hand(self, t, snap):
        """Contrast run: the right gripper reaches over the near wall and pushes the ball toward the goal."""
        from yam_arena.demo import DOWN_GRIP

        if not hasattr(self, "_hq"):
            b = np.asarray(M.BALL_START)
            pts = [b + [0, 0.06, 0.12], b + [0, 0.04, 0.01], b + [0, -0.25, 0.01]]
            q, self._hq = np.asarray(READY, float), []
            for p in pts:
                q, pe, _ = ik.solve(p, DOWN_GRIP, q_init=q, base_pos=self.bases["right"], iters=300)
                self._hq.append(q)
            self.t0 = t
        qs, seg = [np.asarray(READY, float), *self._hq], 1.5
        tt = t - self.t0
        i = min(int(tt / seg), len(qs) - 2)
        f = ik.smoothstep(min((tt - i * seg) / seg, 1.0))
        if tt > seg * 4 and self.finished_at is None:
            self.finished_at = t
        return {"left": (np.asarray(READY, float), 1.0), "right": (qs[i] + f * (qs[i + 1] - qs[i]), 0.0)}
