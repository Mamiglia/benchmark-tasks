"""Closed-loop scripted demo for rotary-dial-pencil, played by ``yam_scripted`` (see ``dialkit._patch_scripted``).

The right arm does everything; the left arm stays at READY.

1. **Pick up the pencil** from the top, 35 mm from its middle toward the eraser, fingers closing across it; lift.
2. **Turn it eraser-down:** the wrist pitches the gripper from pointing down to pointing forward, so the pencil
   hangs vertically below the fingers, eraser down, about 60 mm out.
3. **Each digit:** the eraser goes 30 mm straight above the digit's hole and straight down into it, 3.5 mm above
   the dial face (2.5 mm above the number plate). It is then dragged clockwise round the hole circle (37 mm
   radius) at up to 90 deg/s, the commanded point kept at most 6 degrees ahead of the hole, until it presses on the
   finger stop (the commanded point 2 degrees past the contact). Once the wheel has stood at the stop for 0.25 s
   the eraser lifts straight up out of the hole (30 mm, along the pencil), and the controller waits for the wheel to
   run back to rest.
4. After the third digit the pencil is lifted 8 cm clear and held.

Every control step (20 Hz) it reads the wheel angle, the pencil's pose and the gripper's pose (simulator state; a
person sees them) and commands the gripper so that the pencil's eraser end, where the pencil now sits in the
fingers, goes to the wanted point: the offset between gripper and eraser is measured each step, so a pencil that
slips or turns in the fingers is followed.

``DEMO_CHEAT`` (contrast runs): ``short`` lets go of each digit 25 degrees before the stop (the naive "pull it most
of the way round" dial); ``hold`` keeps the eraser in the hole and rides the wheel back at 60 deg/s instead of
letting it run back on its own.
"""

from __future__ import annotations

import math
import os

import numpy as np
from yam_arena import ik
from yam_arena.constants import READY
from yam_arena.demo import DOWN_GRIP
from yam_arena.urdf import quat_xyzw_to_matrix

import dialkit
import dialmodel as M
import task

GRIP_CLOSED = 0.0
GRIP_FROM_MID = 0.035  # grip 35 mm from the middle: the eraser end about 60 mm below the pads
IN_H, HOVER_UP, LIFT = 0.0035, 0.03, 0.03
DRAG_SPEED, LEAD_MAX, PRESS = 90.0, 6.0, 2.0
INIT_R = [0.28, 1.58, 0.05, 1.53, -0.51, 0.0]


def rodrigues(axis, ang) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


class Controller:
    def __init__(self, bases, dt):
        self.dt = dt
        self.base = np.asarray(bases["right"], float)
        self.cheat = os.environ.get("DEMO_CHEAT", "")
        self.R_down = quat_xyzw_to_matrix(np.asarray(DOWN_GRIP))
        # pitch about the closing axis until the eraser end (toward the arms, -x) points straight down
        x_g = self.R_down[:, 0]
        for s in (1, -1):
            R = rodrigues(x_g, s * math.pi / 2) @ self.R_down
            if (R @ (self.R_down.T @ np.array([-1.0, 0, 0])))[2] < -0.9:
                self.R_front = R
        self.say(f"front grip approach {np.round(self.R_front[:, 2], 2)}")
        c = np.asarray(task.PENCIL.pos, float)
        self.grasp = c + np.array([-GRIP_FROM_MID, 0, 0])
        self.grasp[2] = task.TOP + task.PAD_SIZE[2] + 0.0145  # pad tips 1 mm over the notepad
        self.q = np.asarray(INIT_R, float)
        self.plan = []  # joint waypoints for the pick
        for p in (self.grasp + [0, 0, 0.08], self.grasp, self.grasp, self.grasp + [0, 0, 0.10]):
            self.q = self._ik(p, self.R_down, self.q, 300)
            self.plan.append(self.q.copy())
        self.phase, self.t_phase = "pick", 0.0
        self.digit_i = 0
        self.o_g = None  # eraser offset in the gripper frame
        self.tip_cmd = None
        self.finished_at = None
        self.psi_cmd = None
        self.k = 0

    def say(self, msg):
        print(f"[dialdemo] {msg}", flush=True)

    def _ik(self, p, R, q0, iters=60):
        q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(R), q_init=q0, base_pos=self.base, iters=iters)
        if pe > 0.003:
            self.say(f"IK miss {pe * 1000:.1f} mm at {np.round(p, 3)}")
        return q

    # -- geometry
    def eraser(self):
        pos, R = dialkit.SNAP.pencil
        return pos + R @ np.array([-task.PENCIL_LEN / 2, 0, 0]), R

    def dial_world(self, psi, h, r=M.HOLE_R):
        return dialkit.SNAP.dial_origin + dialkit.SNAP.dial_R @ M.dial_point(psi, r, h)

    def tcp_for_tip(self, tip, R):
        """Gripper TCP that puts the eraser at ``tip`` (offset as now measured)."""
        tcp, Rg = dialkit.SNAP.tcp["right"]
        e, _ = self.eraser()
        o = Rg.T @ (e - tcp)
        self.o_g = o if self.o_g is None else 0.8 * self.o_g + 0.2 * o
        return np.asarray(tip) - R @ self.o_g

    # -- main loop
    def act(self, obs):
        t = float(obs.state_time)
        snap = dialkit.SNAP
        grip = GRIP_CLOSED
        q_left = np.asarray(READY, float)
        tp = t - self.t_phase
        n = M.hole_for_digit(M.NUMBER[min(self.digit_i, 2)])

        def go(phase):
            self.phase, self.t_phase = phase, t
            self.say(f"t {t:6.2f} -> {phase} (digit {self.digit_i}, theta {snap.theta:.1f}, digits {snap.digits}, "
                     f"hand clear {snap.hand_clear * 1000:.0f} mm)")

        if self.phase == "pick":
            segs = [(2.0, np.asarray(READY, float), self.plan[0], 1.0), (1.5, self.plan[0], self.plan[1], 1.0),
                    (0.8, self.plan[1], self.plan[2], 0.0), (1.2, self.plan[2], self.plan[3], 0.0)]
            tt = tp
            for dur, a, b, g in segs:
                if tt <= dur:
                    f = ik.smoothstep(tt / dur)
                    q = a + f * (b - a)
                    grip = g if g == 1.0 else 1.0 - min(tt / dur * 1.5, 1.0)
                    if dur == 1.2:
                        grip = 0.0
                    break
                tt -= dur
            else:
                q = self.plan[3]
                go("turn")
                self.q = q.copy()
                tcp, Rg = snap.tcp["right"]
                e, _ = self.eraser()
                self.o_g = Rg.T @ (e - tcp)
                self.turn_from = tcp.copy()
            self.q = q
        elif self.phase == "turn":  # pitch to eraser-down while moving over the first hole
            T = 3.0
            f = ik.smoothstep(min(tp / T, 1.0))
            R = ik.slerp_matrix(self.R_down, self.R_front, f)
            tip_end = self.dial_world(M.hole_psi(n, 0.0), IN_H) + [0, 0, HOVER_UP + 0.03]
            tip_now = self.turn_from + R @ self.o_g
            p_tip = (1 - f) * tip_now + f * tip_end
            p = p_tip - R @ self.o_g
            self.q = self._ik(p, R, self.q)
            if tp >= T + 0.5:
                go("move")
                self.tip_from = self.eraser()[0]
        elif self.phase in ("move", "enter", "drag", "press", "lift", "wait", "away"):
            self.q = self._servo(t, tp, n, go)
        q = self.q
        self.k += 1
        if os.environ.get("DEMO_TRACE") and self.phase in ("enter", "drag", "press", "lift"):
            e, _ = self.eraser()
            b = snap.dial_R.T @ (e - snap.dial_origin)
            self.say(f"trace t {t:6.2f} {self.phase} theta {snap.theta:6.1f} eraser r {math.hypot(b[0], b[1]) * 1000:5.1f} "
                     f"psi {math.degrees(math.atan2(b[1], b[0])):6.1f} h {b[2] * 1000:5.1f} cmd {self.psi_cmd} "
                     f"o_g {np.round(self.o_g * 1000, 1)} tcp err "
                     f"{np.round((getattr(self, 'tcp_cmd', snap.tcp['right'][0]) - snap.tcp['right'][0]) * 1000, 1)}")
        if self.k % 10 == 0 and self.phase not in ("pick",):
            e, _ = self.eraser()
            self.say(f"t {t:6.2f} {self.phase} theta {snap.theta:6.1f} eraser {np.round(e, 4)} "
                     f"o_g {np.round(self.o_g * 1000, 1)} clear {snap.hand_clear * 1000:.0f} mm")
        return (np.r_[q_left, 1.0], np.r_[q, grip], self.finished_at)

    def _servo(self, t, tp, n, go):
        snap = dialkit.SNAP
        R = self.R_front
        th = snap.theta
        if self.phase == "move":  # to 25 mm over the hole, at rest
            T = 1.2
            f = ik.smoothstep(min(tp / T, 1.0))
            goal = self.dial_world(M.hole_psi(n, 0.0), IN_H) + [0, 0, HOVER_UP]
            tip = (1 - f) * self.tip_from + f * goal
            if tp > T + 0.3:
                go("enter")
        elif self.phase == "enter":
            T = 1.0
            f = ik.smoothstep(min(tp / T, 1.0))
            b = self.dial_world(M.hole_psi(n, th), IN_H)
            a = b + [0, 0, HOVER_UP]
            tip = (1 - f) * a + f * b
            if tp > T + 0.2:
                go("drag")
                self.psi_cmd = M.hole_psi(n, th)
        elif self.phase in ("drag", "press"):
            contact = M.hole_psi(n, M.hole_theta(n)) - M.HOLE_LAG / 2  # eraser centre at the stop
            stop_at = M.hole_theta(n) - (25.0 if self.cheat == "short" else 0.0)
            floor = contact - PRESS if self.cheat != "short" else M.hole_psi(n, stop_at) - M.HOLE_LAG / 2
            hole_now = M.hole_psi(n, th) - M.HOLE_LAG / 2
            self.psi_cmd = max(self.psi_cmd - DRAG_SPEED * self.dt, hole_now - LEAD_MAX, floor)
            tip = self.dial_world(self.psi_cmd, IN_H)
            if self.phase == "drag" and th >= stop_at - 3.0:
                go("press")
            elif self.phase == "press" and tp > 0.25:
                go("lift")
                self.lift_from = tip
        elif self.phase == "lift":
            T = 0.3
            f = min(tp / T, 1.0)
            if self.cheat == "hold":  # ride the wheel back, slowly, in the hole
                self.psi_cmd = min(self.psi_cmd + 60.0 * self.dt, M.hole_psi(n, 0.0))
                tip = self.dial_world(self.psi_cmd, IN_H)
                done = th < 1.0
            else:
                tip = self.lift_from + np.array([0.0, 0.0, LIFT]) * f  # straight up, along the pencil
                done = tp > T
            if done:
                go("wait")
                self.tip_from = tip
        elif self.phase == "wait":
            tip = self.tip_from
            if tp > 0.4 and th < 1.0 and abs(snap.omega) < 5.0:
                self.digit_i += 1
                if self.digit_i >= len(M.NUMBER):
                    go("away")
                else:
                    go("move")
                    self.tip_from = self.eraser()[0]
        else:  # away
            tip = self.tip_from + np.array([0, 0, 0.08]) * min(tp / 1.5, 1.0)
            if tp > 2.0 and self.finished_at is None:
                self.finished_at = t
        self.tcp_cmd = self.tcp_for_tip(tip, R)
        return self._ik(self.tcp_cmd, R, self.q)
