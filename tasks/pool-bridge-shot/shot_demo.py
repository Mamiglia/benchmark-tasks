"""shot_demo: the closed-loop scripted demo that yam_scripted runs for this task. NumPy only.

The right arm picks the cue off the near rail at its balance point (pointing down, fingers closing across the cue),
lifts it, and lays it on the aim line: the line from the cue ball's centre through the ghost-ball point (where the cue
ball must be when it touches the object ball for the object ball to head into the taped pocket), butt raised 6 degrees
to clear the end rail. While the tip goes down, the hand steers the measured tip onto its path (the arm sags under the
cue). The left arm comes down over the shaft 17 cm behind the cue ball and closes its fingers to a light pinch (about
3 N per finger) that the smooth shaft slides through: the bridge. It then shifts across the line until the tip is on
it, while the hand turns the cue about the bridge to line it up (cue pose from simulator state, as a camera tracker
would give it). The hand draws the cue back 5 cm, pauses, and strokes it through the bridge at ``POOL_V`` (m/s, 0.38),
its joint commands leading each joint's PD lag so the stroke stays straight, stopping 3 cm past the contact point.

``DEMO_MODE`` plays the loophole checks: ``push`` (rolls the cue ball forward slowly with the tip in contact: a push),
``hand`` (the left gripper sweeps the object ball into the pocket). ``POOL_V`` and ``POOL_FOLLOW`` play the pace
checks (a hard stroke scratches; a long follow-through hits the cue ball twice).
"""

from __future__ import annotations

import math
import os

import numpy as np
from inspect_robots_yam.packing import pack

from yam_arena import ik
from yam_arena.constants import DOWN_XYZW, READY
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

R_DOWN = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.pi) @ quat_xyzw_to_matrix(np.array(DOWN_XYZW))
RX = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), -math.pi / 2) @ R_DOWN  # pointing down, fingers close along world x
GRIP_CUE = 0.12  # on the 25 mm butt: about 35 N per finger
BRIDGE = 0.125  # fingers set 11.8 mm apart on the 13 mm shaft: a light pinch (about 3 N per finger) it slides through
V = float(os.environ.get("POOL_V", "0.38"))  # cue speed at contact (command)
FOLLOW = float(os.environ.get("POOL_FOLLOW", "0.03"))  # m: the stroke's command ends this far past the contact point
ELEV = math.radians(float(os.environ.get("POOL_ELEV", "6")))
BRIDGE_D = 0.17  # bridge this far behind the cue ball's centre
GRIP_TO_BRIDGE = 0.44  # m, hand to bridge along the cue
BRIDGE_LEVER = 0.44 / (0.44 + 0.13)  # bridge motion per tip motion (the cue pivots about the hand)
LEADCOMP = os.environ.get("POOL_LEAD", "1") == "1"
DEBUG = bool(os.environ.get("POOL_DEBUG"))


def smooth(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


# Each joint drive is a PD toward its target (i2rt gains), so a joint trails a moving target by about d/k: 0.06 s at the
# shoulder (80 N m/rad, 5 N m s/rad), 0.15 s at the wrist (10, 1.5). Joints trailing by different times bend a straight
# stroke sideways, so during the stroke the command leads each joint by its own lag.
LAG_S = np.array([5 / 80] * 3 + [1.5 / 10] * 3)


class Arm:
    def __init__(self, base, dt):
        self.base = np.asarray(base, float)
        self.q = np.array(READY, float)
        self.ready_pos, self.ready_rot = ik.tcp_pose(self.q, self.base)
        self.dt, self.lead = dt, False

    def to(self, pos, rot, g) -> np.ndarray:
        q_prev = self.q.copy()
        self.q, self.err, _ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base,
                                       iters=40)
        q = self.q + LAG_S * (self.q - q_prev) / self.dt if self.lead else self.q
        return np.r_[np.clip(q, ik.LOWER, ik.UPPER), g]


def cue_axis(quat) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(quat, float))[:, 0]


class Controller:
    def __init__(self, bases, dt, cue_pos, cue, cb_xy, ob_xy, ball_z, radius, pocket, felt, mode=""):
        self.R, self.L = Arm(bases["right"], dt), Arm(bases["left"], dt)
        self.dt, self.mode, self.t = dt, mode, 0.0
        self.cue_p, self.r, self.felt = cue, radius, felt
        self.pick = np.asarray(cue_pos, float)
        self.cb = np.r_[cb_xy, ball_z]
        ob = np.r_[ob_xy, ball_z]
        # aim: the object ball must leave toward the middle of the pocket hole (3 cm inside the corner)
        aim = np.r_[np.asarray(pocket, float) + np.array([0.03, -0.03]), ball_z]
        d = (aim - ob) / np.linalg.norm(aim - ob)
        ghost = ob - 2 * radius * d
        self.aim = aim
        u = ghost - self.cb
        self.u = u / np.linalg.norm(u)  # stroke direction (level)
        self.ob = ob
        tilt = axis_angle_matrix(np.cross(self.u, [0.0, 0.0, 1.0]), -ELEV)  # tip down, butt up
        yaw = math.atan2(self.u[1], self.u[0]) - math.pi / 2
        self.rot = tilt @ axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw) @ RX
        self.axis = tilt @ self.u  # tip direction, pointing slightly down
        self.off = None  # tip from the tool point, in the gripper frame
        self.corr = np.zeros(3)
        self.bcorr = np.zeros(3)
        self.want_prev = None
        self.tip = None
        self.log_t = -1.0

    # tip position on the line: s along the line from the contact point (s = 0: tip touching the ball)
    def tip_at(self, s: float, lift: float = 0.0) -> np.ndarray:
        contact = self.cb - self.axis * (self.r + self.cue_p.tip_r * 0.0)
        return contact + self.axis * s + np.array([0.0, 0.0, lift])

    def tcp_for_tip(self, tip, rot) -> np.ndarray:
        return tip - rot @ self.off

    def act(self, observation, states):
        self.t += self.dt
        right = self.right(states)
        left = self.left()
        return pack(left, right), self.t > 26.0

    # timeline (s)
    T_LOWER, T_ADDR, T_PINCH, T_AIM, T_BACK, T_PAUSE, T_AWAY = 8.0, 9.5, 12.0, 14.0, 15.0, 15.5, 22.0

    def right(self, states) -> np.ndarray:
        t, arm = self.t, self.R
        if self.mode == "hand":
            return arm.to(arm.ready_pos, arm.ready_rot, 1.0)
        grip = self.pick + np.array([0.0, 0.0, 0.001])
        above = grip + np.array([0.0, 0.0, 0.08])
        r0 = RX
        if t < 2.5:
            s = smooth(t / 2.3)
            return arm.to(arm.ready_pos + (above - arm.ready_pos) * s, ik.slerp_matrix(arm.ready_rot, r0, s), 1.0)
        if t < 3.7:
            return arm.to(above + (grip - above) * smooth((t - 2.5) / 1.0), r0, 1.0)
        if t < 4.4:
            return arm.to(grip, r0, 1.0 + (GRIP_CUE - 1.0) * smooth((t - 3.7) / 0.5))
        cue = states["cue_ball"]
        cpos, cq = np.asarray(cue.cue_pos), np.asarray(cue.cue_quat_xyzw)
        tip = cpos + cue_axis(cq) * self.cue_p.com_from_tip
        self.tip, self.ax = tip, cue_axis(cq)
        if self.off is None:
            tcp, rot = ik.tcp_pose(arm.q, arm.base)
            self.off = rot.T @ (tip - tcp)
            self.lift0 = tcp.copy()
        if t < 5.2:  # lift
            return arm.to(self.lift0 + np.array([0.0, 0.0, 0.07]) * smooth((t - 4.4) / 0.7), r0, GRIP_CUE)
        hi = self.tip_at(-0.08, 0.05)
        tip_start = self.lift0 + np.array([0.0, 0.0, 0.07]) + r0 @ self.off
        addr = self.tip_at(-0.04)
        if t < self.T_ADDR:  # carry the tip to above the line behind the cue ball, then lower it onto the line
            if t < self.T_LOWER:
                s = smooth((t - 5.2) / 2.6)
                want, rot = tip_start + (hi - tip_start) * s, ik.slerp_matrix(r0, self.rot, s)
            else:
                want, rot = hi + (addr - hi) * smooth((t - self.T_LOWER) / 1.3), self.rot
            if t > self.T_LOWER:  # the hand sags under the cue: steer the measured tip onto the path
                self.corr = np.clip(self.corr - 0.2 * (tip - self.want_prev), -0.06, 0.06)
            self.want_prev = want
            return arm.to(self.tcp_for_tip(want + self.corr, rot), rot, GRIP_CUE)
        if self.mode == "push":
            return self.push_shot()
        if self.T_PINCH < t < self.T_AIM:
            # the bridge holds the shaft: the hand now only turns the cue about the bridge onto the line (moving the
            # butt across) and sets the tip's distance behind the ball (moving along the cue)
            e_ax = self.ax - self.axis
            e_ax -= self.axis * float(e_ax @ self.axis)
            self.corr = np.clip(self.corr + 0.15 * e_ax * GRIP_TO_BRIDGE
                                - 0.2 * self.axis * float((tip - addr) @ self.axis), -0.10, 0.10)
        arm.lead = LEADCOMP and t >= self.T_AIM
        if t < self.T_AIM:
            s = -0.04
        elif t < self.T_BACK:
            s = -0.04 - 0.05 * smooth((t - self.T_AIM) / 1.0)
        elif t < self.T_PAUSE:
            s = -0.09
        else:  # forward: ramp to V over 3 cm, steady through contact, stop 2 cm past it
            tt = t - self.T_PAUSE
            ramp = 2 * 0.03 / V
            s = -0.09 + 0.5 * V * tt**2 / ramp if tt < ramp else min(-0.06 + V * (tt - ramp), FOLLOW)
        if DEBUG and t - self.log_t > 0.099:
            self.log_t = t
            print(f"[pool] t {t:5.2f} s {s:+.3f} tip {np.round(tip, 4)} want {np.round(self.tip_at(s), 4)} "
                  f"cmd {np.round(self.tcp_for_tip(self.tip_at(s) + self.corr, self.rot), 3)} ikerr {arm.err:.4f} ax {np.round(self.ax, 3)} corr {np.round(self.corr, 4)} bcorr {np.round(self.bcorr, 4)} "
                  f"v {np.round(np.asarray(cue.lin_vel), 3)} strikes {cue.strikes} hit {cue.hit} foul {cue.foul} "
                  f"{cue.foul_reason} pocket {states['object_ball'].pocket}", flush=True)
        return arm.to(self.tcp_for_tip(self.tip_at(s) + self.corr, self.rot), self.rot, GRIP_CUE)

    def push_shot(self) -> np.ndarray:
        """Loophole check: put the tip on the ball and roll it forward 30 cm at 0.15 m/s, tip in contact."""
        tt = self.t - self.T_ADDR
        s = -0.04 + min(0.15 * max(tt - 1.0, 0.0), 0.34)
        return self.R.to(self.tcp_for_tip(self.tip_at(s) + self.corr, self.rot), self.rot, GRIP_CUE)

    def hand(self) -> np.ndarray:
        """Loophole check: the left gripper (fingers closed) sweeps the object ball into the pocket."""
        t, arm = self.t, self.L
        d = self.aim - self.ob
        d /= np.linalg.norm(d)
        behind = self.ob - d * 0.045
        p0 = behind + np.array([0.0, 0.0, 0.08])
        goal = self.ob + d * 0.12
        if t < 3.0:
            s = smooth(t / 2.8)
            return arm.to(arm.ready_pos + (p0 - arm.ready_pos) * s, ik.slerp_matrix(arm.ready_rot, RX, s), 0.0)
        if t < 4.5:
            return arm.to(p0 + (behind - p0) * smooth((t - 3.0) / 1.4), RX, 0.0)
        return arm.to(behind + (goal - behind) * smooth((t - 4.5) / 3.0), RX, 0.0)

    # -- the bridge hand ----------------------------------------------------------------------------------------------

    def left(self) -> np.ndarray:
        t, arm = self.t, self.L
        if self.mode == "hand":
            return self.hand()
        b = self.tip_at(-BRIDGE_D + self.r)  # on the cue's planned axis, BRIDGE_D behind the ball centre
        above = b + np.array([0.0, 0.0, 0.08])
        if t < self.T_LOWER:
            return arm.to(arm.ready_pos, arm.ready_rot, 1.0)
        if t < 10.0:
            s = smooth((t - self.T_LOWER) / 2.0)
            return arm.to(arm.ready_pos + (above - arm.ready_pos) * s, ik.slerp_matrix(arm.ready_rot, self.rot, s), 1.0)
        if t < 11.3:
            return arm.to(above + (b - above) * smooth((t - 10.0) / 1.2), self.rot, 1.0)
        if t < self.T_PINCH:
            return arm.to(b, self.rot, 1.0 + (BRIDGE - 1.0) * smooth((t - 11.3) / 0.6))
        if t < self.T_AIM and self.tip is not None:  # move the bridge across the line until the tip is on it
            e = self.tip - self.tip_at(-0.04)
            e -= self.axis * float(e @ self.axis)
            self.bcorr = np.clip(self.bcorr - 0.15 * e * BRIDGE_LEVER, -0.03, 0.03)
        if t < self.T_AWAY:
            return arm.to(b + self.bcorr, self.rot, BRIDGE)
        s = smooth((t - self.T_AWAY) / 1.5)
        return arm.to(b + self.bcorr + np.array([0.0, 0.0, 0.10]) * s, self.rot, BRIDGE + (1.0 - BRIDGE) * s)
