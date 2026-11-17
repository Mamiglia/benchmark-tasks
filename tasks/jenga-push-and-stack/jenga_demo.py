"""jenga_demo: the closed-loop scripted demo that yam_scripted runs for this task. NumPy only.

It uses only the robot's proprioception: the 14-D joint positions in each observation (gripper openings included)
and the commands it sent itself. It knows the nominal start layout (where the tower stands, block size, 18 levels),
as every yam_scripted demo does, never which blocks are loose or the blocks' actual thicknesses. It does not use the
cameras.

Hand force is read the way a position-controlled robot reports it (motor current would give the same): each arm
joint's stiffness times how far it lags its command, mapped to a force at the tool point through the arm's Jacobian.
The tip's real position is the forward kinematics of the measured joints.

One turn, one hand at a time:
1. Left hand, fingers closed, pointing down: lowers onto the top level until it feels 1 N, which gives the tower's
   real height (levels are not exactly 15 mm), and leaves.
2. Left hand, fingers closed, pointing along -y at a candidate block's end face (even levels run along y): advances
   at 6 mm/s. If the fingertip stops while the command runs on more than 5 mm ahead of it (about 1.5 N) within the
   first 3 mm, the block is pinched: back off, try the next candidate (middle blocks of levels 10, 8, 12, 6, 14, 16,
   then their edge blocks). A loose block is pushed 15 mm in (the fingertips are 13 mm thick, the level 15 mm, so
   they can go no deeper without rubbing the levels above and below), so it sticks out 15 mm on the far side. The
   left hand leaves the tower.
3. Right hand, pointing along +y, jaws above and below the protruding end: grips it, draws it straight out along its
   length, turns it to point along x and lays it on the top level, crosswise, 18 mm off centre towards the robot so
   the lower jaw clears the tower's face; lets go and backs away.

``DEMO_MODE`` (environment variable) plays a loophole instead, to check that the scorer gives it nothing:
``noprobe`` (push level 6's middle block 15 mm with no force limit, then carry on as usual), ``brace`` (the right
hand steadies the tower while the left one pushes). ``parallel`` lays the block along the top level instead of across
it (partial credit).
"""

from __future__ import annotations

import os

import numpy as np
from inspect_robots_yam.packing import STATE_KEY, pack

from yam_arena import ik
from yam_arena.constants import READY, SHOULDER, WRIST

K_JOINT = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)  # N m/rad
TIP = 0.013  # pad tips past the tool point
FINGER_BELOW = 0.0033  # touch-off offset: measured tool point minus the touched surface (calibrated once on the fingers, not per tower)
OPEN_G = 1.0
DEBUG = bool(os.environ.get("JENGA_DEBUG"))


def frame(approach, closing) -> np.ndarray:
    z, x = np.asarray(approach, float), np.asarray(closing, float)
    return np.c_[x, np.cross(z, x), z]


DOWN = frame((0, 0, -1), (0, 1, 0))
PUSH = frame((0, -1, 0), (0, 0, 1))  # left hand: pointing along -y, jaws closing vertically
if os.environ.get("GRIP_AXIS") == "x":  # jaws on the block's side faces
    PULL = frame((0, 1, 0), (1, 0, 0))
    PLACE = frame((1, 0, 0), (0, -1, 0))
else:  # jaws on its top and bottom faces
    PULL = frame((0, 1, 0), (0, 0, 1))  # right hand: pointing along +y
    PLACE = frame((1, 0, 0), (0, 0, 1))  # right hand: pointing along +x
PLACE_PAR = frame((0, 1, 0), (0, 0, 1))

PROBE_SPEED = 0.006  # m/s of command
TOUCH_LEAD = 0.002  # touching the top: the tip lags its command this much
TIGHT_LEAD = 0.005  # command this far ahead of the fingertip ...
TIGHT_DEPTH = 0.003  # ... before the tip is this deep: the block is pinched
PUSH_DEPTH = 0.015
STUCK_LEAD = 0.008  # past TIGHT_DEPTH: give up if the command leads by this much
STANDOFF = 0.010
GRIP_GAP = 0.0015  # right pad tips stop this far short of the tower's face
OVERHANG = 0.018  # the placed block overhangs the tower's near face by this
PULL_OUT = 0.095


def hand_force(q_cmd, q_meas, base) -> np.ndarray:
    """Force (N, world) the arm exerts at its tool point, from the joint lags: tau = K (q_cmd - q_meas) = J^T f."""
    q_meas = np.asarray(q_meas, float)
    tau = K_JOINT * (np.asarray(q_cmd, float) - q_meas)
    p0 = ik.tcp_pose(q_meas, base)[0]
    jac = np.zeros((3, 6))
    for i in range(6):
        dq = q_meas.copy()
        dq[i] += 1e-6
        jac[:, i] = (ik.tcp_pose(dq, base)[0] - p0) / 1e-6
    return np.linalg.lstsq(jac.T, tau, rcond=None)[0]


def g_of(opening: float) -> float:
    return float(np.clip(opening / 0.0939, 0.0, 1.0))


class Arm:
    def __init__(self, side: str, base, ctl: "Controller"):
        self.side, self.base, self.ctl = side, np.asarray(base, float), ctl
        self.q = np.array(READY, float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.home = self.pos.copy()
        self.g = OPEN_G

    # -- what the robot measures --
    def meas(self) -> np.ndarray:
        return ik.tcp_pose(self.ctl.joints[self.side][:6], self.base)[0]

    def force(self) -> np.ndarray:
        return hand_force(self.q, self.ctl.joints[self.side][:6], self.base)

    def note(self, msg: str):
        if DEBUG:
            print(f"[jenga_demo] t {self.ctl.t:6.2f} {self.side}: {msg}", flush=True)

    # -- motions --
    def move(self, to, duration: float, rot=None, g: float | None = None):
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

    def wait(self, duration: float):
        for _ in range(max(1, round(duration / self.ctl.dt))):
            yield

    def servo(self, target, duration: float = 1.0, axes=(0, 1, 2)):
        """Shift the command until the measured tool point sits on ``target`` (the compliant arm sags)."""
        target = np.asarray(target, float)
        for _ in range(max(1, round(duration / self.ctl.dt))):
            err = target - self.meas()
            for a in axes:
                self.pos[a] += 0.3 * err[a]
            yield

    # -- 1. how tall is the tower --
    def touch_top(self, tower_xy, nominal_top):
        """Lower the closed fingertips, held as for pushing, onto the top level's edge: the tower's real height."""
        x, y = tower_xy
        face = y + self.ctl.length / 2
        p = np.array([x, face - 0.008 + TIP, nominal_top + FINGER_BELOW + 0.015])  # tips 8 mm in, 15 mm up
        yield from self.move(p + (0.0, 0.06, 0.04), 2.5, rot=PUSH, g=0.0)
        # The compliant arm sags under gravity: settle the measured tool point clear of the tower first, then move
        # relative to that command.
        yield from self.servo(p + (0.0, 0.06, 0.04), 1.5)
        yield from self.move(self.pos - (0.0, 0.06, 0.0), 1.5)
        yield from self.servo(p + (0.0, 0.0, 0.04), 0.8)
        yield from self.move(self.pos - (0.0, 0.0, 0.04), 1.5)
        yield from self.wait(0.8)
        cmd0, meas0, n = self.pos[2], self.meas()[2], 0
        while self.pos[2] > nominal_top - 0.02:
            self.pos = self.pos - (0.0, 0.0, 0.006 * self.ctl.dt)
            yield
            lead = (cmd0 - self.pos[2]) - (meas0 - self.meas()[2])  # command below the measured tip
            n = n + 1 if lead > TOUCH_LEAD else 0
            if n >= 3:
                break
        top = float(self.meas()[2] - FINGER_BELOW)
        self.note(f"top of the tower at {top - self.ctl.table_z:.4f} m above the table")
        yield from self.move(self.pos + (0.0, 0.0, 0.02), 0.6)
        yield from self.move(self.pos + (0.0, 0.07, 0.0), 1.0)
        self.ctl.top = top

    # -- 2. find a loose block and push it through --
    def probe(self, x, z, face_y, limit=True):
        """Push the end face at (x, face_y, z) along -y. Returns via self.result: depth pushed (m) or None."""
        approach = np.array([0.0, -1.0, 0.0])
        start = np.array([x, face_y + STANDOFF + TIP, z])
        yield from self.move(start + (0.0, 0.03, 0.0), 1.0, rot=PUSH, g=0.0)
        yield from self.move(start, 0.6)
        yield from self.servo(start, 0.8)
        yield from self.wait(0.2)
        cmd0 = self.pos.copy()
        meas0 = self.meas()
        self.result = None
        while True:
            self.pos = self.pos + approach * PROBE_SPEED * self.ctl.dt
            for a in (0, 2):  # keep the tip on the end face's centre line
                self.pos[a] += 0.3 * (start[a] - self.meas()[a])
            yield
            tip = self.meas()[1] - TIP
            depth = face_y - tip
            lead = (meas0[1] - self.meas()[1]) - (cmd0[1] - self.pos[1])
            lead = -lead  # command ahead of the measured tip (m)
            if limit and depth < TIGHT_DEPTH and lead > TIGHT_LEAD:
                self.note(f"x {x:+.4f} z {z - self.ctl.table_z:.4f}: pinched (lead {lead*1e3:.1f} mm at depth "
                          f"{depth*1e3:.1f} mm, force {-self.force()[1]:.2f} N)")
                break
            if limit and depth >= TIGHT_DEPTH and lead > STUCK_LEAD:
                self.note(f"x {x:+.4f}: stuck at depth {depth*1e3:.1f} mm")
                break
            if depth >= PUSH_DEPTH:
                self.result = depth
                self.note(f"x {x:+.4f} z {z - self.ctl.table_z:.4f}: loose, pushed {depth*1e3:.1f} mm "
                          f"(force {-self.force()[1]:.2f} N)")
                break
            if depth >= PUSH_DEPTH + 0.004 or (cmd0[1] - self.pos[1]) > PUSH_DEPTH + STANDOFF + 0.02:
                break
        yield from self.move(start + (0.0, 0.01, 0.0), 0.8)
        yield from self.move(start + (0.0, 0.05, 0.0), 0.6)

    def find_and_push(self, ctl, limit=True, only=None):
        cands = only or ctl.candidates()
        for x, z, k, i in cands:
            yield from self.probe(x, z, ctl.tower_xy[1] + ctl.length / 2, limit=limit)
            if self.result is not None:
                ctl.found = (x, z, k, i, self.result)
                break
        yield from self.move(self.home, 2.0, rot=ik.tcp_pose(np.array(READY), self.base)[1], g=OPEN_G)

    # -- 3. draw it out and stack it --
    def take_and_stack(self, ctl, parallel=False):
        x, z, k, i, depth = ctl.found
        face = ctl.tower_xy[1] - ctl.length / 2
        grip = np.array([x, face - GRIP_GAP - TIP, z])
        yield from self.move(grip + (0.0, -0.08, 0.04), 2.0, rot=PULL, g=g_of(0.032))
        yield from self.move(grip + (0.0, -0.06, 0.0), 1.0)
        yield from self.servo(grip + (0.0, -0.06, 0.0), 0.6)
        yield from self.move(self.pos + (0.0, 0.06, 0.0), 1.5)
        yield from self.servo(grip, 0.6)
        self.note(f"at grip: tool point {np.round(self.meas(), 4)} target {np.round(grip, 4)} "
                  f"opening {self.ctl.joints[self.side][6]:.3f}")
        yield from self.move(self.pos, 0.8, g=g_of(0.011))  # close: pads on the top and bottom faces, about 20 N
        yield from self.wait(0.4)
        self.note(f"gripped, opening {self.ctl.joints[self.side][6]:.3f}")
        yield from self.move(self.pos + (0.0, -PULL_OUT, 0.0), 4.0)
        yield from self.move(self.pos + (0.0, -0.03, 0.0), 0.8)
        top = ctl.top
        lay_z = top + ctl.level_h / 2 + 0.0015
        if parallel:
            lay = np.array([ctl.tower_xy[0], ctl.tower_xy[1] - ctl.length / 2 - OVERHANG, lay_z])
            rot = PLACE_PAR
        else:
            lay = np.array([ctl.tower_xy[0] - ctl.length / 2 - OVERHANG, ctl.tower_xy[1], lay_z])
            rot = PLACE
        back = -rot[:, 2]
        yield from self.move(self.pos + (0.0, 0.0, top + 0.05 - self.pos[2]), 2.0)
        yield from self.move(lay + back * 0.05 + (0.0, 0.0, 0.03), 3.0, rot=rot)
        yield from self.move(lay + (0.0, 0.0, 0.03), 1.5)
        yield from self.servo(lay + (0.0, 0.0, 0.03), 0.6)
        yield from self.move(self.pos - (0.0, 0.0, 0.03), 2.0)
        yield from self.servo(lay, 0.6, axes=(0, 1))
        yield from self.wait(0.3)
        yield from self.move(self.pos, 0.8, g=g_of(0.030))  # let go
        yield from self.wait(0.3)
        yield from self.move(self.pos + back * 0.06, 1.2)
        yield from self.move(self.home, 2.5, rot=ik.tcp_pose(np.array(READY), self.base)[1], g=OPEN_G)


class Controller:
    def __init__(self, bases, dt, tower_xy, table_z, top_z, pitch, level_h, levels, mode="", length=0.075):
        self.dt, self.t = float(dt), 0.0
        self.tower_xy, self.table_z, self.pitch, self.level_h, self.levels = tower_xy, table_z, pitch, level_h, levels
        self.length = length
        self.nominal_top = table_z + levels * level_h  # what the box says; the real tower differs
        self.top = self.nominal_top
        self.joints = {"left": np.r_[READY, 1.0], "right": np.r_[READY, 1.0]}
        self.arms = {s: Arm(s, bases[s], self) for s in ("left", "right")}
        self.found = None
        self.mode = mode
        self.script = self.run()
        self.done_t = None

    def level_z(self, k: int) -> float:
        """Centre height of level k, sharing the measured tower height evenly between the levels."""
        return self.table_z + (k - 0.5) * (self.top - self.table_z) / self.levels

    def candidates(self):
        levels = (10, 8, 12, 6, 14, 16)
        mid = [(self.tower_xy[0], self.level_z(k), k, 1) for k in levels]
        edge = [(self.tower_xy[0] + s * self.pitch, self.level_z(k), k, i) for k in levels for i, s in ((0, -1), (2, 1))]
        return mid + edge

    def run(self):
        left, right = self.arms["left"], self.arms["right"]
        if self.mode == "still":  # no robot motion: the tower standing test
            for _ in left.wait(60.0):
                yield
            self.done_t = self.t - 7.0
            while True:
                yield
        yield from left.touch_top(self.tower_xy, self.nominal_top)
        if self.mode == "noprobe":
            only = [(self.tower_xy[0], self.level_z(6), 6, 1)]
            yield from left.find_and_push(self, limit=False, only=only)
            if self.found is None:  # pushed anyway: say it went in
                self.found = (*only[0], 0.015)
        elif self.mode == "brace":
            # the right hand steadies the tower's -y face (level 13) while the left hand pushes
            x, z = self.tower_xy[0], self.level_z(13)
            brace = np.array([x, self.tower_xy[1] - self.length / 2 - TIP + 0.002, z])
            gen_r = right.move(brace + (0.0, -0.05, 0.0), 2.0, rot=PULL, g=0.0)
            for _ in gen_r:
                yield
            for _ in right.move(brace, 1.0):
                yield
            yield from left.find_and_push(self)
            for _ in right.move(brace + (0.0, -0.06, 0.0), 1.0):
                yield
        else:
            yield from left.find_and_push(self)
        if self.found is None:
            left.note("no loose block found")
        else:
            yield from right.take_and_stack(self, parallel=self.mode == "parallel")
        self.done_t = self.t
        while True:
            yield

    def act(self, observation):
        state = np.asarray(observation.state[STATE_KEY], float).reshape(-1)
        self.joints = {"left": state[:7], "right": state[7:14]}
        next(self.script)
        self.t += self.dt
        out = []
        for side in ("left", "right"):
            arm = self.arms[side]
            q, _, _ = ik.solve(arm.pos, ik.matrix_to_quat_xyzw(arm.rot), arm.q, base_pos=arm.base, iters=60)
            arm.q = q
            out.append(np.r_[q, arm.g])
        done = self.done_t is not None and self.t > self.done_t + 7.0
        return pack(*out), done
