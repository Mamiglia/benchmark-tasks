"""grip_demo: the closed-loop scripted demo that yam_scripted runs for this task. NumPy only.

It uses only the robot's proprioception: the 14-D joint positions in each observation (each gripper's measured
opening included) and the commands it sent itself. It knows the start layout of the cups and the tray, as every
yam_scripted demo does, and its own drive gains (i2rt's yam_v1.yml / linear_4310.yml, which yam_arena also uses). It
never reads the fills or any simulator state, and it does not use the cameras (the wrist cameras see only the fingers
during a grasp, and the top camera looks straight down onto the gripper).

Two forces are read the way a position-controlled robot reports them (motor current would give the same):
- grip force: the finger drive's stiffness (5000 N/m) times how far the measured opening lags the command;
- hand load: each arm joint's stiffness times how far it lags its command, mapped to a force at the tool point
  through the arm's Jacobian (least squares). Each reading is a difference: the load 4 to 7 mm up minus the load before
  rising, minus the same difference measured with the fingers open (a tare that removes the arm's own lag). Only
  ratios of these readings are used, so their scale does not matter.

Per cup, each arm:
1. comes down over the cup, fingers open across it (closing along world y);
2. tares the lift test with the fingers still open, then closes slowly until the grip force reads 0.8 N (gentle
   enough for the lightest cup); from then on the gripper regulates its force (force mode);
3. lift tests: raises the hand until its measured height has risen 4 mm (at most 7 mm after settling) (the arm is compliant: a heavier load
   needs a higher command), reads the load, lowers the hand back. While the pads slide up the wall (the cup
   stays on the table) the load is the sliding friction, which grows with the grip; once the cup comes up with the
   hand the load is its weight and stops growing. So it raises the grip 1.4 times after each test until the load
   rises by less than 15 %, then carries at 1.2 x that grip. The pads never slide more
   than about 7 mm;
4. lifts the cup 4.5 cm, carries it to its place on the tray, lowers it until the measured hand height puts the cup
   4 mm above the tray floor, opens, leaves.

``DEMO_CHEAT`` (environment variable) plays a loophole instead, to check that the scorer gives it nothing:
``squeeze`` (grip every cup with the gripper fully closed, its 50 N), ``push`` (shove the cups onto the tray with
closed fingers, never lifting them), ``seal`` (press closed fingertips down on each seal and drag the cup there).
"""

from __future__ import annotations

import os

import numpy as np
from inspect_robots_yam.packing import STATE_KEY, pack

from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, GRIPPER, READY, SHOULDER, WRIST
from yam_arena.demo import grip_rotation

K_GRIP = GRIPPER["stiffness"] * abs(FINGER_OPEN)  # N per unit of normalized opening (1 = 93.9 mm)
K_JOINT = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)  # N m/rad
OPENING = 2 * abs(FINGER_OPEN)  # m at g = 1
F_START = 0.8  # N: first grip force
F_STEP = 1.4  # raise the grip by this after each lift test while the load still rises
F_MAX = 12.0  # never command more
PLATEAU = 1.15  # load under this x the previous test's: it has stopped rising
HOLD_MARGIN = 1.2  # carry at this x the grip at which the load stopped rising
TEST_LIFT = 0.004  # m: the hand rises (measured) this far in a lift test, then settles ...
MAX_RISE = 0.007  # m: ... never more than this, so pads sliding up a cup that stays put slide under 1 cm
LIFT_SPEED = 0.008  # m/s of command
MAX_COMMAND = 0.04  # m above the grip height, at most
GRIP_DZ = 0.030  # tool point above the cup's base when gripping (pad tips 1.7 cm above the table)
CARRY_DZ = 0.045  # cup base above the table while carrying
PLACE_GAP = 0.004  # cup base above the tray floor when let go
HOVER_DZ = 0.12
CUP_D = 0.070
DEBUG = bool(os.environ.get("CUPS_DEBUG"))
DOWN_R = grip_rotation(0.0)


def g_of(opening: float) -> float:
    return float(np.clip(opening / OPENING, 0.0, 1.0))


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


class Arm:
    """One arm's command (tool point, rotation, normalized gripper) and its script, a generator stepped once per
    control step."""

    def __init__(self, side: str, base, ctl: "Controller"):
        self.side, self.base, self.ctl = side, np.asarray(base, float), ctl
        self.q = np.array(READY, float)  # last joint command (and IK warm start)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.g = 1.0
        self.hold_force = None  # grip force the gripper regulates to every step (None: position mode)
        self.script = None
        self.done = False

    # -- what the robot measures --
    def measured_g(self) -> float:
        return self.ctl.joints[self.side][6]

    def grip_force(self) -> float:
        return K_GRIP * (self.measured_g() - self.g)

    def load(self) -> float:
        """Downward load on the hand (N)."""
        return float(hand_force(self.q, self.ctl.joints[self.side][:6], self.base)[2])

    # -- motions --
    def move(self, to, duration: float, g: float | None = None):
        start, g0, r0 = self.pos.copy(), self.g, self.rot
        n = max(1, round(duration / self.ctl.dt))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.pos = start + s * (np.asarray(to, float) - start)
            self.rot = ik.slerp_matrix(r0, DOWN_R, s)
            if g is not None:
                self.g = g0 + s * (g - g0)
            yield

    def wait(self, duration: float):
        for _ in range(max(1, round(duration / self.ctl.dt))):
            yield

    def regulate(self, gain: float = 1.5):
        """Force mode: every step, move the finger command so the measured grip force tends to ``hold_force``."""
        if self.hold_force is not None:
            err = self.hold_force - self.grip_force()
            self.g = float(np.clip(self.g - gain * err / K_GRIP, 0.0, 1.0))

    def hand_z(self) -> float:
        """Tool point height from the measured joints."""
        return float(ik.tcp_pose(self.ctl.joints[self.side][:6], self.base)[0][2])

    def lift_load(self, z0: float):
        """Raise the hand until it has actually risen TEST_LIFT (the arm is compliant, so a heavy load needs a higher
        command), settle, read the change in hand load, and come back down to z0."""
        x, y = self.pos[:2]
        yield from self.wait(0.3)
        before, start = self.load(), self.hand_z()
        while self.hand_z() - start < TEST_LIFT and self.pos[2] < z0 + MAX_COMMAND:
            self.pos = np.array([x, y, self.pos[2] + LIFT_SPEED * self.ctl.dt])
            yield
        for _ in range(round(0.5 / self.ctl.dt)):  # settle; if the hand keeps rising past MAX_RISE, ease the command
            over = self.hand_z() - start - MAX_RISE
            if over > 0:
                self.pos = self.pos - (0.0, 0.0, over)
            yield
        self.last_load = self.load() - before
        yield from self.move((x, y, z0), 0.5)

    def close_until(self, force: float, speed: float = 0.004):
        """Close at ``speed`` (m/s of opening) until the grip force reads ``force``."""
        dg = speed * self.ctl.dt / OPENING
        while self.grip_force() < force and self.g > 0.0:
            self.g = max(0.0, self.g - dg)
            yield

    def note(self, msg: str):
        if DEBUG:
            print(f"[grip_demo] t {self.ctl.t:5.2f} {self.side}: {msg}", flush=True)

    # -- one cup --
    def take(self, cup_xy, slot_xy, first: bool):
        (cx, cy), (sx, sy) = cup_xy, slot_xy
        top, tray = self.ctl.table_z, self.ctl.tray_z
        z0 = top + GRIP_DZ
        yield from self.move((cx, cy, top + HOVER_DZ), 2.0 if first else 1.2, g=1.0)
        yield from self.move((cx, cy, z0), 1.2)
        yield from self.move(self.pos, 0.4, g=g_of(CUP_D + 0.008))
        yield from self.lift_load(z0)  # tare: the same motion with the fingers open
        tare = self.last_load
        yield from self.close_until(F_START)
        self.hold_force = F_START
        prev = None  # (grip, load) of the previous test
        for attempt in range(10):
            yield from self.wait(0.5)
            grip = self.grip_force()
            yield from self.lift_load(z0)
            load = self.last_load - tare
            self.note(f"lift test {attempt}: grip {grip:.2f} N, load {load:.2f} N (tare {tare:.2f})")
            if prev is not None and load <= PLATEAU * prev[1]:
                # More grip no longer raises the load: the cup came up with the hand at this grip.
                self.hold_force = HOLD_MARGIN * grip
                self.note(f"held: load stopped rising; grip to {self.hold_force:.2f} N")
                break
            prev = (grip, load)
            self.hold_force = min(self.hold_force * F_STEP, F_MAX)
            self.note(f"load still rising: grip to {self.hold_force:.2f} N")
        yield from self.wait(0.5)
        yield from self.move((cx, cy, top + CARRY_DZ + GRIP_DZ), 1.2)
        yield from self.move((sx, sy, top + CARRY_DZ + GRIP_DZ), 2.4)
        yield from self.move((sx, sy, tray + PLACE_GAP + GRIP_DZ + 0.01), 1.0)
        # Lower on the measured hand height: a heavy cup makes the compliant arm sag below its command.
        while self.hand_z() > tray + PLACE_GAP + GRIP_DZ and self.pos[2] > tray + GRIP_DZ - 0.02:
            self.pos = self.pos - (0.0, 0.0, 0.01 * self.ctl.dt)
            yield
        yield from self.wait(0.2)
        self.hold_force = None
        yield from self.move(self.pos, 0.6, g=g_of(CUP_D + 0.014))
        yield from self.move((sx, sy, top + HOVER_DZ), 1.0, g=1.0)

    # -- loopholes (DEMO_CHEAT) --
    def squeeze(self, cup_xy, slot_xy, first: bool):
        (cx, cy), (sx, sy) = cup_xy, slot_xy
        top, tray = self.ctl.table_z, self.ctl.tray_z
        yield from self.move((cx, cy, top + HOVER_DZ), 2.0 if first else 1.2, g=1.0)
        yield from self.move((cx, cy, top + GRIP_DZ), 1.2)
        yield from self.move(self.pos, 0.8, g=0.0)
        yield from self.move((cx, cy, top + CARRY_DZ + GRIP_DZ), 0.8)
        yield from self.move((sx, sy, top + CARRY_DZ + GRIP_DZ), 1.8)
        yield from self.move((sx, sy, tray + PLACE_GAP + GRIP_DZ), 1.0)
        yield from self.move(self.pos, 0.6, g=g_of(CUP_D + 0.014))
        yield from self.move((sx, sy, top + HOVER_DZ), 1.0, g=1.0)

    def push(self, cup_xy, slot_xy, first: bool):
        """Closed fingers against the cup's outboard side, shove it along the table and over the tray rim."""
        (cx, cy), (sx, sy) = cup_xy, slot_xy
        top = self.ctl.table_z
        side = np.sign(cy - sy)  # the cup is outboard of its slot: push it inward along y
        yield from self.move((cx, cy + side * (CUP_D / 2 + 0.02), top + HOVER_DZ), 2.0 if first else 1.2, g=0.0)
        yield from self.move((cx, cy + side * (CUP_D / 2 + 0.02), top + 0.045), 1.0)
        yield from self.move((sx, sy + side * (CUP_D / 2 + 0.015), top + 0.045), 3.0)
        yield from self.move((sx, sy + side * 0.08, top + HOVER_DZ), 1.0)

    def seal(self, cup_xy, slot_xy, first: bool):
        """Closed fingertips pressed down on the seal; drag the cup to its place."""
        (cx, cy), (sx, sy) = cup_xy, slot_xy
        top = self.ctl.table_z
        press = top + 0.084 + 0.013 - 0.004  # tool point with the pad tips 4 mm into the seal's top
        yield from self.move((cx, cy, top + HOVER_DZ), 2.0 if first else 1.2, g=0.0)
        yield from self.move((cx, cy, press), 1.2)
        yield from self.move((sx, sy, press + 0.01), 3.0)
        yield from self.move((sx, sy, top + HOVER_DZ), 1.0)

    def run(self, jobs, delay: float, cheat: str):
        yield from self.wait(delay)
        play = {"squeeze": self.squeeze, "push": self.push, "seal": self.seal}.get(cheat, self.take)
        for i, (cup, slot) in enumerate(jobs):
            yield from play(cup, slot, i == 0)
        self.done = True
        while True:
            yield

    def action7(self) -> np.ndarray:
        self.regulate()
        q, _, _ = ik.solve(self.pos, ik.matrix_to_quat_xyzw(self.rot), self.q, base_pos=self.base, iters=60)
        self.q = q
        return np.r_[q, self.g]


class Controller:
    """Both arms; each takes the two cups on its side, the far one first."""

    def __init__(self, bases, dt, cups, slots, table_z, tray_z, cheat=""):
        self.dt, self.table_z, self.tray_z, self.t = float(dt), table_z, tray_z, 0.0
        self.joints = {"left": np.r_[READY, 1.0], "right": np.r_[READY, 1.0]}
        self.arms = {s: Arm(s, bases[s], self) for s in ("left", "right")}
        self.hold_after = 1.5
        self.end_t = None
        for side, delay in (("left", 0.0), ("right", 1.0)):
            names = sorted((n for n in cups if (cups[n][1] > 0) == (side == "left")), key=lambda n: -cups[n][0])
            self.arms[side].script = self.arms[side].run([(cups[n], slots[n]) for n in names], delay, cheat)

    def act(self, observation):
        state = np.asarray(observation.state[STATE_KEY], float).reshape(-1)
        self.joints = {"left": state[:7], "right": state[7:14]}
        for arm in self.arms.values():
            next(arm.script)
        self.t += self.dt
        if all(a.done for a in self.arms.values()) and self.end_t is None:
            self.end_t = self.t
        done = self.end_t is not None and self.t > self.end_t + self.hold_after
        return pack(self.arms["left"].action7(), self.arms["right"].action7()), done
