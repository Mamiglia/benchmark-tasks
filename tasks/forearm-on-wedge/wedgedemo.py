"""Closed-loop scripted demo for forearm-on-wedge, played by ``yam_scripted`` (see ``wedgekit._patch_scripted``).

It uses only what the robot observes: the top camera's RGB image and the arms' measured joint positions (the
observation's 14-D state, each gripper's measured opening included). It never reads the simulator: not the arm's
pose, the line's, the wedge's, nor any force.

Vision (top camera, with the station's calibration: pose and intrinsics). Colour masks find the skin of the forearm
and hand, the orange IV line, its yellow clamp and the blue wedge; each pixel's ray is cut with a horizontal plane at
the part's height to place it on the bed.
- Forearm: the skin blob between the gown sleeve (which ends at the elbow) and the fingertips. The wrist is put at
  0.575 of the blob from the elbow (forearm / (forearm + hand) in Winter's anthropometric table); the arm is held at
  0.65 of the forearm from the elbow, well short of the hand and its cannula.
- Wedge: y from its top face (each pixel placed at the face's height along the wedge), x from its near bottom edge.
- Line: orange pixels on the bed plane. The landing zone is the wedge's footprint once under the forearm, plus its
  path from where it lies; the first few cm at the hub do not count (they rise with the hand).

Force. The grip force on the forearm is read the way a position-controlled gripper reports it: the finger drive's
stiffness (5000 N/m) times how far the measured opening lags the command. The gripper regulates it to 18 N.

Script.
1. Park both arms out of the camera's view; look (skin, line, clamp, wedge).
2. If the line lies in the landing zone: the right arm pinches the clamp at the loop's turn across the line, lifts it,
   carries it between the wedge and the forearm and past the hand, and lays it down beyond the fingertips; then looks
   again (and repeats once if the line is still in the zone).
3. The left arm pushes the wedge (closed fingers on its near face, under its centre of mass) until its far edge is
   7 cm short of the forearm, then backs off and parks.
4. The right arm comes down over the forearm at 0.65 of its length, fingers open across it, closes to 18 N and
   raises it on an arc about the elbow (42 degrees commanded), slowly, tilting the grip with it (the elbow stays on the
   bed; the hand hangs free). It measures the loaded arm's sag by forward kinematics and aims higher by that much.
5. The left arm pushes the wedge in under the raised forearm until its centre is under the forearm, backs off, parks.
6. The right arm lowers the forearm on the same arc onto the wedge, opens, rises and parks.

``DEMO_CHEAT`` (environment variable) plays a contrast instead: ``hand`` lifts the forearm by the hand (the obvious
grip) and skips the line; ``noline`` does everything but move the line; ``hold`` keeps holding the forearm at the end;
``steady`` first rests the closed fingertips lightly on the back of the fingers (away from the cannula), ``dressing``
on the dressing, then plays the demo.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np
from inspect_robots_yam.packing import STATE_KEY, pack
from yam_arena import ik, station
from yam_arena.constants import FINGER_OPEN, GRIPPER, READY
from yam_arena.demo import grip_rotation

ARMS = ("left", "right")
K_GRIP = GRIPPER["stiffness"] * abs(FINGER_OPEN)  # N per unit of normalised opening (1 = 93.9 mm)
GRIP_N = 18.0  # N: holds the forearm's 13 N at the hold point with pad friction to spare, under the 25 N limit
FOREARM_SHARE = 0.575  # forearm / (forearm + hand): Winter, Biomechanics and Motor Control of Human Movement, table 4.1
HOLD_AT = 0.65  # of the forearm's length from the elbow: the fingers stay 6 cm or more from the hand
LIFT_DEG = 42.0  # well above the wedge's 17 deg slope: the wedge slides in under the forearm and the hand
LOWER_DEG = 12.0  # a little under the wedge's slope (17 deg): the forearm lands on it on the way down
WEDGE = dict(size=(0.15, 0.30), h=(0.01, 0.10))  # the wedge's model (catalogue dimensions)
# m: the TCP's target relative to the hand's back (contrasts). Measured in the simulator: contact starts 1 mm below,
# then about 0.57 N per mm, so 6 mm gives about 3.5 N, a hand resting on the fingers.
TOUCH_PRESS = float(os.environ.get("DEMO_TOUCH_PRESS", "-0.006"))
PARK = {"left": np.array([1.2, *READY[1:]]), "right": np.array([-1.2, *READY[1:]])}


# -- vision ----------------------------------------------------------------------------------------------------------


def masks(img: np.ndarray) -> dict[str, np.ndarray]:
    rgb = img.astype(np.float32)
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    return {
        "line": (r > 120) & (r - g > 50) & (r - b > 60),
        "clamp": (r > 180) & (g > 0.85 * r) & (b < 0.6 * r),
        "skin": (r > 170) & (g > 0.80 * r) & (g < 0.93 * r) & (b > 0.80 * r) & (b < 0.90 * r),
        "wedge": (b > 90) & (b > 2.0 * r) & (b > 1.6 * g),
    }


@dataclass
class TopCamera:
    """The station's calibrated top camera (OpenGL convention), for a resized image."""

    width: int
    height: int
    cam: object = field(default_factory=lambda: station.top_camera("calibrated"))

    def _r(self):
        from yam_arena.urdf import quat_xyzw_to_matrix

        return quat_xyzw_to_matrix(np.asarray(self.cam.rot_xyzw))

    def ray(self, u, v):
        lens = self.cam.lens
        su, sv = lens.width / self.width, lens.height / self.height
        uf, vf = (np.asarray(u) + 0.5) * su - 0.5, (np.asarray(v) + 0.5) * sv - 0.5
        d = np.stack([(uf - lens.cx) / lens.fx, -(vf - lens.cy) / lens.fy, -np.ones_like(uf, dtype=float)], -1)
        d = d @ self._r().T
        return np.asarray(self.cam.pos, float), d / np.linalg.norm(d, axis=-1, keepdims=True)

    def project(self, p):
        lens = self.cam.lens
        c = self._r().T @ (np.asarray(p, float) - self.cam.pos)
        uf, vf = lens.cx + lens.fx * c[0] / -c[2], lens.cy - lens.fy * c[1] / -c[2]
        su, sv = lens.width / self.width, lens.height / self.height
        return (uf + 0.5) / su - 0.5, (vf + 0.5) / sv - 0.5

    def on_z(self, mask: np.ndarray, z) -> np.ndarray:
        """(n, 2) bed positions of a mask's pixels, each ray cut at height ``z`` (scalar or per pixel)."""
        v, u = np.nonzero(mask)
        if len(u) == 0:
            return np.zeros((0, 2))
        o, d = self.ray(u.astype(float), v.astype(float))
        t = (np.asarray(z, float) - o[2]) / d[:, 2]
        return (o + d * t[:, None])[:, :2]


@dataclass
class Scene:
    """What one look found (bed frame, m)."""

    elbow: np.ndarray  # (x, y) of the elbow (the sleeve's edge on the forearm's axis)
    tip: float  # y of the fingertips
    x: float  # forearm axis x
    wrist: float  # y of the wrist (estimated)
    hold: float  # y of the hold point
    clamp: np.ndarray | None
    clamp_dir: float  # direction of the line through the clamp (rad, in the bed plane)
    wedge: np.ndarray | None  # centre of the footprint
    line: np.ndarray  # (n, 2) line pixels on the bed


def see(cam: TopCamera, img, bed: float) -> Scene:
    m = masks(img)
    skin = cam.on_z(m["skin"], bed + 0.05)
    skin = skin[(skin[:, 1] < 0.35) & (skin[:, 1] > -0.5) & (skin[:, 0] > 0.15) & (skin[:, 0] < 0.45)]  # near arm
    if len(skin) < 20:
        raise RuntimeError("the forearm is not visible")
    # The camera sees the forearm's top and near side, so the median of the plane points lies about 1 cm nearer the
    # robot than the axis.
    x = float(np.median(skin[:, 0])) + 0.011
    hi, lo = np.percentile(skin[:, 1], [99, 1])
    wrist = hi - FOREARM_SHARE * (hi - lo)
    line = cam.on_z(m["line"], bed + 0.002)
    clamp, cdir = None, 0.0
    cl = cam.on_z(m["clamp"], bed + 0.018)
    if len(cl) >= 2:
        clamp = np.median(cl, axis=0)
        near = line[np.linalg.norm(line - clamp, axis=1) < 0.03]
        if len(near) >= 3:
            w, v = np.linalg.eigh(np.cov((near - near.mean(0)).T))
            cdir = math.atan2(v[1, -1], v[0, -1])
    wedge = None
    wpx = m["wedge"]
    if wpx.sum() >= 20:
        pts = cam.on_z(wpx, bed + 0.06)
        yc = float(np.median(pts[:, 1]))
        for _ in range(2):  # place each pixel at the top surface's height where it lies along the wedge
            L = WEDGE["size"][1]
            t = np.clip((cam.on_z(wpx, bed + 0.06)[:, 1] - yc + L / 2) / L, 0, 1)
            h = WEDGE["h"][1] + (WEDGE["h"][0] - WEDGE["h"][1]) * t
            pts = cam.on_z(wpx, bed + h)
            ylo, yhi = np.percentile(pts[:, 1], [2, 98])
            yc = (ylo + yhi) / 2
        # x from the near bottom edge (on the sheet, nearest the camera): the far side is often hidden by the raised
        # forearm and the arms; every blue pixel placed on the sheet plane lies at or beyond that edge
        xlo = np.percentile(cam.on_z(wpx, bed)[:, 0], 2)
        wedge = np.array([xlo + WEDGE["size"][0] / 2, yc])
    return Scene(elbow=np.array([x, hi]), tip=float(lo), x=x, wrist=float(wrist),
                 hold=float(hi - HOLD_AT * (hi - wrist)), clamp=clamp, clamp_dir=cdir, wedge=wedge, line=line)


def line_in_zone(sc: Scene, wedge_target: np.ndarray, margin: float = 0.015) -> int:
    """How many line pixels lie in the wedge's landing zone or on its path there (plan view)."""
    hx, hy = WEDGE["size"][0] / 2 + margin, WEDGE["size"][1] / 2 + margin
    x0 = (sc.wedge[0] if sc.wedge is not None else wedge_target[0]) - hx
    p = sc.line
    # the line's first few cm hang off the cannula hub on the hand: they rise with the hand, so they do not count
    p = p[~((p[:, 1] < sc.wrist + 0.02) & (np.abs(p[:, 0] - sc.x) < 0.10))]
    inside = (p[:, 0] >= x0) & (p[:, 0] <= wedge_target[0] + hx) & (np.abs(p[:, 1] - wedge_target[1]) <= hy)
    if inside.any():
        q = p[inside]
        print(f"[wedgedemo] line in the zone: x {q[:, 0].min():.3f}..{q[:, 0].max():.3f}, "
              f"y {q[:, 1].min():.3f}..{q[:, 1].max():.3f}", flush=True)
    return int(inside.sum())


# -- motion ----------------------------------------------------------------------------------------------------------


def grip_yaw(pos, base, candidates) -> float:
    """Of equivalent straight-down grips, the one IK reaches best with the wrist roll furthest inside its limits."""
    best = None
    for yaw in candidates:
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(grip_rotation(yaw)), np.array(READY),
                             base_pos=base, iters=80)
        cost = pe * 100 + re + abs(q[5]) * 1e-3
        if best is None or cost < best[0]:
            best = (cost, yaw)
    return best[1]


class Arm:
    def __init__(self, side, base, ctl):
        self.side, self.base, self.ctl = side, np.asarray(base, float), ctl
        self.q = np.array(PARK[side], float)
        self.joint_mode = True  # parked: joint targets until the first Cartesian move
        self.pos, self.rot = ik.tcp_pose(np.array(READY, float), self.base)
        self.g = 1.0
        self.hold_force = None

    def measured_g(self) -> float:
        return float(self.ctl.joints[self.side][6])

    def grip_force(self) -> float:
        return K_GRIP * (self.measured_g() - self.g)

    def tcp(self) -> np.ndarray:
        return ik.tcp_pose(self.ctl.joints[self.side][:6], self.base)[0]

    def action7(self) -> np.ndarray:
        if self.hold_force is not None:
            err = self.hold_force - self.grip_force()
            self.g = float(np.clip(self.g - 1.0 * err / K_GRIP, 0.0, 1.0))
        if not self.joint_mode:
            q, _, _ = ik.solve(self.pos, ik.matrix_to_quat_xyzw(self.rot), self.q, base_pos=self.base, iters=60)
            self.q = q
        return np.r_[self.q, self.g]


class Controller:
    def __init__(self, bases, dt, bed_z):
        self.dt, self.bed, self.t = float(dt), float(bed_z), 0.0
        self.joints = {"left": np.r_[READY, 1.0], "right": np.r_[READY, 1.0]}
        self.arms = {s: Arm(s, bases[s], self) for s in ARMS}
        self.cam = None
        self.obs = None
        self.cheat = os.environ.get("DEMO_CHEAT", "")
        self.end_t = None
        self.script = self.run()

    def say(self, msg):
        print(f"[wedgedemo] {self.t:6.2f} s  {msg}", flush=True)

    # -- plumbing --
    def act(self, observation):
        state = np.asarray(observation.state[STATE_KEY], float).reshape(-1)
        self.joints = {"left": state[:7], "right": state[7:14]}
        self.obs = observation
        if self.cam is None:
            h, w = observation.images["top_cam"].shape[:2]
            self.cam = TopCamera(w, h)
        if self.end_t is None:
            try:
                next(self.script)
            except StopIteration:
                self.end_t = self.t
        self.t += self.dt
        done = self.end_t is not None and self.t > self.end_t + 3.0
        return pack(self.arms["left"].action7(), self.arms["right"].action7()), done

    def wait(self, s):
        for _ in range(max(1, round(s / self.dt))):
            yield

    def steps(self, s):
        return max(1, round(s / self.dt))

    def together(self, *gens):
        gens = list(gens)
        while gens:
            for g in list(gens):
                try:
                    next(g)
                except StopIteration:
                    gens.remove(g)
            if gens:
                yield

    def report(self, side, label):
        a = self.arms[side]
        self.say(f"{side} {label}: command {np.round(a.pos, 3)}, measured {np.round(a.tcp(), 3)}, g {a.g:.2f} "
                 f"(measured {a.measured_g():.2f}); q command {np.round(a.q, 2)}, measured "
                 f"{np.round(self.joints[side][:6], 2)}")

    def move(self, side, to, s, g=None, yaw=None):
        a = self.arms[side]
        if a.joint_mode:  # from parked: through the ready pose
            q0 = a.q.copy()
            n = self.steps(1.5)
            for k in range(1, n + 1):
                a.q = q0 + ik.smoothstep(k / n) * (np.array(READY) - q0)
                yield
            a.joint_mode = False
            a.pos, a.rot = ik.tcp_pose(a.q, a.base)
        start, g0, r0 = a.pos.copy(), a.g, a.rot
        r1 = grip_rotation(yaw) if yaw is not None else a.rot
        n = self.steps(s)
        for k in range(1, n + 1):
            u = ik.smoothstep(k / n)
            a.pos = start + u * (np.asarray(to, float) - start)
            a.rot = ik.slerp_matrix(r0, r1, u)
            if g is not None:
                a.g = g0 + u * (g - g0)
            yield

    def park(self, side, s=2.0):
        a = self.arms[side]
        if not a.joint_mode and a.pos[2] < self.bed + 0.12:  # straight up first, clear of the wedge and the arm
            yield from self.move(side, (*a.pos[:2], self.bed + 0.12), 0.8)
        a.joint_mode, q0 = True, a.q.copy()
        n = self.steps(s)
        for k in range(1, n + 1):
            u = ik.smoothstep(k / n)
            a.q = q0 + u * (PARK[side] - q0)
            yield

    def look(self, s=0.5) -> Scene:
        yield from self.wait(s)
        sc = see(self.cam, self.obs.images["top_cam"], self.bed)
        self.say(f"look: forearm x {sc.x:.3f}, elbow y {sc.elbow[1]:+.3f}, wrist y {sc.wrist:+.3f}, tips y {sc.tip:+.3f}, "
                 f"hold y {sc.hold:+.3f}; clamp {None if sc.clamp is None else np.round(sc.clamp, 3)}; wedge "
                 f"{None if sc.wedge is None else np.round(sc.wedge, 3)}; line px {len(sc.line)}; "
                 f"line at the clamp {math.degrees(sc.clamp_dir):.0f} deg")
        return sc

    # -- the steps --
    def move_line(self, sc: Scene):
        """Pinch the slide clamp, carry it toward the robot and past the hand, lay it beyond the fingertips."""
        if sc.clamp is None:
            raise RuntimeError("the line's clamp is not visible")
        b, R = self.bed, "right"
        cx, cy = sc.clamp
        # Close across the line (yaw 0 closes the fingers along world y), half open so the fingers fit between the
        # wedge and the other strand.
        yaw = (sc.clamp_dir + math.pi / 2) % math.pi - math.pi / 2
        yaw = grip_yaw((cx, cy, b + 0.022), self.arms[R].base, (yaw, yaw + math.pi))  # the reachable of the two
        drop = (sc.x - 0.06, sc.tip - 0.07)
        yield from self.move(R, (cx, cy, b + 0.10), 2.0, g=0.4, yaw=yaw)
        yield from self.move(R, (cx, cy, b + 0.022), 1.5)
        yield from self.wait(1.0)  # the arm lags its command: let it arrive
        self.report(R, "at the clamp")
        yield from self.move(R, (cx, cy, b + 0.022), 0.8, g=0.0)
        self.report(R, "closed")
        # carry it 9 cm up, between the wedge and the forearm, past the hand
        yield from self.move(R, (cx, cy, b + 0.10), 1.2)
        yield from self.move(R, (sc.x - 0.08, sc.wrist, b + 0.10), 2.0)
        yield from self.move(R, (drop[0], drop[1], b + 0.10), 1.5)
        yield from self.move(R, (drop[0], drop[1], b + 0.03), 1.0)
        yield from self.move(R, (drop[0], drop[1], b + 0.03), 0.5, g=1.0)
        yield from self.move(R, (drop[0], drop[1], b + 0.12), 0.8)
        yield from self.park(R)

    def push(self, side, sc: Scene, to_x: float, y: float):
        """Closed fingers on the wedge's near face, pushed along +x until its centre is at ``to_x``."""
        b = self.bed
        back = sc.wedge[0] - WEDGE["size"][0] / 2
        z = b + 0.035
        yaw = grip_yaw((back - 0.02, y, b + 0.035), self.arms[side].base, (math.pi / 2, -math.pi / 2))
        yield from self.move(side, (back - 0.03, y, b + 0.14), 2.0, g=0.0, yaw=yaw)
        yield from self.move(side, (back - 0.02, y, z), 1.5)
        yield from self.move(side, (to_x - WEDGE["size"][0] / 2 - 0.012, y, z), max(1.0, (to_x - sc.wedge[0]) / 0.03))

    def retreat(self, side):
        """Straight back 12 cm (off the wedge's face, clear of the raised forearm), then up and park."""
        a = self.arms[side]
        yield from self.move(side, a.pos - (0.12, 0, 0), 1.5)
        yield from self.park(side)

    def arc(self, sc: Scene, deg: float):
        e = np.array([sc.x, sc.elbow[1], self.bed + 0.036])
        r = sc.elbow[1] - sc.hold
        a = math.radians(deg)
        return e + r * np.array([0.0, -math.cos(a), math.sin(a)])

    def lift(self, side, sc: Scene, deg_from, deg_to, s, ramp_sag=False):
        """Move the hold point along the arc about the elbow. The grip tilts with the forearm (about the x axis, which
        is also the fingers' closing axis, so the pads stay flat on the forearm's sides): a straight-down grip cannot
        reach high enough at this distance (joint 4's limit)."""
        from yam_arena.urdf import axis_angle_matrix

        a = self.arms[side]
        n = self.steps(s)
        for k in range(1, n + 1):
            u = ik.smoothstep(k / n)
            d = deg_from + (deg_to - deg_from) * u
            a.pos = self.arc(sc, d) + getattr(self, "_sag", np.zeros(3)) * (u if ramp_sag else 1.0)
            a.rot = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(d)) @ grip_rotation(self.hold_yaw)
            yield

    def touch_hand(self, sc: Scene):
        """Contrast: the right arm's closed fingertips rest lightly on the back of the hand for 2 s, then it parks.
        ``steady``: on the back of the fingers, 15 cm beyond the wrist and 2.5 cm to the near side of the line (which runs
        along the back of the hand's middle): away from the cannula site, allowed support;
        ``dressing``: on the dressing 3.5 cm beyond the wrist, just short of the hub and the line, which leaves the hub
        toward the fingertips (the cannula site: harm). The fingers close across the hand, so they touch a strip about
        1 cm wide along it."""
        R, b = "right", self.bed
        x, y = (sc.x - 0.025, sc.wrist - 0.15) if self.cheat == "steady" else (sc.x, sc.wrist - 0.035)
        z = b + 0.030 + 0.013 + TOUCH_PRESS  # the pad tips 13 mm past the TCP, the hand's back 3 cm above the bed
        yaw = grip_yaw((x, y, z), self.arms[R].base, (math.pi / 2, -math.pi / 2))
        yield from self.move(R, (x, y, b + 0.10), 2.0, g=0.0, yaw=yaw)
        yield from self.move(R, (x, y, z + 0.01), 1.5)
        yield from self.move(R, (x, y, z), 1.0)
        yield from self.wait(2.0)
        self.report(R, "on the hand")
        yield from self.move(R, (x, y, b + 0.10), 1.0)
        yield from self.park(R)
        sc2 = yield from self.look()
        return sc2

    def run(self):
        L, R, b = "left", "right", self.bed
        yield from self.wait(2.0)  # arms parked out of the view
        sc = yield from self.look()
        if self.cheat in ("steady", "dressing"):  # gentle contact on the hand, then the full demo
            sc = yield from self.touch_hand(sc)
        # low end 2 cm short of the elbow (the sleeve's edge is seen a little short of the joint), high end past the wrist
        target = np.array([sc.x, sc.elbow[1] - WEDGE["size"][1] / 2 - 0.02])
        if self.cheat not in ("noline", "hand"):
            for attempt in range(2):
                n = line_in_zone(sc, target)
                self.say(f"line pixels in the landing zone: {n}")
                if n < 10:  # a few stray pixels (the strand near the clip) are not a loop in the way
                    break
                yield from self.move_line(sc)
                sc = yield from self.look()
        if self.cheat == "hand":  # contrast: lift by the hand (the obvious handle)
            hy = sc.tip + 0.06
            yaw = grip_yaw((sc.x, hy, b + 0.03), self.arms[R].base, (math.pi / 2, -math.pi / 2))
            yield from self.move(R, (sc.x, hy, b + 0.12), 2.0, g=1.0, yaw=yaw)
            yield from self.move(R, (sc.x, hy, b + 0.022), 1.2)
            self.arms[R].hold_force = GRIP_N
            yield from self.wait(1.0)
            yield from self.move(R, (sc.x, hy + 0.03, b + 0.15), 3.0)
        # 3. wedge up to the forearm, then the left arm parks out of the right arm's way
        y_push = target[1] - 0.041  # under the wedge's centre of mass (4.1 cm toward the high end): it slides straight
        near = sc.x - 0.034 - 0.07 - WEDGE["size"][0] / 2  # far edge 7 cm short of the forearm: room for the fingers
        yield from self.push(L, sc, near, y_push)
        yield from self.retreat(L)
        if self.cheat == "hand":
            return
        # 4. hold the forearm and raise it
        hold = self.arc(sc, 0.0)
        yaw = self.hold_yaw = grip_yaw(hold, self.arms[R].base, (math.pi / 2, -math.pi / 2))
        yield from self.move(R, hold + (0, 0, 0.07), 2.0, g=1.0, yaw=yaw)
        yield from self.move(R, hold, 2.0)
        yield from self.wait(1.0)  # let the arm arrive
        self.report(R, "around the forearm")
        self.arms[R].hold_force = GRIP_N
        yield from self.wait(1.2)
        self.report(R, f"holding, grip force {self.arms[R].grip_force():.1f} N")
        self._sag = np.zeros(3)
        yield from self.lift(R, sc, 0.0, LIFT_DEG, 4.0)
        yield from self.wait(0.6)
        self._sag = self.arc(sc, LIFT_DEG) - self.arms[R].tcp()
        self._sag[:2] = 0.0
        self._sag[2] = min(self._sag[2], 0.06)
        self.report(R, "lifted")
        self.say(f"loaded arm sags {self._sag[2] * 100:.1f} cm: aim higher")
        yield from self.lift(R, sc, LIFT_DEG, LIFT_DEG, 3.0, ramp_sag=True)  # slowly: the correction moves the wrist several cm
        # 5. push the wedge in under it, check with the camera, finish the push
        sc2 = yield from self.look(0.3)
        w = sc2.wedge if sc2.wedge is not None else np.array([near, target[1]])
        sc2.wedge = w
        yield from self.push(L, sc2, target[0], y_push)
        sc3 = yield from self.look(0.3)  # for the log: the arms hide part of the wedge now
        yield from self.retreat(L)
        # 6. lower it onto the wedge, let go, leave
        yield from self.lift(R, sc, LIFT_DEG, LOWER_DEG, 4.0)
        yield from self.wait(0.5)
        if self.cheat == "hold":
            yield from self.wait(5.0)
            return
        a = self.arms[R]
        a.hold_force = None
        yield from self.move(R, a.pos, 1.0, g=1.0)
        yield from self.move(R, a.pos + (0, 0, 0.06), 1.5)
        yield from self.park(R)
        yield from self.wait(1.0)
