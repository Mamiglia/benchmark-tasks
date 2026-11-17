"""dripkit: a staked 1/2 in drip line, an emitter punch and a barbed dripper, for a yam_arena task file.

yam_arena task files only take single primitives. This module adds one object, :class:`DripBed`, spawned as a static
(``BASE``) Arena asset holding these bodies:

- ``bench`` (kinematic): the bed board covered in landscape fabric, two tubing stakes over the line, a shrub in its
  nursery pot beside the mark, and the drilled pine tool block (a 6 mm hole for the punch, a 5 mm hole for the
  dripper's barb).
- ``line`` (kinematic): 1/2 in drip tubing (17.8 mm OD) lying along y on the bed, held by the stakes.
- ``mark`` (kinematic, drawn only): the white paint-pen ring where the dripper goes (its place varies with the seed).
- ``hole0``, ``hole1`` (kinematic, drawn only): the holes the punch makes (hidden until made).
- ``punch`` (dynamic): the emitter punch. Body frame: origin at the point, +z up the tool. Steel point 3 mm x 15 mm
  (collides), a stop shoulder 8 mm across, the plastic handle (drawn 28 mm round, a 22 mm square collider) to 105 mm.
- ``emitter`` (dynamic): the barbed dripper. Body frame: origin at the barb's tip, +z up. Barb 4 mm x 7 mm
  (collides), body 30 mm across, 21 mm tall above it (a 22 mm square collider inside the drawn round body).

The tubing wall (the rule standing in for the plastic, in the fidelity note). Before every physics step (240 Hz, a
wrapper on ``InteractiveScene.write_data_to_sim``) the kit reads the PhysX contact force of the punch and of the
dripper on the line.

- A hole is made where the punch's point is (within ``near_m`` of the tube's surface) once the punch has pressed on
  the line with at least ``punch_n`` for ``punch_s``. It is round when the punch was within ``punch_tilt_deg`` of the
  surface's normal there, torn (a slit that leaks) otherwise. Pressing again within ``same_hole_m`` of a hole makes
  no new hole.
- The dripper seats in a hole once its barb's tip is on that hole (within ``engage_m`` across, at the surface) and it
  has pressed on the line with at least ``seat_n`` for ``seat_s``: square (within ``seat_tilt_deg`` of the hole's
  normal) it pops in to its collar and stays there; pushed in crooked it tears the hole instead.

``object_states`` reports a :class:`DripState`. Importing the module also hooks ``yam_arena.scripted``: when the task
file defines ``controller(bases, dt)``, ``yam_scripted`` asks it for every action. yam_arena itself is not modified.
Pure Python at import; Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass, field, replace

import numpy as np
from yam_arena.task_file import Box, ObjectState

Color = tuple[float, float, float]
ARMS = ("left_robot", "right_robot")


# -- the tubing wall (pure Python, unit-tested) ----------------------------------------------------------------------


@dataclass(frozen=True)
class Law:
    """The punch-and-seat rule (SI units)."""

    punch_n: float = 25.0  # push on the line that drives the point through the wall
    punch_s: float = 0.10  # ... held this long in all, net of the time below it (not a contact spike)
    smooth_s: float = 0.03  # pushes are read through a first-order filter with this time constant
    punch_tilt_deg: float = 15.0  # further off the surface normal: a torn hole
    near_m: float = 0.002  # the point within this of the tube's surface
    same_hole_m: float = 0.004  # pressing again this close to a hole: the same hole
    seat_n: float = 15.0  # push that pops the barb through the hole to its collar
    seat_s: float = 0.10
    engage_m: float = 0.0015  # barb tip this close to the hole's centre, across the normal: in the hole's mouth
    mouth_n: float = 2.0  # pressed this hard there, the barb stays in the mouth ...
    hold_m: float = 0.004  # ... while within this across (the rigid tube cannot hold it the way a hole does)
    seat_tilt_deg: float = 15.0  # further off the hole's normal: the barb tears the hole
    mark_m: float = 0.010  # a hole counts when its centre is this close to the mark, along the line


@dataclass
class Hole:
    p: np.ndarray  # centre, on the tube's surface
    n: np.ndarray  # outward normal there
    tilt: float  # the punch's angle to the normal when it went through (deg)
    torn: bool = False
    why: str = ""


def _deg(a, b) -> float:
    return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0))))


@dataclass
class Line:
    """The drip line as the rule sees it: a straight tube along y with its axis at (``x``, ``z``)."""

    law: Law
    x: float
    z: float
    radius: float
    mark_y: float
    holes: list = field(default_factory=list)
    seated: int | None = None
    mouth: int | None = None  # the hole whose mouth the barb's tip is in
    punch_t: float = 0.0
    seat_t: float = 0.0
    punch_f: float = 0.0  # filtered pushes (N)
    seat_f: float = 0.0

    def _filter(self, old: float, push: float, dt: float) -> float:
        return old + (push - old) * min(1.0, dt / self.law.smooth_s)

    def surface(self, p):
        """Outward normal at the surface point nearest ``p``, and ``p``'s height above the surface (m)."""
        r = np.array([p[0] - self.x, 0.0, p[2] - self.z])
        d = float(np.linalg.norm(r))
        n = r / d if d > 1e-9 else np.array([0.0, 0.0, 1.0])
        return n, d - self.radius

    def punch_step(self, tip, axis, push: float, dt: float) -> str | None:
        """One physics step of the punch: ``tip`` its point, ``axis`` unit from the point up the tool, ``push`` its
        contact force on the line (N). Returns an event to log, if any."""
        law = self.law
        tip = np.asarray(tip, float)
        n, h = self.surface(tip)
        push = self.punch_f = self._filter(self.punch_f, push, dt)
        if push < law.punch_n or h > law.near_m:
            self.punch_t = max(0.0, self.punch_t - dt)  # leaky: contact chatter does not restart the count
            return None
        self.punch_t += dt
        if self.punch_t < law.punch_s:
            return None
        p = np.array([self.x, tip[1], self.z]) + self.radius * n
        if any(np.linalg.norm(p - hole.p) <= law.same_hole_m for hole in self.holes):
            return None
        tilt = _deg(axis, n)
        hole = Hole(p=p, n=n, tilt=tilt, torn=tilt > law.punch_tilt_deg,
                    why=f"punched {tilt:.1f} deg off square" if tilt > law.punch_tilt_deg else "")
        self.holes.append(hole)
        return (f"HOLE {len(self.holes) - 1}: {'TORN' if hole.torn else 'round'} at y {p[1]:+.4f} "
                f"({(p[1] - self.mark_y) * 1000:+.1f} mm from the mark), punch {tilt:.1f} deg off square, {push:.1f} N")

    def emitter_step(self, tip, axis, push: float, dt: float) -> str | None:
        """One physics step of the dripper: ``tip`` its barb's tip, ``axis`` unit from the barb up the body."""
        law = self.law
        if self.seated is not None:
            return None
        push = self.seat_f = self._filter(self.seat_f, push, dt)
        tip = np.asarray(tip, float)
        on = None
        for i, hole in enumerate(self.holes):
            rel = tip - hole.p
            along = float(rel @ hole.n)
            across = float(np.linalg.norm(rel - along * hole.n))
            reach = law.hold_m if i == self.mouth else law.engage_m
            if across <= reach and -0.002 <= along <= 0.003:
                on = i
        if on is not None and push >= law.mouth_n:
            self.mouth = on
        elif on is None or push < law.mouth_n:
            self.mouth = None if on is None else self.mouth
        if on is None or push < law.seat_n:
            self.seat_t = 0.0 if on is None else max(0.0, self.seat_t - dt)
            return None
        self.seat_t += dt
        if self.seat_t < law.seat_s:
            return None
        hole = self.holes[on]
        tilt = _deg(axis, hole.n)
        if tilt > law.seat_tilt_deg:
            if not hole.torn:
                hole.torn, hole.why = True, f"dripper forced in {tilt:.1f} deg off square"
                return f"HOLE {on} TORN: the dripper was pushed in {tilt:.1f} deg off square ({push:.1f} N)"
            return None
        self.seated = on
        return f"SEATED in hole {on}, {tilt:.1f} deg off square, {push:.1f} N"

    def on_mark(self, hole: Hole) -> bool:
        return not hole.torn and abs(float(hole.p[1]) - self.mark_y) <= self.law.mark_m

    @property
    def leak(self) -> bool:
        """A torn hole, or more than one hole: the line leaks however the dripper ends."""
        return len(self.holes) > 1 or any(h.torn for h in self.holes)


def score_line(line: Line, punch_home: bool) -> float:
    """0.4 a round hole within 10 mm of the mark; +0.5 the dripper seated in it; +0.1 then the punch back in its
    block hole, let go. Any torn hole, or a second hole, caps the score at 0.2."""
    good = [h for h in line.holes if line.on_mark(h)]
    s = 0.4 if good else 0.0
    if s and line.seated is not None and line.on_mark(line.holes[line.seated]):
        s += 0.5
        if punch_home:
            s += 0.1
    return round(min(s, 0.2) if line.leak else s, 4)


# -- the object ------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class DripBed(Box):
    """Positions in the env frame (x forward, y left, z up, m). ``pos`` is the bed board's top-face centre."""

    table_z: float = 0.0
    board: tuple[float, float, float] = (0.30, 0.64, 0.012)  # x, y, thickness
    line_x: float = 0.30
    line_y: tuple[float, float] = (-0.33, 0.29)
    line_r: float = 0.0089  # 0.700 in OD
    mark_y: float = -0.08  # mean; each seed moves it
    mark_jitter: float = 0.015
    stake_y: tuple[float, float] = (-0.205, 0.045)
    pot_xy: tuple[float, float] = (0.4525, -0.08)
    pot: tuple[float, float] = (0.0825, 0.17)  # radius, height (1 gal nursery pot)
    block_xy: tuple[float, float] = (0.40, -0.25)
    block: tuple[float, float, float] = (0.089, 0.120, 0.038)  # pine 2x4 offcut: x, y, height
    punch_hole: tuple[float, float, float] = (0.0, -0.038, 0.006)  # from the block's centre: x, y, width
    emitter_hole: tuple[float, float, float] = (0.0, 0.038, 0.005)
    pin: tuple[float, float] = (0.0015, 0.015)  # radius, length
    stop: tuple[float, float] = (0.004, 0.018)  # radius, end
    handle: tuple[float, float] = (0.014, 0.105)  # radius, end
    punch_mass: float = 0.040
    barb: tuple[float, float] = (0.002, 0.007)
    body: tuple[float, float] = (0.015, 0.028)  # radius, end (from the barb's tip)
    emitter_mass: float = 0.020
    punch_jitter: float = 0.10  # the wall's punch force varies by this fraction with the seed
    law: Law = Law()
    fabric_color: Color = (0.17, 0.14, 0.11)
    tube_color: Color = (0.10, 0.09, 0.08)
    mark_color: Color = (0.95, 0.95, 0.92)
    steel_color: Color = (0.70, 0.71, 0.73)
    handle_color: Color = (0.95, 0.75, 0.10)
    emitter_color: Color = (0.20, 0.55, 0.85)
    pine_color: Color = (0.85, 0.72, 0.50)
    pot_color: Color = (0.08, 0.08, 0.08)
    leaf_color: Color = (0.20, 0.45, 0.18)
    hole_color: Color = (0.02, 0.02, 0.02)

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a DripBed is a fixed asset (mass None); its bodies carry their own")

    @property
    def top(self) -> float:
        return self.table_z + self.board[2]

    @property
    def line_z(self) -> float:
        return self.top + self.line_r

    @property
    def block_top(self) -> float:
        return self.top + self.block[2]

    def hole_xy(self, which: str) -> np.ndarray:
        h = self.punch_hole if which == "punch" else self.emitter_hole
        return np.array(self.block_xy) + np.array(h[:2])

    def punch_rest(self) -> np.ndarray:
        """The punch's point standing in its block hole, the stop on the block's top."""
        return np.array([*self.hole_xy("punch"), self.block_top - self.pin[1]])

    def emitter_rest(self) -> np.ndarray:
        """The dripper's barb tip, barb in its block hole, body on the block's top."""
        return np.array([*self.hole_xy("emitter"), self.block_top - self.barb[1]])


def seeded(spec: DripBed, seed: int | None):
    """Per seed: the mark's place along the line (m) and the wall's punch force (N)."""
    rng = np.random.default_rng(0 if seed is None else int(seed))
    mark = float(spec.mark_y + rng.uniform(-spec.mark_jitter, spec.mark_jitter))
    punch_n = float(spec.law.punch_n * (1.0 + rng.uniform(-spec.punch_jitter, spec.punch_jitter)))
    return mark, punch_n


def new_line(spec: DripBed, seed) -> Line:
    mark, punch_n = seeded(spec, seed)
    return Line(law=replace(spec.law, punch_n=punch_n), x=spec.line_x, z=spec.line_z, radius=spec.line_r, mark_y=mark)


def in_block(spec: DripBed, tip, axis) -> bool:
    """The punch's point down in its block hole, the tool upright within 20 degrees."""
    xy = spec.hole_xy("punch")
    return (float(np.linalg.norm(np.asarray(tip[:2]) - xy)) <= spec.punch_hole[2] / 2 + 0.001
            and tip[2] <= spec.block_top - 0.005 and float(axis[2]) >= math.cos(math.radians(20)))


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(R) -> np.ndarray:
    """xyzw."""
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def frame_from_z(z) -> np.ndarray:
    """A rotation whose +z is ``z``."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    y = np.array([0.0, 1.0, 0.0]) - z[1] * z
    if np.linalg.norm(y) < 1e-6:
        y = np.array([1.0, 0.0, 0.0]) - z[0] * z
    y /= np.linalg.norm(y)
    return np.column_stack([np.cross(y, z), y, z])


@dataclass(frozen=True)
class DripState(ObjectState):
    """After a control step (env frame). ``pos`` is the punch's point."""

    line: Line = None  # a copy
    punch_home: bool = False  # in its block hole and let go
    punch_held: str | None = None
    emitter_tip: np.ndarray = None
    punch_n: float = 0.0  # largest punch-on-line push during the step
    emitter_n: float = 0.0


# -- spawning (Isaac Sim only) ---------------------------------------------------------------------------------------

_SPECS: dict[str, DripBed] = {}
_PATHS: dict[str, str] = {}


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    s = _SPECS[cfg.rig_name]
    _PATHS.setdefault(s.name, prim_path)
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    looks: dict = {}
    mats: dict = {}

    def material(mu) -> str:
        if mu not in mats:
            mats[mu] = f"{prim_path}/Looks/m{len(mats)}"
            mat = sim_utils.RigidBodyMaterialCfg(static_friction=mu, dynamic_friction=mu)
            mat.func(mats[mu], mat)
        return mats[mu]

    def look(color, rough=0.6) -> str:
        key = (tuple(color), rough)
        if key not in looks:
            looks[key] = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough,
                                              metallic=0.8 if rough < 0.4 else 0.0)
            vis.func(looks[key], vis)
        return looks[key]

    def collide(prim, mu) -> None:
        UsdPhysics.CollisionAPI.Apply(prim)
        col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        col.CreateContactOffsetAttr(0.001)
        col.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), material(mu), stage=stage)

    def body(name, pos, kinematic=False, depen=0.5):
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*map(float, pos)))
        xf.AddOrientOp(precision=dbl).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(kinematic)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(depen)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(4)
        if not kinematic:
            rb.CreateEnableCCDAttr(True)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return path, prim

    def finish(g, color, mu, rough):
        if mu is not None:
            collide(g.GetPrim(), mu)
        if color is None:
            UsdGeom.Imageable(g.GetPrim()).MakeInvisible()
        else:
            bind_visual_material(str(g.GetPath()), look(color, rough), stage=stage)

    def cube(parent, name, centre, size, color=None, mu=None, rough=0.6):
        c = UsdGeom.Cube.Define(stage, f"{parent}/{name}")
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        finish(c, color, mu, rough)

    def cyl(parent, name, centre, height, radius, color, mu=None, rough=0.6, axis="Z"):
        g = UsdGeom.Cylinder.Define(stage, f"{parent}/{name}")
        g.CreateAxisAttr(axis)
        g.CreateHeightAttr(float(height))
        g.CreateRadiusAttr(float(radius))
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        finish(g, color, mu, rough)

    def ball(parent, name, centre, radius, color, mu=None):
        g = UsdGeom.Sphere.Define(stage, f"{parent}/{name}")
        g.CreateRadiusAttr(float(radius))
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        finish(g, color, mu, 0.8)

    z0 = s.table_z
    bt = s.board[2]
    bench, _ = body("bench", (0.0, 0.0, z0), kinematic=True)
    cube(bench, "board", (s.pos[0], s.pos[1], bt / 2), (s.board[0], s.board[1], bt), s.fabric_color, mu=0.9, rough=0.95)
    # tubing stakes: a wire hoop over the line
    for i, y in enumerate(s.stake_y):
        for j, dx in enumerate((-1, 1)):
            cyl(bench, f"stake{i}_{j}", (s.line_x + dx * (s.line_r + 0.002), y, bt + 0.012), 0.024, 0.0015,
                s.steel_color, mu=0.5, rough=0.35)
        cube(bench, f"stake{i}_top", (s.line_x, y, bt + 2 * s.line_r + 0.0035), (2 * s.line_r + 0.007, 0.003, 0.003),
             s.steel_color, mu=0.5, rough=0.35)
    # the shrub in its nursery pot
    pr, ph = s.pot
    cyl(bench, "pot", (*s.pot_xy, bt + ph / 2), ph, pr, s.pot_color, mu=0.8)
    ball(bench, "shrub", (s.pot_xy[0] + 0.01, s.pot_xy[1], bt + ph + 0.045), 0.07, s.leaf_color)
    # the drilled pine block: built round its two square holes
    bx, by, bh = s.block
    cx, cy = s.block_xy
    zc = bt + bh / 2
    holes = sorted([s.punch_hole, s.emitter_hole], key=lambda h: h[1])
    y_lo = -by / 2
    for k, (hx, hy, w) in enumerate(holes):
        cube(bench, f"block_seg{k}", (cx, cy + (y_lo + hy - w / 2) / 2, zc), (bx, hy - w / 2 - y_lo, bh),
             s.pine_color, mu=0.7)
        for j, sgn in enumerate((-1, 1)):
            xa, xb = (hx + w / 2, bx / 2) if sgn > 0 else (-bx / 2, hx - w / 2)
            cube(bench, f"block_side{k}_{j}", (cx + (xa + xb) / 2, cy + hy, zc), (xb - xa, w, bh), s.pine_color, mu=0.7)
        cyl(bench, f"block_rim{k}", (cx + hx, cy + hy, bt + bh + 0.0001), 0.0002, w / 2 + 0.0005, (0.35, 0.25, 0.15))
        y_lo = hy + w / 2
    cube(bench, "block_end", (cx, cy + (y_lo + by / 2) / 2, zc), (bx, by / 2 - y_lo, bh), s.pine_color, mu=0.7)
    cube(bench, "block_floor", (cx, cy, bt + 0.002), (bx, by, 0.004), s.pine_color, mu=0.7)

    # the drip line
    line, _ = body("line", (0.0, 0.0, z0), kinematic=True)
    y0, y1 = s.line_y
    cyl(line, "tube", (s.line_x, (y0 + y1) / 2, bt + s.line_r), y1 - y0, s.line_r, s.tube_color, mu=1.0, rough=0.5,
        axis="Y")
    for i, y in enumerate((y0, y1)):  # figure-8 end closures, drawn
        cyl(line, f"end{i}", (s.line_x, y, bt + s.line_r), 0.012, s.line_r + 0.002, (0.15, 0.15, 0.15), axis="Y")
    # the mark: a ring, drawn; moved to the seed's place at reset
    mark, _ = body("mark", (s.line_x, s.mark_y, s.line_z), kinematic=True)
    cyl(mark, "ring", (0.0, 0.0, 0.0), 0.003, s.line_r + 0.0003, s.mark_color, axis="Y")
    # holes: drawn discs, out of sight until made (origin at the hole's centre, +z its normal)
    for i in range(2):
        hole, _ = body(f"hole{i}", (s.line_x, 0.0, z0 - 0.05 - 0.01 * i), kinematic=True)
        cyl(hole, "round", (0.0, 0.0, 0.0002), 0.0006, 0.0017, s.hole_color)
        slit, _ = body(f"slit{i}", (s.line_x, 0.05, z0 - 0.05 - 0.01 * i), kinematic=True)
        cube(slit, "slit", (0.0, 0.0, 0.0002), (0.0065, 0.0016, 0.0006), s.hole_color)

    # the punch, standing in its block hole
    punch, prim = body("punch", s.punch_rest())
    r0 = s.pin[0]  # the point: a 1.5 mm round tip (one steady contact on the round tube), then the shank
    ball(punch, "point", (0.0, 0.0, r0), r0, s.steel_color, mu=1.5)
    cyl(punch, "pin", (0.0, 0.0, (r0 + s.pin[1]) / 2), s.pin[1] - r0, r0, s.steel_color, mu=1.0, rough=0.3)
    cyl(punch, "stop", (0.0, 0.0, (s.pin[1] + s.stop[1]) / 2), s.stop[1] - s.pin[1], s.stop[0], s.steel_color,
        mu=0.4, rough=0.3)
    hl = s.handle[1] - s.stop[1]
    cyl(punch, "handle_look", (0.0, 0.0, s.stop[1] + hl / 2), hl, s.handle[0], s.handle_color, rough=0.5)
    cube(punch, "handle", (0.0, 0.0, s.stop[1] + hl / 2), (0.022, 0.022, hl), None, mu=2.0)
    m = UsdPhysics.MassAPI.Apply(prim)
    m.CreateMassAttr(s.punch_mass)
    m.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.06))
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(4e-5, 4e-5, 4e-6))

    # the dripper, barb down in its block hole
    em, prim = body("emitter", s.emitter_rest())
    rb = 0.0012  # the barb's rounded point, then its shank
    ball(em, "barb_point", (0.0, 0.0, rb), rb, (0.15, 0.15, 0.15), mu=1.5)
    cyl(em, "barb", (0.0, 0.0, (rb + s.barb[1]) / 2), s.barb[1] - rb, s.barb[0], (0.15, 0.15, 0.15), mu=0.5)
    h = s.body[1] - s.barb[1]
    cyl(em, "body_look", (0.0, 0.0, s.barb[1] + h / 2), h, s.body[0], s.emitter_color, rough=0.5)
    cube(em, "body", (0.0, 0.0, s.barb[1] + h / 2), (0.022, 0.022, h), None, mu=1.5)  # flat sides for the pads
    cyl(em, "cap", (0.0, 0.0, s.body[1] + 0.0005), 0.001, s.body[0] * 0.6, (0.9, 0.9, 0.9))
    m = UsdPhysics.MassAPI.Apply(prim)
    m.CreateMassAttr(s.emitter_mass)
    m.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.015))
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(1.2e-6, 1.2e-6, 1.2e-6))
    return root.GetPrim()


def _arena_object(spec: DripBed):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class _Cfg(SpawnerCfg):
        func: object = clone(_spawn)
        rig_name: str = ""

    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=_Cfg(rig_name=spec.name),
                  initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


# -- runtime ---------------------------------------------------------------------------------------------------------


def _arr(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def _like(ref, data: np.ndarray, dtype="float32"):
    if hasattr(ref, "detach"):
        import torch

        return torch.tensor(data, dtype=getattr(torch, dtype), device=ref.device)
    import warp as wp

    return wp.array(np.asarray(data).astype(dtype), dtype=getattr(wp, dtype), device=getattr(ref, "device", None))


class _Body:
    def __init__(self, view, path):
        self.v = view.create_rigid_body_view([path])
        self.start = _arr(self.v.get_transforms()).copy()

    def pose(self):
        return _arr(self.v.get_transforms())[0]

    def vel(self):
        return _arr(self.v.get_velocities())[0]

    def put(self, tr, vel=None):
        cur = self.v.get_transforms()
        idx = _like(cur, np.zeros(1), "int32")
        self.v.set_transforms(_like(cur, np.asarray(tr, float).reshape(1, 7)), idx)
        if vel is not None:
            self.v.set_velocities(_like(cur, np.asarray(vel, float).reshape(1, 6)), idx)


class _Runtime:
    """PhysX views, the tubing rule and the latches for one bed (env 0)."""

    def __init__(self, base, spec: DripBed):
        from isaaclab_physx.physics import PhysxManager

        self.spec, self.base = spec, base
        root = _PATHS[spec.name]
        view = PhysxManager.get_physics_sim_view()
        self.punch = _Body(view, f"{root}/punch")
        self.emitter = _Body(view, f"{root}/emitter")
        self.mark = _Body(view, f"{root}/mark")
        self.hole_bodies = [_Body(view, f"{root}/hole{i}") for i in range(2)]
        self.slit_bodies = [_Body(view, f"{root}/slit{i}") for i in range(2)]
        links, link_arm = [], []
        for arm in ARMS:
            paths = base.scene[arm].root_physx_view.link_paths
            paths = list(paths[0] if paths and isinstance(paths[0], (list, tuple)) else paths)
            links += paths
            link_arm += [arm] * len(paths)
        names = [p.rsplit("/", 1)[-1] for p in links]
        self.tip_idx = {arm: [i for i, n in enumerate(names) if link_arm[i] == arm and n in ("tip_left", "tip_right")]
                        for arm in ARMS}
        self.n_links = len(links)
        flt = [*links, f"{root}/line"]
        self.c_punch = view.create_rigid_contact_view([f"{root}/punch"], filter_patterns=[flt],
                                                      max_contact_data_count=256)
        self.c_em = view.create_rigid_contact_view([f"{root}/emitter"], filter_patterns=[flt],
                                                   max_contact_data_count=256)
        self.dt = float(base.physics_dt)
        self.origin = _arr(base.scene.env_origins)[0]
        self.debug = bool(os.environ.get("DRIP_DEBUG"))
        self.n = 0
        self.seed = None
        self.clear()

    def clear(self) -> None:
        self.line = new_line(self.spec, self.seed)
        self.held = None
        self.grip_n = self.egrip_n = 0.0
        self.punch_peak = self.em_peak = 0.0
        self.pin_pose = None
        self.events: list[str] = []
        tr = self.mark.start[0].copy()
        tr[:3] = self.origin + (self.spec.line_x, self.line.mark_y, self.spec.line_z)
        self.mark.put(tr)
        for b in self.hole_bodies + self.slit_bodies:
            b.put(b.start[0].copy())

    def say(self, msg: str) -> None:
        self.events.append(msg)
        print(f"[dripkit] t {self.n * self.dt:6.2f} {msg}", flush=True)

    def reset(self, seed) -> None:
        self.seed = int(os.environ.get("DRIP_SEED", "0"))  # the run's --seed does not reach the reset as given
        self.punch.put(self.punch.start[0].copy(), np.zeros(6))
        self.emitter.put(self.emitter.start[0].copy(), np.zeros(6))
        self.clear()
        print(f"[dripkit] seed {self.seed}: mark at y {self.line.mark_y:+.4f}, wall gives at "
              f"{self.line.law.punch_n:.1f} N", flush=True)

    def _tool(self, b: _Body):
        tr = b.pose()
        return tr[:3] - self.origin, quat_to_matrix(tr[3:7])[:, 2]

    def _draw_holes(self) -> None:
        """Round holes as a 3.4 mm dark disc, torn ones as a 6.5 mm slit (only the first two holes are drawn)."""
        for i, h in enumerate(self.line.holes[:2]):
            show, hide = (self.slit_bodies[i], self.hole_bodies[i]) if h.torn else (self.hole_bodies[i],
                                                                                    self.slit_bodies[i])
            tr = show.start[0].copy()
            tr[:3] = self.origin + h.p
            tr[3:7] = matrix_to_quat(frame_from_z(h.n))
            show.put(tr)
            hide.put(hide.start[0].copy())

    def physics_step(self) -> None:
        s, line = self.spec, self.line
        self.n += 1
        fp = _arr(self.c_punch.get_contact_force_matrix(dt=self.dt)).reshape(-1, 3)
        fe = _arr(self.c_em.get_contact_force_matrix(dt=self.dt)).reshape(-1, 3)
        p_push = float(np.linalg.norm(fp[self.n_links]))
        e_push = float(np.linalg.norm(fe[self.n_links]))
        self.punch_peak = max(self.punch_peak, p_push)
        self.em_peak = max(self.em_peak, e_push)
        self.grip_n = max(float(np.linalg.norm(fp[i])) for i in self.tip_idx["right_robot"])
        self.egrip_n = max(float(np.linalg.norm(fe[i])) for i in self.tip_idx["right_robot"])
        held = None
        for arm in ARMS:
            if all(np.linalg.norm(fp[i]) > 0.3 for i in self.tip_idx[arm]):
                held = arm
        if held != self.held:
            self.say(f"punch {'held by ' + held if held else 'let go'}")
        self.held = held
        tip, axis = self._tool(self.punch)
        ev = line.punch_step(tip, axis, p_push, self.dt)
        if ev:
            self.say(ev)
            self._draw_holes()
        if line.seated is None:
            etip, eaxis = self._tool(self.emitter)
            ev = line.emitter_step(etip, eaxis, e_push, self.dt)
            if ev:
                self.say(ev)
                self._draw_holes()
            if line.seated is not None:
                h = line.holes[line.seated]
                tr = self.emitter.start[0].copy()
                tr[:3] = self.origin + h.p - h.n * s.barb[1]
                tr[3:7] = matrix_to_quat(frame_from_z(h.n))
                self.pin_pose = tr
        if self.pin_pose is not None:  # seated: the barb is through the wall, the collar on the tube
            self.emitter.put(self.pin_pose, np.zeros(6))

    def _tcp_pose(self):
        """The right arm's tool point and its rotation (approach = +z), env frame."""
        rr = self.base.scene["right_robot"]
        i = rr.find_bodies("gripper")[0][0]
        g = _arr(rr.data.body_link_pose_w)[0][i]
        R = quat_to_matrix(g[3:7])
        return g[:3] - self.origin + R[:, 2] * 0.134, R

    def _tcp(self):
        return self._tcp_pose()[0]

    def state(self) -> DripState:
        tip, axis = self._tool(self.punch)
        v = self.punch.vel()
        etip, _ = self._tool(self.emitter)
        line = Line(**{**self.line.__dict__, "holes": [Hole(**h.__dict__) for h in self.line.holes]})
        home = in_block(self.spec, tip, axis) and self.held is None
        st = DripState(pos=tip, quat_xyzw=self.punch.pose()[3:7], lin_vel=v[:3], ang_vel=v[3:],
                       low=np.minimum(tip, tip + axis * 0.105) - 0.014, high=np.maximum(tip, tip + axis * 0.105) + 0.014,
                       line=line, punch_home=home, punch_held=self.held, emitter_tip=etip,
                       punch_n=self.punch_peak, emitter_n=self.em_peak)
        if self.debug and self.n % 24 == 0:
            n, h = line.surface(tip)
            print(f"[dripkit] t {self.n * self.dt:6.2f} punch tip {np.round(tip, 4)} axis {np.round(axis, 3)} "
                  f"h {h * 1000:+.1f} mm push {self.punch_peak:5.1f} N | emitter tip {np.round(etip, 4)} push "
                  f"{self.em_peak:5.1f} N | grip {self.grip_n:5.1f}/{self.egrip_n:5.1f} N | tcp {np.round(self._tcp(), 4)} | held {self.held} home {home} holes {len(line.holes)} seated {line.seated}",
                  flush=True)
        self.punch_peak = self.em_peak = 0.0
        return st


_RT: dict[str, _Runtime] = {}


def _runtime(base, name: str) -> _Runtime:
    if name not in _RT:
        _RT[name] = _Runtime(base, _SPECS[name])
    return _RT[name]


def runtime(name: str) -> _Runtime | None:
    """The running rule for a bed (for the scripted demo, which may read the simulator's state)."""
    return _RT.get(name)


# -- hooks (as in our other tasks' kits) -----------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_dripkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, DripBed):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _runtime(environment.unwrapped, n).state() for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._dripkit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_dripkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for name in _SPECS:
            _runtime(base, name).physics_step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._dripkit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_dripkit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def _ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
        return out

    def _reset(self, scene, *, seed=None):
        self._ensure_sim(None)
        for name in _SPECS:
            _runtime(self._base, name).reset(seed)
        obs = reset(self, scene, seed=seed)
        for name in _SPECS:
            rt = _runtime(self._base, name)
            rt.clear()  # the settle steps are not part of the trial
            rt.n = 0
        return obs

    cls._ensure_sim, cls.reset, cls._dripkit = _ensure_sim, _reset, True


def _patch_scripted(module) -> None:
    """``yam_scripted`` plays the task file's closed-loop ``controller`` when it has one."""
    cls = module.YamScriptedPolicy
    if getattr(cls, "_dripkit", False):
        return
    reset, act = cls.reset, cls.act

    def _make(self):
        task = self._task
        make = getattr(task.module, "controller", None) if task is not None else None
        return make(dict(self._bases), self._dt) if make is not None else None

    def _reset(self, scene):
        self._ctl = _make(self)
        if self._ctl is None:
            return reset(self, scene)

    def _act(self, observation):
        if getattr(self, "_ctl", None) is None:
            return act(self, observation)
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import pack

        left, right, finished_at = self._ctl.act(observation)
        stop = finished_at is not None and float(observation.state_time) > finished_at + self.hold_s
        return ActionChunk(actions=[Action(data=pack(left, right), meta={"request_stop": True} if stop else {})],
                           control_hz=1.0 / self._dt)

    cls.reset, cls.act, cls._dripkit = _reset, _act, True


def _patch_demo(demo) -> None:
    if getattr(demo, "_dripkit", False):
        return
    grip_rotation = demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else grip_rotation(yaw)

    demo.grip_rotation, demo._dripkit = _grip_rotation, True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
            "yam_arena.scripted": _patch_scripted, "yam_arena.demo": _patch_demo}


class _Finder(importlib.abc.MetaPathFinder):
    """Patches each target module right after it is first imported."""

    def find_spec(self, fullname, path, target=None):
        if fullname not in _TARGETS:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        exec_module = spec.loader.exec_module

        def _exec(module):
            exec_module(module)
            _TARGETS[fullname](module)

        spec.loader.exec_module = _exec
        return spec


def install() -> None:
    for name, patch in _TARGETS.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
