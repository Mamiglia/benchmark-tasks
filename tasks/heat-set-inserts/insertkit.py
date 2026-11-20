"""insertkit: a soldering iron in its stand, a printed bracket and four M3 heat-set inserts, for a yam_arena task file.

yam_arena task files only take single primitives. This module adds one object, :class:`InsertRig`, spawned as a
static (``BASE``) Arena asset holding these bodies:

- ``bench`` (kinematic): a silicone soldering mat and, lying on it, the PETG bracket (80 x 40 x 12 mm) with its four
  4.0 mm holes; the iron stand's weighted base, clamped to the table.
- ``stand`` (kinematic): the stand's holder, a tube (30 mm inside, 90 mm long, closed at its lower end) tilted
  ``stand_pitch`` below the horizontal, pointing away from the arms.
- ``insert0`` .. ``insert3`` (kinematic): brass M3 x 5.7 mm inserts (4.6 mm across), each standing in its hole,
  ``start_proud`` above the bracket's top face. Only this kit moves them.
- ``flat`` (dynamic): the flat block, an aluminium bar offcut 19 x 19 x 50 mm (49 g) standing on its end on the mat,
  for pressing a just-sunk insert flush and square while the plastic is still soft.
- ``iron`` (dynamic): the soldering iron with an M3 heat-set tip. Body frame: origin at the end of the tip's pin,
  +z along the iron toward the cord. Pin 2.1 mm x 5 mm (seen, no collider: it sits inside the insert's bore),
  shoulder 6.5 mm across, barrel 8 mm across to 70 mm, then the grip (20 mm across) from 80 to 175 mm.

The plastic (the rule standing in for melting, in the fidelity note). Before every physics step (240 Hz, a wrapper
on ``InteractiveScene.write_data_to_sim``) the kit reads the PhysX contact force of the iron on each insert.
An insert is *engaged* when the iron's pin is within ``engage_m`` of its axis and the iron presses on it. While
engaged it heats; after ``heat_s`` of contact the plastic round it is soft, and from then on it sinks at
``k (F - f_min)`` (F: the iron's push along the hole, N), at most ``v_max``. Pushed harder than ``f_max`` for
``over_s`` while it sinks, it is *forced* (driven faster than the plastic flows: it ends loose, latched). Sinking while
the iron leans more than ``tilt_deg`` from the hole's axis sets it *crooked* (latched; the insert then leans with the
iron). Off the iron for ``cool_s``, it cools and must heat again. It stops at the hole's bottom, 1 mm past flush.
Any part of the iron pressing on the bracket for ``mark_s`` in all leaves a melt mark, charged to the insert nearest
the tip (latched). An insert sunk by the iron has a *lip* (plastic raised round it, its top not registered to the
face). The flat block pressed on it within ``cool_s`` of the iron leaving (still soft) sinks its last ``flat_reach``
at the same rate, stops at flush (the block lands on the face) and, pressed level, flattens the lip; leaning more than
``tilt_deg`` while it sinks, it sets the insert crooked; over ``f_max``, forced.

Latches over the whole trial: ``burned`` (a hot part of the iron, tip to the end of the barrel, touched the robot),
``dropped`` (the iron out of the stand and in no grip for ``drop_s``).

``object_states`` reports an :class:`InsertState`. Importing the module also hooks ``yam_arena.scripted``: when the task
file defines ``controller(bases, dt)``, ``yam_scripted`` asks it for every action, passing it the observation (camera
images and joint positions). yam_arena itself is not modified. Pure Python at import; Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

Color = tuple[float, float, float]
ARMS = ("left_robot", "right_robot")
N_INSERTS = 4


# -- the plastic (pure Python, unit-tested) --------------------------------------------------------------------------


@dataclass(frozen=True)
class Law:
    """The softening-plastic rule (SI units)."""

    touch_n: float = 0.5  # contact counts from this force
    engage_m: float = 0.0015  # pin within this of the insert's axis: its tapered point finds the bore
    heat_s: float = 2.5  # contact time before the plastic round the insert is soft
    cool_s: float = 3.0  # off the iron this long: cold again (the window for the flat block)
    f_min: float = 1.5  # N: below this a soft insert does not sink
    k: float = 0.00030  # m/s per N above f_min (0.3 mm/s per N)
    v_max: float = 0.003  # m/s
    f_max: float = 15.0  # N: pushed harder than this while sinking ...
    over_s: float = 0.25  # ... for this long: forced
    tilt_deg: float = 5.0  # iron's lean from the hole's axis while sinking: crooked
    mark_n: float = 0.5  # the iron pressing on the bracket ...
    mark_s: float = 0.5  # ... this long in all: a melt mark
    flat_reach: float = 0.0015  # the flat block, pressed on a still-soft insert, sinks it only from this close to flush


@dataclass
class Insert:
    """One insert's state. ``depth``: how far it has sunk from its start (m); ``travel``: the depth at flush;
    ``bottom``: the depth at which it bottoms in the hole."""

    travel: float
    bottom: float
    depth: float = 0.0
    heat: float = 0.0
    off: float = 0.0
    over: float = 0.0
    sank: bool = False  # sunk at all
    crooked: bool = False
    forced: bool = False
    marked: bool = False
    lip: bool = False  # sunk by the iron: plastic raised round it, its top not yet pressed flat with the block
    lean: tuple = (0.0, 0.0, 1.0)  # its axis once crooked

    @property
    def proud(self) -> float:
        """Its top face above the bracket's top face (m); negative when sunk below it."""
        return self.travel - self.depth

    def step(self, law: Law, push: float, engaged: bool, tilt: float, axis, dt: float, flat: float = 0.0,
             flat_tilt: float = 0.0, flat_axis=(0.0, 0.0, 1.0)) -> float:
        """One physics step: ``push`` (N) along the hole by the iron, ``engaged`` (pin in the bore), ``tilt`` (deg)
        of the iron from the hole's axis, ``axis`` the iron's axis (unit); ``flat`` (N) the flat block's push on it,
        ``flat_tilt`` / ``flat_axis`` the block's. Returns how much it sank (m)."""
        if engaged and push >= law.touch_n:
            self.heat += dt
            self.off = 0.0
        else:
            self.off += dt
            if self.off >= law.cool_s:
                self.heat = 0.0
            if self.heat >= law.heat_s and self.sank and flat >= law.touch_n and not self.forced:
                return self._flat(law, flat, flat_tilt, flat_axis, dt)
            self.over = 0.0
            return 0.0
        if self.heat < law.heat_s or self.forced:
            return 0.0
        v = min(law.k * max(0.0, push - law.f_min), law.v_max)
        dz = min(v * dt, self.bottom - self.depth)
        if dz <= 0.0:
            return 0.0
        self.over = self.over + dt if push > law.f_max else 0.0
        if self.over >= law.over_s:
            self.forced = True
        if tilt > law.tilt_deg and not self.crooked:
            self.crooked = True
            self.lean = tuple(float(a) for a in axis)
        self.depth += dz
        self.sank = self.lip = True
        return dz

    def _flat(self, law: Law, push: float, tilt: float, axis, dt: float) -> float:
        """The flat block on the insert while the plastic is still soft: it sinks the last ``flat_reach`` at the
        plastic's rate and stops at flush (the block lands on the face); pressed level it flattens the lip."""
        if self.proud > law.flat_reach:
            return 0.0
        v = min(law.k * max(0.0, push - law.f_min), law.v_max)
        dz = min(v * dt, max(0.0, self.proud))
        if dz > 0.0:
            self.over = self.over + dt if push > law.f_max else 0.0
            if self.over >= law.over_s:
                self.forced = True
            if tilt > law.tilt_deg and not self.crooked:
                self.crooked = True
                self.lean = tuple(float(a) for a in axis)
            self.depth += dz
        if tilt <= law.tilt_deg and push >= law.f_min:
            self.lip = False
        return dz


def insert_credit(ins: Insert, flush_m: float = 0.0005, seated_m: float = 0.0025) -> float:
    """0.25 flush (top within 0.5 mm of the face, either way) and pressed flat with the block while soft; 0.10
    seated but not both (top at most 2.5 mm proud, or sunk past 0.5 mm, or flush with its lip left); 0 otherwise or
    if crooked, forced or marked."""
    if ins.crooked or ins.forced or ins.marked or not ins.sank:
        return 0.0
    if abs(ins.proud) <= flush_m:
        return 0.10 if ins.lip else 0.25
    return 0.10 if ins.proud <= seated_m else 0.0


# -- the object ------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class InsertRig(Box):
    """Positions in the env frame (x forward, y left, z up, m). ``pos`` is the bracket's top-face centre."""

    table_z: float = 0.0
    mat: tuple[float, float, float] = (0.20, 0.16, 0.002)  # silicone mat, x, y, thickness
    mat_xy: tuple[float, float] = (0.27, -0.045)
    bracket: tuple[float, float, float] = (0.040, 0.080, 0.012)  # x, y (long side along y), height
    holes: tuple = ((-0.010, -0.028), (0.010, -0.028), (-0.010, 0.028), (0.010, 0.028))  # from the bracket centre
    insert_r: float = 0.0023
    insert_len: float = 0.0057
    start_proud: float = 0.0045  # mean; each insert's own from the seed
    proud_jitter: float = 0.0003
    hole_extra: float = 0.0010  # hole depth past flush
    soft_jitter: float = 0.25  # the bracket's print: k varies by this fraction with the seed
    stand_xy: tuple[float, float] = (0.42, -0.27)  # the holder tube's closed end, over the base
    stand_pitch_deg: float = 50.0
    tube_in: float = 0.030
    tube_len: float = 0.090
    tube_bottom_z: float = 0.030  # above the table
    base: tuple[float, float, float] = (0.11, 0.09, 0.022)
    pin: tuple[float, float] = (0.00105, 0.005)  # radius, length
    shoulder: tuple[float, float] = (0.00324, 0.003)
    barrel: tuple[float, float] = (0.004, 0.070)  # radius, end (from the pin's end)
    collar: tuple[float, float] = (0.0065, 0.080)
    grip: tuple[float, float] = (0.0095, 0.175)  # half-width, end
    tail: tuple[float, float] = (0.006, 0.205)
    iron_mass: float = 0.060
    hot_end: float = 0.075  # tip to here is hot
    flat_size: tuple[float, float, float] = (0.019, 0.019, 0.050)  # the flat block: x, y, height
    flat_xy: tuple[float, float] = (0.22, 0.005)  # where it stands on the mat
    flat_mass: float = 0.049
    alu_color: Color = (0.78, 0.79, 0.80)
    law: Law = Law()
    drop_s: float = 2.0
    mat_color: Color = (0.20, 0.42, 0.55)
    petg_color: Color = (0.95, 0.45, 0.10)
    brass_color: Color = (0.80, 0.62, 0.25)
    steel_color: Color = (0.70, 0.71, 0.73)
    grip_color: Color = (0.12, 0.30, 0.65)
    dark_color: Color = (0.12, 0.12, 0.13)
    sponge_color: Color = (0.90, 0.80, 0.30)

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: an InsertRig is a fixed asset (mass None); its bodies carry their own")

    @property
    def face_z(self) -> float:
        """The bracket's top face, absolute."""
        return self.table_z + self.mat[2] + self.bracket[2]

    def hole_xy(self, i: int) -> np.ndarray:
        return np.array(self.pos[:2]) + np.array(self.holes[i])

    @property
    def tube_axis(self) -> np.ndarray:
        """Unit vector down the holder tube (mouth to closed end): the iron's tip direction in the stand."""
        p = math.radians(self.stand_pitch_deg)
        return np.array([math.cos(p), 0.0, -math.sin(p)])

    @property
    def tube_down(self) -> np.ndarray:
        """Unit vector across the tube toward its lower wall."""
        p = math.radians(self.stand_pitch_deg)
        return -np.array([math.sin(p), 0.0, math.cos(p)])

    @property
    def rest_drop(self) -> float:
        """How far the resting iron's axis sits below the tube's axis (its grip lies on the lower wall)."""
        return self.tube_in / 2 - self.grip[0]

    @property
    def tube_bottom(self) -> np.ndarray:
        return np.array([*self.stand_xy, self.table_z + self.tube_bottom_z])


def seeded(spec: InsertRig, seed: int | None):
    """Per seed: each insert's start height above the face (m) and the bracket's softness factor."""
    rng = np.random.default_rng(0 if seed is None else int(seed))
    proud = [float(spec.start_proud + rng.uniform(-spec.proud_jitter, spec.proud_jitter)) for _ in range(N_INSERTS)]
    soft = float(1.0 + rng.uniform(-spec.soft_jitter, spec.soft_jitter))
    return proud, soft


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
    """A rotation whose +z is ``z`` and whose +y is world y when ``z`` lies in the x-z plane."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    y = np.array([0.0, 1.0, 0.0]) - z[1] * z
    if np.linalg.norm(y) < 1e-6:
        y = np.array([1.0, 0.0, 0.0]) - z[0] * z
    y /= np.linalg.norm(y)
    return np.column_stack([np.cross(y, z), y, z])


def tilt_deg(axis) -> float:
    return math.degrees(math.acos(min(1.0, abs(float(np.asarray(axis)[2])))))


def in_stand(spec: InsertRig, tip, axis) -> bool:
    """The iron's tip inside the holder tube, the iron along it within 30 degrees (it rests leaning across the tube)."""
    a = spec.tube_axis
    rel = np.asarray(tip, float) - spec.tube_bottom
    along = float(rel @ -a)
    radial = float(np.linalg.norm(rel - along * -a))
    return -0.005 <= along <= spec.tube_len and radial <= spec.tube_in / 2 + 0.002 and float(-a @ axis) >= math.cos(
        math.radians(30))


@dataclass(frozen=True)
class InsertState(ObjectState):
    """After a control step (env frame). ``pos`` is the iron's tip (pin end)."""

    axis: np.ndarray = None  # the iron's axis, tip to cord
    inserts: tuple = ()  # Insert per hole (copies)
    held: str | None = None
    in_stand: bool = False
    burned: bool = False
    dropped: bool = False
    push_n: float = 0.0  # largest iron-on-insert push during the step
    bracket_n: float = 0.0  # largest iron-on-bracket force during the step


# -- spawning (Isaac Sim only) ---------------------------------------------------------------------------------------

_SPECS: dict[str, InsertRig] = {}
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

    def body(name, pos, kinematic=False, R=None):
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*map(float, pos)))
        if R is not None:
            x, y, z, w = matrix_to_quat(R)
            xf.AddOrientOp(precision=dbl).Set(Gf.Quatd(float(w), float(x), float(y), float(z)))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(kinematic)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(4)
        if not kinematic:
            rb.CreateEnableCCDAttr(True)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return path, prim

    def cube(parent, name, centre, size, color=None, mu=None, rough=0.6):
        c = UsdGeom.Cube.Define(stage, f"{parent}/{name}")
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        if mu is not None:
            collide(c.GetPrim(), mu)
        if color is None:
            UsdGeom.Imageable(c.GetPrim()).MakeInvisible()
        else:
            bind_visual_material(str(c.GetPath()), look(color, rough), stage=stage)

    def cyl(parent, name, z0, z1, radius, color, mu=None, rough=0.6, xy=(0.0, 0.0)):
        g = UsdGeom.Cylinder.Define(stage, f"{parent}/{name}")
        g.CreateAxisAttr("Z")
        g.CreateHeightAttr(float(z1 - z0))
        g.CreateRadiusAttr(float(radius))
        g.AddTranslateOp().Set(Gf.Vec3d(float(xy[0]), float(xy[1]), float((z0 + z1) / 2)))
        if mu is not None:
            collide(g.GetPrim(), mu)
        if color is None:
            UsdGeom.Imageable(g.GetPrim()).MakeInvisible()
        else:
            bind_visual_material(str(g.GetPath()), look(color, rough), stage=stage)

    z0 = s.table_z
    # bench: mat, bracket (one solid collider: the inserts are kinematic, so they never collide with it), stand base
    bench, _ = body("bench", (0.0, 0.0, z0), kinematic=True)
    mx, my, mt = s.mat
    cube(bench, "mat", (*s.mat_xy, mt / 2), (mx, my, mt), s.mat_color, mu=1.0, rough=0.9)
    bx, by, bh = s.bracket
    cube(bench, "bracket", (s.pos[0], s.pos[1], mt + bh / 2), (bx, by, bh), s.petg_color, mu=0.6, rough=0.5)
    for i in range(N_INSERTS):  # the hole's dark rim, seen round each insert
        cyl(bench, f"hole{i}", mt + bh, mt + bh + 0.0002, 0.0026, s.dark_color, xy=s.hole_xy(i))
    ax = s.tube_axis
    bxy = np.array(s.stand_xy) - 0.035 * ax[:2] / max(np.linalg.norm(ax[:2]), 1e-9)
    cube(bench, "stand_base", (*bxy, s.base[2] / 2), s.base, s.dark_color, mu=0.8)
    cube(bench, "sponge", (bxy[0] - 0.03, bxy[1] + 0.02, s.base[2] + 0.004), (0.035, 0.035, 0.008), s.sponge_color)
    post = s.tube_bottom_z - s.base[2]
    cyl(bench, "stand_post", s.base[2], s.base[2] + post, 0.006, s.dark_color, xy=s.stand_xy)

    # the holder tube: local +z up the tube (bottom to mouth)
    R_tube = frame_from_z(-ax)
    stand, _ = body("stand", s.tube_bottom, kinematic=True, R=R_tube)
    w, L, t = s.tube_in, s.tube_len, 0.002
    for i, (cx, cy, sx, sy) in enumerate(((w / 2 + t / 2, 0, t, w + 2 * t), (-w / 2 - t / 2, 0, t, w + 2 * t),
                                          (0, w / 2 + t / 2, w, t), (0, -w / 2 - t / 2, w, t))):
        cube(stand, f"wall{i}", (cx, cy, L / 2), (sx, sy, L), s.steel_color, mu=0.4, rough=0.3)
    cube(stand, "floor", (0, 0, -t / 2), (w + 2 * t, w + 2 * t, t), s.dark_color, mu=0.4)

    # inserts: origin at the top face's centre, body below it
    for i in range(N_INSERTS):
        x, y = s.hole_xy(i)
        ins, _ = body(f"insert{i}", (x, y, s.face_z - z0 + z0 + s.start_proud), kinematic=True)
        cyl(ins, "brass", -s.insert_len, 0.0, s.insert_r, s.brass_color, mu=0.1, rough=0.35)
        cyl(ins, "bore", -0.0004, 0.00005, 0.00125, s.dark_color)

    # the flat block, standing on its end on the mat: origin at its bottom face's centre
    fx, fy, fh = s.flat_size
    flat, fprim = body("flat", (*s.flat_xy, z0 + mt))
    cube(flat, "block", (0, 0, fh / 2), (fx, fy, fh), s.alu_color, mu=1.0, rough=0.35)
    UsdPhysics.MassAPI.Apply(fprim).CreateMassAttr(s.flat_mass)

    # iron, resting in the holder: origin at the pin's end, +z toward the cord
    tip0 = s.tube_bottom + 0.002 * -ax + s.rest_drop * s.tube_down
    iron, prim = body("iron", tip0, R=frame_from_z(-ax))
    cyl(iron, "pin", 0.0, s.pin[1], s.pin[0], s.steel_color, rough=0.3)
    sz = s.pin[1]
    cyl(iron, "shoulder", sz, sz + s.shoulder[1], s.shoulder[0], s.steel_color, mu=0.1, rough=0.3)
    cyl(iron, "barrel", sz + s.shoulder[1], s.barrel[1], s.barrel[0], s.steel_color, mu=0.5, rough=0.3)
    cyl(iron, "collar", s.barrel[1], s.collar[1], s.collar[0], s.dark_color, mu=0.8)
    g = s.grip[0]
    cyl(iron, "grip_look", s.collar[1], s.grip[1], g + 0.0005, s.grip_color)
    cube(iron, "grip", (0, 0, (s.collar[1] + s.grip[1]) / 2), (2 * g, 2 * g, s.grip[1] - s.collar[1]), None, mu=2.0)
    cyl(iron, "tail", s.grip[1], s.tail[1], s.tail[0], s.dark_color, mu=0.8)
    m = UsdPhysics.MassAPI.Apply(prim)
    m.CreateMassAttr(s.iron_mass)
    m.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.12))
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(2.2e-4, 2.2e-4, 3e-6))
    return root.GetPrim()


def _arena_object(spec: InsertRig):
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
    """PhysX views, the plastic rule and the latches for one rig (env 0)."""

    def __init__(self, base, spec: InsertRig):
        from isaaclab_physx.physics import PhysxManager

        self.spec, self.base = spec, base
        root = _PATHS[spec.name]
        view = PhysxManager.get_physics_sim_view()
        self.iron = _Body(view, f"{root}/iron")
        self.flat = _Body(view, f"{root}/flat")
        self.ins = [_Body(view, f"{root}/insert{i}") for i in range(N_INSERTS)]
        links, self.link_arm = [], []
        for arm in ARMS:
            paths = base.scene[arm].root_physx_view.link_paths
            paths = list(paths[0] if paths and isinstance(paths[0], (list, tuple)) else paths)
            links += paths
            self.link_arm += [arm] * len(paths)
        self.links = links
        self.link_names = [p.split("/")[-3] + "/" + p.rsplit("/", 1)[-1] for p in links]
        self.link_arm = np.array(self.link_arm)
        self.tip_idx = {arm: [i for i, n in enumerate(self.link_names)
                              if self.link_arm[i] == arm and n.endswith(("tip_left", "tip_right"))] for arm in ARMS}
        # iron vs [robot links..., bench, stand]; each insert vs iron
        self.c_iron = view.create_rigid_contact_view([f"{root}/iron"],
                                                     filter_patterns=[[*links, f"{root}/bench", f"{root}/stand"]],
                                                     max_contact_data_count=512)
        self.n_links = len(links)
        self.c_ins = view.create_rigid_contact_view([f"{root}/insert{i}" for i in range(N_INSERTS)],
                                                    filter_patterns=[[f"{root}/iron", f"{root}/flat"]] * N_INSERTS,
                                                    max_contact_data_count=256)
        self.dt = float(base.physics_dt)
        self.origin = _arr(base.scene.env_origins)[0]
        self.debug = bool(os.environ.get("HSI_DEBUG"))
        self.g_id = base.scene["right_robot"].find_bodies("gripper")[0][0]
        self.n = 0
        self.seed = None
        self.clear()

    def clear(self) -> None:
        s = self.spec
        proud, soft = seeded(s, self.seed)
        self.law = Law(**{**s.law.__dict__, "k": s.law.k * soft})
        self.proud0 = proud
        self.st = [Insert(travel=p, bottom=p + s.hole_extra) for p in proud]
        self.burned = self.dropped = False
        self.mark_t = 0.0
        self.loose_t = 0.0
        self.held = None
        self.push_peak = self.bracket_peak = self.flat_peak = 0.0
        self.events: list[str] = []
        self._place_inserts()

    def say(self, msg: str) -> None:
        self.events.append(msg)
        print(f"[insertkit] t {self.n * self.dt:6.2f} {msg}", flush=True)

    def _place_inserts(self) -> None:
        s = self.spec
        for i, (b, st) in enumerate(zip(self.ins, self.st)):
            tr = b.start[0].copy()
            x, y = s.hole_xy(i)
            tr[:3] = self.origin + (x, y, s.face_z + st.proud)
            R = frame_from_z(st.lean) if st.crooked else np.eye(3)
            tr[3:7] = matrix_to_quat(R)
            b.put(tr)

    def reset(self, seed) -> None:
        self.seed = int(os.environ.get("HSI_SEED", "0"))
        tr = self.iron.start[0].copy()
        self.iron.put(tr, np.zeros(6))
        self.flat.put(self.flat.start[0].copy(), np.zeros(6))
        self.clear()
        print(f"[insertkit] seed {self.seed}: inserts start {np.round(np.array(self.proud0) * 1000, 2)} mm proud, "
              f"plastic k {self.law.k * 1000:.3f} mm/s per N", flush=True)

    def _iron(self):
        tr = self.iron.pose()
        R = quat_to_matrix(tr[3:7])
        return tr[:3] - self.origin, R[:, 2]

    def physics_step(self) -> None:
        s, law = self.spec, self.law
        self.n += 1
        tip, axis = self._iron()
        tilt = tilt_deg(axis)
        fi = _arr(self.c_iron.get_contact_force_matrix(dt=self.dt)).reshape(-1, 3)
        fins = _arr(self.c_ins.get_contact_force_matrix(dt=self.dt)).reshape(N_INSERTS, 2, 3)
        f_axis = quat_to_matrix(self.flat.pose()[3:7])[:, 2]
        f_tilt = tilt_deg(f_axis)
        link_n = np.linalg.norm(fi[: self.n_links], axis=1)
        bracket_n = float(np.linalg.norm(fi[self.n_links]))
        self.bracket_peak = max(self.bracket_peak, bracket_n)
        # hot part on the robot: contact points along the iron's axis nearer the tip than hot_end
        if link_n.max(initial=0.0) > 0.05 and not self.burned:
            data = self.c_iron.get_contact_data(dt=self.dt)
            forces, points = _arr(data[0]).reshape(-1), _arr(data[1]).reshape(-1, 3)
            counts, starts = _arr(data[4]).reshape(-1).astype(int), _arr(data[5]).reshape(-1).astype(int)
            for j in range(self.n_links):
                for c in range(starts[j], starts[j] + counts[j]):
                    along = float((points[c] - self.origin - tip) @ axis)
                    if along < s.hot_end and forces[c] > 0.05:
                        self.burned = True
                        self.say(f"BURNED: the iron's hot part ({along * 1000:.0f} mm from the tip) touched "
                                 f"{self.link_names[j]}")
                        break
                if self.burned:
                    break
        held = None
        for arm in ARMS:
            if all(link_n[i] > 0.3 for i in self.tip_idx[arm]):
                held = arm
        if held != self.held:
            self.say(f"iron {'held by ' + held if held else 'let go'}")
        self.held = held
        stand = in_stand(s, tip, axis)
        self.loose_t = 0.0 if (held or stand) else self.loose_t + self.dt
        if self.loose_t >= s.drop_s and not self.dropped:
            self.dropped = True
            self.say(f"DROPPED: the iron is out of the stand and in no grip (tip at {np.round(tip, 3)})")
        # the bracket: melt mark
        if bracket_n > law.mark_n:
            self.mark_t += self.dt
            if self.mark_t >= law.mark_s:
                d = [float(np.linalg.norm(tip[:2] - s.hole_xy(i))) for i in range(N_INSERTS)]
                i = int(np.argmin(d))
                if not self.st[i].marked:
                    self.st[i].marked = True
                    self.say(f"MARK: the hot iron pressed on the bracket by insert {i} for {self.mark_t:.2f} s")
        # the inserts
        moved = False
        for i, st in enumerate(self.st):
            push = max(0.0, -float(fins[i, 0, 2]))
            flat = max(0.0, -float(fins[i, 1, 2]))
            self.push_peak = max(self.push_peak, push)
            self.flat_peak = max(self.flat_peak, flat)
            off = float(np.linalg.norm(tip[:2] - s.hole_xy(i)))
            shoulder_z = tip[2] + axis[2] * s.pin[1]
            engaged = off <= law.engage_m and abs(shoulder_z - (s.face_z + st.proud)) < 0.002
            was = (st.crooked, st.forced, st.heat >= law.heat_s, st.lip)
            dz = st.step(law, push, engaged, tilt, axis, self.dt, flat, f_tilt, f_axis)
            moved |= dz > 0 or st.crooked != was[0]
            if st.crooked and not was[0]:
                self.say(f"CROOKED: insert {i} sank leaning (iron {tilt:.1f} deg, block {f_tilt:.1f} deg)")
            if st.forced and not was[1]:
                self.say(f"FORCED: insert {i} pushed at {push:.1f} N (> {law.f_max} N) while sinking")
            if was[3] and not st.lip:
                self.say(f"insert {i} pressed flat with the block ({flat:.1f} N, block leaning {f_tilt:.1f} deg, "
                         f"{st.off:.2f} s after the iron left, {st.proud * 1000:+.2f} mm)")
            if st.heat >= law.heat_s and not was[2]:
                self.say(f"insert {i} hot (push {push:.1f} N, pin {off * 1000:.2f} mm off its axis)")
        if moved:
            self._place_inserts()

    def state(self) -> InsertState:
        tip, axis = self._iron()
        v = self.iron.vel()
        st = InsertState(pos=tip, quat_xyzw=self.iron.pose()[3:7], lin_vel=v[:3], ang_vel=v[3:],
                         low=np.minimum(tip, tip + axis * 0.2) - 0.01, high=np.maximum(tip, tip + axis * 0.2) + 0.01,
                         axis=axis, inserts=tuple(Insert(**x.__dict__) for x in self.st), held=self.held,
                         in_stand=in_stand(self.spec, tip, axis), burned=self.burned, dropped=self.dropped,
                         push_n=self.push_peak, bracket_n=self.bracket_peak)
        if self.debug and self.n % 24 == 0:
            g = _arr(self.base.scene["right_robot"].data.body_link_pose_w)[0][self.g_id]
            Rg = quat_to_matrix(g[3:7])
            tcp = g[:3] - self.origin + Rg[:, 2] * 0.134
            rel = Rg.T @ (tip - tcp)
            print(f"[insertkit] tcp {np.round(tcp, 4)} approach {np.round(Rg[:, 2], 3)} tip in tcp frame "
                  f"{np.round(rel, 4)} iron axis in tcp frame {np.round(Rg.T @ axis, 3)}", flush=True)
            print(f"[insertkit] t {self.n * self.dt:6.2f} tip {np.round(tip, 4)} tilt {tilt_deg(axis):4.1f} push "
                  f"{self.push_peak:5.2f} N flat {self.flat_peak:5.2f} N bracket {self.bracket_peak:4.2f} N "
                  f"held {self.held} stand {st.in_stand} proud {[round(x.proud * 1000, 2) for x in self.st]} "
                  f"lip {[int(x.lip) for x in self.st]} heat {[round(x.heat, 1) for x in self.st]}",
                  flush=True)
        self.push_peak = self.bracket_peak = self.flat_peak = 0.0
        return st


_RT: dict[str, _Runtime] = {}


def _runtime(base, name: str) -> _Runtime:
    if name not in _RT:
        _RT[name] = _Runtime(base, _SPECS[name])
    return _RT[name]


# -- hooks (as in other task kits) ----------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_insertkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, InsertRig):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _runtime(environment.unwrapped, n).state() for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._insertkit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_insertkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for name in _SPECS:
            _runtime(base, name).physics_step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._insertkit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_insertkit", False):
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

    cls._ensure_sim, cls.reset, cls._insertkit = _ensure_sim, _reset, True


def _patch_scripted(module) -> None:
    """``yam_scripted`` plays the task file's closed-loop ``controller`` when it has one."""
    cls = module.YamScriptedPolicy
    if getattr(cls, "_insertkit", False):
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

    cls.reset, cls.act, cls._insertkit = _reset, _act, True


def _patch_demo(demo) -> None:
    if getattr(demo, "_insertkit", False):
        return
    grip_rotation = demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else grip_rotation(yaw)

    demo.grip_rotation, demo._insertkit = _grip_rotation, True


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
