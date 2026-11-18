"""tfkit: a tabletop football (foosball) table and its ball, for a yam_arena task file.

yam_arena task files only take single primitives. This module adds two objects and hooks yam_arena (without
modifying it):

- :class:`FootballTable`: a FIXED-base articulation (the table is clamped to the bench). Root ``cabinet``: the
  playfield, walls, goals, and the six rods the robot does not play (figures hanging straight down: the robot team's
  goalie and 2-bar, and the opponent's four rods, clamped). The robot team's 5-bar and 3-bar each sit on a prismatic
  joint along x (``slide5``, ``slide3``; the rod sliding in its bushings) carrying a revolute joint about x
  (``spin5``, ``spin3``) with the rod, its figures and its rubber handle.
- :class:`Ball`: one dynamic sphere.

Before every physics step (240 Hz, via a wrapper on ``InteractiveScene.write_data_to_sim``) the kit applies each
rod's bushing friction (stick-slip, as joint efforts), reads the rods and the ball, and steps the trial's latched
progress and fouls (``tfmodel.Progress``). ``object_states`` reports a :class:`TableState`. Importing the module also
hooks ``yam_arena.scripted`` so that a task file's ``controller(bases, dt)`` is played by ``yam_scripted``.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import pathlib
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import tfmodel as M  # noqa: E402

ARMS = ("left_robot", "right_robot")
TCP_OFFSET = 0.134
JOINTS = ("slide5", "spin5", "slide3", "spin3")
ROD_OF = {4: ("slide5", "spin5"), 6: ("slide3", "spin3")}
GREEN, LINE, MDF, MDF_DARK = (0.10, 0.46, 0.20), (0.95, 0.95, 0.93), (0.20, 0.20, 0.22), (0.12, 0.12, 0.13)
RED, BLUE, STEEL, RUBBER, WHITE = (0.80, 0.10, 0.08), (0.10, 0.25, 0.75), (0.75, 0.77, 0.80), (0.08, 0.08, 0.08), (
    0.96, 0.96, 0.94)
SLIDE_FRICTION_N = 1.5  # bushing friction along the rod (measure: spring scale on the handle)
SPIN_FRICTION_NM = 0.004
DEBUG = bool(os.environ.get("TF_DEBUG"))


@dataclass(frozen=True)
class FootballTable(Box):
    """Geometry comes from tfmodel; ``pos`` and ``size`` only describe its bounding box."""


@dataclass(frozen=True)
class Ball(Box):
    """A fixed (BASE) asset carrying one dynamic ball body, starting at ``start`` (world)."""

    start: tuple = M.BALL_START


@dataclass(frozen=True)
class TableState(ObjectState):
    ball: np.ndarray = field(default_factory=lambda: np.zeros(3))
    ball_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rods: dict = field(default_factory=dict)  # {4: (slide m, theta rad), 6: ...}
    stage: str = "none"
    goal: bool = False
    fouls: tuple = ()
    score: float = 0.0


# -- spawning -------------------------------------------------------------------------------------------------------

_SPECS: dict[str, object] = {}
_PATHS: dict[str, str] = {}


def _tools(prim_path, stage):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    looks: dict = {}

    def material(name, mu, restitution=0.0) -> str:
        path = f"{prim_path}/Looks/{name}"
        cfg = sim_utils.RigidBodyMaterialCfg(static_friction=mu, dynamic_friction=mu, restitution=restitution)
        cfg.func(path, cfg)
        return path

    def look(color, rough=0.6, metal=0.0) -> str:
        key = (tuple(round(c, 4) for c in color), rough, metal)
        if key not in looks:
            looks[key] = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal)
            vis.func(looks[key], vis)
        return looks[key]

    def collide(prim, mat, offset=0.002):
        UsdPhysics.CollisionAPI.Apply(prim)
        col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        col.CreateContactOffsetAttr(offset)
        col.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), mat, stage=stage)

    def cube(path, centre, size, color, mat=None, rough=0.6):
        c = UsdGeom.Cube.Define(stage, path)
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*centre))
        c.AddScaleOp().Set(Gf.Vec3f(*size))
        if mat:
            collide(c.GetPrim(), mat)
        bind_visual_material(str(c.GetPath()), look(color, rough), stage=stage)
        return c

    def cyl(path, centre, radius, height, axis, color, mat=None, rough=0.6, metal=0.0):
        c = UsdGeom.Cylinder.Define(stage, path)
        c.CreateAxisAttr(axis)
        c.CreateRadiusAttr(radius)
        c.CreateHeightAttr(height)
        c.AddTranslateOp().Set(Gf.Vec3d(*centre))
        if mat:
            collide(c.GetPrim(), mat)
        bind_visual_material(str(c.GetPath()), look(color, rough, metal), stage=stage)
        return c

    return material, look, collide, cube, cyl


def _box(a, b):
    """(centre, size) of the box between corners a and b."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    return tuple((a + b) / 2), tuple(np.abs(b - a))


def _spawn_table(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    _PATHS[cfg.rig_name] = prim_path
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    root.AddOrientOp(precision=dbl).Set(Gf.Quatd(1, 0, 0, 0))
    material, look, collide, cube, cyl = _tools(prim_path, stage)
    laminate = material("laminate", 0.3, 0.3)
    wood = material("mdf", 0.4, 0.4)
    plastic = material("figure", 0.4, 0.45)
    rubber = material("rubber", 1.2, 0.0)

    def body(name, pos, mass) -> str:
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*pos))
        xf.AddOrientOp(precision=dbl).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        return path

    T, x0, x1 = M.WALL_T, M.X_NEAR, M.X_FAR
    y0, y1 = M.Y_GOAL, M.Y_OWN
    cab = body("cabinet", (0.0, 0.0, 0.0), 6.8)
    n = 0

    def slab(a, b, color, mat=None, rough=0.6):
        nonlocal n
        c, s = _box(a, b)
        cube(f"{cab}/part_{n}", c, s, color, mat, rough)
        n += 1

    base_z = M.TOP + 0.020
    for fx in (x0 - T + 0.03, x1 + T - 0.03):  # four feet
        for fy in (y0 - T + 0.03, y1 + T - 0.03):
            slab((fx - 0.02, fy - 0.02, M.TOP), (fx + 0.02, fy + 0.02, base_z), RUBBER, wood)
    slab((x0 - T, y0 - T - M.POCKET - T, base_z), (x1 + T, y1 + T + M.POCKET + T, M.FLOOR_Z - 0.001), MDF_DARK, wood)
    slab((x0, y0 - M.POCKET, M.FLOOR_Z - 0.001), (x1, y1 + M.POCKET, M.FLOOR_Z), GREEN, laminate, 0.8)
    # painted lines (visual): centre line, centre circle as a square ring of strips, goal areas
    for (a, b) in (((x0, M.Y_C - 0.002), (x1, M.Y_C + 0.002)),):
        slab((a[0], a[1], M.FLOOR_Z), (b[0], b[1], M.FLOOR_Z + 0.0004), LINE)
    for i in range(24):
        a = 2 * math.pi * i / 24
        cx, cy = M.X_C + 0.07 * math.cos(a), M.Y_C + 0.07 * math.sin(a)
        slab((cx - 0.0025, cy - 0.0025, M.FLOOR_Z), (cx + 0.0025, cy + 0.0025, M.FLOOR_Z + 0.0004), LINE)
    for ys, sgn in ((y0, 1), (y1, -1)):
        slab((M.X_C - 0.12, ys, M.FLOOR_Z), (M.X_C - 0.116, ys + sgn * 0.08, M.FLOOR_Z + 0.0004), LINE)
        slab((M.X_C + 0.116, ys, M.FLOOR_Z), (M.X_C + 0.12, ys + sgn * 0.08, M.FLOOR_Z + 0.0004), LINE)
        slab((M.X_C - 0.12, ys + sgn * 0.078, M.FLOOR_Z), (M.X_C + 0.12, ys + sgn * 0.082, M.FLOOR_Z + 0.0004), LINE)
    # the ball's start spot
    bx, by, _ = M.BALL_START
    slab((bx - 0.004, by - 0.004, M.FLOOR_Z), (bx + 0.004, by + 0.004, M.FLOOR_Z + 0.0005), LINE)
    # side walls (near: the handle side) and end walls with the goal mouths
    slab((x0 - T, y0 - T, M.FLOOR_Z), (x0, y1 + T, M.WALL_TOP), MDF, wood)
    slab((x1, y0 - T, M.FLOOR_Z), (x1 + T, y1 + T, M.WALL_TOP), MDF, wood)
    gx0, gx1, gz = M.X_C - M.GOAL_W / 2, M.X_C + M.GOAL_W / 2, M.FLOOR_Z + M.GOAL_H
    for ya, yb, sgn in ((y0 - T, y0, -1), (y1, y1 + T, 1)):
        slab((x0, ya, M.FLOOR_Z), (gx0, yb, M.WALL_TOP), MDF, wood)
        slab((gx1, ya, M.FLOOR_Z), (x1, yb, M.WALL_TOP), MDF, wood)
        slab((gx0, ya, gz), (gx1, yb, M.WALL_TOP), MDF, wood)
        # goal pocket behind the mouth: back, sides, roof
        yo = yb if sgn > 0 else ya
        yp = yo + sgn * M.POCKET
        slab((gx0 - T, min(yo, yp), M.FLOOR_Z), (gx0, max(yo, yp), gz + T), MDF_DARK, wood)
        slab((gx1, min(yo, yp), M.FLOOR_Z), (gx1 + T, max(yo, yp), gz + T), MDF_DARK, wood)
        slab((gx0 - T, min(yp, yp + sgn * T), M.FLOOR_Z), (gx1 + T, max(yp, yp + sgn * T), gz + T), MDF_DARK, wood)
        slab((gx0 - T, min(yo, yp), gz), (gx1 + T, max(yo, yp), gz + T), MDF_DARK, wood)

    def figures(parent, k, color, mat, at=(0.0, 0.0, 0.0)):
        """Figures of rod k (boxes) in ``parent``'s frame, rod axis through ``at``."""
        for i, fx in enumerate(M.FIG_X[k]):
            for j, (c, h) in enumerate(M.FIGURE_BOXES):
                cube(f"{parent}/fig{k}_{i}_{j}", (at[0] + fx + c[0], at[1] + c[1], at[2] + c[2]),
                     tuple(2 * np.asarray(h)), color, mat)
            cube(f"{parent}/fig{k}_{i}_hair", (at[0] + fx, at[1], at[2] + 0.035), (0.016, 0.014, 0.004),
                 (0.10, 0.07, 0.05))

    far_handle = x1 + (x0 - M.HANDLE_X)
    for k in (1, 2, 3, 5, 7, 8):  # rods not played: clamped (theirs) or untouched (the robot's goalie and 2-bar)
        y = M.rod_y(k)
        ours = k in (1, 2)
        figures(cab, k, RED if ours else BLUE, plastic, (M.X_C, y, M.ROD_Z))
        hx = M.HANDLE_X if ours else far_handle
        xa, xb = (hx + M.HANDLE_L / 2, x1 + T) if ours else (x0 - T, hx - M.HANDLE_L / 2)
        cyl(f"{cab}/rod{k}", ((xa + xb) / 2, y, M.ROD_Z), M.ROD_R, xb - xa, "X", STEEL, rough=0.3, metal=0.8)
        cyl(f"{cab}/handle{k}", (hx, y, M.ROD_Z), M.HANDLE_R, M.HANDLE_L, "X", RUBBER)
        if not ours:  # the spring clamp holding the rod at its bushing (visual)
            cube(f"{cab}/clamp{k}", (x1 + T + 0.025, y, M.ROD_Z), (0.03, 0.035, 0.05), (0.85, 0.65, 0.05))

    rods = {}
    for k, (sl, sp) in ROD_OF.items():
        y = M.rod_y(k)
        slider = body(f"{sl}_body", (M.X_C, y, M.ROD_Z), 0.01)
        UsdPhysics.MassAPI(stage.GetPrimAtPath(slider)).CreateDiagonalInertiaAttr(Gf.Vec3f(1e-5, 1e-5, 1e-5))
        rod = body(f"{sp}_body", (M.X_C, y, M.ROD_Z), 0.10)
        hx = M.HANDLE_X - M.X_C
        xa, xb = hx + M.HANDLE_L / 2, x1 + T - M.X_C
        cyl(f"{rod}/rod", ((xa + xb) / 2, 0, 0), M.ROD_R, xb - xa, "X", STEEL, rough=0.3, metal=0.8)
        cyl(f"{rod}/handle", (hx, 0, 0), M.HANDLE_R, M.HANDLE_L, "X", RUBBER, rubber)
        cube(f"{rod}/handle_mark", (hx - M.HANDLE_L / 2 - 0.0003, 0, 0.008), (0.0008, 0.004, 0.012), WHITE)
        figures(rod, k, RED, plastic)
        rods[k] = (slider, rod)

    def joint(name, kind, parent, child, pos, lo, hi):
        j = kind.Define(stage, f"{prim_path}/{name}")
        j.CreateAxisAttr("X")
        j.CreateBody0Rel().SetTargets([Sdf.Path(parent)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*pos))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(lo)
        j.CreateUpperLimitAttr(hi)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear" if kind is UsdPhysics.PrismaticJoint else "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(0.0)

    for k, (sl, sp) in ROD_OF.items():
        slider, rod = rods[k]
        tr = M.TRAVEL[k]
        joint(sl, UsdPhysics.PrismaticJoint, cab, slider, (M.X_C, M.rod_y(k), M.ROD_Z), -tr, tr)
        joint(sp, UsdPhysics.RevoluteJoint, slider, rod, (0.0, 0.0, 0.0), -720.0, 720.0)

    anchor = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/fixed_to_world")
    anchor.CreateBody1Rel().SetTargets([Sdf.Path(cab)])
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    art = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _spawn_ball(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.utils import bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    s: Ball = _SPECS[cfg.rig_name]
    _PATHS[s.name] = prim_path
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    material, look, collide, cube, cyl = _tools(prim_path, stage)
    hard = material("ball", 0.3, 0.5)
    xf = UsdGeom.Xform.Define(stage, f"{prim_path}/ball")
    xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*s.start))
    prim = xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(M.BALL_MASS)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(0.5)
    rb.CreateSolverPositionIterationCountAttr(16)
    rb.CreateAngularDampingAttr(0.3)  # rolling resistance on the laminate
    rb.CreateLinearDampingAttr(0.05)
    rb.CreateEnableCCDAttr(True)
    sph = UsdGeom.Sphere.Define(stage, f"{prim_path}/ball/shape")
    sph.CreateRadiusAttr(M.BALL_R)
    collide(sph.GetPrim(), hard, offset=0.002)
    bind_visual_material(str(sph.GetPath()), look((0.97, 0.97, 0.95), 0.4), stage=stage)
    return root.GetPrim()


def _arena_object(spec):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    if isinstance(spec, Ball):
        @configclass
        class _BallCfg(SpawnerCfg):
            func: object = clone(_spawn_ball)
            rig_name: str = ""

        return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=_BallCfg(rig_name=spec.name),
                      initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))

    @configclass
    class _TableCfg(SpawnerCfg):
        func: object = clone(_spawn_table)
        rig_name: str = ""

    class TableObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {
                "slides": ImplicitActuatorCfg(joint_names_expr=["slide5", "slide3"], stiffness=0.0, damping=0.5,
                                              armature=0.002, effort_limit_sim=1e6),
                "spins": ImplicitActuatorCfg(joint_names_expr=["spin5", "spin3"], stiffness=0.0, damping=0.0005,
                                             armature=0.00005, effort_limit_sim=1e6),
            }
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            cfg = self._add_initial_pose_to_cfg(cfg)
            cfg.init_state.joint_pos = dict.fromkeys(JOINTS, 0.0)
            return cfg

    return TableObject(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=_TableCfg(rig_name=spec.name),
                       initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


# -- runtime --------------------------------------------------------------------------------------------------------


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

    return wp.array(data.astype(dtype), dtype=getattr(wp, dtype), device=getattr(ref, "device", None))


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _stick(anchor: float, x: float, stiffness: float, limit: float) -> tuple[float, float]:
    """Elasto-plastic friction: a stiff spring to an anchor that slips past ``limit``. (force, new anchor)."""
    f = stiffness * (anchor - x)
    if abs(f) > limit:
        f = math.copysign(limit, f)
        anchor = x + f / stiffness
    return f, anchor


@dataclass
class Snapshot:
    """The latest physics step, for the scripted demo (simulator state, which a scripted demo may read)."""

    t: float = 0.0
    ball: np.ndarray = field(default_factory=lambda: np.zeros(3))
    ball_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rods: dict = field(default_factory=lambda: {4: (0.0, 0.0), 6: (0.0, 0.0)})
    stage: str = "none"
    goal: bool = False


SNAP = Snapshot()


class _Runtime:
    def __init__(self, base, table: FootballTable, ball: Ball):
        from isaaclab_physx.physics import PhysxManager

        self.base, self.table, self.ball = base, table, ball
        self.asset = base.scene[table.name]
        names = list(self.asset.joint_names)
        self.jid = {n: names.index(n) for n in JOINTS}
        view = PhysxManager.get_physics_sim_view()
        self.bview = view.create_rigid_body_view([f"{_PATHS[ball.name]}/ball"])
        self.dt = float(base.physics_dt)
        self.origin = _arr(base.scene.env_origins)[0]
        self.start = _arr(self.bview.get_transforms()).copy()
        self.start[0, :3] = np.asarray(ball.start) + self.origin
        self.start[0, 3:7] = (0, 0, 0, 1)
        self.pad_ids = {arm: [base.scene[arm].find_bodies(b)[0][0] for b in ("tip_left", "tip_right", "gripper")]
                        for arm in ARMS}
        self.reset_latches()

    def reset_latches(self):
        self.progress = M.Progress()
        self.anchor = dict.fromkeys(JOINTS, 0.0)
        self.n = 0
        q = _arr(self.asset.data.joint_pos)[0]
        for j in JOINTS:
            self.anchor[j] = float(q[self.jid[j]])

    def reset_ball(self):
        cur = self.bview.get_transforms()
        idx = _like(cur, np.array([0], np.int32), "int32")
        self.bview.set_transforms(_like(cur, self.start), idx)
        self.bview.set_velocities(_like(cur, np.zeros((1, 6))), idx)

    def step(self):
        import torch

        self.n += 1
        q = _arr(self.asset.data.joint_pos)[0]
        eff = []
        for j in JOINTS:
            stiff, lim = (400.0, SLIDE_FRICTION_N) if j.startswith("slide") else (2.0, SPIN_FRICTION_NM)
            f, self.anchor[j] = _stick(self.anchor[j], float(q[self.jid[j]]), stiff, lim)
            eff.append(f)
        self.asset.actuators.target_command.set_effort_index(
            value=torch.tensor([eff], dtype=torch.float32, device=self.base.device),
            joint_ids=[self.jid[j] for j in JOINTS])
        rods = {k: (float(q[self.jid[sl]]), float(q[self.jid[sp]])) for k, (sl, sp) in ROD_OF.items()}
        tr = _arr(self.bview.get_transforms())[0]
        vel = _arr(self.bview.get_velocities())[0]
        ball = tr[:3] - self.origin
        gap, inside = self._hands(ball)
        self.progress.step(self.dt, ball, vel[:3], rods, gap, inside)
        SNAP.t = self.n * self.dt
        SNAP.ball, SNAP.ball_vel, SNAP.rods = ball.copy(), vel[:3].copy(), rods
        SNAP.stage, SNAP.goal = self.progress.stage, self.progress.goal
        if DEBUG and self.n % 24 == 0:
            p = self.progress
            print(f"[tfkit] t {SNAP.t:6.2f} ball {np.round(ball * 1000, 1)} v {np.round(vel[:3], 2)} "
                  f"rod5 {rods[4][0] * 1000:5.1f} mm {math.degrees(rods[4][1]):6.1f} deg rod3 {rods[6][0] * 1000:5.1f} "
                  f"mm {math.degrees(rods[6][1]):6.1f} deg stage {p.stage} last {p.last_own} still {p.still_t:.2f} "
                  f"fouls {sorted(p.fouls)} hand {gap * 1000:.0f} mm", flush=True)

    def _hands(self, ball):
        """(nearest finger/tool point's gap to the ball's surface, any of them inside the table's playfield)."""
        gap, inside = 1e9, False
        for arm in ARMS:
            poses = _arr(self.base.scene[arm].data.body_link_pose_w)[0]
            ids = self.pad_ids[arm]
            g = poses[ids[2]]
            pts = [poses[i, :3] for i in ids[:2]] + [g[:3] + quat_to_matrix(g[3:7])[:, 2] * TCP_OFFSET]
            for p in pts:
                p = p - self.origin
                gap = min(gap, float(np.linalg.norm(p - ball)) - M.BALL_R)
                if M.X_NEAR < p[0] < M.X_FAR and M.Y_GOAL < p[1] < M.Y_OWN and p[2] < M.WALL_TOP + 0.01:
                    inside = True
        return gap, inside

    def state(self) -> TableState:
        p = self.progress
        z = np.zeros(3)
        lo = np.array([M.X_NEAR - M.WALL_T, M.Y_GOAL - M.WALL_T, M.TOP])
        hi = np.array([M.X_FAR + M.WALL_T, M.Y_OWN + M.WALL_T, M.WALL_TOP])
        return TableState(pos=(lo + hi) / 2, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=z, ang_vel=z, low=lo, high=hi,
                          ball=SNAP.ball.copy(), ball_vel=SNAP.ball_vel.copy(), rods=dict(SNAP.rods), stage=p.stage,
                          goal=p.goal, fouls=tuple(sorted(p.fouls)), score=p.score())


_RT: dict[str, _Runtime] = {}


def _runtime(base) -> _Runtime | None:
    if "rt" not in _RT:
        tables = [s for s in _SPECS.values() if isinstance(s, FootballTable)]
        balls = [s for s in _SPECS.values() if isinstance(s, Ball)]
        if not tables or not balls:
            return None
        _RT["rt"] = _Runtime(base, tables[0], balls[0])
    return _RT["rt"]


# -- hooks ----------------------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_tfkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, (FootballTable, Ball)):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        rt = _runtime(environment.unwrapped)
        for n in names:
            if n in _SPECS:
                out[n] = rt.state()
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._tfkit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_tfkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        rt = _runtime(base)
        if rt is not None:
            rt.step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._tfkit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_tfkit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def _ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
        return out

    def _reset(self, scene, *, seed=None):
        self._ensure_sim(None)
        rt = _runtime(self._base)
        if rt is not None:
            rt.reset_ball()
        obs = reset(self, scene, seed=seed)
        if rt is not None:
            rt.reset_latches()  # the settle steps are not part of the trial
        return obs

    cls._ensure_sim, cls.reset, cls._tfkit = _ensure_sim, _reset, True


def _patch_scripted(module) -> None:
    """``yam_scripted`` plays the task file's closed-loop ``controller`` when it has one."""
    cls = module.YamScriptedPolicy
    if getattr(cls, "_tfkit", False):
        return
    reset, act = cls.reset, cls.act

    def _reset(self, scene):
        task = self._task
        make = getattr(task.module, "controller", None) if task is not None else None
        self._ctl = make(dict(self._bases), self._dt) if make is not None else None
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

    cls.reset, cls.act, cls._tfkit = _reset, _act, True


def _patch_demo(demo) -> None:
    if getattr(demo, "_tfkit", False):
        return
    grip_rotation = demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else grip_rotation(yaw)

    demo.grip_rotation, demo._tfkit = _grip_rotation, True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
            "yam_arena.scripted": _patch_scripted, "yam_arena.demo": _patch_demo}


class _Finder(importlib.abc.MetaPathFinder):
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
