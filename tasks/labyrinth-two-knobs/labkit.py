"""labkit: a BRIO tilting labyrinth on two knobs and its steel ball, for a yam_arena task file.

yam_arena task files only take single primitives. This module adds two objects and hooks yam_arena (without
modifying it):

- :class:`Labyrinth`: a FLOATING-base articulation (it stands loose on its riser). Root ``box`` (the wooden box);
  ``frame`` on revolute ``tilt_x`` (about the box's x); ``board`` on revolute ``tilt_y`` (about the frame's y), with
  its rim, the partitions and the printed path and holes; ``knob_x`` (on the +x end face) and ``knob_y``
  (on the -y long face) on their own revolute shafts.
- :class:`SteelBall`: one dynamic sphere.

Before every physics step (240 Hz, via a wrapper on ``InteractiveScene.write_data_to_sim``) the kit:

1. reads each knob's angle and sets its tilt joint's position target through the gearing (``labmodel``): the tilt
   joints are stiff position drives, as the strings and springs hold the real board;
2. applies the knob's turning resistance (stick-slip friction) as a joint effort;
3. finds the ball on the board; if its centre is over a hole it has fallen in: the ball is moved to the box's floor
   under that hole (the holed board is not simulated as a collider) and the trial's progress stops there;
4. latches the fouls: a gripper finger or tool point inside the box (below its rim + 1 cm) or within 1 cm of the
   ball; the box lifted 1 cm, tipped 5 degrees or slid 5 cm.

``object_states`` reports a :class:`LabState`. Importing the module also hooks ``yam_arena.scripted`` so that a task
file's ``controller(bases, dt)`` is played by ``yam_scripted``.
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
import labmodel as M  # noqa: E402

ARMS = ("left_robot", "right_robot")
TCP_OFFSET = 0.134
FINGER_OPEN = -0.04695
JOINTS = ("tilt_x", "tilt_y", "knob_x", "knob_y")
WOOD, WOOD_DARK, BOARD_C = (0.80, 0.62, 0.40), (0.62, 0.43, 0.24), (0.93, 0.85, 0.66)
INK, HOLE_C, KNOB_C, STEEL = (0.12, 0.10, 0.09), (0.03, 0.03, 0.03), (0.55, 0.30, 0.16), (0.78, 0.80, 0.83)
DEBUG = bool(os.environ.get("LAB_DEBUG"))


@dataclass(frozen=True)
class Labyrinth(Box):
    """``pos``: the box's underside centre (world); ``yaw`` its turn about the vertical. Fixed spec fields."""

    box_mass: float = 0.85
    friction: float = 0.5  # wood

    def to_world(self, p_box) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x, y, z = p_box
        return np.array([self.pos[0] + c * x - s * y, self.pos[1] + s * x + c * y, self.pos[2] + z])


@dataclass(frozen=True)
class SteelBall(Box):
    """A fixed (BASE) asset carrying one dynamic ball body, starting at ``start`` (world)."""

    start: tuple = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class LabState(ObjectState):
    ball_uv: np.ndarray = field(default_factory=lambda: np.zeros(2))  # board frame
    ball_vel_uv: np.ndarray = field(default_factory=lambda: np.zeros(2))
    knob_deg: dict = field(default_factory=dict)  # "x", "y": about each knob's outward axis
    tilt_deg: dict = field(default_factory=dict)
    passed: int = 0
    fell: int | None = None
    fouls: tuple = ()  # latched foul names


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

    def cyl(path, centre, radius, height, axis, color, mat=None):
        c = UsdGeom.Cylinder.Define(stage, path)
        c.CreateAxisAttr(axis)
        c.CreateRadiusAttr(radius)
        c.CreateHeightAttr(height)
        c.AddTranslateOp().Set(Gf.Vec3d(*centre))
        if mat:
            collide(c.GetPrim(), mat)
        bind_visual_material(str(c.GetPath()), look(color), stage=stage)
        return c

    return material, look, collide, cube, cyl


def _spawn_lab(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    s: Labyrinth = _SPECS[cfg.rig_name]
    _PATHS[s.name] = prim_path
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=dbl).Set(Gf.Quatd(w, x, y, z))
    material, look, collide, cube, cyl = _tools(prim_path, stage)
    wood = material("wood", s.friction, 0.1)

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

    L, W, H = M.BOX
    t = M.WALL_T
    box = body("box", (0.0, 0.0, 0.0), s.box_mass)
    cube(f"{box}/floor", (0, 0, M.FLOOR_T / 2), (L, W, M.FLOOR_T), WOOD_DARK, wood)
    for i, (c, sz) in enumerate((((0, W / 2 - t / 2, H / 2), (L, t, H)), ((0, -W / 2 + t / 2, H / 2), (L, t, H)),
                                 ((L / 2 - t / 2, 0, H / 2), (t, W - 2 * t, H)),
                                 ((-L / 2 + t / 2, 0, H / 2), (t, W - 2 * t, H)))):
        cube(f"{box}/wall_{i}", c, sz, WOOD, wood)

    pivot = (0.0, 0.0, M.BOARD_TOP - M.BOARD[2] / 2)
    frame = body("frame", pivot, 0.08)
    fu, fv = M.BOARD[0] / 2 + M.RIM_T + 0.004, M.BOARD[1] / 2 + M.RIM_T + 0.004  # the frame's bars, outside the rim
    for i, (c, sz) in enumerate((((0, fv, -0.012), (2 * fu, 0.004, 0.006)), ((0, -fv, -0.012), (2 * fu, 0.004, 0.006)),
                                 ((fu, 0, -0.012), (0.004, 2 * fv, 0.006)), ((-fu, 0, -0.012), (0.004, 2 * fv, 0.006)))):
        cube(f"{frame}/bar_{i}", c, sz, WOOD_DARK, wood)

    board = body("board", pivot, 0.30)
    top = M.BOARD[2] / 2  # board top surface in the board body's frame
    cube(f"{board}/plate", (0, 0, 0), M.BOARD, BOARD_C, wood, rough=0.8)
    bu, bv = M.BOARD[0] / 2, M.BOARD[1] / 2
    rz, rt = top + M.RIM_H / 2, M.RIM_T
    for i, (c, sz) in enumerate((((0, bv + rt / 2, rz), (2 * bu + 2 * rt, rt, M.RIM_H)),
                                 ((0, -bv - rt / 2, rz), (2 * bu + 2 * rt, rt, M.RIM_H)),
                                 ((bu + rt / 2, 0, rz), (rt, 2 * bv, M.RIM_H)),
                                 ((-bu - rt / 2, 0, rz), (rt, 2 * bv, M.RIM_H)))):
        cube(f"{board}/rim_{i}", c, sz, WOOD, wood)
    for i, (u0, u1, v0, v1) in enumerate(M.WALLS):
        cube(f"{board}/wall_{i}", ((u0 + u1) / 2, (v0 + v1) / 2, top + M.WALL_H / 2), (u1 - u0, v1 - v0, M.WALL_H),
             INK, wood)
    # printed on the board (visual only): the path line as 3 mm dots every 4 mm, the holes, START and FINISH
    for i, s in enumerate(np.arange(0.0, M.PATH.length, 0.004)):
        u, v = M.PATH.point(s)
        cube(f"{board}/line_{i}", (u, v, top + 0.0002), (0.003, 0.003, 0.0004), (0.35, 0.33, 0.31))
    for k, (u, v) in enumerate(M.HOLES):
        cyl(f"{board}/hole_{k}", (u, v, top + 0.0003), M.HOLE_R, 0.0006, "Z", HOLE_C)
    cube(f"{board}/start", (*(M.START + (0.010, 0.0)), top + 0.0003), (0.006, 0.012, 0.0006), (0.10, 0.55, 0.20))
    cube(f"{board}/finish", (*(M.FINISH - (0.010, 0.0)), top + 0.0003), (0.006, 0.012, 0.0006), (0.80, 0.12, 0.10))

    knob_mat = material("knob", 1.0)
    knobs = {}
    for name, pos, axis in (("knob_x", (L / 2, 0.0, M.KNOB_Z), "X"), ("knob_y", (0.0, -W / 2, M.KNOB_Z), "Y")):
        kb = body(f"{name}_body", pos, 0.02)
        out = 1.0 if axis == "X" else -1.0  # outward along the joint axis
        c = M.KNOB_GAP + M.KNOB_LEN / 2
        cen = (out * c, 0, 0) if axis == "X" else (0, out * c, 0)
        shaft = (out * M.KNOB_GAP / 2, 0, 0) if axis == "X" else (0, out * M.KNOB_GAP / 2, 0)
        cyl(f"{kb}/knob", cen, M.KNOB_R, M.KNOB_LEN, axis, KNOB_C, knob_mat)
        cyl(f"{kb}/shaft", shaft, 0.004, M.KNOB_GAP, axis, STEEL)
        e = M.KNOB_GAP + M.KNOB_LEN + 0.0003  # a white line across the knob's end, to show its turn
        cube(f"{kb}/mark", (out * e, 0, 0.008) if axis == "X" else (0, out * e, 0.008),
             (0.0008, 0.004, 0.016) if axis == "X" else (0.004, 0.0008, 0.016), (0.95, 0.95, 0.92))
        knobs[name] = (kb, pos, axis)

    def joint(name, parent, child, pos, axis, limit_deg):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/{name}")
        j.CreateAxisAttr(axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(parent)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*pos))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(-limit_deg)
        j.CreateUpperLimitAttr(limit_deg)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(0.0)

    tmax = math.degrees(M.TILT_MAX) + 0.5
    joint("tilt_x", box, frame, pivot, "X", tmax)
    joint("tilt_y", frame, board, (0.0, 0.0, 0.0), "Y", tmax)
    for name, (kb, pos, axis) in knobs.items():
        joint(name, box, kb, pos, axis, math.degrees(M.KNOB_LIMIT))

    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(box))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(box))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _spawn_ball(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    s: SteelBall = _SPECS[cfg.rig_name]
    _PATHS[s.name] = prim_path
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    material, look, collide, cube, cyl = _tools(prim_path, stage)
    steel = material("steel", 0.3, 0.3)
    xf = UsdGeom.Xform.Define(stage, f"{prim_path}/ball")
    xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*s.start))
    prim = xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(M.BALL_MASS)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(0.5)
    rb.CreateSolverPositionIterationCountAttr(16)
    rb.CreateAngularDampingAttr(0.15)  # rolling resistance of the steel ball on the wood
    rb.CreateLinearDampingAttr(0.02)
    rb.CreateEnableCCDAttr(True)
    sph = UsdGeom.Sphere.Define(stage, f"{prim_path}/ball/shape")
    sph.CreateRadiusAttr(M.BALL_R)
    collide(sph.GetPrim(), steel, offset=0.001)
    from isaaclab.sim.utils import bind_visual_material

    bind_visual_material(str(sph.GetPath()), look((0.36, 0.38, 0.42), 0.3, 0.5), stage=stage)
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

    if isinstance(spec, SteelBall):
        @configclass
        class _BallCfg(SpawnerCfg):
            func: object = clone(_spawn_ball)
            rig_name: str = ""

        return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=_BallCfg(rig_name=spec.name),
                      initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))

    @configclass
    class _LabCfg(SpawnerCfg):
        func: object = clone(_spawn_lab)
        rig_name: str = ""

    class LabObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {
                "tilt": ImplicitActuatorCfg(joint_names_expr=["tilt_x", "tilt_y"], stiffness=400.0, damping=8.0,
                                            armature=0.002),
                "knobs": ImplicitActuatorCfg(joint_names_expr=["knob_x", "knob_y"], stiffness=0.0, damping=0.004,
                                             armature=0.0005),
            }
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return LabObject(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=_LabCfg(rig_name=spec.name),
                     initial_pose=Pose(position_xyz=tuple(spec.pos),
                                       rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))


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
    ball_uv: np.ndarray = field(default_factory=lambda: np.zeros(2))
    ball_vel_uv: np.ndarray = field(default_factory=lambda: np.zeros(2))
    knob: dict = field(default_factory=lambda: {"x": 0.0, "y": 0.0})  # rad, outward-axis sense
    tilt: dict = field(default_factory=lambda: {"x": 0.0, "y": 0.0})  # rad
    fell: int | None = None
    passed: int = 0
    box_pose: tuple = ()


SNAP = Snapshot()


class _Runtime:
    def __init__(self, base, lab: Labyrinth, ball: SteelBall):
        from isaaclab_physx.physics import PhysxManager

        self.base, self.lab, self.ball = base, lab, ball
        self.asset = base.scene[lab.name]
        names = list(self.asset.joint_names)
        self.jid = {n: names.index(n) for n in JOINTS}
        self.board_id = self.asset.find_bodies("board")[0][0]
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
        self.anchor = {"knob_x": 0.0, "knob_y": 0.0}
        self.fouls: set[str] = set()
        self.n = 0
        self.box0 = None
        self.worst = {"lift_mm": 0.0, "tilt_deg": 0.0, "slide_mm": 0.0}

    def reset_ball(self):
        cur = self.bview.get_transforms()
        idx = _like(cur, np.array([0], np.int32), "int32")
        self.bview.set_transforms(_like(cur, self.start), idx)
        self.bview.set_velocities(_like(cur, np.zeros((1, 6))), idx)

    # one physics step
    def step(self):
        import torch

        self.n += 1
        q = _arr(self.asset.data.joint_pos)[0]
        knob = {"x": float(q[self.jid["knob_x"]]), "y": -float(q[self.jid["knob_y"]])}  # outward-axis sense
        tx, ty = M.tilt_from_knob(knob["x"]), M.tilt_from_knob(knob["y"])
        dev = self.base.device
        self.asset.actuators.target_command.set_position_index(
            value=torch.tensor([[tx, ty]], dtype=torch.float32, device=dev),
            joint_ids=[self.jid["tilt_x"], self.jid["tilt_y"]])
        eff = []
        for j in ("knob_x", "knob_y"):
            f, self.anchor[j] = _stick(self.anchor[j], float(q[self.jid[j]]), 2.0, M.KNOB_FRICTION_NM)
            eff.append(f)
        self.asset.actuators.target_command.set_effort_index(
            value=torch.tensor([eff], dtype=torch.float32, device=dev),
            joint_ids=[self.jid["knob_x"], self.jid["knob_y"]])

        bp = _arr(self.asset.data.body_link_pose_w)[0, self.board_id]
        Rb = quat_to_matrix(bp[3:7])
        top = bp[:3] + Rb[:, 2] * (M.BOARD[2] / 2)  # board's top-surface centre (world)
        tr = _arr(self.bview.get_transforms())[0]
        vel = _arr(self.bview.get_velocities())[0]
        rel = Rb.T @ (tr[:3] - top)
        on_board = -0.002 < rel[2] < 0.03 and abs(rel[0]) < M.BOARD[0] / 2 and abs(rel[1]) < M.BOARD[1] / 2
        if on_board:
            self.progress.step(rel[:2])
            if self.progress.fell is not None:
                self._drop(self.progress.fell, Rb, top)
        root = _arr(self.asset.data.root_link_pose_w)[0]
        self._fouls(root, tr[:3])
        SNAP.t = self.n * self.dt
        SNAP.ball_uv, SNAP.ball_vel_uv = rel[:2].copy(), (Rb.T @ vel[:3])[:2]
        SNAP.knob, SNAP.tilt = knob, {"x": float(q[self.jid["tilt_x"]]), "y": float(q[self.jid["tilt_y"]])}
        SNAP.passed = self.progress.passed
        SNAP.fell, SNAP.box_pose = self.progress.fell, (root[:3] - self.origin, root[3:7])
        if DEBUG and self.n % 48 == 0:
            print(f"[labkit] t {SNAP.t:6.2f} ball uv {np.round(rel[:2] * 1000, 1)} mm v {np.round(SNAP.ball_vel_uv, 3)} "
                  f"knob x {math.degrees(knob['x']):6.1f} y {math.degrees(knob['y']):6.1f} tilt x "
                  f"{math.degrees(SNAP.tilt['x']):5.2f} y {math.degrees(SNAP.tilt['y']):5.2f} passed "
                  f"{self.progress.passed} fell {self.progress.fell} fouls {sorted(self.fouls)}", flush=True)

    def _drop(self, k: int, Rb, top):
        """The ball has gone down hole ``k``: put it on the box floor under the hole, still."""
        if getattr(self, "_dropped", None) == k:
            return
        self._dropped = k
        u, v = M.HOLES[k - 1]
        p = top + Rb @ np.array([u, v, 0.0])
        p[2] = self.lab.pos[2] + self.origin[2] + M.FLOOR_T + M.BALL_R + 0.001
        cur = self.bview.get_transforms()
        data = _arr(cur).copy()
        data[0, :3] = p
        idx = _like(cur, np.array([0], np.int32), "int32")
        self.bview.set_transforms(_like(cur, data), idx)
        self.bview.set_velocities(_like(cur, np.zeros((1, 6))), idx)
        print(f"[labkit] ball fell into hole {k} at {self.n * self.dt:.2f} s", flush=True)

    def _fouls(self, root, ball_w):
        pos = root[:3] - self.origin
        if self.box0 is None:
            self.box0 = pos.copy()
        R = quat_to_matrix(root[3:7])
        lift = (pos[2] - self.box0[2]) * 1000
        tilt = math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1))))
        slide = float(np.linalg.norm(pos[:2] - self.box0[:2])) * 1000
        for k, v in (("lift_mm", lift), ("tilt_deg", tilt), ("slide_mm", slide)):
            self.worst[k] = max(self.worst[k], v)
        if lift > 10:
            self.fouls.add("box lifted")
        if tilt > 5:
            self.fouls.add("box tipped")
        if slide > 50:
            self.fouls.add("box slid")
        L, W, H = M.BOX
        for arm in ARMS:
            poses = _arr(self.base.scene[arm].data.body_link_pose_w)[0]
            ids = self.pad_ids[arm]
            g = poses[ids[2]]
            pts = [poses[i, :3] for i in ids[:2]] + [g[:3] + quat_to_matrix(g[3:7])[:, 2] * TCP_OFFSET]
            for p in pts:
                b = R.T @ (p - root[:3])
                if abs(b[0]) < L / 2 - M.WALL_T and abs(b[1]) < W / 2 - M.WALL_T and b[2] < H + 0.01:
                    self.fouls.add(f"{arm.split('_')[0]} hand in the box")
                if np.linalg.norm(p - ball_w) < M.BALL_R + 0.01:
                    self.fouls.add(f"{arm.split('_')[0]} hand at the ball")

    def state(self) -> LabState:
        root = _arr(self.asset.data.root_link_pose_w)[0]
        pos = root[:3] - self.origin
        half = np.asarray(M.BOX) / 2
        return LabState(pos=pos, quat_xyzw=root[3:7], lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                        low=pos - half * (1, 1, 0), high=pos + half * (1, 1, 2), ball_uv=SNAP.ball_uv.copy(),
                        ball_vel_uv=SNAP.ball_vel_uv.copy(),
                        knob_deg={k: math.degrees(v) for k, v in SNAP.knob.items()},
                        tilt_deg={k: math.degrees(v) for k, v in SNAP.tilt.items()}, passed=self.progress.passed,
                        fell=self.progress.fell, fouls=tuple(sorted(self.fouls)))


_RT: dict[str, _Runtime] = {}


def _runtime(base) -> _Runtime | None:
    if "rt" not in _RT:
        labs = [s for s in _SPECS.values() if isinstance(s, Labyrinth)]
        balls = [s for s in _SPECS.values() if isinstance(s, SteelBall)]
        if not labs or not balls:
            return None
        _RT["rt"] = _Runtime(base, labs[0], balls[0])
    return _RT["rt"]


# -- hooks ----------------------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_labkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, (Labyrinth, SteelBall)):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        rt = _runtime(environment.unwrapped)
        for n in names:
            if n in _SPECS:
                st = rt.state()
                out[n] = st
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._labkit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_labkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        rt = _runtime(base)
        if rt is not None:
            rt.step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._labkit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_labkit", False):
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

    cls._ensure_sim, cls.reset, cls._labkit = _ensure_sim, _reset, True


def _patch_scripted(module) -> None:
    """``yam_scripted`` plays the task file's closed-loop ``controller`` when it has one."""
    cls = module.YamScriptedPolicy
    if getattr(cls, "_labkit", False):
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

    cls.reset, cls.act, cls._labkit = _reset, _act, True


def _patch_demo(demo) -> None:
    if getattr(demo, "_labkit", False):
        return
    grip_rotation = demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else grip_rotation(yaw)

    demo.grip_rotation, demo._labkit = _grip_rotation, True


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
