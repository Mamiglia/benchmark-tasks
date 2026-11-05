"""boxkit: a loose toolbox (base, hinged lid, two front latches) as one articulation, for yam_arena task files.

yam_arena task files only take rigid primitives. This module adds one articulated object, :class:`ToolBox`, and
hooks yam_arena (without modifying it) so that:

- ``yam_arena.scene.task_object`` builds a ToolBox as a FLOATING-base articulation. The root is the base body
  (origin on the table under its centre). Its children: the lid on the ``lid_hinge`` joint (the base's rear top
  edge, axis y, 0 = shut, positive = open) and two latch levers on ``latch_l`` / ``latch_r`` (axis y, on the base's
  front face; joint 0 = open, leaning out against a stop, ``open_deg`` = closed, flat against the box).
- Before every PHYSICS step (240 Hz, via a wrapper on ``InteractiveScene.write_data_to_sim``), a :class:`Latches`
  turns the joint angles into joint efforts: a latch within ``snap_deg`` of closed is pulled shut by its
  over-centre spring; a latch that closes while the lid is shut ENGAGES, and while any latch is engaged a stiff
  spring holds the lid shut (the latch hook on the lid's keeper). That spring is the run-time lock.
- ``yam_arena.scene.object_states`` reports a :class:`ToolBoxState`: the base pose (its origin), the lid angle, how
  far each latch is from closed, and which latches are engaged.
- ``yam_arena.demo``: a ``Waypoint.yaw`` may be a 3x3 world rotation matrix (grips other than straight down).

Self-collision inside the articulation is off: the lid rests on the base through its joint's lower limit, and the
latches pass the lid's front without contact (their engagement is the lock above, not a contact hook).

The top half (``Latches``) is plain Python and is what the tests exercise. Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import Box, ObjectState

LID, LATCHES = "lid_hinge", ("latch_l", "latch_r")


# -- the mechanism (pure Python) --------------------------------------------------------------------------------


@dataclass(frozen=True)
class LatchParams:
    open_deg: float = 40.0  # latch travel from its open stop to closed
    snap_deg: float = 30.0  # within this of closed, the over-centre spring pulls the latch shut
    release_deg: float = 34.0  # an engaged latch pulled further open than this lets go of the lid
    hold_nm: float = 0.04  # over-centre spring torque holding a latch shut
    lid_shut_deg: float = 3.0  # a latch engages only if the lid is within this of shut when it closes
    lock_stiffness: float = 60.0  # N m/rad: latched lid held shut (0.5 deg of give under the loaded box)
    lock_damping: float = 0.5  # N m s/rad
    lid_damping: float = 0.002  # implicit joint damping (N m s/rad), always on: a free-swinging hinge
    lid_armature: float = 0.003  # kg m^2, keeps the 240 Hz explicit lock spring stable
    latch_damping: float = 0.002
    latch_armature: float = 2e-5


class Latches:
    """``step(lid_deg, lid_vel, latch_closed_deg)`` -> (lid torque, [latch torques], both in the joints' positive
    sense). ``latch_closed_deg[i]``: how far latch i is from closed (0 = closed, open_deg = on its open stop)."""

    def __init__(self, params: LatchParams = LatchParams()):
        self.p = params
        self.reset()

    def reset(self) -> None:
        self.engaged = [False, False]
        self.was_open = [True, True]  # a latch engages on the stroke that closes it from open, with the lid shut

    def step(self, lid_deg: float, lid_vel: float, closed_deg) -> tuple[float, list[float]]:
        p = self.p
        taus = []
        for i, c in enumerate(closed_deg):
            if c > p.release_deg:
                self.engaged[i] = False
            if c < p.snap_deg:
                if self.was_open[i] and lid_deg < p.lid_shut_deg:
                    self.engaged[i] = True
                self.was_open[i] = False
                taus.append(p.hold_nm)
            else:
                self.was_open[i] = True
                taus.append(0.0)
        lid = 0.0
        if any(self.engaged):
            lid = -p.lock_stiffness * math.radians(max(lid_deg, 0.0)) - p.lock_damping * lid_vel
        return lid, taus


# -- the task-file object ---------------------------------------------------------------------------------------

BoxSpec = tuple  # (centre, size, colour) or (centre, size, colour, yaw_deg), in the frame of its body (m)


@dataclass(frozen=True)
class ToolBox(Box):
    """A loose toolbox. ``pos``: the base body origin (on the table under the base centre); ``mass``: the base body's
    mass. The lid frame's origin is the hinge axis (``lid_at`` in the base frame), its boxes drawn shut. Each latch
    frame's origin is its pivot (``latch_at``), its boxes drawn CLOSED (upright); it spawns open, leaning out."""

    base_boxes: tuple[BoxSpec, ...] = ()
    lid_at: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lid_boxes: tuple[BoxSpec, ...] = ()
    lid_mass: float = 0.1
    lid_open_deg: float = 180.0
    latch_at: tuple[tuple[float, float, float], ...] = ()
    latch_boxes: tuple[BoxSpec, ...] = ()
    latch_mass: float = 0.01
    friction: float = 0.8
    params: LatchParams = LatchParams()

    def _check(self) -> None:
        super()._check()
        if self.mass is None or not (self.base_boxes and self.lid_boxes and self.latch_boxes):
            raise ValueError(f"{self.name}: a ToolBox is loose and needs base, lid and latch boxes")
        if len(self.latch_at) != 2:
            raise ValueError(f"{self.name}: two latches")


@dataclass(frozen=True)
class ToolBoxState(ObjectState):
    """The base body's pose (``pos`` is its origin), the lid angle (deg, 0 shut), how far each latch is from closed
    (deg) and whether each latch is engaged (holding the lid)."""

    lid_deg: float = 0.0
    latch_deg: tuple[float, float] = (0.0, 0.0)
    engaged: tuple[bool, bool] = (False, False)
    R: np.ndarray | None = None

    def rotation(self) -> np.ndarray:
        if self.R is not None:
            return self.R
        x, y, z, w = np.asarray(self.quat_xyzw, float) / np.linalg.norm(self.quat_xyzw)
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

    def to_local(self, p) -> np.ndarray:
        return self.rotation().T @ (np.asarray(p, float) - self.pos)

    def tilt_deg(self) -> float:
        return math.degrees(math.acos(max(-1.0, min(1.0, float(self.rotation()[2, 2])))))


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# -- Isaac Sim side ---------------------------------------------------------------------------------------------

_SPECS: dict[str, ToolBox] = {}
_MECH: dict[str, Latches] = {}
_DEBUG = [0] if __import__("os").environ.get("BOX_DEBUG") else []


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.box_name]
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=dbl).Set(Gf.Quatd(w, x, y, z))
    mat = f"{prim_path}/Looks/physics"
    m = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    m.func(mat, m)
    looks: dict = {}

    def gf_quat(a, cls=Gf.Quatd):  # rotation by ``a`` rad about y
        return cls(math.cos(a / 2), 0.0, math.sin(a / 2), 0.0)

    def body(name, pos, rot, mass, specs) -> str:
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*pos))
        xf.AddOrientOp(precision=dbl).Set(gf_quat(rot))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        for i, (centre, size, color, *yaw) in enumerate(specs):
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
            if yaw:
                cube.AddRotateZOp().Set(float(yaw[0]))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            c = cube.GetPrim()
            UsdPhysics.CollisionAPI.Apply(c)
            col = PhysxSchema.PhysxCollisionAPI.Apply(c)
            col.CreateContactOffsetAttr(0.002)
            col.CreateRestOffsetAttr(0.0)
            color = tuple(color)
            if color not in looks:
                looks[color] = f"{prim_path}/Looks/c{len(looks)}"
                vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
                vis.func(looks[color], vis)
            bind_physics_material(str(c.GetPath()), mat, stage=stage)
            bind_visual_material(str(c.GetPath()), looks[color], stage=stage)
        return path

    base = body("base", (0.0, 0.0, 0.0), 0.0, spec.mass, spec.base_boxes)
    lid = body("lid", spec.lid_at, 0.0, spec.lid_mass, spec.lid_boxes)
    tilt = -math.radians(spec.params.open_deg)  # a latch spawns open: its top leans out toward -x
    latches = [body(f"{n}_body", at, tilt, spec.latch_mass, spec.latch_boxes) for n, at in zip(LATCHES, spec.latch_at)]

    def joint(name, child, pos, rot0, limits):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/{name}")
        j.CreateAxisAttr("Y")
        j.CreateBody0Rel().SetTargets([Sdf.Path(base)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*pos))
        j.CreateLocalRot0Attr(gf_quat(rot0, Gf.Quatf))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(limits[0])  # degrees
        j.CreateUpperLimitAttr(limits[1])
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")  # gains come from the actuator config
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(0.0)

    joint(LID, lid, spec.lid_at, 0.0, (0.0, spec.lid_open_deg))
    for n, child, at in zip(LATCHES, latches, spec.latch_at):
        joint(n, child, at, tilt, (0.0, spec.params.open_deg))

    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(base))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(base))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _arena_object(spec: ToolBox):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    p = spec.params

    @configclass
    class BoxSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn)
        box_name: str = ""

    class BoxObject(Object):
        def _generate_articulation_cfg(self):
            acts = {
                "lid": ImplicitActuatorCfg(joint_names_expr=[LID], stiffness=0.0, damping=p.lid_damping,
                                           armature=p.lid_armature),
                "latches": ImplicitActuatorCfg(joint_names_expr=list(LATCHES), stiffness=0.0,
                                               damping=p.latch_damping, armature=p.latch_armature),
            }
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=acts)
            return self._add_initial_pose_to_cfg(cfg)

    return BoxObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                     spawner_cfg=BoxSpawnerCfg(box_name=spec.name),
                     initial_pose=Pose(position_xyz=tuple(spec.pos),
                                       rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy().astype(np.float64)


def _joints(base, spec: ToolBox):
    asset = base.scene[spec.name]
    names = list(asset.joint_names)
    ids = [names.index(LID)] + [names.index(n) for n in LATCHES]
    q, qd = _np(asset.data.joint_pos)[0], _np(asset.data.joint_vel)[0]
    lid_deg = math.degrees(q[ids[0]])
    closed = [spec.params.open_deg - math.degrees(q[i]) for i in ids[1:]]
    return asset, ids, lid_deg, float(qd[ids[0]]), closed


def _apply(base) -> None:
    import torch

    for name, spec in _SPECS.items():
        asset, ids, lid_deg, lid_vel, closed = _joints(base, spec)
        lid, taus = _MECH[name].step(lid_deg, lid_vel, closed)
        effort = torch.zeros((1, asset.num_joints), dtype=torch.float32, device=base.device)
        effort[0, ids[0]] = lid
        for i, t in zip(ids[1:], taus):
            effort[0, i] = t
        asset.actuators.target_command.set_effort_index(value=effort)


def _state(base, spec: ToolBox) -> ToolBoxState:
    asset, _, lid_deg, _, closed = _joints(base, spec)
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0]
    pos = pose[:3] - _np(base.scene.env_origins)[0]
    mech = _MECH[spec.name]
    probe = ToolBoxState(pos, pose[3:7], vel[:3], vel[3:6], pos, pos)
    r = probe.rotation()
    half = np.asarray(spec.size, float) / 2
    corners = np.array([[sx * half[0], sy * half[1], z] for sx in (-1, 1) for sy in (-1, 1)
                        for z in (0.0, spec.size[2])]) @ r.T + pos
    if _DEBUG:
        _DEBUG[0] += 1
        if _DEBUG[0] % 10 == 0:
            for arm in ("left_robot", "right_robot"):
                rob = base.scene[arm]
                g = _np(rob.data.body_link_pose_w)[0][list(rob.body_names).index("gripper")]
                x, y, z, w = g[3:7]
                rz = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
                tp = g[:3] + 0.134 * rz - _np(base.scene.env_origins)[0]
                print(f"[boxkit] {arm} tcp {np.round(tp, 3)}", flush=True)
            print(f"[boxkit] {_DEBUG[0]:5d} base {np.round(pos, 3)} tilt {probe.tilt_deg():5.1f} lid {lid_deg:6.2f} "
                  f"latches {np.round(closed, 1)} engaged {mech.engaged} v {np.round(vel[:3], 2)}", flush=True)
    return ToolBoxState(pos=pos, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=corners.min(0),
                        high=corners.max(0), lid_deg=lid_deg, latch_deg=tuple(closed),
                        engaged=tuple(mech.engaged), R=r)


# -- hooking yam_arena ------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_boxkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def box_task_object(spec):
        if isinstance(spec, ToolBox):
            _SPECS[spec.name] = spec
            _MECH.setdefault(spec.name, Latches(spec.params))
            return _arena_object(spec)
        return task_object(spec)

    def box_object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _state(environment.unwrapped, _SPECS[n]) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._boxkit = box_task_object, box_object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_boxkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        _apply(base)
        return write(*a, **k)

    isc.write_data_to_sim, isc._boxkit = write_data_to_sim, True


_RESET: list = []


def on_reset(fn):
    _RESET.append(fn)
    return fn


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_boxkit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def box_ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
        return out

    def box_reset(self, *a, **k):
        for mech in _MECH.values():
            mech.reset()
        for fn in _RESET:
            fn()
        return reset(self, *a, **k)

    cls._ensure_sim, cls.reset, cls._boxkit = box_ensure_sim, box_reset, True


def _patch_demo(demo) -> None:
    if getattr(demo, "_boxkit", False):
        return
    grip_rotation = demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else grip_rotation(yaw)

    demo.grip_rotation, demo._boxkit = _grip_rotation, True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
            "yam_arena.demo": _patch_demo}


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

        def patched(module):
            exec_module(module)
            _TARGETS[fullname](module)

        spec.loader.exec_module = patched
        return spec


def install() -> None:
    for name, patch in _TARGETS.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
