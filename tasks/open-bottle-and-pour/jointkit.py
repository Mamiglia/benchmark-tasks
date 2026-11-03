"""jointkit: multi-box rigid parts, optionally with one spring-driven hinge, for a yam_arena task file.

yam_arena task files only take single primitives (Box, Cylinder, Sphere, library assets). This module adds:

- ``Part``: one loose rigid body made of several boxes (a hollow bottle, a cup, a cap with a hole in it). With a
  ``Hinge`` it becomes a two-link articulation: the part and a child body on a revolute joint with a spring drive
  (an implicit actuator), whose target the task file can change at run time (``set_hinge_target``).
- ``PartState``: what ``success()/score()`` get for a Part: the body origin's pose (not a bounding-box centre), its
  velocities, a world bounding box, and the hinge angle (rad).
- ``on_step`` / ``on_reset``: callbacks run with the Isaac Lab env after every control step (before scoring) and
  after every reset.
- ``Attach``: a revolute joint from another Part to this one (outside any articulation), with a spring-damper drive
  whose target (``set_attach_target``) and enabled state (``set_attached``) the task file can change at run time.
- ``ball``: a sphere spawned with the same solver settings as the parts.

Importing it hooks ``yam_arena.scene`` (spawning and state), ``yam_arena.inspect_embodiment`` (step and reset) and
``yam_arena.demo`` (a ``Waypoint.yaw`` may be a 3x3 world rotation matrix, so ``yam_scripted`` can play grips other
than straight down). yam_arena itself is not modified. Pure Python at import; Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

Color = tuple[float, float, float]
BoxSpec = tuple[tuple[float, float, float], tuple[float, float, float], Color]  # centre, size (m), sRGB colour


@dataclass(frozen=True)
class Hinge:
    """A child body on a revolute joint. ``anchor`` is the joint's point in the part frame and the child frame's
    origin; ``boxes`` are in the child frame with the hinge closed (angle 0). The joint turns about the part's x axis
    rotated ``yaw`` about z. A drive of ``stiffness`` (N m/rad) and ``damping`` (N m s/rad) pulls it to its target."""

    name: str
    anchor: tuple[float, float, float]
    boxes: tuple[BoxSpec, ...]
    lower: float = 0.0  # rad
    upper: float = 2.0
    yaw: float = 0.0
    stiffness: float = 0.3
    damping: float = 0.002
    mass: float = 0.003


@dataclass(frozen=True)
class Attach:
    """A free-spinning revolute joint (about z of the joint frame) from another Part's body to this Part's body,
    outside any articulation. ``pos0``/``rot0``: joint frame in the other part; ``pos1``/``rot1``: in this part.
    An angular drive (``stiffness`` N m/rad toward a target angle that ``set_attach_target`` moves at run time, plus
    ``damping`` N m s/rad) resists turning and reacts on both bodies. ``set_attached`` switches the joint off (and on
    again) at run time."""

    to: str
    pos0: tuple[float, float, float]
    pos1: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot0: float = 0.0  # yaw (rad) of the joint frame in the other part
    rot1: float = 0.0
    stiffness: float = 0.0
    damping: float = 0.08


@dataclass(frozen=True)
class Part(Box):
    """A loose rigid body made of ``boxes`` (part frame, origin at ``pos``). Subclasses Box so yam_arena's task-file
    checks accept it; ``size`` is the boxes' extent and ``lo``/``hi`` its corners in the part frame."""

    boxes: tuple[BoxSpec, ...] = ()
    hinge: Hinge | None = None
    attach: Attach | None = None
    friction: float = 0.9
    lo: tuple[float, float, float] = (0.0, 0.0, 0.0)
    hi: tuple[float, float, float] = (0.0, 0.0, 0.0)
    radius: float = 0.0  # > 0: one sphere collider instead of boxes

    def _check(self) -> None:
        super()._check()
        if not (self.boxes or self.radius > 0) or self.mass is None:
            raise ValueError(f"{self.name}: a Part needs boxes (or a radius) and a mass")


def part(name, pos, boxes, mass, yaw=0.0, hinge=None, color=(0.5, 0.5, 0.5), friction=0.9, attach=None) -> Part:
    lo = np.min([np.subtract(c, np.divide(s, 2)) for c, s, _ in boxes], axis=0)
    hi = np.max([np.add(c, np.divide(s, 2)) for c, s, _ in boxes], axis=0)
    return Part(name=name, pos=tuple(pos), size=tuple((hi - lo).tolist()), mass=mass, color=color, yaw=yaw,
                boxes=tuple(boxes), hinge=hinge, attach=attach, friction=friction, lo=tuple(lo.tolist()),
                hi=tuple(hi.tolist()))


def ball(name, pos, radius, mass, color=(0.5, 0.5, 0.5), friction=0.3) -> Part:
    """A sphere, spawned by this kit so that it gets the same solver settings (32 position iterations, 1 m/s
    depenetration) as the parts it rolls around in."""
    r = float(radius)
    return Part(name=name, pos=tuple(pos), size=(2 * r,) * 3, mass=mass, color=color, friction=friction,
                lo=(-r, -r, -r), hi=(r, r, r), radius=r)


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(r: np.ndarray) -> tuple[float, float, float, float]:
    """XYZW, robust for any rotation (Shepperd)."""
    t = np.trace(r)
    if t > 0:
        s = 2 * math.sqrt(t + 1)
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4)
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + r[i, i] - r[j, j] - r[k, k])
    q = [0.0, 0.0, 0.0, (r[k, j] - r[j, k]) / s]
    q[i], q[j], q[k] = s / 4, (r[j, i] + r[i, j]) / s, (r[k, i] + r[i, k]) / s
    return tuple(q)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


@dataclass(frozen=True)
class PartState(ObjectState):
    """A Part after a control step: body origin pose, velocities, world bounding box, hinge angle (rad)."""

    hinge: float = 0.0
    R: np.ndarray = field(default=None, repr=False)

    def rotation(self) -> np.ndarray:
        return quat_to_matrix(self.quat_xyzw) if self.R is None else self.R

    def to_local(self, p) -> np.ndarray:
        return self.rotation().T @ (np.asarray(p, float) - self.pos)

    def to_world(self, p) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(p, float)

    def tilt(self) -> float:
        """Angle (rad) between the part's z axis and the vertical."""
        return math.acos(max(-1.0, min(1.0, float(self.rotation()[2, 2]))))


# -- spawning (Isaac Sim only) ----------------------------------------------------------------------------------

_SPECS: dict[str, Part] = {}
_JOINTS: dict[str, str] = {}  # part name -> its attach joint's prim path (the first one spawned: env 0)


def _gf_quat(r: np.ndarray, cls):
    x, y, z, w = matrix_to_quat(r)
    return cls(w, x, y, z)


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.part_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    material = f"{prim_path}/Looks/physics"
    mat = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    mat.func(material, mat)
    looks: dict = {}

    def body(path, offset, mass, boxes, radius=0.0):
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*offset))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        shapes = list(enumerate(boxes)) + ([(-1, ((0, 0, 0), None, spec.color))] if radius > 0 else [])
        for i, (centre, size, color) in shapes:
            if size is None:
                geom = UsdGeom.Sphere.Define(stage, f"{path}/sphere")
                geom.CreateRadiusAttr(radius)
            else:
                geom = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
                geom.CreateSizeAttr(1.0)
                geom.AddTranslateOp().Set(Gf.Vec3d(*centre))
                geom.AddScaleOp().Set(Gf.Vec3f(*size))
            c = geom.GetPrim()
            UsdPhysics.CollisionAPI.Apply(c)
            col = PhysxSchema.PhysxCollisionAPI.Apply(c)
            col.CreateContactOffsetAttr(0.001)
            col.CreateRestOffsetAttr(0.0)
            if color not in looks:
                looks[color] = f"{prim_path}/Looks/c{len(looks)}"
                vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
                vis.func(looks[color], vis)
            bind_physics_material(str(c.GetPath()), material, stage=stage)
            bind_visual_material(str(c.GetPath()), looks[color], stage=stage)
        return prim

    main = body(f"{prim_path}/body", (0.0, 0.0, 0.0), spec.mass, spec.boxes, spec.radius)
    h = spec.hinge
    if h is not None:
        child_path = f"{prim_path}/{h.name}_body"
        body(child_path, h.anchor, h.mass, h.boxes)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{child_path}/{h.name}")
        j.CreateAxisAttr("X")
        j.CreateBody0Rel().SetTargets([main.GetPath()])
        j.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*h.anchor))
        j.CreateLocalRot0Attr(_gf_quat(rot_z(h.yaw), Gf.Quatf))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(_gf_quat(rot_z(h.yaw), Gf.Quatf))
        j.CreateLowerLimitAttr(math.degrees(h.lower))  # USD revolute limits and drive gains are per degree
        j.CreateUpperLimitAttr(math.degrees(h.upper))
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(h.stiffness * math.pi / 180)
        drive.CreateDampingAttr(h.damping * math.pi / 180)
        drive.CreateTargetPositionAttr(0.0)
        UsdPhysics.ArticulationRootAPI.Apply(main)
        art = PhysxSchema.PhysxArticulationAPI.Apply(main)
        art.CreateEnabledSelfCollisionsAttr(False)
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)
    a = spec.attach
    if a is not None:
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/{spec.name}_attach")
        _JOINTS.setdefault(spec.name, str(j.GetPath()))
        j.CreateAxisAttr("Z")
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path.rsplit('/', 1)[0]}/{a.to}/body")])
        j.CreateBody1Rel().SetTargets([main.GetPath()])
        j.CreateLocalPos0Attr(Gf.Vec3f(*a.pos0))
        j.CreateLocalRot0Attr(_gf_quat(rot_z(a.rot0), Gf.Quatf))
        j.CreateLocalPos1Attr(Gf.Vec3f(*a.pos1))
        j.CreateLocalRot1Attr(_gf_quat(rot_z(a.rot1), Gf.Quatf))
        j.CreateExcludeFromArticulationAttr(True)
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(a.stiffness * math.pi / 180)
        drive.CreateDampingAttr(a.damping * math.pi / 180)
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateTargetVelocityAttr(0.0)
    return root.GetPrim()


def _attach_prim(name: str):
    from isaaclab.sim.utils.stage import get_current_stage

    prim = get_current_stage().GetPrimAtPath(_JOINTS[name])
    if not prim.IsValid():
        raise RuntimeError(f"{name}: no attach joint at {_JOINTS[name]}")
    return prim


def set_attached(environment, name: str, on: bool) -> None:
    """Switch a Part's ``attach`` joint on or off (env 0)."""
    from pxr import UsdPhysics

    UsdPhysics.Joint(_attach_prim(name)).GetJointEnabledAttr().Set(bool(on))


def set_attach_target(environment, name: str, angle: float) -> None:
    """Move the target angle (rad, in (-pi, pi]) of a Part's ``attach`` drive (env 0)."""
    from pxr import UsdPhysics

    UsdPhysics.DriveAPI(_attach_prim(name), "angular").GetTargetPositionAttr().Set(math.degrees(angle))


def _arena_object(spec: Part):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class _Cfg(SpawnerCfg):
        func: object = clone(_spawn)
        part_name: str = ""

    h = spec.hinge

    class _Obj(Object):
        def _generate_articulation_cfg(self):
            acts = {h.name: ImplicitActuatorCfg(joint_names_expr=[h.name], stiffness=h.stiffness, damping=h.damping)}
            return self._add_initial_pose_to_cfg(ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg,
                                                                 actuators=acts))

    return _Obj(name=spec.name, object_type=ObjectType.ARTICULATION if h else ObjectType.RIGID,
                spawner_cfg=_Cfg(part_name=spec.name),
                initial_pose=Pose(position_xyz=tuple(spec.pos),
                                  rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))


# -- reading and writing state ------------------------------------------------------------------------------------


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _hinge_id(asset, h: Hinge) -> int:
    return asset.find_joints([h.name])[0][0]


def _part_state(environment, spec: Part) -> PartState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0]
    pos = pose[:3] - _np(base.scene.env_origins)[0]
    r = quat_to_matrix(pose[3:7])
    corners = np.array([[a, b, c] for a in (spec.lo[0], spec.hi[0]) for b in (spec.lo[1], spec.hi[1])
                        for c in (spec.lo[2], spec.hi[2])]) @ r.T + pos
    angle = float(_np(asset.data.joint_pos)[0, _hinge_id(asset, spec.hinge)]) if spec.hinge else 0.0
    return PartState(pos=pos, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=corners.min(0),
                     high=corners.max(0), hinge=angle, R=r)


def set_hinge_target(environment, name: str, angle: float) -> None:
    """Move a Part's hinge drive target (rad); the drive's stiffness then pulls the child toward it."""
    import torch

    asset = environment.unwrapped.scene[name]
    jid = _hinge_id(asset, _SPECS[name].hinge)
    value = torch.full((1, 1), float(angle), device=asset.device)
    asset.actuators.target_command.set_position_index(value=value, joint_ids=[jid])


# -- hooks ------------------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def _patch_scene(scene) -> None:
    if getattr(scene, "_jointkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, Part):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        out = object_states(environment, [n for n in names if n not in _SPECS])
        out.update({n: _part_state(environment, _SPECS[n]) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._jointkit = _task_object, _object_states, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_jointkit", False):
        return
    task_step, reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        for fn in _STEP:
            fn(self._env)
        return task_step(self, *a, **k)

    def _reset(self, *a, **k):
        obs = reset(self, *a, **k)
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset, cls._jointkit = _task_step, _reset, True


def _patch_demo(demo) -> None:
    if getattr(demo, "_jointkit", False):
        return
    grip_rotation = demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else grip_rotation(yaw)

    demo.grip_rotation, demo._jointkit = _grip_rotation, True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
            "yam_arena.demo": _patch_demo}


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
