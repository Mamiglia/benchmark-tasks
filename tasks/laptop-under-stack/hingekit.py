"""hingekit: a base and a lid on a friction hinge, as one object in a yam_arena task file.

yam_arena task files only hold rigid primitives. Importing this module hooks ``yam_arena.scene`` (and the
embodiment's reset and step) without modifying yam_arena, so a task file can list a :class:`Hinged` object:
an articulation of two bodies built from boxes, joined by a revolute joint along the base's y axis.

The hinge is a constant-torque (Coulomb) friction hinge, as in a laptop: it holds the lid at any angle and
gives way only to a torque above ``hold_torque``. PhysX articulation joint friction scales with the joint's
constraint force, so it can't model that. Instead a stiff position drive whose force is capped at
``hold_torque`` holds the lid. After every control step its target is dragged along behind the lid, never more
than ``hold_torque / STIFFNESS`` away (a spring-slider): the lid sticks until pushed with more than ``hold_torque``,
then slides against ``hold_torque``, and doesn't creep under smaller loads.

``success()`` and ``score()`` receive a :class:`HingeState` for the object: the base body's pose (its origin,
not a bounding-box centre), velocities, the base's world-aligned bounds, and ``angle``, the hinge angle in
radians (0 closed, positive open).

Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import Box, ObjectState

Color = tuple[float, float, float]
BoxSpec = tuple[tuple[float, float, float], tuple[float, float, float], Color]  # centre, size (m), sRGB

STIFFNESS = 200.0  # N m/rad: the lid gives 0.46 deg before a 1.6 N m cap is reached
DAMPING = 1.0  # N m s/rad


def extent(boxes) -> tuple[float, float, float]:
    lo = np.min([np.subtract(c, np.divide(s, 2)) for c, s, _ in boxes], axis=0)
    hi = np.max([np.add(c, np.divide(s, 2)) for c, s, _ in boxes], axis=0)
    return tuple(float(v) for v in hi - lo)


@dataclass(frozen=True)
class Hinged(Box):
    """A base and a lid on a friction hinge. ``pos`` is the base body's origin (world, m); ``base_boxes`` are in
    the base frame. ``hinge`` is the hinge axis's point in the base frame; the axis is the base's +y. The lid's
    frame has its origin on the hinge and equals the base frame when closed; ``lid_boxes`` are in it. The lid
    opens by turning about +y, so it should reach out along -x. Subclasses Box only so yam_arena's task-file
    checks accept it; ``size`` is the base's bounding box."""

    base_boxes: tuple[BoxSpec, ...] = ()
    lid_boxes: tuple[BoxSpec, ...] = ()
    hinge: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lid_mass: float = 0.1  # kg; ``mass`` is the base's
    hold_torque: float = 1.0  # N m: the hinge's friction torque
    upper_deg: float = 135.0  # the hinge's open stop
    friction: float = 0.8  # static = dynamic friction coefficient of every box

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if not (self.base_boxes and self.lid_boxes):
            raise ValueError(f"{self.name}: a Hinged object needs base and lid boxes")
        if not (self.mass and self.mass > 0 and self.lid_mass > 0 and self.hold_torque > 0):
            raise ValueError(f"{self.name}: masses and hold_torque must be positive")
        if not 0 < self.upper_deg <= 180:
            raise ValueError(f"{self.name}: upper_deg must be in (0, 180]")


@dataclass(frozen=True)
class HingeState(ObjectState):
    angle: float = 0.0  # rad, 0 closed

    def rotation(self) -> np.ndarray:
        x, y, z, w = self.quat_xyzw
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )


def base_bounds(spec: Hinged, pos, rot) -> tuple[np.ndarray, np.ndarray]:
    """World-aligned bounds of the base's boxes for the base at ``pos`` turned by ``rot``."""
    pts = [
        np.asarray(pos) + rot @ (np.array(c) + np.array(s) / 2 * np.array([sx, sy, sz]))
        for c, s, _ in spec.base_boxes
        for sx in (-1, 1)
        for sy in (-1, 1)
        for sz in (-1, 1)
    ]
    return np.min(pts, axis=0), np.max(pts, axis=0)


# -- building the articulation (Isaac Sim only) ---------------------------------------------------------------

_SPECS: dict[str, Hinged] = {}


def _joint(spec: Hinged) -> str:
    return f"{spec.name}_hinge"


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.part]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0, 0, 0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    material = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    pm.func(material, pm)
    looks: dict[Color, str] = {}

    def body(path, pos, mass, boxes):
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
        xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        for i, (centre, size, color) in enumerate(boxes):
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            box = cube.GetPrim()
            UsdPhysics.CollisionAPI.Apply(box)
            cp = PhysxSchema.PhysxCollisionAPI.Apply(box)
            cp.CreateContactOffsetAttr(0.003)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(box.GetPath()), material, stage=stage)
            if color not in looks:
                looks[color] = f"{prim_path}/Looks/c{len(looks)}"
                vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
                vis.func(looks[color], vis)
            bind_visual_material(str(box.GetPath()), looks[color], stage=stage)
        return prim

    base_path, lid_path = f"{prim_path}/base", f"{prim_path}/lid"
    base = body(base_path, (0.0, 0.0, 0.0), spec.mass, spec.base_boxes)
    body(lid_path, spec.hinge, spec.lid_mass, spec.lid_boxes)
    j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/{_joint(spec)}")
    j.CreateAxisAttr("Y")
    j.CreateBody0Rel().SetTargets([Sdf.Path(base_path)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(lid_path)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*spec.hinge))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(0.0)  # USD revolute limits are in degrees
    j.CreateUpperLimitAttr(spec.upper_deg)
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")  # USD angular gains are per degree
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(STIFFNESS * math.pi / 180)
    drive.CreateDampingAttr(DAMPING * math.pi / 180)
    drive.CreateMaxForceAttr(spec.hold_torque)
    drive.CreateTargetPositionAttr(0.0)
    UsdPhysics.ArticulationRootAPI.Apply(base)
    art = PhysxSchema.PhysxArticulationAPI.Apply(base)
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _arena_object(spec: Hinged):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class HingedSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn)
        part: str = ""

    class HingedObject(Object):
        def _generate_articulation_cfg(self):
            # The capped drive is the friction hinge (Isaac Lab sets these gains in rad on the PhysX drive).
            hinge = ImplicitActuatorCfg(
                joint_names_expr=[_joint(spec)],
                stiffness=STIFFNESS,
                damping=DAMPING,
                joint_effort_limit=spec.hold_torque,
            )
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators={"hinge": hinge})
            return self._add_initial_pose_to_cfg(cfg)

    return HingedObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=HingedSpawnerCfg(part=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0, 0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
    )


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _joint_index(asset, spec: Hinged) -> int:
    return list(asset.joint_names).index(_joint(spec))


def _state(environment, spec: Hinged) -> HingeState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    pose = _np(asset.data.root_link_pose_w)[0]
    pos = pose[:3] - _np(base.scene.env_origins)[0]
    vel = next((_np(getattr(asset.data, k))[0] for k in ("root_link_vel_w", "root_vel_w") if hasattr(asset.data, k)), np.zeros(6))
    probe = HingeState(pos, pose[3:7], vel[:3], vel[3:6], pos, pos)
    low, high = base_bounds(spec, pos, probe.rotation())
    angle = float(_np(asset.data.joint_pos)[0][_joint_index(asset, spec)])
    return HingeState(pos, pose[3:7], vel[:3], vel[3:6], low, high, angle)


_TARGET: dict = {}


def _set_targets(environment, zero: bool = False) -> None:
    """Spring-slider friction: each hinge's drive target stays put while the drive holds (|target - q| below
    hold_torque / STIFFNESS), and is dragged along at that distance once the lid is forced past it."""
    if environment is None:
        return
    scene = environment.unwrapped.scene
    for spec in _SPECS.values():
        asset = scene[spec.name]
        q = _t(asset.data.joint_pos).clone()
        target = _TARGET.get(spec.name)
        if zero or target is None:
            target = q * 0
        else:
            lim = spec.hold_torque / STIFFNESS
            target = q + (target - q).clamp(-lim, lim)
        _TARGET[spec.name] = target
        command = getattr(getattr(asset, "actuators", None), "target_command", None)
        if command is not None:  # Isaac Lab 3.0, as yam_arena.actions uses it
            command.set_position_index(value=target)
        else:
            asset.set_joint_position_target(target)


# -- hooking yam_arena ----------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []


def on_step(fn):
    """Register ``fn(env)`` to run after every control step, before the task file is scored."""
    _STEP.append(fn)
    return fn


def on_reset(fn):
    """Register ``fn(env)`` to run after every reset."""
    _RESET.append(fn)
    return fn


def _patch_scene(scene) -> None:
    if getattr(scene, "_hingekit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def patched_task_object(spec):
        if isinstance(spec, Hinged):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def patched_object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _state(environment, _SPECS[n]) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = patched_task_object, patched_object_states
    scene._hingekit = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_hingekit", False):
        return
    task_step, reset = cls._task_step, cls.reset

    def patched_task_step(self, *a, **k):
        _set_targets(self._env)
        for fn in _STEP:
            fn(self._env)
        return task_step(self, *a, **k)

    def patched_reset(self, *a, **k):
        _set_targets(getattr(self, "_env", None), zero=True)  # so the lid settles closed
        obs = reset(self, *a, **k)
        _set_targets(self._env, zero=True)
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = patched_task_step, patched_reset
    cls._hingekit = True


class _Finder(importlib.abc.MetaPathFinder):
    """Patches each target module right after it is first imported (yam_arena.scene needs Isaac Sim running)."""

    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        exec_module, patch = spec.loader.exec_module, self.targets[fullname]

        def patched_exec(module):
            exec_module(module)
            patch(module)

        spec.loader.exec_module = patched_exec
        return spec


def install() -> None:
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
