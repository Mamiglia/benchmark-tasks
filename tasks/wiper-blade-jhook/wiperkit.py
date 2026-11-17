"""wiperkit: multi-box rigid parts in any orientation, spring-driven child joints, a fixed-base articulation, and
switchable D6 joints between parts, for a yam_arena task file.

yam_arena task files only take single primitives. This module adds (adapted from the partkit module of our IV-bag task):

- ``Part``: one rigid body made of several boxes, spawned at ``pos`` with orientation ``quat`` (XYZW). With
  ``slides`` it becomes an articulation: each ``Slide`` is a child body on a prismatic or revolute joint with a spring
  drive (an implicit actuator). ``fixed=True``: the articulation root is fixed to the world by a FixedJoint.
- ``Link``: a D6 joint from a body of another Part (``to``: "<part>" for its root body or "<part>/<slide>" for a
  child) to this Part's root body. ``slide=False``: all six axes locked (a weld). ``slide=True``: it slides along
  its frame's x within ``[lower, upper]``, a linear drive acting along it; the other five axes locked.
  ``set_link`` switches a joint on or off at run time; ``set_link_drive`` changes the drive.
- ``PartState``: what ``success()/score()`` get for a Part: the root body's pose, velocities, a world bounding box,
  slide joint positions and velocities.
- ``on_physics`` / ``on_step`` / ``on_reset``: callbacks run before every physics step (240 Hz), after every control
  step and after every reset, with the Isaac Lab env.
- ``yam_scripted`` runs the task module's ``demo_controller`` (if any) closed loop on each observation.
- ``robot_hands(env)``: each gripper's tool point (world, m), approach axis and finger opening (m).

Importing it hooks ``yam_arena.scene``, ``yam_arena.inspect_embodiment`` and ``yam_arena.scripted``; yam_arena itself
is not modified. Pure Python at import; Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

# A box: (centre, size, colour[, collide]) in the body frame (m). Colour is sRGB; collide=False: drawn, no collider.
BoxSpec = tuple


@dataclass(frozen=True)
class Slide:
    """A child body on a prismatic joint along ``axis`` (part frame), or with ``revolute=True`` a hinge about ``axis``
    through ``origin``. The child frame is parallel to the part frame at joint position 0, origin at ``origin``;
    ``boxes`` are in it. A drive of ``stiffness`` (N/m or N m/rad) and ``damping`` pulls it to ``target``."""

    name: str
    origin: tuple[float, float, float]
    axis: tuple[float, float, float]
    boxes: tuple[BoxSpec, ...]
    lower: float
    upper: float
    stiffness: float
    damping: float
    mass: float
    revolute: bool = False
    target: float = 0.0
    armature: float = 0.0


@dataclass(frozen=True)
class Link:
    """A D6 joint from body ``to`` (body 0) to this Part's root (body 1). Frame 0: ``pos0`` / ``rot0`` (3x3, rows) in
    body 0; frame 1: ``pos1`` / ``rot1`` in this part. Joint position = how far frame 1 is along frame 0's x."""

    name: str
    to: str
    pos0: tuple[float, float, float]
    rot0: tuple
    pos1: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot1: tuple = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    slide: bool = False
    lower: float = 0.0
    upper: float = 0.0
    stiffness: float = 0.0
    damping: float = 0.0
    max_force: float = 1e6
    target: float = 0.0
    enabled: bool = False
    break_force: float = 0.0  # N; 0: unbreakable


@dataclass(frozen=True)
class Part(Box):
    """A rigid body made of ``boxes`` (part frame, origin at ``pos``, orientation ``quat``)."""

    boxes: tuple[BoxSpec, ...] = ()
    slides: tuple[Slide, ...] = ()
    links: tuple[Link, ...] = ()
    quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    friction: float = 0.9
    fixed: bool = False
    lo: tuple[float, float, float] = (0.0, 0.0, 0.0)
    hi: tuple[float, float, float] = (0.0, 0.0, 0.0)
    contact_offset: float = 0.001
    com: tuple[float, float, float] | None = None
    inertia: tuple[float, float, float] | None = None

    def _check(self) -> None:
        super()._check()
        if not self.boxes or self.mass is None:
            raise ValueError(f"{self.name}: a Part needs boxes and a mass")


def part(name, pos, boxes, mass, quat=(0.0, 0.0, 0.0, 1.0), color=(0.5, 0.5, 0.5), friction=0.9, slides=(),
         links=(), fixed=False, contact_offset=0.001, com=None, inertia=None) -> Part:
    lo = np.min([np.subtract(b[0], np.divide(b[1], 2)) for b in boxes], axis=0)
    hi = np.max([np.add(b[0], np.divide(b[1], 2)) for b in boxes], axis=0)
    return Part(name=name, pos=tuple(float(v) for v in pos), size=tuple((hi - lo).tolist()), mass=mass, color=color,
                boxes=tuple(boxes), slides=tuple(slides), links=tuple(links), quat=tuple(float(v) for v in quat),
                friction=friction, fixed=fixed, lo=tuple(lo.tolist()), hi=tuple(hi.tolist()),
                contact_offset=contact_offset, com=com, inertia=inertia)


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(r: np.ndarray) -> tuple[float, float, float, float]:
    """XYZW, robust for any rotation (Shepperd)."""
    r = np.asarray(r, float)
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


def frame_x(axis) -> np.ndarray:
    """A rotation whose x axis is ``axis``."""
    x = np.asarray(axis, float) / np.linalg.norm(axis)
    t = np.array([0.0, 0.0, 1.0]) if abs(x[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    y = np.cross(t, x)
    y /= np.linalg.norm(y)
    return np.stack([x, y, np.cross(x, y)], axis=1)


@dataclass(frozen=True)
class PartState(ObjectState):
    """A Part after a step: root pose, velocities, world bounding box, slide positions and velocities."""

    slides: np.ndarray = field(default_factory=lambda: np.zeros(0))
    slide_vel: np.ndarray = field(default_factory=lambda: np.zeros(0))
    R: np.ndarray = field(default=None, repr=False)

    def rotation(self) -> np.ndarray:
        return quat_to_matrix(self.quat_xyzw) if self.R is None else self.R

    def to_local(self, p) -> np.ndarray:
        return self.rotation().T @ (np.asarray(p, float) - self.pos)

    def to_world(self, p) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(p, float)

    def axis(self, v) -> np.ndarray:
        return self.rotation() @ np.asarray(v, float)


# -- spawning (Isaac Sim only) ----------------------------------------------------------------------------------

_SPECS: dict[str, Part] = {}
_LINKS: dict[str, str] = {}  # link name -> joint prim path (env 0)


def _gf_quat(r: np.ndarray, cls):
    x, y, z, w = matrix_to_quat(np.asarray(r, float))
    return cls(w, x, y, z)


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.part_name]
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=dbl).Set(Gf.Quatd(w, x, y, z))
    material = f"{prim_path}/Looks/physics"
    mat = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    mat.func(material, mat)
    looks: dict = {}

    def body(path, offset, mass, boxes, com=None, inertia=None):
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*offset))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(mass)
        if com is not None:
            mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        if inertia is not None:
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        prb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        prb.CreateMaxDepenetrationVelocityAttr(1.0)
        prb.CreateSolverPositionIterationCountAttr(32)
        for i, (centre, size, color, *rest) in enumerate(boxes):
            collide = rest[0] if rest else True
            geom = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            geom.CreateSizeAttr(1.0)
            geom.AddTranslateOp().Set(Gf.Vec3d(*centre))
            geom.AddScaleOp().Set(Gf.Vec3f(*size))
            c = geom.GetPrim()
            if collide:
                UsdPhysics.CollisionAPI.Apply(c)
                col = PhysxSchema.PhysxCollisionAPI.Apply(c)
                col.CreateContactOffsetAttr(spec.contact_offset)
                col.CreateRestOffsetAttr(0.0)
                bind_physics_material(str(c.GetPath()), material, stage=stage)
            key = tuple(color[:3])
            if key not in looks:
                looks[key] = f"{prim_path}/Looks/c{len(looks)}"
                vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(key), roughness=0.45)
                vis.func(looks[key], vis)
            bind_visual_material(str(c.GetPath()), looks[key], stage=stage)
        return prim

    main = body(f"{prim_path}/body", (0.0, 0.0, 0.0), spec.mass, spec.boxes, spec.com, spec.inertia)
    for s in spec.slides:
        child = f"{prim_path}/{s.name}"
        body(child, s.origin, s.mass, s.boxes)
        j = (UsdPhysics.RevoluteJoint if s.revolute else UsdPhysics.PrismaticJoint).Define(stage, f"{child}/{s.name}_joint")
        j.CreateAxisAttr("X")
        r = frame_x(s.axis)
        j.CreateBody0Rel().SetTargets([main.GetPath()])
        j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*s.origin))
        j.CreateLocalRot0Attr(_gf_quat(r, Gf.Quatf))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(_gf_quat(r, Gf.Quatf))
        scale = 180 / math.pi if s.revolute else 1.0  # USD revolute limits are in degrees
        j.CreateLowerLimitAttr(s.lower * scale)
        j.CreateUpperLimitAttr(s.upper * scale)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if s.revolute else "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(0.0)
    if spec.slides:
        if spec.fixed:
            anchor = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/fixed_to_world")
            anchor.CreateBody1Rel().SetTargets([main.GetPath()])
            art_prim = root.GetPrim()
        else:
            art_prim = main
        UsdPhysics.ArticulationRootAPI.Apply(art_prim)
        art = PhysxSchema.PhysxArticulationAPI.Apply(art_prim)
        art.CreateEnabledSelfCollisionsAttr(False)
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)
    elif spec.fixed:
        UsdPhysics.RigidBodyAPI(main).CreateKinematicEnabledAttr(True)
    env = prim_path.rsplit("/", 1)[0]
    for ln in spec.links:
        j = UsdPhysics.Joint.Define(stage, f"{prim_path}/{ln.name}")
        locked = ("transY", "transZ", "rotX", "rotY", "rotZ") + (() if ln.slide else ("transX",))
        for axis in locked:
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        if ln.slide:
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), "transX")
            lim.CreateLowAttr(ln.lower)
            lim.CreateHighAttr(ln.upper)
            lin = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "transX")
            lin.CreateTypeAttr("force")
            lin.CreateStiffnessAttr(ln.stiffness)
            lin.CreateDampingAttr(ln.damping)
            lin.CreateMaxForceAttr(ln.max_force)
            lin.CreateTargetPositionAttr(ln.target)
            lin.CreateTargetVelocityAttr(0.0)
        if ln.break_force > 0:
            j.CreateBreakForceAttr().Set(float(ln.break_force))
        _LINKS.setdefault(ln.name, str(j.GetPath()))
        to = ln.to if "/" in ln.to else f"{ln.to}/body"
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{env}/{to}")])
        j.CreateBody1Rel().SetTargets([main.GetPath()])
        j.CreateLocalPos0Attr(Gf.Vec3f(*ln.pos0))
        j.CreateLocalRot0Attr(_gf_quat(np.asarray(ln.rot0, float), Gf.Quatf))
        j.CreateLocalPos1Attr(Gf.Vec3f(*ln.pos1))
        j.CreateLocalRot1Attr(_gf_quat(np.asarray(ln.rot1, float), Gf.Quatf))
        j.CreateExcludeFromArticulationAttr(True)
        j.CreateJointEnabledAttr(bool(ln.enabled))
    return root.GetPrim()


_PRIMS: dict = {}


def _link_prim(name: str):
    from isaaclab.sim.utils.stage import get_current_stage

    prim = _PRIMS.get(name)
    if prim is None or not prim.IsValid():
        prim = get_current_stage().GetPrimAtPath(_LINKS[name])
        if not prim.IsValid():
            raise RuntimeError(f"{name}: no joint at {_LINKS[name]}")
        _PRIMS[name] = prim
    return prim


def set_link(name: str, on: bool) -> None:
    """Switch a D6 link on or off (env 0)."""
    from pxr import UsdPhysics

    UsdPhysics.Joint(_link_prim(name)).GetJointEnabledAttr().Set(bool(on))


def set_link_drive(name: str, target: float | None = None, stiffness: float | None = None,
                   damping: float | None = None, max_force: float | None = None) -> None:
    from pxr import UsdPhysics

    drive = UsdPhysics.DriveAPI(_link_prim(name), "transX")
    for value, attr in ((target, drive.GetTargetPositionAttr()), (stiffness, drive.GetStiffnessAttr()),
                        (damping, drive.GetDampingAttr()), (max_force, drive.GetMaxForceAttr())):
        if value is not None:
            attr.Set(float(value))


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

    class _Obj(Object):
        def _generate_articulation_cfg(self):
            acts = {s.name: ImplicitActuatorCfg(joint_names_expr=[f"{s.name}_joint"], stiffness=s.stiffness,
                                                damping=s.damping, armature=s.armature, effort_limit_sim=1e6)
                    for s in spec.slides}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=acts)
            cfg = self._add_initial_pose_to_cfg(cfg)
            cfg.init_state.joint_pos = {f"{s.name}_joint": 0.0 for s in spec.slides}
            return cfg

    return _Obj(name=spec.name, object_type=ObjectType.ARTICULATION if spec.slides else ObjectType.RIGID,
                spawner_cfg=_Cfg(part_name=spec.name),
                initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=tuple(spec.quat)))


# -- reading state ------------------------------------------------------------------------------------------------


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


_JIDS: dict = {}


def _jids(asset, spec: Part):
    if spec.name not in _JIDS:
        names = list(asset.joint_names)
        _JIDS[spec.name] = [names.index(f"{s.name}_joint") for s in spec.slides]
    return _JIDS[spec.name]


def _part_state(environment, spec: Part) -> PartState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0]
    pos = pose[:3] - _np(base.scene.env_origins)[0]
    r = quat_to_matrix(pose[3:7])
    corners = np.array([[a, b, c] for a in (spec.lo[0], spec.hi[0]) for b in (spec.lo[1], spec.hi[1])
                        for c in (spec.lo[2], spec.hi[2])]) @ r.T + pos
    slides = slide_vel = np.zeros(0)
    if spec.slides:
        ids = _jids(asset, spec)
        slides = _np(asset.data.joint_pos)[0, ids]
        slide_vel = _np(asset.data.joint_vel)[0, ids]
    return PartState(pos=pos, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=corners.min(0),
                     high=corners.max(0), slides=slides, slide_vel=slide_vel, R=r)


def set_slide_targets(environment, name: str, targets: dict) -> None:
    import torch

    asset = environment.unwrapped.scene[name]
    spec = _SPECS[name]
    ids = [_jids(asset, spec)[[s.name for s in spec.slides].index(k)] for k in targets]
    value = torch.tensor([list(targets.values())], dtype=torch.float32, device=asset.device)
    asset.set_joint_position_target_index(target=value, joint_ids=ids)


TCP_OFFSET = 0.134


def robot_hands(environment) -> dict:
    """{arm: (tool point, gripper rotation (columns: closing axis, -, approach), finger opening m)} (world, env 0)."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins)[0]
    out = {}
    for side in ("left", "right"):
        rob = base.scene[f"{side}_robot"]
        g = _np(rob.data.body_link_pose_w)[0][list(rob.body_names).index("gripper")]
        rg = quat_to_matrix(g[3:7])
        rz = rg[:, 2]
        q = _np(rob.data.joint_pos)[0]
        names = list(rob.joint_names)
        opening = -2.0 * q[names.index("joint7")] if "joint7" in names else 0.0
        out[side] = (g[:3] + TCP_OFFSET * rz - origin, rg, float(opening))
    return out


# -- hooks ------------------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_PHYSICS: list = []


def on_physics(fn):
    _PHYSICS.append(fn)
    return fn


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def _patch_scene(scene) -> None:
    if getattr(scene, "_wiperkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, Part):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _part_state(environment, _SPECS[n]) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._wiperkit = _task_object, _object_states, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_wiperkit", False):
        return
    task_step, reset, ensure_sim = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        isc = self._env.unwrapped.scene if self._env is not None else None
        if isc is not None and not getattr(isc, "_wiperkit", False):  # the env calls this once per physics step
            write, env = isc.write_data_to_sim, self._env

            def write_data_to_sim(*a2, **k2):
                for fn in _PHYSICS:
                    fn(env)
                return write(*a2, **k2)

            isc.write_data_to_sim, isc._wiperkit = write_data_to_sim, True
        return out

    def _task_step(self, *a, **k):
        for fn in _STEP:
            fn(self._env)
        return task_step(self, *a, **k)

    def _reset(self, *a, **k):
        for fn in _RESET:
            fn(None)
        obs = reset(self, *a, **k)
        for fn in _RESET:  # again after the settle, so it does not count
            fn(self._env)
        return obs

    cls._task_step, cls.reset, cls._ensure_sim, cls._wiperkit = _task_step, _reset, _ensure_sim, True


def _patch_scripted(scripted) -> None:
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_wiperkit", False):
        return
    reset, act = cls.reset, cls.act

    def _controller(self):
        task = getattr(self, "_task", None)
        return getattr(task.module, "demo_controller", None) if task is not None else None

    def _reset(self, scene):
        make = _controller(self)
        if make is None:
            return reset(self, scene)
        self._wk_ctl = make(dict(left=self._bases["left"], right=self._bases["right"]), self._dt)
        return None

    def _act(self, observation):
        ctl = getattr(self, "_wk_ctl", None)
        if ctl is None or _controller(self) is None:
            return act(self, observation)
        from inspect_robots.types import Action, ActionChunk

        data, done = ctl.act(observation)
        meta = {"request_stop": True} if done else {}
        return ActionChunk(actions=[Action(data=data, meta=meta)], control_hz=1.0 / self._dt)

    cls.reset, cls.act, cls._wiperkit = _reset, _act, True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
            "yam_arena.scripted": _patch_scripted}


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
