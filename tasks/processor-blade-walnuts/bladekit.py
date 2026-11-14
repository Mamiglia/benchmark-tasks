"""bladekit: multi-box rigid parts and a sliding hub joint, for a yam_arena task file.

yam_arena task files only take single primitives (Box, Cylinder, Sphere, library assets). This module adds:

- ``Part``: one loose (or, with ``fixed=True``, kinematic) rigid body made of several boxes (a bowl, a blade).
- ``Slide``: a joint from another Part to this one (outside any articulation) that lets this part slide along and
  spin about one axis, the way a blade hub sits on a bowl's centre post. Sideways and tilt are held by stiff
  spring drives, so the hub's sideways load and tilting moment can be read from how far it is pushed off the
  axis. The axial drive is a spring whose target the task file moves at run time (``set_slide_target``): that is
  how the task file makes breakaway friction. ``set_slide_enabled`` switches the joint off (and on).
- ``PartState``: what ``success()/score()`` get for a Part: the body origin's pose, velocities, a world bounding box.
- ``on_physics(fn)``: ``fn(env)`` runs before every physics step (240 Hz); ``on_env_reset(fn)``: right after the
  environment's own reset, before yam_arena's settle steps; ``on_reset(fn)``: after the settle steps.
- ``Contacts``: PhysX contact forces between one Part and the robot's finger pads.
- A scripted-policy hook: when the task file defines ``controller(bases, dt)``, ``yam_scripted`` asks that object
  for every action (it gets the observation: camera images and joint positions) instead of playing waypoints.

Importing it hooks ``yam_arena.scene`` (spawning and state), ``yam_arena.inspect_embodiment`` (physics steps and
reset) and ``yam_arena.scripted``. yam_arena itself is not modified. Pure Python at import; Isaac Sim is imported
lazily.
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
BoxSpec = tuple  # (centre, size (m), sRGB colour[, yaw about z (rad)[, friction]]) in the part frame
ARMS = ("left_robot", "right_robot")
PADS = ("tip_left", "tip_right")


@dataclass(frozen=True)
class Slide:
    """A joint from Part ``to`` to this Part. ``pos0`` (in ``to``'s frame) and ``pos1`` (in this part's frame) are
    the same point when the joint is at 0; the joint's axis is the z axis of both frames. Travel along it is
    limited to [0, ``travel``]; spin about it is free. Sideways offsets and tilts are pulled back by springs of
    ``lateral_k`` (N/m) and ``tilt_k`` (N m/rad), damped. The axial drive is a spring of ``axial_k`` (N/m) toward
    a target that the task file moves."""

    to: str
    pos0: tuple[float, float, float]
    pos1: tuple[float, float, float]
    travel: float
    lateral_k: float = 3000.0
    lateral_d: float = 20.0
    tilt_k: float = 6.0
    tilt_d: float = 0.03
    axial_k: float = 2000.0
    axial_d: float = 4.0


@dataclass(frozen=True)
class Part(Box):
    """A rigid body made of ``boxes`` (part frame, origin at ``pos``). Subclasses Box so yam_arena's task-file
    checks accept it; ``size`` is the boxes' extent and ``lo``/``hi`` its corners in the part frame. A Part with
    ``fixed=True`` is kinematic (it still needs a mass for the checks; the mass is not used)."""

    boxes: tuple[BoxSpec, ...] = ()
    slide: Slide | None = None
    friction: float = 0.9
    fixed: bool = False
    contacts: bool = False  # report contacts (needed by Contacts)
    lo: tuple[float, float, float] = (0.0, 0.0, 0.0)
    hi: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def _check(self) -> None:
        super()._check()
        if not self.boxes or self.mass is None:
            raise ValueError(f"{self.name}: a Part needs boxes and a mass")


def part(name, pos, boxes, mass, yaw=0.0, color=(0.5, 0.5, 0.5), friction=0.9, slide=None, fixed=False,
         contacts=False) -> Part:
    corners = []
    for c, s, _color, *rest in boxes:
        r = rot_z(rest[0] if rest else 0.0)
        corners += [np.add(c, r @ (np.array([a, b, d]) * np.divide(s, 2))) for a in (-1, 1) for b in (-1, 1)
                    for d in (-1, 1)]
    lo, hi = np.min(corners, axis=0), np.max(corners, axis=0)
    return Part(name=name, pos=tuple(pos), size=tuple((hi - lo).tolist()), mass=mass, color=color, yaw=yaw,
                boxes=tuple(boxes), slide=slide, friction=friction, fixed=fixed, contacts=contacts,
                lo=tuple(lo.tolist()), hi=tuple(hi.tolist()))


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


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


@dataclass(frozen=True)
class PartState(ObjectState):
    """A Part after a control step: body origin pose, velocities, world bounding box."""

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
_PATHS: dict[str, str] = {}  # part name -> its prim path in env 0
_JOINTS: dict[str, str] = {}  # part name -> its slide joint's prim path in env 0
# PhysX's D6 joint twists about its frame's x axis: the slide's frames turn z (the hub axis) onto x.
_Z_TO_X = rot_y(-math.pi / 2)


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
    _PATHS.setdefault(spec.name, prim_path)
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=dbl).Set(Gf.Quatd(w, x, y, z))
    material = f"{prim_path}/Looks/physics"
    mat = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    mat.func(material, mat)
    mats = {material}
    looks: dict = {}

    path = f"{prim_path}/body"
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    main = xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(main).CreateKinematicEnabledAttr(bool(spec.fixed))
    UsdPhysics.MassAPI.Apply(main).CreateMassAttr(spec.mass)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(main)
    rb.CreateMaxDepenetrationVelocityAttr(1.0)
    rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateSolverVelocityIterationCountAttr(4)
    if spec.contacts:
        PhysxSchema.PhysxContactReportAPI.Apply(main).CreateThresholdAttr(0.0)
    for i, (centre, size, color, *rest) in enumerate(spec.boxes):
        geom = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
        geom.CreateSizeAttr(1.0)
        geom.AddTranslateOp().Set(Gf.Vec3d(*centre))
        if rest:
            geom.AddRotateZOp().Set(math.degrees(rest[0]))  # rest: (yaw[, friction])
        geom.AddScaleOp().Set(Gf.Vec3f(*size))
        c = geom.GetPrim()
        UsdPhysics.CollisionAPI.Apply(c)
        col = PhysxSchema.PhysxCollisionAPI.Apply(c)
        col.CreateContactOffsetAttr(0.001)
        col.CreateRestOffsetAttr(0.0)
        if color not in looks:
            looks[color] = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.45)
            vis.func(looks[color], vis)
        mu = rest[1] if len(rest) > 1 and rest[1] is not None else None
        own = material if mu is None else f"{prim_path}/Looks/physics_mu{round(mu * 100)}"
        if own not in mats:
            mats.add(own)
            m2 = sim_utils.RigidBodyMaterialCfg(static_friction=mu, dynamic_friction=mu)
            m2.func(own, m2)
        bind_physics_material(str(c.GetPath()), own, stage=stage)
        bind_visual_material(str(c.GetPath()), looks[color], stage=stage)

    s = spec.slide
    if s is not None:
        j = UsdPhysics.Joint.Define(stage, f"{prim_path}/{spec.name}_slide")
        _JOINTS.setdefault(spec.name, str(j.GetPath()))
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path.rsplit('/', 1)[0]}/{s.to}/body")])
        j.CreateBody1Rel().SetTargets([main.GetPath()])
        j.CreateLocalPos0Attr(Gf.Vec3f(*s.pos0))
        j.CreateLocalRot0Attr(_gf_quat(_Z_TO_X, Gf.Quatf))
        j.CreateLocalPos1Attr(Gf.Vec3f(*s.pos1))
        j.CreateLocalRot1Attr(_gf_quat(_Z_TO_X, Gf.Quatf))
        j.CreateExcludeFromArticulationAttr(True)
        prim = j.GetPrim()
        lim = UsdPhysics.LimitAPI.Apply(prim, "transX")  # along the hub axis
        lim.CreateLowAttr(0.0)
        lim.CreateHighAttr(s.travel + 0.02)
        for dof, k, d in (("transX", s.axial_k, s.axial_d), ("transY", s.lateral_k, s.lateral_d),
                          ("transZ", s.lateral_k, s.lateral_d), ("rotY", math.radians(s.tilt_k),
                                                                   math.radians(s.tilt_d)),
                          ("rotZ", math.radians(s.tilt_k), math.radians(s.tilt_d))):
            drive = UsdPhysics.DriveAPI.Apply(prim, dof)  # angular gains are per degree in USD
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(k)
            drive.CreateDampingAttr(d)
            drive.CreateTargetPositionAttr(0.0)
            drive.CreateTargetVelocityAttr(0.0)
        # rotX (spin about the hub axis) has neither limit nor drive: free
    return root.GetPrim()


def _joint_prim(name: str):
    from isaaclab.sim.utils.stage import get_current_stage

    prim = get_current_stage().GetPrimAtPath(_JOINTS[name])
    if not prim.IsValid():
        raise RuntimeError(f"{name}: no slide joint at {_JOINTS[name]}")
    return prim


def set_slide_enabled(name: str, on: bool) -> None:
    """Switch a Part's slide joint on or off (env 0)."""
    from pxr import UsdPhysics

    UsdPhysics.Joint(_joint_prim(name)).GetJointEnabledAttr().Set(bool(on))


def set_slide_target(name: str, travel: float) -> None:
    """Move the axial spring's target (m along the axis) of a Part's slide joint (env 0)."""
    from pxr import UsdPhysics

    UsdPhysics.DriveAPI(_joint_prim(name), "transX").GetTargetPositionAttr().Set(float(travel))


def _arena_object(spec: Part):
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

    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=_Cfg(part_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(spec.pos),
                                    rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))


# -- reading state ------------------------------------------------------------------------------------------------


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def part_state(environment, name: str) -> PartState:
    spec = _SPECS[name]
    base = environment.unwrapped
    asset = base.scene[name]
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0]
    pos = pose[:3] - _np(base.scene.env_origins)[0]
    r = quat_to_matrix(pose[3:7])
    corners = np.array([[a, b, c] for a in (spec.lo[0], spec.hi[0]) for b in (spec.lo[1], spec.hi[1])
                        for c in (spec.lo[2], spec.hi[2])]) @ r.T + pos
    return PartState(pos=pos, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=corners.min(0),
                     high=corners.max(0), R=r)


class Contacts:
    """PhysX contact forces (N) between Part ``name`` and each finger pad of both arms (env 0)."""

    def __init__(self, environment, name: str):
        from isaaclab_physx.physics import PhysxManager

        base = environment.unwrapped
        view = PhysxManager.get_physics_sim_view()
        self.pads = []
        for arm in ARMS:
            paths = base.scene[arm].root_physx_view.link_paths
            paths = list(paths[0] if paths and isinstance(paths[0], (list, tuple)) else paths)
            self.pads += [(arm, p) for p in paths if p.rsplit("/", 1)[-1] in PADS]
        body = f"{_PATHS[name]}/body"
        self.view = view.create_rigid_contact_view([body], filter_patterns=[[p for _, p in self.pads]],
                                                   max_contact_data_count=256)
        self.dt = float(base.physics_dt)

    def forces(self) -> dict[str, list[float]]:
        """Arm -> the contact force on each of its two pads."""
        fm = _np(self.view.get_contact_force_matrix(dt=self.dt)).reshape(-1, 3)
        out: dict[str, list[float]] = {a: [] for a in ARMS}
        for (arm, _), f in zip(self.pads, fm):
            out[arm].append(float(np.linalg.norm(f)))
        return out


# -- hooks ------------------------------------------------------------------------------------------------------

_PHYSICS: list = []
_ENV_RESET: list = []
_RESET: list = []


def on_physics(fn):
    _PHYSICS.append(fn)
    return fn


def on_env_reset(fn):
    _ENV_RESET.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def _patch_scene(scene) -> None:
    if getattr(scene, "_bladekit", False):
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
        out.update({n: part_state(environment, n) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._bladekit = _task_object, _object_states, True


def _hook_env(env) -> None:
    base = env.unwrapped
    isc = base.scene
    if getattr(isc, "_bladekit", False):
        return
    write, reset = isc.write_data_to_sim, env.reset

    def write_data_to_sim(*a, **k):
        for fn in _PHYSICS:
            fn(env)
        return write(*a, **k)

    def _reset(*a, **k):
        out = reset(*a, **k)
        for fn in _ENV_RESET:
            fn(env)
        return out

    isc.write_data_to_sim, env.reset, isc._bladekit = write_data_to_sim, _reset, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_bladekit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def _ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_env(self._env)
        return out

    def _reset(self, *a, **k):
        obs = reset(self, *a, **k)
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._ensure_sim, cls.reset, cls._bladekit = _ensure_sim, _reset, True


def _patch_scripted(module) -> None:
    """``yam_scripted`` plays the task file's closed-loop ``controller`` when it has one."""
    cls = module.YamScriptedPolicy
    if getattr(cls, "_bladekit", False):
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

    cls.reset, cls.act, cls._bladekit = _reset, _act, True


def _patch_demo(demo) -> None:
    """A ``Waypoint.yaw`` may be a 3x3 world rotation matrix (any grip, not only straight down)."""
    if getattr(demo, "_bladekit", False):
        return
    grip_rotation = demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else grip_rotation(yaw)

    demo.grip_rotation, demo._bladekit = _grip_rotation, True


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
