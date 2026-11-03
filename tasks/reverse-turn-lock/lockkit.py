"""lockkit: a loose box with a key on a revolute joint and a lid on a prismatic joint, for yam_arena task files.

yam_arena task files only take rigid primitives. This module adds one articulated object, :class:`LockBox`, and
hooks yam_arena (without modifying it) so that:

- ``yam_arena.scene.task_object`` builds a LockBox as a FLOATING-base articulation. The root is the box body, which
  rests on its feet. Its children are the ``key_turn`` joint (key axis along the box's z) and the ``lid_slide``
  joint.
- Before every PHYSICS step (240 Hz, via a wrapper on ``InteractiveScene.write_data_to_sim``), a :class:`Lock`
  turns the key angle and lid travel into joint efforts: the resistance curve, the wrong-way stop with its
  shear-pin damage latch, lid friction and the lid catch. Joint efforts act between child and parent, so every
  reaction torque and force reaches the box. A one-handed turn or pull moves the box unless something holds it.
- ``yam_arena.scene.object_states`` reports a :class:`LockBoxState`: the box pose, key angle, lid travel, the
  lock's latches and the box's worst tilt and drop so far. The task file's ``success()`` / ``score()`` read it.
- Each embodiment reset resets the latches.

Angles in this module are in the UNLOCK sense: ``u > 0`` turns toward unlocking. With ``LockParams.reversed`` (the
default), unlocking is clockwise seen from above, which is the joint's negative direction.

The top half (``LockParams``, ``resistance_nm``, ``Lock``) is plain NumPy, and it is what the tests exercise.
Everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import Box, ObjectState

KEY_JOINT, LID_JOINT = "key_turn", "lid_slide"


# -- the lock's mechanics (pure Python) ---------------------------------------------------------------------------


@dataclass(frozen=True)
class LockParams:
    """Every number of the lock. Angles in degrees in the unlock sense, torques in N m, forces in N."""

    # Resistance curve: the friction torque limit vs key angle, piecewise linear between (deg, N m) points: a
    # breakaway peak (a stiff lock "unsticking"), steady travel, then the cam pushing the bolt back near the end.
    curve: tuple[tuple[float, float], ...] = ((0.0, 0.20), (6.0, 0.40), (15.0, 0.30), (70.0, 0.30), (90.0, 0.40))
    reversed: bool = True  # unlock clockwise seen from above (False: the conventional counter-clockwise)
    stop_deg: float = 0.0  # wrong-way stop: the key starts against it
    stop_stiffness: float = 12.0  # N m/rad: the stop and key shank's give before the pin shears
    break_nm: float = 0.6  # shear-pin strength: stop torque above this ...
    break_hold_steps: int = 3  # ... for this many physics steps in a row (12.5 ms at 240 Hz) = damage
    unlock_deg: float = 75.0  # the cam clears the lid's notch from here on
    open_end_deg: float = 95.0  # solid end stop on the unlock side (a joint limit)
    broken_travel_deg: float = 45.0  # how far the decoupled key can swing the wrong way once the pin has sheared
    stick_stiffness: float = 20.0  # N m/rad: pre-slip stiffness of the key friction (1.1 deg of give at 0.4 N m)
    lid_friction_n: float = 3.0  # N: rail friction of the lid
    lid_stick_stiffness: float = 2000.0  # N/m: its pre-slip stiffness (1.5 mm of give)
    catch_stiffness: float = 3000.0  # N/m: the cam holding a locked lid (30 N per cm)
    catch_engage_m: float = 0.005  # the cam drops into the notch when the lid is closer to shut than this
    pry_m: float = 0.01  # a locked lid pulled further open than this means the catch was pried (damage)
    key_damping: float = 0.1  # N m s/rad, implicit joint damping
    key_armature: float = 0.002  # kg m^2, keeps the 240 Hz explicit torque update stable
    lid_damping: float = 10.0  # N s/m
    lid_armature: float = 0.05  # kg


def resistance_nm(u_deg: float, p: LockParams = LockParams()) -> float:
    """Friction torque limit (N m) at key angle ``u_deg``; flat beyond the curve's ends."""
    deg, nm = zip(*p.curve, strict=True)
    return float(np.interp(u_deg, deg, nm))


def _stick(anchor: float, x: float, stiffness: float, limit: float) -> tuple[float, float]:
    """Elasto-plastic friction: a stiff spring to an anchor that slips once the spring force exceeds ``limit``.
    Returns (force, new anchor)."""
    force = stiffness * (anchor - x)
    if abs(force) > limit:
        force = math.copysign(limit, force)
        anchor = x + force / stiffness
    return force, anchor


class Lock:
    """The lock's state and per-physics-step efforts. Call :meth:`step` with the key angle (deg, unlock sense) and the
    lid travel (m); it returns (key torque in the unlock sense in N m, lid force along its travel in N) and updates
    the latches. :meth:`track_box` keeps the box's worst tilt and drop."""

    def __init__(self, params: LockParams = LockParams()):
        self.p = params
        self.reset()

    def reset(self) -> None:
        self.anchor = 0.0  # rad: where the key friction is "stuck"
        self.lid_anchor = 0.0  # m
        self.over = 0  # physics steps in a row above the shear strength
        self.peak_stop_nm = 0.0
        self.damaged = False  # shear pin broken: the key no longer drives the cam
        self.pried = False  # locked lid forced open past pry_m
        self.unlocked = False  # the cam has cleared the notch at some point
        self.catch = True  # the cam is in the lid's notch
        self.max_tilt_deg = 0.0  # box
        self.max_drop_m = 0.0

    def step(self, u_deg: float, lid_m: float) -> tuple[float, float]:
        p, u = self.p, math.radians(u_deg)
        tau = 0.0
        if not self.damaged:
            pen = math.radians(p.stop_deg) - u
            stop = p.stop_stiffness * pen if pen > 0 else 0.0
            self.peak_stop_nm = max(self.peak_stop_nm, stop)
            self.over = self.over + 1 if stop > p.break_nm else 0
            self.damaged = self.over >= p.break_hold_steps
        if not self.damaged:
            spring, self.anchor = _stick(self.anchor, u, p.stick_stiffness, resistance_nm(u_deg, p))
            tau = stop + spring
        released = not self.damaged and u_deg >= p.unlock_deg
        self.unlocked |= released
        if released:
            self.catch = False
        elif lid_m < p.catch_engage_m:
            self.catch = True
        force, self.lid_anchor = _stick(self.lid_anchor, lid_m, p.lid_stick_stiffness, p.lid_friction_n)
        if self.catch:
            force -= p.catch_stiffness * max(lid_m, 0.0)
        self.pried |= self.catch and lid_m > p.pry_m
        return tau, force

    def track_box(self, tilt_deg: float, drop_m: float) -> None:
        self.max_tilt_deg = max(self.max_tilt_deg, tilt_deg)
        self.max_drop_m = max(self.max_drop_m, drop_m)


# -- the task-file object ------------------------------------------------------------------------------------------

# (centre, size, colour) or (centre, size, colour, yaw_deg), in the frame of the body it belongs to (m)
BoxSpec = tuple


@dataclass(frozen=True)
class LockBox(Box):
    """A loose box (``pos`` is its body origin, on the table under its centre; ``size`` its bounding box; ``mass`` the
    box body's mass) standing on ``feet`` (their own friction), a key turning about the body's z axis at
    ``key_pos``, and a lid sliding ``lid_dir`` (+1 or -1) along the body's y axis from ``lid_pos``."""

    body_boxes: tuple[BoxSpec, ...] = ()
    feet: tuple[BoxSpec, ...] = ()
    marks: tuple[BoxSpec, ...] = ()  # visual only (the dial)
    key_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    key_boxes: tuple[BoxSpec, ...] = ()
    key_mass: float = 0.03
    lid_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lid_boxes: tuple[BoxSpec, ...] = ()
    lid_mass: float = 0.12
    lid_travel: float = 0.10
    lid_dir: float = 1.0
    friction: float = 1.0  # walls, key, lid: grippy PLA
    feet_friction: float = 0.3  # feet on the table (PhysX "min" combine, so this is the table contact)
    params: LockParams = LockParams()

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a LockBox is loose; give the box body a mass")
        if not (self.body_boxes and self.feet and self.key_boxes and self.lid_boxes):
            raise ValueError(f"{self.name}: body, feet, key and lid each need at least one box")
        if self.lid_dir not in (1.0, -1.0):
            raise ValueError(f"{self.name}: lid_dir must be +1 or -1")

    @property
    def total_mass(self) -> float:
        return self.mass + self.key_mass + self.lid_mass

    def key_limits_deg(self) -> tuple[float, float]:
        """The key joint's limits in its own (counter-clockwise positive) sense."""
        p, s = self.params, _sign(self.params)
        return tuple(sorted((-s * p.broken_travel_deg, s * p.open_end_deg)))

    def lid_limits_m(self) -> tuple[float, float]:
        return tuple(sorted((0.0, self.lid_dir * self.lid_travel)))


def _sign(p: LockParams) -> float:
    """Joint angle (counter-clockwise about +z) = sign * unlock angle."""
    return -1.0 if p.reversed else 1.0


@dataclass(frozen=True)
class LockBoxState(ObjectState):
    """The box body's pose (``pos`` is its origin), plus the key angle (deg, unlock sense), the lid travel (m), the
    lock's latches and the box's worst tilt (deg) and drop below its start height (m) so far."""

    key_deg: float = 0.0
    lid_m: float = 0.0
    unlocked: bool = False
    damaged: bool = False
    pried: bool = False
    peak_stop_nm: float = 0.0
    max_tilt_deg: float = 0.0
    max_drop_m: float = 0.0

    def yaw_deg(self) -> float:
        x, y, z, w = self.quat_xyzw
        return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))

    def tilt_deg(self) -> float:
        x, y, _, _ = self.quat_xyzw
        return math.degrees(math.acos(min(1.0, max(-1.0, 1 - 2 * (x * x + y * y)))))


# -- Isaac Sim side ------------------------------------------------------------------------------------------------

_SPECS: dict[str, LockBox] = {}
_LOCKS: dict[str, Lock] = {}
_DEBUG = [0] if __import__("os").environ.get("LOCK_DEBUG") else []  # LOCK_DEBUG=1: log the lock every 10 steps


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the lock box under ``prim_path`` (pxr only)."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.lock_name]
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=dbl).Set(Gf.Quatd(w, x, y, z))

    def material(name: str, mu: float, combine: str) -> str:
        path = f"{prim_path}/Looks/{name}"
        cfg_ = sim_utils.RigidBodyMaterialCfg(static_friction=mu, dynamic_friction=mu, friction_combine_mode=combine)
        cfg_.func(path, cfg_)
        return path

    grippy = material("grippy", spec.friction, "average")
    feet = material("feet", spec.feet_friction, "min")
    looks: dict = {}

    def look(color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def body(name: str, pos, mass: float) -> str:
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

    def boxes(path: str, specs, mat: str | None, tag: str) -> None:
        for i, (centre, size, color, *yaw) in enumerate(specs):
            cube = UsdGeom.Cube.Define(stage, f"{path}/{tag}_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
            if yaw:
                cube.AddRotateZOp().Set(float(yaw[0]))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            prim = cube.GetPrim()
            if mat is not None:
                UsdPhysics.CollisionAPI.Apply(prim)
                cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                cp.CreateContactOffsetAttr(0.003)
                cp.CreateRestOffsetAttr(0.0)
                bind_physics_material(str(prim.GetPath()), mat, stage=stage)
            bind_visual_material(str(prim.GetPath()), look(tuple(color)), stage=stage)

    box = body("box", (0.0, 0.0, 0.0), spec.mass)
    boxes(box, spec.body_boxes, grippy, "box")
    boxes(box, spec.feet, feet, "foot")
    boxes(box, spec.marks, None, "mark")
    key = body("key", spec.key_pos, spec.key_mass)
    boxes(key, spec.key_boxes, grippy, "box")
    lid = body("lid", spec.lid_pos, spec.lid_mass)
    boxes(lid, spec.lid_boxes, grippy, "box")

    def joint(cls, name, child, pos, axis, limits, drive):
        j = cls.Define(stage, f"{prim_path}/{name}")
        j.CreateAxisAttr(axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(box)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*pos))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(limits[0])
        j.CreateUpperLimitAttr(limits[1])
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), drive)  # gains come from the actuator config
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(0.0)

    joint(UsdPhysics.RevoluteJoint, KEY_JOINT, key, spec.key_pos, "Z", spec.key_limits_deg(), "angular")  # degrees
    joint(UsdPhysics.PrismaticJoint, LID_JOINT, lid, spec.lid_pos, "Y", spec.lid_limits_m(), "linear")

    # Floating base: the articulation root is the box body itself.
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(box))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(box))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _arena_object(spec: LockBox):
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
    class LockSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn)
        lock_name: str = ""

    class LockObject(Object):
        def _generate_articulation_cfg(self):
            # No stiffness: the joints are passive. Damping and armature here; the lock's efforts come as effort
            # targets from _apply() every physics step.
            actuators = {
                "key": ImplicitActuatorCfg(
                    joint_names_expr=[KEY_JOINT], stiffness=0.0, damping=p.key_damping, armature=p.key_armature
                ),
                "lid": ImplicitActuatorCfg(
                    joint_names_expr=[LID_JOINT], stiffness=0.0, damping=p.lid_damping, armature=p.lid_armature
                ),
            }
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return LockObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=LockSpawnerCfg(lock_name=spec.name),
        initial_pose=Pose(
            position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))
        ),
    )


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)  # Isaac Lab 3.0 data are warp arrays with a .torch view
    return x.detach().cpu().numpy().astype(np.float64)


def _joints(base, spec: LockBox):
    """(asset, key index, lid index, key angle in deg unlock sense, lid travel in m)."""
    asset = base.scene[spec.name]
    names = list(asset.joint_names)
    ik, il = names.index(KEY_JOINT), names.index(LID_JOINT)
    q = _np(asset.data.joint_pos)[0]
    return asset, ik, il, _sign(spec.params) * math.degrees(q[ik]), spec.lid_dir * float(q[il])


def _apply(base) -> None:
    """One physics step of the lock: key torque and lid force as joint effort targets (they act between the joint's
    two bodies, so the box gets the reaction)."""
    import torch

    for name, spec in _SPECS.items():
        asset, ik, il, u_deg, lid_m = _joints(base, spec)
        tau, force = _LOCKS[name].step(u_deg, lid_m)
        effort = torch.zeros((1, asset.num_joints), dtype=torch.float32, device=base.device)
        effort[0, ik] = _sign(spec.params) * tau
        effort[0, il] = spec.lid_dir * force
        asset.actuators.target_command.set_effort_index(value=effort)  # Isaac Lab 3.0, keyword-only


def _state(base, spec: LockBox) -> LockBoxState:
    asset, _, _, u_deg, lid_m = _joints(base, spec)
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    pos = pose[:3] - _np(base.scene.env_origins)[0]
    lock = _LOCKS[spec.name]
    probe = LockBoxState(pos, pose[3:7], vel[:3], vel[3:6], pos, pos)
    lock.track_box(probe.tilt_deg(), spec.pos[2] - pos[2])
    half = np.asarray(spec.size, float) / 2  # bounds: world-aligned box of the unrotated size (approximate)
    if _DEBUG:
        _DEBUG[0] += 1
        if _DEBUG[0] % 10 == 0:
            print(f"[lockkit] {_DEBUG[0]:5d} key {u_deg:7.2f} deg  lid {lid_m * 100:5.2f} cm  box {pos.round(3)} "
                  f"yaw {probe.yaw_deg():6.2f} tilt {probe.tilt_deg():5.2f}  stop_peak {lock.peak_stop_nm:.2f}  "
                  f"unlocked {lock.unlocked} damaged {lock.damaged} pried {lock.pried}", flush=True)
    return LockBoxState(
        pos=pos, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=pos - half * (1, 1, 0),
        high=pos + half * (1, 1, 2), key_deg=u_deg, lid_m=lid_m, unlocked=lock.unlocked, damaged=lock.damaged,
        pried=lock.pried, peak_stop_nm=lock.peak_stop_nm, max_tilt_deg=lock.max_tilt_deg, max_drop_m=lock.max_drop_m,
    )  # fmt: skip


# -- hooking yam_arena ---------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_lockkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def lock_task_object(spec):
        if isinstance(spec, LockBox):
            _SPECS[spec.name] = spec
            _LOCKS.setdefault(spec.name, Lock(spec.params))
            return _arena_object(spec)
        return task_object(spec)

    def lock_object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _state(environment.unwrapped, _SPECS[n]) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = lock_task_object, lock_object_states
    scene._lockkit = True


def _hook_physics(environment) -> None:
    """Run the lock before every physics step: the env calls scene.write_data_to_sim() once per substep."""
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_lockkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        _apply(base)
        return write(*a, **k)

    isc.write_data_to_sim = write_data_to_sim
    isc._lockkit = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_lockkit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def lock_ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
        return out

    def lock_reset(self, *a, **k):
        # env.reset() inside puts the box, key and lid back to their defaults; the settle steps after it already run
        # with fresh latches. The settle can let the box sink a fraction of a mm onto its feet, well inside the
        # scoring margins.
        for lock in _LOCKS.values():
            lock.reset()
        return reset(self, *a, **k)

    cls._ensure_sim, cls.reset = lock_ensure_sim, lock_reset
    cls._lockkit = True


class _Finder(importlib.abc.MetaPathFinder):
    """Patch yam_arena.scene (imported only after Isaac Sim starts) and the embodiment as they are imported."""

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
    """Hook yam_arena: modules already imported now, the others when they are. Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
__all__ = ["Lock", "LockBox", "LockBoxState", "LockParams", "resistance_nm"]
