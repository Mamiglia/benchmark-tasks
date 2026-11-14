"""mixkit: a test-tube rack with two open tubes of layered liquid, for a yam_arena task file.

yam_arena task files only take single primitives. This module adds one object, :class:`TubeRack`, spawned as a static
(``BASE``) Arena asset holding three bodies:

- ``rack``: a kinematic (fixed) wooden rack made of boxes: a base board, a top board with square holes, two end posts.
- ``tube_0``, ``tube_1``: one rigid body each, a 16 x 100 mm glass test tube (cylinder collider, mass of the glass
  and the liquid). The liquid is drawn inside as two stacked cylinders (dark blue copper(II) sulfate solution under
  clear water); the glass is drawn as thin edges (a rim, a foot ring and four hairlines) so the liquid shows. Five
  sets of liquid cylinders, one per mixing stage, are switched visible as the layers mix (:func:`stage_for`).

Before every physics step (240 Hz, via a wrapper on ``InteractiveScene.write_data_to_sim``) the kit reads each tube's
pose and steps its :class:`liquid.TubeLiquid`: the liquid's height on the wall, the spill latch, the mixing contrast.

``object_states`` reports a :class:`RackState`: per tube its pose, axis, tilt, bottom point, liquid state, the finger
pads' positions and how far each gripper is open.

Importing the module also hooks ``yam_arena.scripted``: when the task file defines ``controller(bases, dt)``,
``yam_scripted`` asks that object for every action, passing it the observation (camera images and joint positions),
instead of playing fixed waypoints. yam_arena itself is not modified. Pure Python at import; Isaac Sim is imported
lazily.
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
from liquid import LiquidParams, TubeLiquid  # noqa: E402

Color = tuple[float, float, float]
ARMS = ("left_robot", "right_robot")
FINGER_OPEN = -0.04695  # yam_arena.constants.FINGER_OPEN
TCP_OFFSET = 0.134  # yam_arena.constants.TCP_OFFSET, on the gripper link's z axis
N_STAGES = 5
CUSO4, WATER = (0.05, 0.30, 0.78), (0.86, 0.91, 0.95)  # sRGB: 0.5 M copper(II) sulfate; water
GLASS = (0.80, 0.84, 0.86)


def stage_for(contrast: float) -> int:
    """Which drawn mixing stage (0: two layers .. 4: one even colour) shows a contrast."""
    return 4 - int(np.searchsorted([0.1, 0.3, 0.55, 0.8], contrast, side="left"))


def stage_colors(stage: int) -> tuple[Color, Color]:
    """(lower, upper) colour of a stage: both move toward the mixed colour, the even blend of the two."""
    k = stage / (N_STAGES - 1)
    mix = tuple((a + b) / 2 for a, b in zip(CUSO4, WATER))
    return (tuple(a + k * (m - a) for a, m in zip(CUSO4, mix)), tuple(b + k * (m - b) for b, m in zip(WATER, mix)))


@dataclass(frozen=True)
class TubeRack(Box):
    """Positions in the env frame (x forward, y left, z up, m). ``pos``: the rack's base centre on the table."""

    tube_xy: tuple = ((0.26, 0.08), (0.26, -0.08))  # tube centres (in the rack's holes)
    tube_radius: float = 0.008  # 16 mm outside
    tube_len: float = 0.100
    glass_mass: float = 0.0095  # 16 x 100 mm borosilicate tube
    liquid: LiquidParams = field(default_factory=LiquidParams)
    rack_boxes: tuple = ()  # (centre, size, colour), env frame, kinematic
    floor_z: float = 0.0  # where a racked tube's bottom stands (top of the base board)
    table_z: float = 0.0
    tube_friction: float = 1.2  # glass on the rubber finger pads
    rack_friction: float = 0.6

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a TubeRack is a fixed asset (mass None); its bodies carry their own")

    @property
    def liquid_mass(self) -> float:
        p = self.liquid
        return math.pi * p.radius**2 * (p.lower * 1080.0 + p.upper * 1000.0)  # 0.5 M CuSO4 about 1.08 g/mL


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def surface_gap(p, centre, axis, length: float, radius: float) -> float:
    """Distance from point ``p`` to a closed cylinder's surface (0 inside)."""
    v = np.asarray(p, float) - np.asarray(centre, float)
    t = float(v @ axis)
    radial = float(np.linalg.norm(v - t * np.asarray(axis)))
    out_r, out_t = max(0.0, radial - radius), max(0.0, abs(t) - length / 2)
    return math.hypot(out_r, out_t)


@dataclass(frozen=True)
class TubeState:
    pos: np.ndarray  # centre
    axis: np.ndarray  # unit, bottom -> mouth
    bottom: np.ndarray
    mouth: np.ndarray
    tilt_deg: float  # from upright
    lin_vel: np.ndarray
    ang_vel: np.ndarray
    spilled: bool
    contrast: float
    mixed: bool
    strokes: int
    wall_mm: float  # how high the liquid stands on the wall now (mm from the bottom)
    peak_wall_mm: float


@dataclass(frozen=True)
class RackState(ObjectState):
    tubes: tuple = ()  # TubeState per tube
    pads: dict = field(default_factory=dict)  # arm -> (3, 3) finger pad tips and tool point
    open_frac: dict = field(default_factory=dict)  # arm -> 0 (closed) .. 1 (open)


# -- spawning (Isaac Sim only) ----------------------------------------------------------------------------------

_SPECS: dict[str, TubeRack] = {}
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

    def material(name, mu) -> str:
        path = f"{prim_path}/Looks/{name}"
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=mu, dynamic_friction=mu)
        mat.func(path, mat)
        return path

    def look(color) -> str:
        color = tuple(round(c, 4) for c in color)
        if color not in looks:
            looks[color] = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.3)
            vis.func(looks[color], vis)
        return looks[color]

    def collider(prim, mat) -> None:
        UsdPhysics.CollisionAPI.Apply(prim)
        col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        col.CreateContactOffsetAttr(0.002)
        col.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), mat, stage=stage)

    def body(name, pos, kinematic=False) -> tuple[str, object]:
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*pos))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(kinematic)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateAngularDampingAttr(0.05)
        return path, prim

    def cyl(path, radius, height, z, color, collide=None):
        c = UsdGeom.Cylinder.Define(stage, path)
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(radius)
        c.CreateHeightAttr(height)
        c.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z))
        if collide:
            collider(c.GetPrim(), collide)
        if color is not None:
            bind_visual_material(str(c.GetPath()), look(color), stage=stage)
        return c

    # rack (fixed)
    rack_pos = np.mean([b[0] for b in s.rack_boxes], axis=0)
    rack, _ = body("rack", tuple(rack_pos), kinematic=True)
    rmat = material("rack", s.rack_friction)
    for i, (centre, size, color) in enumerate(s.rack_boxes):
        cube = UsdGeom.Cube.Define(stage, f"{rack}/box_{i}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*np.subtract(centre, rack_pos)))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        collider(cube.GetPrim(), rmat)
        bind_visual_material(str(cube.GetPath()), look(tuple(color)), stage=stage)

    # tubes: body origin at the tube's centre
    p, L, r = s.liquid, s.tube_len, s.tube_radius
    tmat = material("tube", s.tube_friction)
    for k, (tx, ty) in enumerate(s.tube_xy):
        tube, prim = body(f"tube_{k}", (tx, ty, s.floor_z + L / 2 + 0.0005))
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(s.glass_mass + s.liquid_mass)
        m.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        shell = cyl(f"{tube}/shell", r, L, 0.0, None, collide=tmat)
        col = PhysxSchema.PhysxCollisionAPI(shell.GetPrim())  # rubber finger pads grip over a patch, not a line:
        col.CreateTorsionalPatchRadiusAttr(0.006)  # torsional friction over a 6 mm patch keeps the tube from
        col.CreateMinTorsionalPatchRadiusAttr(0.006)  # pivoting between the pads
        UsdGeom.Imageable(shell.GetPrim()).MakeInvisible()  # glass: collides, drawn as edges below
        cyl(f"{tube}/rim", r + 0.0008, 0.003, L / 2 - 0.0015, GLASS)
        cyl(f"{tube}/foot", r, 0.002, -L / 2 + 0.001, GLASS)
        for j in range(4):
            a = j * math.pi / 2 + math.pi / 4
            h = UsdGeom.Cylinder.Define(stage, f"{tube}/edge_{j}")
            h.CreateAxisAttr("Z")
            h.CreateRadiusAttr(0.0004)
            h.CreateHeightAttr(L)
            h.AddTranslateOp().Set(Gf.Vec3d(r * math.cos(a), r * math.sin(a), 0.0))
            bind_visual_material(str(h.GetPath()), look(GLASS), stage=stage)
        bottom = -L / 2 + 0.001  # 1 mm glass bottom
        for st in range(N_STAGES):
            lo, up = stage_colors(st)
            g = UsdGeom.Xform.Define(stage, f"{tube}/liquid_{st}")
            cyl(f"{tube}/liquid_{st}/lower", p.radius - 0.0003, p.lower, bottom + p.lower / 2, lo)
            cyl(f"{tube}/liquid_{st}/upper", p.radius - 0.0003, p.upper, bottom + p.lower + p.upper / 2, up)
            if st:
                UsdGeom.Imageable(g.GetPrim()).MakeInvisible()
    return root.GetPrim()


def _arena_object(spec: TubeRack):
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


# -- runtime ----------------------------------------------------------------------------------------------------


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


class _Runtime:
    """PhysX view, liquids and drawn stages for one rack (env 0)."""

    def __init__(self, base, spec: TubeRack):
        from isaaclab_physx.physics import PhysxManager

        self.spec, self.base = spec, base
        self.root = _PATHS[spec.name]
        self.nt = len(spec.tube_xy)
        view = PhysxManager.get_physics_sim_view()
        self.body = view.create_rigid_body_view([f"{self.root}/tube_{k}" for k in range(self.nt)])
        self.dt = float(base.physics_dt)
        self.start = _arr(self.body.get_transforms()).copy()
        self.origin = _arr(base.scene.env_origins)[0]
        self.debug = bool(os.environ.get("MIX_DEBUG"))
        self.liquids = [TubeLiquid(spec.liquid) for _ in range(self.nt)]
        self.shown = [0] * self.nt
        self.n = 0
        self.pad_ids = {arm: [base.scene[arm].find_bodies(b)[0][0] for b in ("tip_left", "tip_right", "gripper")]
                        for arm in ARMS}
        self.finger_ids = {arm: base.scene[arm].find_joints(["joint7", "joint8"])[0] for arm in ARMS}

    def reset(self) -> None:
        cur = self.body.get_transforms()
        idx = np.arange(self.nt, dtype=np.int32)
        self.body.set_transforms(_like(cur, self.start), _like(cur, idx, "int32"))
        self.body.set_velocities(_like(cur, np.zeros((self.nt, 6))), _like(cur, idx, "int32"))
        self.clear()

    def clear(self) -> None:
        for liq in self.liquids:
            liq.reset()
        self.n = 0
        self._show()

    def _poses(self):
        tr = _arr(self.body.get_transforms())
        return [(t[:3] - self.origin, quat_to_matrix(t[3:7]), t[3:7]) for t in tr]

    def physics_step(self) -> None:
        self.n += 1
        L = self.spec.tube_len
        for k, (p, R, _) in enumerate(self._poses()):
            liq = self.liquids[k]
            was = liq.spilled
            liq.step(p - R[:, 2] * (L / 2 - 0.001), R, self.dt)
            if liq.spilled and not was:
                tilt = math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1))))
                print(f"[mixkit] tube {k} SPILLED at {self.n * self.dt:.2f} s: liquid reached the mouth "
                      f"(wall {liq.wall * 1000:.0f} mm, tilt {tilt:.0f} deg)", flush=True)

    def _show(self) -> None:
        """Switch each tube's drawn liquid to the stage its contrast is at."""
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdGeom

        stage = get_current_stage()
        for k, liq in enumerate(self.liquids):
            st = stage_for(liq.contrast)
            if st == self.shown[k]:
                continue
            for j in range(N_STAGES):
                img = UsdGeom.Imageable(stage.GetPrimAtPath(f"{self.root}/tube_{k}/liquid_{j}"))
                img.MakeVisible() if j == st else img.MakeInvisible()
            self.shown[k] = st

    def state(self) -> RackState:
        s = self.spec
        self._show()
        vel = _arr(self.body.get_velocities())
        tubes = []
        for k, (p, R, q) in enumerate(self._poses()):
            ax = R[:, 2]
            liq = self.liquids[k]
            tubes.append(TubeState(pos=p, axis=ax, bottom=p - ax * s.tube_len / 2, mouth=p + ax * s.tube_len / 2,
                                   tilt_deg=math.degrees(math.acos(float(np.clip(ax[2], -1, 1)))),
                                   lin_vel=vel[k, :3], ang_vel=vel[k, 3:], spilled=liq.spilled,
                                   contrast=liq.contrast, mixed=liq.mixed, strokes=liq.strokes,
                                   wall_mm=liq.wall * 1000, peak_wall_mm=liq.peak_wall * 1000))
        pads, opened = {}, {}
        for arm in ARMS:
            asset = self.base.scene[arm]
            poses = _arr(asset.data.body_link_pose_w)[0]
            tips = [poses[i, :3] - self.origin for i in self.pad_ids[arm][:2]]
            g = poses[self.pad_ids[arm][2]]
            tcp = g[:3] - self.origin + quat_to_matrix(g[3:7])[:, 2] * TCP_OFFSET
            pads[arm] = np.array([*tips, tcp])
            fingers = _arr(asset.data.joint_pos)[0, self.finger_ids[arm]]
            opened[arm] = float(np.clip(fingers.mean() / FINGER_OPEN, 0.0, 1.0))
        lo = np.min([t.pos for t in tubes], 0) - 0.05
        hi = np.max([t.pos for t in tubes], 0) + 0.05
        st = RackState(pos=tubes[0].pos, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=tubes[0].lin_vel,
                       ang_vel=tubes[0].ang_vel, low=lo, high=hi, tubes=tuple(tubes), pads=pads, open_frac=opened)
        if self.debug and self.n % 120 < 12:
            for k, t in enumerate(tubes):
                print(f"[mixkit] t {self.n * self.dt:6.2f} tube {k} pos {np.round(t.pos, 3)} tilt {t.tilt_deg:5.1f} "
                      f"wall {t.wall_mm:5.1f} (peak {t.peak_wall_mm:5.1f}) mm strokes {t.strokes} contrast "
                      f"{t.contrast:.3f} spilled {t.spilled} open {opened}", flush=True)
        return st


_RT: dict[str, _Runtime] = {}


def _runtime(base, name: str) -> _Runtime:
    if name not in _RT:
        _RT[name] = _Runtime(base, _SPECS[name])
    return _RT[name]


# -- hooks ------------------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_mixkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, TubeRack):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _runtime(environment.unwrapped, n).state() for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._mixkit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_mixkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for name in _SPECS:
            _runtime(base, name).physics_step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._mixkit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_mixkit", False):
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
            _runtime(self._base, name).reset()
        obs = reset(self, scene, seed=seed)
        for name in _SPECS:
            _runtime(self._base, name).clear()  # the settle steps are not part of the trial
        return obs

    cls._ensure_sim, cls.reset, cls._mixkit = _ensure_sim, _reset, True


def _patch_scripted(module) -> None:
    """``yam_scripted`` plays the task file's closed-loop ``controller`` when it has one."""
    cls = module.YamScriptedPolicy
    if getattr(cls, "_mixkit", False):
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

    cls.reset, cls.act, cls._mixkit = _reset, _act, True


def _patch_demo(demo) -> None:
    if getattr(demo, "_mixkit", False):
        return
    grip_rotation = demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else grip_rotation(yaw)

    demo.grip_rotation, demo._mixkit = _grip_rotation, True


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
