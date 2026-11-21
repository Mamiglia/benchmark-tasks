"""smearkit: a base slide with a drop on it, a spreader slide in a stand and bench paper, for a yam_arena task file.

yam_arena task files only take single primitives. This module adds one object, :class:`SmearRig`, spawned as a static
(``BASE``) Arena asset holding these bodies:

- ``bench`` (kinematic): the bench paper (white, taped down) and the slotted stand the spreader stands in.
- ``base`` (dynamic): the base slide, 75 x 25 x 1 mm glass with a 20 mm frosted end, lying on the paper. It carries
  the drop and the drawn film (look only, no collider).
- ``spreader`` (dynamic): the spreader slide, 75 x 25 x 1 mm glass, standing on its short end in the stand. Body
  frame: origin at the centre of its lower end (the spreading edge), +z up the slide, x through its thickness.

Before every physics step (240 Hz, a wrapper on ``InteractiveScene.write_data_to_sim``) the kit reads the spreader's
pose and the PhysX contact force between the spreader and the base slide, and steps :class:`smearmodel.Smear`: the
edge's place along the base slide, contact (force 0.02 N or more), the spreader's angle to the slide. The film is
drawn as 1.5 mm segments (three shades by thickness) under the base slide's body.

``object_states`` reports a :class:`SmearState`. Importing the module also hooks ``yam_arena.scripted``: when the task
file defines ``controller(bases, dt)``, ``yam_scripted`` asks it for every action, passing it the observation (camera
images and joint positions). yam_arena itself is not modified. Pure Python at import; Isaac Sim is imported lazily.
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
from smearmodel import Law, Smear  # noqa: E402

Color = tuple[float, float, float]
SEG = 0.0015  # drawn film segment length
N_SEG = 50
SHADES = ((0.55, 0.04, 0.06), (0.78, 0.22, 0.24), (0.93, 0.62, 0.62))  # thick, mid, feathered


@dataclass(frozen=True)
class SmearRig(Box):
    """Positions in the env frame (x forward, y left, z up, m). ``pos``: the base slide's centre on the paper."""

    table_z: float = 0.0
    slide: tuple[float, float, float] = (0.075, 0.025, 0.001)  # both slides: length, width, thickness
    frost: float = 0.020  # frosted band at the base slide's near end
    glass_mass: float = 0.0047
    stand_xy: tuple[float, float] = (0.30, -0.20)  # the spreader's slot centre
    stand_size: tuple[float, float, float] = (0.024, 0.040, 0.015)  # x, y, z
    slot: tuple[float, float] = (0.0016, 0.012)  # slot width (x) and depth
    paper: tuple[float, float, float] = (0.30, 0.30, 0.0003)  # bench paper, centred on (paper_xy)
    paper_xy: tuple[float, float] = (0.30, -0.12)
    grip_zone: float = 0.020  # the top of the spreader the fingers hold ...
    grip_collider: float = float(os.environ.get("SMEAR_GRIPC", "0.003"))  # ... collided this thick (see _spawn)
    mu_glass: float = 0.15  # glass on glass lubricated by the film (both slides' material; PhysX averages)
    mu_paper: float = 0.8
    mu_grip: float = 1.5
    law: Law = Law()
    touch_n: float = 0.02  # contact force from which the edge counts as on the slide
    drop_from_near: float = 0.024  # drop centre from the base slide's near (frosted) end
    glass_color: Color = (0.78, 0.86, 0.88)
    frost_color: Color = (0.97, 0.97, 0.96)
    paper_color: Color = (0.95, 0.95, 0.93)
    stand_color: Color = (0.20, 0.22, 0.26)

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a SmearRig is a fixed asset (mass None); its bodies carry their own")

    @property
    def paper_top(self) -> float:
        return self.table_z + self.paper[2]

    @property
    def slide_top(self) -> float:
        return self.paper_top + self.slide[2]

    @property
    def near_x(self) -> float:
        return self.pos[0] - self.slide[0] / 2

    @property
    def drop_xy(self) -> tuple[float, float]:
        return (self.near_x + self.drop_from_near, self.pos[1])

    @property
    def spreader_bottom(self) -> float:
        return self.table_z + self.stand_size[2] - self.slot[1]


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def edge_on_slide(edge, axis, slide_pos, slide_R, near_offset: float):
    """The spreader's edge in the base slide's frame (x from its near end, y across) and its angle to the slide on
    the trailing side (deg; over 90: leaning forward). ``axis``: unit vector up the spreader."""
    d = np.asarray(edge, float) - np.asarray(slide_pos, float)
    ex, ey, ez = slide_R[:, 0], slide_R[:, 1], slide_R[:, 2]
    ax, az = float(axis @ ex), float(axis @ ez)
    return float(d @ ex) + near_offset, float(d @ ey), math.degrees(math.atan2(az, -ax))


@dataclass(frozen=True)
class SmearState(ObjectState):
    """After a control step (env frame). ``pos`` is the base slide's centre."""

    smear: dict = field(default_factory=dict)  # Smear.summary()
    score_in: tuple = ()  # (Smear copy for scoring, base slide moved m)
    edge: tuple = ()  # spreader edge: x, y on the slide (m), angle (deg), contact force (N)
    spreader_pos: np.ndarray = None
    base_moved: float = 0.0


# -- spawning (Isaac Sim only) ---------------------------------------------------------------------------------------

_SPECS: dict[str, SmearRig] = {}
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
            if mu == s.mu_grip:  # rubber pads on glass: the pad's grip, not the average with the robot's material
                PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(mats[mu])).CreateFrictionCombineModeAttr("max")
        return mats[mu]

    def look(color, rough=0.5, opacity=1.0) -> str:
        key = (tuple(color), rough, opacity)
        if key not in looks:
            looks[key] = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=0.0,
                                              opacity=opacity)
            vis.func(looks[key], vis)
        return looks[key]

    def collide(prim, mu, offset=0.0005, patch_r=0.0) -> None:
        UsdPhysics.CollisionAPI.Apply(prim)
        col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        col.CreateContactOffsetAttr(offset)
        col.CreateRestOffsetAttr(0.0)
        if patch_r > 0:
            col.CreateTorsionalPatchRadiusAttr(patch_r)
            col.CreateMinTorsionalPatchRadiusAttr(patch_r)
        bind_physics_material(str(prim.GetPath()), material(mu), stage=stage)

    def body(name, pos, kinematic=False, report=False):
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*map(float, pos)))
        xf.AddOrientOp(precision=dbl).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(kinematic)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(4)
        if not kinematic:
            rb.CreateEnableCCDAttr(True)
        if report:
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return path, prim

    def cube(parent, name, centre, size, color=None, mu=None, rough=0.5, offset=0.0005, patch_r=0.0, opacity=1.0):
        c = UsdGeom.Cube.Define(stage, f"{parent}/{name}")
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        if mu is not None:
            collide(c.GetPrim(), mu, offset, patch_r)
        if color is None:
            UsdGeom.Imageable(c.GetPrim()).MakeInvisible()
        else:
            bind_visual_material(str(c.GetPath()), look(color, rough, opacity), stage=stage)
        return c

    def mass(prim, m, size, com_z=0.0):
        mp = UsdPhysics.MassAPI.Apply(prim)
        mp.CreateMassAttr(m)
        mp.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, com_z))
        a, b, c = size
        mp.CreateDiagonalInertiaAttr(Gf.Vec3f(m * (b * b + c * c) / 12, m * (a * a + c * c) / 12,
                                              m * (a * a + b * b) / 12))

    z0 = s.table_z
    L, W, T = s.slide
    # bench: paper and the spreader stand (a block with one slot through its width)
    bench, _ = body("bench", (0.0, 0.0, z0), kinematic=True)
    px, py = s.paper_xy
    cube(bench, "paper", (px, py, s.paper[2] / 2), s.paper, s.paper_color, mu=s.mu_paper, rough=0.95)
    sx, sy = s.stand_xy
    dx, dy, dz = s.stand_size
    gap, depth = s.slot
    wall = (dx - gap) / 2
    for k, sgn in enumerate((-1, 1)):
        cube(bench, f"stand_wall{k}", (sx + sgn * (gap / 2 + wall / 2), sy, dz / 2), (wall, dy, dz), s.stand_color,
             mu=0.4)
    cube(bench, "stand_floor", (sx, sy, (dz - depth) / 2), (gap + 0.0002, dy, dz - depth), s.stand_color, mu=0.4)

    # base slide: origin at its centre (mid-thickness)
    base, bprim = body("base", (s.pos[0], s.pos[1], s.paper_top + T / 2), report=True)
    cube(base, "glass", (0, 0, 0), (L, W, T), s.glass_color, mu=s.mu_glass, rough=0.1, opacity=0.6)
    cube(base, "frost", (-L / 2 + s.frost / 2, 0, T / 2 + 0.00002), (s.frost, W, 0.00004), s.frost_color, rough=0.9)
    mass(bprim, s.glass_mass, (L, W, T))
    dropx = -L / 2 + s.drop_from_near
    g = UsdGeom.Cylinder.Define(stage, f"{base}/drop")
    g.CreateAxisAttr("Z")
    g.CreateHeightAttr(0.0008)
    g.CreateRadiusAttr(s.law.drop_r)
    g.AddTranslateOp().Set(Gf.Vec3d(dropx, 0.0, T / 2 + 0.0004))
    bind_visual_material(str(g.GetPath()), look(SHADES[0], 0.2), stage=stage)
    for i in range(N_SEG):
        for j, col in enumerate(SHADES):
            c = cube(base, f"film{i}_{j}", (-L / 2 + (i + 0.5) * SEG, 0.0, T / 2 + 0.00005), (SEG + 0.0001, 0.02,
                                                                                               0.00006), col, rough=0.6)
            UsdGeom.Imageable(c.GetPrim()).MakeInvisible()

    # spreader: standing in the slot, origin at its lower edge's centre, faces normal to x
    sp, sprim = body("spreader", (sx, sy, s.spreader_bottom + 0.0002), report=True)
    zg = L - s.grip_zone
    cube(sp, "glass", (0, 0, zg / 2), (T, W, zg), None, mu=s.mu_glass)
    # the simulated pads stop about 3 mm apart when fully closed (real rubber pads close on the 1 mm glass), so where
    # the fingers hold it the collider is thicker: the grip a real gripper gets on the slide
    cube(sp, "grip", (0, 0, zg + s.grip_zone / 2), (s.grip_collider, W, s.grip_zone), None, mu=s.mu_grip,
         patch_r=0.006)
    cube(sp, "look", (0, 0, L / 2), (T, W, L), s.glass_color, rough=0.1, opacity=0.6)
    cube(sp, "label", (0, 0, L - 0.010), (T + 0.0002, W, 0.020), s.frost_color, rough=0.9)
    cube(sp, "bead", (0, 0, 0.0004), (0.0016, 0.001, 0.0008), SHADES[0], rough=0.2)
    UsdGeom.Imageable(stage.GetPrimAtPath(f"{sp}/bead")).MakeInvisible()
    mass(sprim, s.glass_mass, (T, W, L), com_z=L / 2)
    return root.GetPrim()


def _arena_object(spec: SmearRig):
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


class _Runtime:
    """PhysX views, the smear rule and the drawn film for one rig (env 0)."""

    def __init__(self, base, spec: SmearRig):
        from isaaclab_physx.physics import PhysxManager

        self.spec, self.base = spec, base
        self.root = _PATHS[spec.name]
        view = PhysxManager.get_physics_sim_view()
        self.bodies = view.create_rigid_body_view([f"{self.root}/base", f"{self.root}/spreader"])
        self.start = _arr(self.bodies.get_transforms()).copy()
        self.contact = view.create_rigid_contact_view([f"{self.root}/spreader"],
                                                      filter_patterns=[[f"{self.root}/base"]],
                                                      max_contact_data_count=64)
        self.c_bench = view.create_rigid_contact_view([f"{self.root}/spreader"],
                                                      filter_patterns=[[f"{self.root}/bench"]],
                                                      max_contact_data_count=64)
        self.dt = float(base.physics_dt)
        self.origin = _arr(base.scene.env_origins)[0]
        self.debug = bool(os.environ.get("SMEAR_DEBUG"))
        self.n = 0
        self.shown: dict = {}
        self.clear()

    def reset(self) -> None:
        cur = self.bodies.get_transforms()
        idx = _like(cur, np.arange(2), "int32")
        self.bodies.set_transforms(_like(cur, self.start), idx)
        self.bodies.set_velocities(_like(cur, np.zeros((2, 6))), idx)
        self.clear()

    def clear(self) -> None:
        s = self.spec
        self.smear = Smear(law=s.law, slide_l=s.slide[0], slide_w=s.slide[1], drop_x=s.drop_from_near,
                           spreader_w=s.slide[1])
        self.base0 = None
        self.last_touch = -100
        self.moved = 0.0
        self.force = 0.0
        self.edge = (0.0, 0.0, 0.0, 0.0)
        self.n = 0
        self._draw(force=True)

    def say(self, msg: str) -> None:
        print(f"[smearkit] t {self.n * self.dt:6.2f} {msg}", flush=True)

    def _poses(self):
        tr = _arr(self.bodies.get_transforms())
        return [(t[:3] - self.origin, quat_to_matrix(t[3:7])) for t in tr]

    def physics_step(self) -> None:
        s = self.spec
        self.n += 1
        (bp, bR), (sp, sR) = self._poses()
        if self.base0 is None:
            self.base0 = bp.copy()
        self.moved = max(self.moved, float(np.linalg.norm((bp - self.base0)[:2])))
        f = _arr(self.contact.get_contact_force_matrix(dt=self.dt)).reshape(-1, 3)
        self.force = float(np.linalg.norm(f.sum(axis=0)))
        x, y, ang = edge_on_slide(sp, sR[:, 2], bp, bR, s.slide[0] / 2)
        self.edge = (x, y, ang, self.force)
        # PhysX reports the force of a light sliding contact on most substeps but not all: the edge counts as on the
        # glass if it pushed 0.02 N or more within the last 6 substeps (25 ms)
        self.last_touch = self.n if self.force >= s.touch_n else self.last_touch
        msg = self.smear.step(self.dt, x, y, self.n - self.last_touch < 6, ang)
        if msg:
            self.say(f"{msg} (edge {x * 1000:.1f} mm, angle {ang:.1f} deg, force {self.force:.2f} N, speed "
                     f"{self.smear.v * 1000:.0f} mm/s)")
        if self.debug and self.n % 24 == 0:
            fb = float(np.linalg.norm(_arr(self.c_bench.get_contact_force_matrix(dt=self.dt)).reshape(-1, 3).sum(0)))
            ez = (sp[2] - bp[2] - s.slide[2] / 2) * 1000
            self.say(f"edge z {ez:5.2f} mm roll {math.degrees(math.asin(float(np.clip(sR[2, 1], -1, 1)))):5.1f} "
                     f"bench force {fb:.3f} N spreader {np.round(sp * 1000, 1)}")
            self.say(f"edge x {x * 1000:6.1f} y {y * 1000:5.1f} mm angle {ang:5.1f} force {self.force:.3f} N "
                     f"state {self.smear.state} width {self.smear.width * 1000:.1f} moved {self.moved * 1000:.2f} mm")

    def _vis(self, path: str, on: bool) -> None:
        if self.shown.get(path) == on:
            return
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdGeom

        img = UsdGeom.Imageable(get_current_stage().GetPrimAtPath(path))
        img.MakeVisible() if on else img.MakeInvisible()
        self.shown[path] = on

    def _scale(self, path: str, size) -> None:
        key = path + ":scale"
        size = tuple(round(float(v), 5) for v in size)
        if self.shown.get(key) == size:
            return
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        prim = get_current_stage().GetPrimAtPath(path)
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                op.Set(Gf.Vec3f(*size))
        self.shown[key] = size

    def _draw(self, force: bool = False) -> None:
        """The drop, the bead on the spreader's edge and the film segments."""
        if force:
            self.shown = {}
        sm, r = self.smear, self.root
        self._vis(f"{r}/base/drop", sm.state == "drop")
        bead = sm.state in ("wicking", "carrying", "spreading", "blunt", "runoff") and sm.vol > 0
        self._vis(f"{r}/spreader/bead", bead)
        if bead:
            self._scale(f"{r}/spreader/bead", (0.0016, max(sm.width, 0.003), 0.0008))
        k = int(round(SEG / sm.law.bin_m))
        h = sm.h[: N_SEG * k].reshape(N_SEG, k).max(axis=1) if len(sm.h) >= N_SEG * k else np.zeros(N_SEG)
        top = h.max(initial=0.0)
        for i in range(N_SEG):
            shade = -1
            if h[i] > 0:
                shade = 0 if h[i] > 0.6 * top else (1 if h[i] > 0.25 * top else 2)
            for j in range(len(SHADES)):
                p = f"{r}/base/film{i}_{j}"
                self._vis(p, j == shade)
                if j == shade:
                    self._scale(p, (SEG + 0.0001, max(sm.width, 0.002), 0.00006))

    def state(self) -> SmearState:
        import copy

        s = self.spec
        self._draw()
        (bp, bR), (sp, _) = self._poses()
        st = SmearState(pos=bp, quat_xyzw=np.array([0.0, 0, 0, 1]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                        low=bp - np.array([s.slide[0] / 2, s.slide[1] / 2, 0.001]),
                        high=bp + np.array([s.slide[0] / 2, s.slide[1] / 2, 0.001]),
                        smear=self.smear.summary(), score_in=(copy.deepcopy(self.smear), self.moved),
                        edge=self.edge, spreader_pos=sp, base_moved=self.moved)
        return st


_RT: dict[str, _Runtime] = {}


def _runtime(base, name: str) -> _Runtime:
    if name not in _RT:
        _RT[name] = _Runtime(base, _SPECS[name])
    return _RT[name]


# -- hooks (as in the other tasks' kits) -----------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_smearkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, SmearRig):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _runtime(environment.unwrapped, n).state() for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._smearkit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_smearkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for name in _SPECS:
            _runtime(base, name).physics_step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._smearkit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_smearkit", False):
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

    cls._ensure_sim, cls.reset, cls._smearkit = _ensure_sim, _reset, True


def _patch_scripted(module) -> None:
    """``yam_scripted`` plays the task file's closed-loop ``controller`` when it has one."""
    cls = module.YamScriptedPolicy
    if getattr(cls, "_smearkit", False):
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

    cls.reset, cls.act, cls._smearkit = _reset, _act, True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
            "yam_arena.scripted": _patch_scripted}


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
