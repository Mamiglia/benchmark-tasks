"""titrkit: a burette on a retort stand over an Erlenmeyer flask on a white tile, for a yam_arena task file.

yam_arena task files only take single primitives. This module adds one object, :class:`TitrationRig`, spawned as a
static (``BASE``) Arena asset holding three bodies:

- ``stand`` (kinematic): the stand's base plate and rod, the burette clamp, the glass burette with its stopcock
  barrel and its tip, and the white tile. The tip has no collider (the tip's contact with the flask's glass is
  latched by the model instead, see :mod:`titration`). The drops and the stream are drawn here.
- ``key`` (dynamic): the stopcock's PTFE key, on a revolute joint to the stand along the barrel's axis: a plug and a
  two-winged handle (the collider the fingers grip). The key's friction is an elasto-plastic stick (``KEY_FRICTION``
  N m, ``KEY_STICK`` N m/rad of give) applied as a torque every physics step.
- ``flask`` (dynamic): a 125 mL Erlenmeyer, colliders a stack of cylinders for the conical body and the neck. Its
  liquid is drawn in colour stages (colourless to magenta) with a pink patch under the tip in size stages.

Before every physics step (240 Hz, via a wrapper on ``InteractiveScene.write_data_to_sim``) the kit reads the key's
angle and the flask's pose and steps :class:`titration.Titration`. ``object_states`` reports a :class:`RigState`.

Importing the module also hooks ``yam_arena.scripted`` (as in the other kits): when the task file defines
``controller(bases, dt)``, ``yam_scripted`` asks that object for every action, passing it the observation (camera
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
from titration import Titration, TitrationParams  # noqa: E402

ARMS = ("left_robot", "right_robot")
FINGER_OPEN = -0.04695
TCP_OFFSET = 0.134
KEY_FRICTION = 0.05  # N m: a PTFE burette key with its tension nut set for a firm turn
KEY_STICK = 8.0  # N m/rad pre-slip stiffness (0.36 deg of give at the friction torque)
KEY_INERTIA = 2.0e-3  # kg m^2: raised from the real ~1e-6 so the 240 Hz friction torque is stable

WATER = (0.90, 0.93, 0.95)
PINKS = (WATER, (0.98, 0.80, 0.88), (0.96, 0.55, 0.75), (0.85, 0.15, 0.55))  # colourless, faint, pink, magenta
PATCH = (0.93, 0.35, 0.65)
PATCH_R = (0.004, 0.007, 0.011, 0.016)  # m, by unmixed volume
PATCH_AT = (0.02, 0.15, 0.40, 1.00)  # mL
GLASS = (0.78, 0.83, 0.86)
PTFE = (0.97, 0.97, 0.95)
IRON, ROD, TILE = (0.20, 0.22, 0.25), (0.70, 0.72, 0.74), (0.98, 0.98, 0.97)


def colour_stage(pinkness: float) -> int:
    return 0 if pinkness <= 0 else 1 if pinkness < 0.4 else 2 if pinkness < 0.8 else 3


def patch_stage(u: float) -> int:
    """0: no patch; 1..4 sizes."""
    return int(np.searchsorted(PATCH_AT, u, side="right"))


@dataclass(frozen=True)
class TitrationRig(Box):
    """Positions in the env frame (x forward, y left, z up, m). ``pos``: the burette's axis on the table."""

    tip_z: float = 0.0  # burette tip (world z)
    cock_z: float = 0.0  # stopcock axis height
    key_dir: tuple = (1.0, 0.0, 0.0)  # unit, horizontal: from the handle into the barrel
    handle_off: float = 0.022  # handle wings' centre from the burette's axis, m
    top_z: float = 0.0  # burette's top
    flask_xy: tuple = (0.0, 0.0)
    floor_z: float = 0.0  # where the flask stands (the tile's top)
    stand_boxes: tuple = ()  # (centre, size, colour) world, kinematic with colliders
    rod: tuple = ()  # (x, y, z0, z1, r)
    clamp_boxes: tuple = ()  # (centre, size, colour), visual only
    titration: TitrationParams = field(default_factory=TitrationParams)
    flask_mass: float = 0.115  # 125 mL borosilicate Erlenmeyer (about 85 g) with 30 mL of solution
    flask_friction: float = 1.0
    key_mass: float = 0.008


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def yaw_quat_wxyz(d) -> tuple:
    a = math.atan2(d[1], d[0])
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


# flask profile: (z0, z1, outer radius) from its bottom; 125 mL narrow-mouth Erlenmeyer, 67 mm x 114 mm
FLASK_RINGS = ((0.0, 0.022, 0.0335), (0.022, 0.042, 0.029), (0.042, 0.062, 0.024), (0.062, 0.084, 0.019),
               (0.084, 0.114, 0.0145))
LIQUID_H = 0.010  # 30 mL in the base
LIQUID_R = 0.031


@dataclass(frozen=True)
class RigState(ObjectState):
    key_deg: float = 0.0
    flow: float = 0.0  # mL/s
    reading: float = 0.0  # burette reading, mL
    landed: float = 0.0
    lost: float = 0.0
    unmixed: float = 0.0
    excess: float = 0.0  # bulk past equivalence, mL
    final_excess: float = 0.0
    final_pink: bool = False
    v_e: float = 0.0
    swirl: float = 0.0
    struck: bool = False
    spilled: bool = False
    ran_dry: bool = False
    max_tilt: float = 0.0
    flask_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))  # bottom centre
    flask_axis: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))
    flask_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    pads: dict = field(default_factory=dict)  # arm -> (3, 3) finger pad tips and tool point
    drops: int = 0


# -- spawning (Isaac Sim only) ----------------------------------------------------------------------------------

_SPECS: dict[str, TitrationRig] = {}
_PATHS: dict[str, str] = {}


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
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

    def body(name, pos, kinematic=False, orient=None):
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*pos))
        if orient is not None:
            xf.AddOrientOp(precision=dbl).Set(Gf.Quatd(*orient))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(kinematic)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(4)
        return path, prim

    def cube(path, centre, size, color, mat=None, visible=True):
        c = UsdGeom.Cube.Define(stage, path)
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*centre))
        c.AddScaleOp().Set(Gf.Vec3f(*size))
        if mat:
            collider(c.GetPrim(), mat)
        bind_visual_material(str(c.GetPath()), look(color), stage=stage)
        if not visible:
            UsdGeom.Imageable(c.GetPrim()).MakeInvisible()
        return c

    def cyl(path, radius, a, b, color, mat=None, visible=True):
        """A cylinder from point a to point b (in its parent's frame)."""
        a, b = np.asarray(a, float), np.asarray(b, float)
        d = b - a
        h = float(np.linalg.norm(d))
        c = UsdGeom.Cylinder.Define(stage, path)
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(radius)
        c.CreateHeightAttr(h)
        c.AddTranslateOp().Set(Gf.Vec3d(*((a + b) / 2)))
        z = d / h
        ax = np.cross([0.0, 0.0, 1.0], z)
        n = float(np.linalg.norm(ax))
        if n > 1e-9:
            ang = math.atan2(n, z[2])
            ax = ax / n * math.sin(ang / 2)
            c.AddOrientOp().Set(Gf.Quatf(math.cos(ang / 2), *ax))
        elif z[2] < 0:
            c.AddOrientOp().Set(Gf.Quatf(0.0, 1.0, 0.0, 0.0))
        if mat:
            collider(c.GetPrim(), mat)
        if color is not None:
            bind_visual_material(str(c.GetPath()), look(color), stage=stage)
        if not visible:
            UsdGeom.Imageable(c.GetPrim()).MakeInvisible()
        return c

    def sphere(path, radius, centre, color, visible=True):
        sp = UsdGeom.Sphere.Define(stage, path)
        sp.CreateRadiusAttr(radius)
        sp.AddTranslateOp().Set(Gf.Vec3d(*centre))
        bind_visual_material(str(sp.GetPath()), look(color), stage=stage)
        if not visible:
            UsdGeom.Imageable(sp.GetPrim()).MakeInvisible()

    bx, by = s.pos[0], s.pos[1]
    d = np.asarray(s.key_dir, float)
    # stand (kinematic): origin at the stopcock's centre
    cock = np.array([bx, by, s.cock_z])
    stand, _ = body("stand", tuple(cock), kinematic=True)
    smat = material("stand", 0.8)
    for i, (centre, size, color) in enumerate(s.stand_boxes):
        cube(f"{stand}/box_{i}", np.subtract(centre, cock), size, color, smat)
    rx, ry, rz0, rz1, rr = s.rod
    cyl(f"{stand}/rod", rr, np.subtract((rx, ry, rz0), cock), np.subtract((rx, ry, rz1), cock), ROD, smat)
    for i, (centre, size, color) in enumerate(s.clamp_boxes):
        cube(f"{stand}/clamp_{i}", np.subtract(centre, cock), size, color)
    o = np.zeros(3)
    cyl(f"{stand}/tube", 0.0060, (0, 0, 0.012), (0, 0, s.top_z - s.cock_z), GLASS)  # graduated tube
    cyl(f"{stand}/barrel", 0.0065, -d * 0.014, d * 0.014, GLASS)  # stopcock barrel (visual)
    cyl(f"{stand}/nut", 0.0075, d * 0.014, d * 0.022, (0.15, 0.45, 0.85))  # key tension nut
    cyl(f"{stand}/stem", 0.0040, (0, 0, -0.008), (0, 0, s.tip_z - s.cock_z + 0.012), GLASS)
    cyl(f"{stand}/tip", 0.0030, (0, 0, s.tip_z - s.cock_z + 0.012), (0, 0, s.tip_z - s.cock_z), GLASS)
    tipz = s.tip_z - s.cock_z
    surf = s.floor_z + 0.002 + LIQUID_H - s.cock_z
    sphere(f"{stand}/drop_hang", 0.0023, (0, 0, tipz - 0.002), WATER, visible=False)
    sphere(f"{stand}/drop_fall", 0.0023, (0, 0, (tipz + surf) / 2), WATER, visible=False)
    cyl(f"{stand}/stream", 0.0008, (0, 0, tipz), (0, 0, surf), WATER, visible=False)

    # key (dynamic), origin on the stopcock's axis at the burette's axis, local x along key_dir
    key, kprim = body("key", tuple(cock), orient=yaw_quat_wxyz(d))
    km = UsdPhysics.MassAPI.Apply(kprim)
    km.CreateMassAttr(s.key_mass)
    km.CreateDiagonalInertiaAttr(Gf.Vec3f(KEY_INERTIA, KEY_INERTIA, KEY_INERTIA))
    km.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
    PhysxSchema.PhysxRigidBodyAPI(kprim).CreateAngularDampingAttr(2.0)
    kmat = material("key", 1.0)
    h = s.handle_off
    cyl(f"{key}/plug", 0.0040, (0.014, 0, 0), (-(h - 0.006), 0, 0), PTFE)
    cube(f"{key}/wings", (-h, 0.0, 0.0), (0.012, 0.032, 0.005), PTFE, kmat)  # handle across the burette = closed
    cube(f"{key}/rib", (-h - 0.002, 0.0, 0.0), (0.006, 0.006, 0.010), PTFE)
    j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/key_joint")
    j.CreateAxisAttr("X")
    j.CreateBody0Rel().SetTargets([Sdf.Path(stand)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(key)])
    j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    w, x, y, z = yaw_quat_wxyz(d)
    j.CreateLocalRot0Attr(Gf.Quatf(w, x, y, z))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    # flask (dynamic): origin at its bottom centre
    fx, fy = s.flask_xy
    flask, fprim = body("flask", (fx, fy, s.floor_z + 0.0005))
    fm = UsdPhysics.MassAPI.Apply(fprim)
    fm.CreateMassAttr(s.flask_mass)
    fm.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.022))
    fmat = material("flask", s.flask_friction)
    for i, (z0, z1, r) in enumerate(FLASK_RINGS):
        c = cyl(f"{flask}/shell_{i}", r, (0, 0, z0), (0, 0, z1), None, fmat)
        UsdGeom.Imageable(c.GetPrim()).MakeInvisible()
    cyl(f"{flask}/foot", 0.0335, (0, 0, 0), (0, 0, 0.002), GLASS)
    cyl(f"{flask}/lip", 0.0155, (0, 0, 0.110), (0, 0, 0.114), GLASS)
    for k in range(8):
        a = k * math.pi / 4 + math.pi / 8
        c, sn = math.cos(a), math.sin(a)
        cyl(f"{flask}/edge_{k}", 0.0005, (0.0335 * c, 0.0335 * sn, 0.002), (0.0145 * c, 0.0145 * sn, 0.084), GLASS)
        cyl(f"{flask}/neck_{k}", 0.0005, (0.0145 * c, 0.0145 * sn, 0.084), (0.0145 * c, 0.0145 * sn, 0.110), GLASS)
    for st, col in enumerate(PINKS):
        cyl(f"{flask}/liquid_{st}", LIQUID_R, (0, 0, 0.002), (0, 0, 0.002 + LIQUID_H), col, visible=st == 0)
    for st, r in enumerate(PATCH_R):
        cyl(f"{flask}/patch_{st}", r, (0, 0, 0.002 + LIQUID_H), (0, 0, 0.0025 + LIQUID_H), PATCH, visible=False)
    return root.GetPrim()


def _arena_object(spec: TitrationRig):
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


def ve_for(seed) -> float:
    """The trial's equivalence volume (mL): the acid aliquot, 8.00 to 12.00 mL, from the seed."""
    rng = np.random.default_rng(int(seed or 0) + 7919)
    return round(float(rng.uniform(8.0, 12.0)) * 20) / 20


def stick(anchor: float, x: float, k: float, limit: float) -> tuple[float, float]:
    """Elasto-plastic friction: (torque, new anchor)."""
    t = k * (anchor - x)
    if abs(t) > limit:
        t = math.copysign(limit, t)
        anchor = x + t / k
    return t, anchor


def key_angle(rot_key: np.ndarray, d) -> float:
    """Key angle (deg) about its axis from the closed pose (wings horizontal)."""
    d = np.asarray(d, float)
    y0 = np.cross([0.0, 0.0, 1.0], d)
    y0 /= np.linalg.norm(y0)
    y = rot_key[:, 1]
    return math.degrees(math.atan2(float(y @ np.array([0.0, 0.0, 1.0])), float(y @ y0)))


class _Runtime:
    def __init__(self, base, spec: TitrationRig):
        from isaaclab_physx.physics import PhysxManager

        self.spec, self.base = spec, base
        self.root = _PATHS[spec.name]
        view = PhysxManager.get_physics_sim_view()
        self.body = view.create_rigid_body_view([f"{self.root}/key", f"{self.root}/flask"])
        self.dt = float(base.physics_dt)
        self.start = _arr(self.body.get_transforms()).copy()
        self.origin = _arr(base.scene.env_origins)[0]
        self.debug = bool(os.environ.get("TITR_DEBUG"))
        self.model = Titration(spec.titration)
        self.tip = np.array([spec.pos[0], spec.pos[1], spec.tip_z])
        self.anchor = 0.0
        self.shown = {"colour": 0, "patch": 0, "hang": False, "fall": False, "stream": False}
        self.n = 0
        self.seed = 0
        self.pad_ids = {arm: [base.scene[arm].find_bodies(b)[0][0] for b in ("tip_left", "tip_right", "gripper")]
                        for arm in ARMS}

    def reset(self) -> None:
        cur = self.body.get_transforms()
        idx = np.arange(2, dtype=np.int32)
        self.body.set_transforms(_like(cur, self.start), _like(cur, idx, "int32"))
        self.body.set_velocities(_like(cur, np.zeros((2, 6))), _like(cur, idx, "int32"))
        self.clear()

    def clear(self) -> None:
        self.model.reset(ve_for(self.seed))
        self.anchor = 0.0
        self.n = 0
        print(f"[titrkit] seed {self.seed}: equivalence at {self.model.p.v_e:.2f} mL", flush=True)
        self._show()

    def _poses(self):
        tr = _arr(self.body.get_transforms())
        return [(t[:3] - self.origin, quat_to_matrix(t[3:7])) for t in tr]

    def physics_step(self) -> None:
        import warp as wp

        self.n += 1
        (kp, kr), (fp, fr) = self._poses()
        ang = key_angle(kr, self.spec.key_dir)
        tau, self.anchor = stick(self.anchor, math.radians(ang), KEY_STICK, KEY_FRICTION)
        torque = np.zeros(6)
        torque[:3] = kr[:, 0] * tau
        cur = self.body.get_transforms()
        dev = getattr(cur, "device", None)
        self.body.apply_forces_and_torques_at_position(
            force_data=wp.array(np.zeros(6, np.float32), dtype=wp.float32, device=dev),
            torque_data=wp.array(torque.astype(np.float32), dtype=wp.float32, device=dev),
            position_data=None, indices=wp.array(np.arange(2, dtype=np.int32), dtype=wp.int32, device=dev),
            is_global=True)
        m = self.model
        was = (m.struck, m.spilled, m.lost > 0)
        m.step(ang, self.tip, fp, fr, self.dt)
        now = (m.struck, m.spilled, m.lost > 0)
        if now != was:
            print(f"[titrkit] t {m.t:.2f}: struck {m.struck} spilled {m.spilled} lost {m.lost:.2f} mL "
                  f"(tip in flask frame {np.round(m.tip_in_flask * 1000, 1)} mm)", flush=True)

    def _show(self) -> None:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdGeom

        stage = get_current_stage()
        m = self.model

        def vis(path, on):
            img = UsdGeom.Imageable(stage.GetPrimAtPath(f"{self.root}/{path}"))
            img.MakeVisible() if on else img.MakeInvisible()

        col = colour_stage(m.bulk_pinkness())
        if col != self.shown["colour"]:
            for k in range(len(PINKS)):
                vis(f"flask/liquid_{k}", k == col)
            self.shown["colour"] = col
        pat = 0 if col else patch_stage(m.w)
        if pat != self.shown["patch"]:
            for k in range(len(PATCH_R)):
                vis(f"flask/patch_{k}", k == pat - 1)
            self.shown["patch"] = pat
        want = {"hang": m.hanging > 0.5 * m.p.v_drop and not m.streaming,
                "fall": 0 <= m.t - m.drop_t < 0.15 and m.drop_t >= 0, "stream": m.streaming}
        for k, path in (("hang", "stand/drop_hang"), ("fall", "stand/drop_fall"), ("stream", "stand/stream")):
            if want[k] != self.shown[k]:
                vis(path, want[k])
                self.shown[k] = want[k]

    def state(self) -> RigState:
        self._show()
        m = self.model
        vel = _arr(self.body.get_velocities())
        (kp, kr), (fp, fr) = self._poses()
        pads = {}
        for arm in ARMS:
            poses = _arr(self.base.scene[arm].data.body_link_pose_w)[0]
            tips = [poses[i, :3] - self.origin for i in self.pad_ids[arm][:2]]
            g = poses[self.pad_ids[arm][2]]
            pads[arm] = np.array([*tips, g[:3] - self.origin + quat_to_matrix(g[3:7])[:, 2] * TCP_OFFSET])
        st = RigState(pos=fp, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=vel[1, :3], ang_vel=vel[1, 3:],
                      low=fp - 0.04, high=fp + 0.12, key_deg=m.key_deg, flow=m.q, reading=m.reading,
                      landed=m.landed, lost=m.lost, unmixed=m.u, excess=m.excess, final_excess=m.final_excess,
                      final_pink=m.final_pink, v_e=m.p.v_e, swirl=m.swirl, struck=m.struck, spilled=m.spilled,
                      ran_dry=m.ran_dry, max_tilt=m.max_tilt, flask_pos=fp, flask_axis=fr[:, 2],
                      flask_vel=vel[1, :3], pads=pads, drops=m.drops)
        if self.debug and self.n % 240 < 12:
            print(f"[titrkit] t {m.t:6.2f} key {m.key_deg:6.1f} q {m.q:.3f} read {m.reading:6.2f} u {m.u:.3f} "
                  f"excess {m.excess:+.2f} swirl {m.swirl:.2f} tip {np.round(m.tip_in_flask * 1000, 1)}", flush=True)
        return st


_RT: dict[str, _Runtime] = {}


def _runtime(base, name: str) -> _Runtime:
    if name not in _RT:
        _RT[name] = _Runtime(base, _SPECS[name])
    return _RT[name]


# -- hooks ------------------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_titrkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, TitrationRig):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _runtime(environment.unwrapped, n).state() for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._titrkit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_titrkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for name in _SPECS:
            _runtime(base, name).physics_step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._titrkit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_titrkit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def _ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
        return out

    def _reset(self, scene, *, seed=None):
        self._ensure_sim(None)
        if seed is None:
            seed = int(os.environ.get("TITRATION_SEED", "0"))
        for name in _SPECS:
            rt = _runtime(self._base, name)
            rt.seed = seed
            rt.reset()
        obs = reset(self, scene, seed=seed)
        for name in _SPECS:
            _runtime(self._base, name).clear()  # the settle steps are not part of the trial
        return obs

    cls._ensure_sim, cls.reset, cls._titrkit = _ensure_sim, _reset, True


def _patch_scripted(module) -> None:
    """``yam_scripted`` plays the task file's closed-loop ``controller`` when it has one."""
    cls = module.YamScriptedPolicy
    if getattr(cls, "_titrkit", False):
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

    cls.reset, cls.act, cls._titrkit = _reset, _act, True


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
