"""tubekit: a sealed tube with a hidden slug and a fixed conveyor hook, for a yam_arena task file.

yam_arena task files only take single primitives. This module adds one object, :class:`TubeRig`, spawned as a static
(``BASE``) Arena asset holding two bodies:

- ``tube``: one rigid body, a cylinder collider along the world y axis with a red end plug at +y and a blue one at -y
  (visual rings). Its mass properties are those of the aluminium shell, the two plugs and a steel slug glued inside at
  a position set by the trial's seed (:func:`slug_for_seed`). The slug has no geometry of its own; only the centre of
  mass and inertia show it. They are written with PhysX's tensor API at every reset.
- ``hook``: a kinematic (fixed) frame made of boxes: a post, an overhead arm (the conveyor rail's stand-in) and a
  J-hook hanging from it, whose short cradle is a rubber-lined 90-degree V (two boxes turned 45 degrees about y).

Before every physics step (240 Hz, via a wrapper on ``InteractiveScene.write_data_to_sim``) the kit reads the tube's
pose and velocity and PhysX's contact forces between the tube and the hook frame and every robot link, and latches:
``lifted`` (the tube's lowest point was ``lift_m`` above the table, or the hook alone carried it), ``regrounded``
(after that, it touched the table again or fell below it), ``dropped`` (it fell ``drop_m`` with nothing touching it).

``object_states`` reports a :class:`TubeState`: the tube's pose, axis and tilt, its lowest point, the hook and robot
contact forces in the last control step, the latches, the finger pads' positions and how far each gripper is open.

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
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

Color = tuple[float, float, float]
BoxSpec = tuple  # (centre, size (m), sRGB colour[, rotation about the y axis in degrees])
ARMS = ("left_robot", "right_robot")
FINGER_OPEN = -0.04695  # yam_arena.constants.FINGER_OPEN
TCP_OFFSET = 0.134  # yam_arena.constants.TCP_OFFSET, on the gripper link's z axis
STEEL = 7850.0  # kg/m^3


@dataclass(frozen=True)
class TubeRig(Box):
    """Positions in the env frame (x forward, y left, z up, m). ``pos`` is the tube's centre at the start."""

    length: float = 1.0
    radius: float = 0.025
    shell_mass: float = 0.617  # 6061 tube, 50 x 1.5 mm, 1 m
    plug_mass: float = 0.025  # each end plug
    plug_len: float = 0.03
    slug_radius: float = 0.020
    slug_len: float = 0.035  # 40 mm steel bar, 35 mm long: 0.345 kg
    offset_range: tuple[float, float] = (0.06, 0.14)  # |centre-of-mass offset| from the middle, m
    tube_color: Color = (0.78, 0.79, 0.81)
    plus_color: Color = (0.85, 0.08, 0.08)  # +y plug (red)
    minus_color: Color = (0.08, 0.25, 0.85)  # -y plug (blue)
    hook_boxes: tuple[BoxSpec, ...] = ()  # kinematic, env frame
    tube_friction: float = 0.8
    hook_friction: float = 1.0
    table_z: float = 0.0
    lift_m: float = 0.05  # lowest point this far above the table: lifted clear
    touch_m: float = 0.002  # lowest point within this of the table: touching it
    drop_m: float = 0.02  # free fall this far with nothing touching the tube: dropped
    touch_n: float = 0.05  # N: contact force above this counts as touching

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a TubeRig is a fixed asset (mass None); its bodies carry their own")

    @property
    def slug_mass(self) -> float:
        return STEEL * math.pi * self.slug_radius**2 * self.slug_len

    @property
    def total_mass(self) -> float:
        return self.shell_mass + 2 * self.plug_mass + self.slug_mass


def slug_for_seed(spec: TubeRig, seed: int | None) -> float:
    """Slug centre along the tube (m from the middle, + toward the red end) for a trial seed. The centre-of-mass
    offset is uniform in ``offset_range`` in size, either sign."""
    rng = np.random.default_rng(0 if seed is None else int(seed))
    d = rng.uniform(*spec.offset_range) * (1.0 if rng.random() < 0.5 else -1.0)
    return d * spec.total_mass / spec.slug_mass


def mass_props(spec: TubeRig, slug_y: float) -> tuple[float, np.ndarray]:
    """(centre of mass along the tube, diagonal inertia about it in the tube body frame (x, y along the tube, z))."""
    M, L, r = spec.total_mass, spec.length, spec.radius
    m_s, m_p, m_t = spec.slug_mass, spec.plug_mass, spec.shell_mass
    plug_y = L / 2 - spec.plug_len / 2
    d = (m_s * slug_y) / M
    axial = m_t * r**2 + 2 * m_p * r**2 / 2 + m_s * spec.slug_radius**2 / 2
    trans = (m_t * (L**2 / 12 + r**2 / 2 + d**2) + m_p * ((plug_y - d) ** 2 + (plug_y + d) ** 2)
             + m_s * ((slug_y - d) ** 2 + spec.slug_radius**2 / 4 + spec.slug_len**2 / 12))
    return d, np.array([trans, axial, trans])


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def lowest_point(centre, axis, length: float, radius: float) -> float:
    """Height of a cylinder's lowest point (centre, unit axis)."""
    a = np.asarray(axis, float)
    return float(np.asarray(centre)[2] - abs(a[2]) * length / 2 - radius * math.sqrt(max(0.0, 1 - a[2] ** 2)))


def tilt_deg(axis) -> float:
    """Angle between the tube's axis and the horizontal."""
    return math.degrees(math.asin(min(1.0, abs(float(np.asarray(axis)[2])))))


def surface_gap(p, centre, axis, length: float, radius: float) -> float:
    """Distance from point ``p`` to the tube's surface (0 inside)."""
    v = np.asarray(p, float) - np.asarray(centre, float)
    t = float(np.clip(v @ axis, -length / 2, length / 2))
    return max(0.0, float(np.linalg.norm(v - t * np.asarray(axis))) - radius)


@dataclass(frozen=True)
class TubeState(ObjectState):
    """The rig after a control step (env frame). ``pos`` is the tube's centre."""

    axis: np.ndarray = None  # unit, from the blue end to the red end
    tilt_deg: float = 0.0
    lowest: float = 0.0  # height of the tube's lowest point
    hook_n: float = 0.0  # contact force hook-tube at the end of the control step
    robot_n: float = 0.0  # largest robot-tube contact force during the control step
    lifted: bool = False
    regrounded: bool = False
    dropped: bool = False
    pads: dict = field(default_factory=dict)  # arm -> (3, 3) finger pad tips and tool point
    open_frac: dict = field(default_factory=dict)  # arm -> 0 (closed) .. 1 (open)
    com_offset: float = 0.0  # for logs only; scoring does not read it
    weight_n: float = 0.0


# -- spawning (Isaac Sim only) ----------------------------------------------------------------------------------

_SPECS: dict[str, TubeRig] = {}
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
        if color not in looks:
            looks[color] = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.4)
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

    # tube
    tube, prim = body("tube", tuple(s.pos))
    d, inertia = mass_props(s, slug_for_seed(s, None))
    m = UsdPhysics.MassAPI.Apply(prim)
    m.CreateMassAttr(s.total_mass)
    m.CreateCenterOfMassAttr(Gf.Vec3f(0.0, d, 0.0))
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
    cyl = UsdGeom.Cylinder.Define(stage, f"{tube}/shell")
    cyl.CreateAxisAttr("Y")
    cyl.CreateHeightAttr(s.length)
    cyl.CreateRadiusAttr(s.radius)
    collider(cyl.GetPrim(), material("tube", s.tube_friction))
    bind_visual_material(str(cyl.GetPath()), look(tuple(s.tube_color)), stage=stage)
    for side, color in ((1, s.plus_color), (-1, s.minus_color)):  # visual end rings, no collider
        ring = UsdGeom.Cylinder.Define(stage, f"{tube}/plug_{'red' if side > 0 else 'blue'}")
        ring.CreateAxisAttr("Y")
        ring.CreateHeightAttr(s.plug_len)
        ring.CreateRadiusAttr(s.radius + 0.0008)
        ring.AddTranslateOp().Set(Gf.Vec3d(0.0, side * (s.length / 2 - s.plug_len / 2), 0.0))
        bind_visual_material(str(ring.GetPath()), look(tuple(color)), stage=stage)

    # hook (fixed frame)
    hook_pos = np.mean([b[0] for b in s.hook_boxes], axis=0)
    hook, _ = body("hook", tuple(hook_pos), kinematic=True)
    smat = material("hook", s.hook_friction)
    for i, (centre, size, color, *rot) in enumerate(s.hook_boxes):
        cube = UsdGeom.Cube.Define(stage, f"{hook}/box_{i}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*np.subtract(centre, hook_pos)))
        if rot:
            cube.AddRotateYOp().Set(float(rot[0]))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        collider(cube.GetPrim(), smat)
        bind_visual_material(str(cube.GetPath()), look(tuple(color)), stage=stage)
    return root.GetPrim()


def _arena_object(spec: TubeRig):
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


def _like(ref, data: np.ndarray, dtype="float32", cpu=False):
    """``data`` as the tensor type the view returned ``ref`` in (torch or warp), on its device or the CPU."""
    if hasattr(ref, "detach"):
        import torch

        return torch.tensor(data, dtype=getattr(torch, dtype), device="cpu" if cpu else ref.device)
    import warp as wp

    return wp.array(data.astype(dtype), dtype=getattr(wp, dtype), device="cpu" if cpu else getattr(ref, "device", None))


class _Runtime:
    """PhysX views and latches for one rig (env 0)."""

    def __init__(self, base, spec: TubeRig):
        from isaaclab_physx.physics import PhysxManager

        self.spec, self.base = spec, base
        root = _PATHS[spec.name]
        view = PhysxManager.get_physics_sim_view()
        self.body = view.create_rigid_body_view([f"{root}/tube"])
        links = []
        for arm in ARMS:
            paths = base.scene[arm].root_physx_view.link_paths
            links += list(paths[0] if paths and isinstance(paths[0], (list, tuple)) else paths)
        self.link_names = [p.split("/")[-3] + "/" + p.rsplit("/", 1)[-1] for p in links]
        self.contact = view.create_rigid_contact_view([f"{root}/tube"], filter_patterns=[[f"{root}/hook", *links]],
                                                      max_contact_data_count=256)
        self.dt = float(base.physics_dt)
        self.start = _arr(self.body.get_transforms()).copy()
        self.origin = _arr(base.scene.env_origins)[0]
        self.debug = bool(os.environ.get("TUBE_DEBUG"))
        self.n = 0
        self.slug_y, self.com_offset = 0.0, 0.0
        self.pad_ids = {arm: [base.scene[arm].find_bodies(b)[0][0] for b in ("tip_left", "tip_right", "gripper")]
                        for arm in ARMS}
        self.finger_ids = {arm: base.scene[arm].find_joints(["joint7", "joint8"])[0] for arm in ARMS}
        self.clear_latches()

    def clear_latches(self) -> None:
        self.lifted = self.regrounded = self.dropped = False
        self.z_ref = None
        self.robot_peak = 0.0
        self.robot_link = ""
        self.hook_n = 0.0

    def set_seed(self, seed: int | None) -> None:
        """Glue the slug where this seed puts it: write the tube's centre of mass and inertia."""
        s = self.spec
        self.slug_y = slug_for_seed(s, seed)
        self.com_offset, inertia = mass_props(s, self.slug_y)
        idx = np.zeros(1, dtype=np.int32)
        coms = self.body.get_coms()
        com = np.array([[0.0, self.com_offset, 0.0, 0.0, 0.0, 0.0, 1.0]])
        self.body.set_coms(_like(coms, com, cpu=True), _like(coms, idx, "int32", cpu=True))
        inn = self.body.get_inertias()
        self.body.set_inertias(_like(inn, np.diag(inertia).reshape(1, 9), cpu=True), _like(inn, idx, "int32", cpu=True))
        got = _arr(self.body.get_coms()).reshape(-1)[:3]
        print(f"[tubekit] seed {seed}: slug at {self.slug_y * 100:+.1f} cm, centre of mass {self.com_offset * 100:+.2f} "
              f"cm from the middle (+ toward the red end); PhysX com {np.round(got, 4)}", flush=True)

    def reset(self) -> None:
        """Put the tube back where it started, at rest."""
        cur = self.body.get_transforms()
        idx = np.zeros(1, dtype=np.int32)
        self.body.set_transforms(_like(cur, self.start), _like(cur, idx, "int32"))
        self.body.set_velocities(_like(cur, np.zeros((1, 6))), _like(cur, idx, "int32"))
        self.clear_latches()

    def _pose(self):
        tr = _arr(self.body.get_transforms())[0]
        R = quat_to_matrix(tr[3:7])
        return tr[:3] - self.origin, R, tr[3:7]

    def physics_step(self) -> None:
        s = self.spec
        self.n += 1
        p, R, _ = self._pose()
        axis = R[:, 1]
        vel = _arr(self.body.get_velocities())[0]
        fm = _arr(self.contact.get_contact_force_matrix(dt=self.dt)).reshape(-1, 3)
        self.hook_n = float(np.linalg.norm(fm[0]))
        robot = float(np.linalg.norm(fm[1:], axis=1).max(initial=0.0))
        if robot > self.robot_peak:
            self.robot_peak, self.robot_link = robot, self.link_names[int(np.argmax(np.linalg.norm(fm[1:], axis=1)))]
        low = lowest_point(p, axis, s.length, s.radius)
        on_table = low <= s.table_z + s.touch_m
        if (low >= s.table_z + s.lift_m or (self.hook_n > s.touch_n and not on_table)) and not self.lifted:
            self.lifted = True
            print(f"[tubekit] lifted clear at {self.n * self.dt:.2f} s", flush=True)
        if self.lifted and not self.regrounded and (on_table or p[2] < s.table_z - 0.05):
            self.regrounded = True
            print(f"[tubekit] touched the table again after lifting, at {self.n * self.dt:.2f} s", flush=True)
        if robot > s.touch_n or self.hook_n > s.touch_n or on_table:
            self.z_ref = p[2]
        elif self.z_ref is not None and self.z_ref - p[2] > s.drop_m and not self.dropped:
            self.dropped = True
            print(f"[tubekit] dropped: fell {(self.z_ref - p[2]) * 100:.1f} cm untouched, at {self.n * self.dt:.2f} s "
                  f"(vz {vel[2]:.2f} m/s)", flush=True)

    def state(self) -> TubeState:
        s = self.spec
        p, R, q = self._pose()
        axis = R[:, 1]
        vel = _arr(self.body.get_velocities())[0]
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
        corners = np.array([p + sy * axis * s.length / 2 for sy in (-1, 1)])
        st = TubeState(pos=p, quat_xyzw=q, lin_vel=vel[:3], ang_vel=vel[3:], low=corners.min(0) - s.radius,
                       high=corners.max(0) + s.radius, axis=axis, tilt_deg=tilt_deg(axis),
                       lowest=lowest_point(p, axis, s.length, s.radius), hook_n=self.hook_n,
                       robot_n=self.robot_peak, lifted=self.lifted, regrounded=self.regrounded, dropped=self.dropped,
                       pads=pads, open_frac=opened, com_offset=self.com_offset, weight_n=s.total_mass * 9.81)
        if self.debug and self.n % 120 == 0:
            gaps = {a: round(min(surface_gap(x, p, axis, s.length, s.radius) for x in pads[a]) * 100, 1) for a in ARMS}
            print(f"[tubekit] t {self.n * self.dt:6.2f} tube {np.round(p, 3)} tilt {math.copysign(st.tilt_deg, axis[2]):+5.2f} low "
                  f"{(st.lowest - s.table_z) * 100:5.1f} cm hook {self.hook_n:5.2f} N robot {self.robot_peak:5.2f} N {self.robot_link} "
                  f"v {np.linalg.norm(vel[:3]):.3f} gaps {gaps} open {opened} lifted {self.lifted} "
                  f"regrounded {self.regrounded} dropped {self.dropped}", flush=True)
        self.robot_peak, self.robot_link = 0.0, ""
        return st


_RT: dict[str, _Runtime] = {}


def _runtime(base, name: str) -> _Runtime:
    if name not in _RT:
        _RT[name] = _Runtime(base, _SPECS[name])
    return _RT[name]


# -- hooks ------------------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_tubekit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, TubeRig):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _runtime(environment.unwrapped, n).state() for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._tubekit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_tubekit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for name in _SPECS:
            _runtime(base, name).physics_step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._tubekit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_tubekit", False):
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
            rt = _runtime(self._base, name)
            rt.reset()
            rt.set_seed(seed)
        obs = reset(self, scene, seed=seed)
        for name in _SPECS:
            _runtime(self._base, name).clear_latches()  # the settle steps are not part of the trial
            _runtime(self._base, name).n = 0
        return obs

    cls._ensure_sim, cls.reset, cls._tubekit = _ensure_sim, _reset, True


def _patch_scripted(module) -> None:
    """``yam_scripted`` plays the task file's closed-loop ``controller`` when it has one."""
    cls = module.YamScriptedPolicy
    if getattr(cls, "_tubekit", False):
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

    cls.reset, cls.act, cls._tubekit = _reset, _act, True


def _patch_demo(demo) -> None:
    if getattr(demo, "_tubekit", False):
        return
    grip_rotation = demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else grip_rotation(yaw)

    demo.grip_rotation, demo._tubekit = _grip_rotation, True


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
