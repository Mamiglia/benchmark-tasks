"""tapekit: a printed letter taped to the desk and a correction-tape dispenser lying beside it, for a yam_arena task.

yam_arena task files only take single primitives. This module adds one object, :class:`TapeRig`, spawned as a static
(``BASE``) Arena asset holding two bodies:

- ``sheet`` (kinematic): an A4 sheet, taped flat at its corners, with the printed letter (sheet_tex.png).
- ``tape`` (dynamic): the dispenser, a sideways-pull type. Body frame: origin at the centre of the applicator lip's
  pressing face; +x along the case from the lip to its back end; +y across the case (the way the lip lays tape: it
  is pulled sideways, along +y); +z out of the case's upper (logo) face when it lies flat. The lip's face is at 45
  degrees to the case, so it is flat on the paper when the case is held at 45 degrees, lip down and forward.

Before every physics step (240 Hz, a wrapper on ``InteractiveScene.write_data_to_sim``) the kit reads the
dispenser's pose and its PhysX contact force on the sheet and steps :class:`tapemodel.Tape`; the laid tape is drawn
as white strips on the sheet. ``object_states`` reports a :class:`TapeState`. Importing the module also hooks
``yam_arena.scripted``: when the task file defines ``controller(bases, dt)``, ``yam_scripted`` plays it, passing it
the observation (camera images and joint positions). yam_arena itself is not modified.
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

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from tapemodel import LAYOUT, Law, Tape  # noqa: E402

N_SEG = 160
S45 = math.sqrt(0.5)
LIP_NORMAL = np.array([-S45, 0.0, -S45])  # the lip face's outward normal, body frame
APPLY_B = np.array([0.0, 1.0, 0.0])  # the direction the lip lays tape when pulled, body frame
ACROSS_B = np.array([S45, 0.0, -S45])  # the tape's width on the lip face, body frame


@dataclass(frozen=True)
class TapeRig(Box):
    """``pos``: the sheet's top-left corner as printed (env frame, m); the line of text runs along -y, the page down
    along -x. ``tape_xy``: where the dispenser's lip lies (the case points away from the arms, lip forward)."""

    table_z: float = 0.0
    sheet: tuple[float, float] = (0.210, 0.297)  # A4 portrait: width (along -y), length (along -x)
    paper_t: float = 0.0002
    tape_xy: tuple[float, float] = (0.30, -0.27)
    case: tuple[float, float, float] = (0.073, 0.033, 0.014)  # behind the head: length, width, thickness
    case_x0: float = 0.012  # case starts this far back from the lip
    case_z0: float = -0.0012  # case's lower face (the lip pokes 1 mm below it)
    mass_kg: float = 0.016
    mu_lip: float = 0.15
    mu_case: float = 0.5
    mu_grip: float = 1.5
    mu_paper: float = 0.5
    law: Law = Law()

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a TapeRig is a fixed asset (mass None); its bodies carry their own")

    @property
    def paper_top(self) -> float:
        return self.table_z + self.paper_t

    def sheet_to_env(self, u_mm, w_mm) -> np.ndarray:
        return np.array([self.pos[0] - w_mm / 1000, self.pos[1] - u_mm / 1000])

    def env_to_sheet(self, xy) -> np.ndarray:
        return np.array([(self.pos[1] - xy[0 + 1]) * 1000, (self.pos[0] - xy[0]) * 1000])

    def typo_xy(self) -> np.ndarray:
        w = LAYOUT["words"][LAYOUT["typo"]]
        return self.sheet_to_env((w["u0"] + w["u1"]) / 2, (w["w0"] + w["w1"]) / 2)


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def lip_state(R: np.ndarray):
    """From the dispenser's rotation: the lip's tilt from flat about the tape's width (pitch) and about the pull
    direction (roll, one edge of the tape lifts) in deg, and the env-frame unit vectors (xy) of the pull direction
    and the tape's width."""
    across, apply = R @ ACROSS_B, R @ APPLY_B
    roll = math.degrees(math.asin(float(np.clip(across[2], -1, 1))))
    n = R @ LIP_NORMAL
    c = float(np.clip(-n[2] / max(math.cos(math.radians(roll)), 1e-6), -1, 1))
    pitch = math.degrees(math.acos(c))
    lead = apply[:2] / max(np.linalg.norm(apply[:2]), 1e-9)
    acr = across[:2] / max(np.linalg.norm(across[:2]), 1e-9)
    return pitch, roll, lead, acr


@dataclass(frozen=True)
class TapeState(ObjectState):
    """After a control step (env frame). ``pos`` is the dispenser's lip."""

    result: dict = field(default_factory=dict)  # tapemodel.score_tape
    lip: tuple = ()  # u, w (mm), force (N), pitch, roll (deg)


# -- spawning (Isaac Sim only) ---------------------------------------------------------------------------------------

_SPECS: dict[str, TapeRig] = {}
_PATHS: dict[str, str] = {}


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade, Vt
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
            if mu == s.mu_grip:
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
        if patch_r > 0:  # the finger pads' contact patch resists twisting, as real rubber pads do
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

    def cube(parent, name, centre, size, color=None, mu=None, rough=0.5, opacity=1.0, rot=None, patch_r=0.0):
        c = UsdGeom.Cube.Define(stage, f"{parent}/{name}")
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        if rot is not None:
            c.AddOrientOp().Set(Gf.Quatf(*map(float, rot)))  # w, x, y, z
        c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        if mu is not None:
            collide(c.GetPrim(), mu, patch_r=patch_r)
        if color is None:
            UsdGeom.Imageable(c.GetPrim()).MakeInvisible()
        else:
            bind_visual_material(str(c.GetPath()), look(color, rough, opacity), stage=stage)
        return c

    def textured_quad(parent, name, x0, x1, y0, y1, z, png) -> None:
        """A flat quad with the image on it: image left edge at y1, top edge at x1 (the page as printed)."""
        m = UsdGeom.Mesh.Define(stage, f"{parent}/{name}")
        m.CreatePointsAttr([Gf.Vec3f(x0, y1, z), Gf.Vec3f(x0, y0, z), Gf.Vec3f(x1, y0, z), Gf.Vec3f(x1, y1, z)])
        m.CreateFaceVertexCountsAttr([4])
        m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        m.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
        st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                  UsdGeom.Tokens.vertex)
        st.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
        mat = UsdShade.Material.Define(stage, f"{prim_path}/Looks/{name}")
        sh = UsdShade.Shader.Define(stage, f"{prim_path}/Looks/{name}/surf")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
        reader = UsdShade.Shader.Define(stage, f"{prim_path}/Looks/{name}/st")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tex = UsdShade.Shader.Define(stage, f"{prim_path}/Looks/{name}/tex")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(png))
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)

    z0 = s.table_z
    sw, sl = s.sheet
    X, Y = s.pos[0], s.pos[1]
    sheet, _ = body("sheet", (0.0, 0.0, z0), kinematic=True)
    cube(sheet, "paper", (X - sl / 2, Y - sw / 2, s.paper_t / 2), (sl, sw, s.paper_t), (0.96, 0.96, 0.95),
         mu=s.mu_paper, rough=0.95)
    t0, t1 = LAYOUT["tex_w_mm"]
    textured_quad(sheet, "print", X - t1 / 1000, X - t0 / 1000, Y - sw, Y, s.paper_t + 0.00003, HERE / "sheet_tex.png")
    for k, (cx, cy) in enumerate(((X, Y), (X, Y - sw), (X - sl, Y), (X - sl, Y - sw))):  # clear tape at the corners
        cube(sheet, f"corner{k}", (cx, cy, s.paper_t + 0.00005), (0.03, 0.019, 0.00006), (0.85, 0.85, 0.80),
             rough=0.3, opacity=0.7, rot=(math.cos(math.pi / 8), 0, 0, math.sin(math.pi / 8)))
    for i in range(N_SEG):
        c = cube(sheet, f"tape{i}", (X, Y, s.paper_t + 0.00008), (0.001, 0.001, 0.00005), (1.0, 1.0, 1.0),
                 rough=0.8, rot=(1, 0, 0, 0))
        UsdGeom.Imageable(c.GetPrim()).MakeInvisible()

    # the dispenser, lying flat on its lower face, lip toward +x (away from the arms): body x = env -x, y = env -y
    tx, ty = s.tape_xy
    tp, tprim = body("tape", (tx, ty, z0 - s.case_z0 + 0.0003), report=True)
    q = (0.0, 0.0, 0.0, 1.0)  # Gf.Quatd(w, x, y, z): 180 deg about z
    stage.GetPrimAtPath(tp).GetAttribute("xformOp:orient").Set(Gf.Quatd(*q))
    L, W, T = s.case
    cx = s.case_x0 + L / 2
    cz = s.case_z0 + T / 2
    cube(tp, "case", (cx, 0, cz), (L, W, T), None, mu=s.mu_grip, patch_r=0.006)
    cube(tp, "case_look", (cx, 0, cz), (L, W, T), (0.20, 0.45, 0.85), rough=0.3, opacity=0.75)
    cube(tp, "spool", (cx + 0.008, 0, cz), (0.026, 0.026, T * 0.8), (0.97, 0.97, 0.97), rough=0.7)
    cube(tp, "grip_band", (cx + 0.018, 0, cz), (0.030, W + 0.0004, T + 0.0004), (0.95, 0.50, 0.10), rough=0.6)
    cube(tp, "tongue", (0.0075, 0, 0.0025), (0.012, 0.008, 0.004), (0.95, 0.50, 0.10), mu=s.mu_case)
    # the lip: 3 x 8 mm pad, 1.5 mm thick, its face through the origin with outward normal LIP_NORMAL
    rot = (math.cos(math.pi / 8), 0.0, math.sin(math.pi / 8), 0.0)  # +45 deg about body y: its z -> -LIP_NORMAL
    cube(tp, "lip", (S45 * 0.00075, 0, S45 * 0.00075), (0.003, 0.008, 0.0015), (0.98, 0.98, 0.98), mu=s.mu_lip,
         rot=rot)
    mp = UsdPhysics.MassAPI.Apply(tprim)
    mp.CreateMassAttr(s.mass_kg)
    mp.CreateCenterOfMassAttr(Gf.Vec3f(cx, 0.0, cz))
    m = s.mass_kg
    mp.CreateDiagonalInertiaAttr(Gf.Vec3f(m * (W * W + T * T) / 12, m * (L * L + T * T) / 12, m * (L * L + W * W) / 12))
    return root.GetPrim()


def _arena_object(spec: TapeRig):
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
    """PhysX views, the tape rule and the drawn tape for one rig (env 0)."""

    def __init__(self, base, spec: TapeRig):
        from isaaclab_physx.physics import PhysxManager

        self.spec, self.base = spec, base
        self.root = _PATHS[spec.name]
        view = PhysxManager.get_physics_sim_view()
        self.bodies = view.create_rigid_body_view([f"{self.root}/tape"])
        self.start = _arr(self.bodies.get_transforms()).copy()
        self.contact = view.create_rigid_contact_view([f"{self.root}/tape"], filter_patterns=[[f"{self.root}/sheet"]],
                                                      max_contact_data_count=64)
        self.dt = float(base.physics_dt)
        self.origin = _arr(base.scene.env_origins)[0]
        self.debug = bool(os.environ.get("TAPE_DEBUG"))
        self.shown: dict = {}
        self.clear()

    def reset(self) -> None:
        cur = self.bodies.get_transforms()
        idx = _like(cur, np.arange(1), "int32")
        self.bodies.set_transforms(_like(cur, self.start), idx)
        self.bodies.set_velocities(_like(cur, np.zeros((1, 6))), idx)
        self.clear()

    def clear(self) -> None:
        self.tape = Tape(law=self.spec.law)
        self.segs: list = []  # drawn strips: [a_uw, b_uw, across_uw]
        self.lip = (0.0, 0.0, 0.0, 0.0, 0.0)
        self.forces: list = []
        self.n = 0
        self._draw(force=True)

    def say(self, msg: str) -> None:
        print(f"[tapekit] t {self.n * self.dt:6.2f} {msg}", flush=True)

    def _pose(self):
        t = _arr(self.bodies.get_transforms())[0]
        return t[:3] - self.origin, quat_to_matrix(t[3:7])

    def physics_step(self) -> None:
        s = self.spec
        self.n += 1
        p, R = self._pose()
        f = _arr(self.contact.get_contact_force_matrix(dt=self.dt)).reshape(-1, 3)
        # PhysX reports a light sliding contact's force on most substeps but not all: the lip's force is the largest
        # reading over the last 12 substeps (50 ms)
        self.forces = (self.forces + [float(np.linalg.norm(f.sum(axis=0)))])[-12:]
        force = max(self.forces)
        pitch, roll, lead, acr = lip_state(R)
        uw = s.env_to_sheet(p[:2])
        lead_uw = np.array([-lead[1], -lead[0]])
        acr_uw = np.array([-acr[1], -acr[0]])
        laid = self.tape.laid_mm
        strokes = self.tape.strokes
        msg = self.tape.step(self.dt, uw, force, pitch, roll, lead_uw, acr_uw)
        if self.tape.laid_mm > laid:
            seg = self.segs[-1] if self.segs and self.tape.strokes == strokes else None
            if seg is None or np.linalg.norm(seg[1] - seg[0]) >= 1.0:
                start = seg[1] if seg is not None else self.tape.stroke_from
                self.segs.append([np.array(start, float), uw.copy(), acr_uw.copy()])
            else:
                seg[1] = uw.copy()
        self.lip = (uw[0], uw[1], force, pitch, roll)
        if msg:
            self.say(f"{msg} (force {force:.2f} N, pitch {pitch:.1f}, roll {roll:.1f} deg)")
        if self.debug and self.n % 24 == 0:
            self.say(f"lip u {uw[0]:7.2f} w {uw[1]:7.2f} z {(p[2] - s.paper_top) * 1000:6.2f} mm force {force:.2f} "
                     f"pitch {pitch:5.1f} roll {roll:5.1f} laid {self.tape.laid_mm:.1f} mm")

    def _vis(self, path: str, on: bool) -> None:
        if self.shown.get(path) == on:
            return
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdGeom

        img = UsdGeom.Imageable(get_current_stage().GetPrimAtPath(path))
        img.MakeVisible() if on else img.MakeInvisible()
        self.shown[path] = on

    def _place(self, path: str, a, b, acr) -> None:
        key = (tuple(np.round(a, 3)), tuple(np.round(b, 3)))
        if self.shown.get(path + ":pose") == key:
            return
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        s = self.spec
        pa, pb = s.sheet_to_env(*a), s.sheet_to_env(*b)
        mid = (pa + pb) / 2
        d = pb - pa
        ln = float(np.linalg.norm(d)) + s.law.tape_w / 1000 * 0.05
        yaw = math.atan2(d[1], d[0]) if np.linalg.norm(d) > 1e-6 else math.atan2(-1.0, 0.0)
        prim = get_current_stage().GetPrimAtPath(path)
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(float(mid[0]), float(mid[1]), s.paper_t + 0.00008))
            elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                op.Set(Gf.Quatf(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))
            elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
                op.Set(Gf.Vec3f(max(ln, 0.0002), s.law.tape_w / 1000, 0.00005))
        self.shown[path + ":pose"] = key

    def _draw(self, force: bool = False) -> None:
        if force:
            self.shown = {}
        r = self.root
        for i in range(N_SEG):
            on = i < len(self.segs)
            if on:
                self._place(f"{r}/sheet/tape{i}", *self.segs[i])
            self._vis(f"{r}/sheet/tape{i}", on)

    def state(self) -> TapeState:
        from tapemodel import score_tape

        self._draw()
        p, R = self._pose()
        result = score_tape(self.tape)
        if self.debug and result != getattr(self, "_last_result", None):
            self.say(f"result {result}")
            self._last_result = result
        return TapeState(pos=p, quat_xyzw=np.array([0.0, 0, 0, 1]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                         low=p - 0.01, high=p + 0.01, result=result, lip=self.lip)


_RT: dict[str, _Runtime] = {}


def _runtime(base, name: str) -> _Runtime:
    if name not in _RT:
        _RT[name] = _Runtime(base, _SPECS[name])
    return _RT[name]


# -- hooks ---------------------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_tapekit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, TapeRig):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _runtime(environment.unwrapped, n).state() for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._tapekit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_tapekit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for name in _SPECS:
            _runtime(base, name).physics_step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._tapekit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_tapekit", False):
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

    cls._ensure_sim, cls.reset, cls._tapekit = _ensure_sim, _reset, True


def _patch_scripted(module) -> None:
    """``yam_scripted`` plays the task file's closed-loop ``controller`` when it has one."""
    cls = module.YamScriptedPolicy
    if getattr(cls, "_tapekit", False):
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

    cls.reset, cls.act, cls._tapekit = _reset, _act, True


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
