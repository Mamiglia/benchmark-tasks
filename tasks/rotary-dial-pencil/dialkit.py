"""dialkit: a rotary-dial telephone and its handset for a yam_arena task file.

yam_arena task files take single primitives only. This module adds two object types and hooks yam_arena (without
modifying it):

- :class:`Phone`: a FLOATING-base articulation (the phone stands loose on the desk). Root ``base`` (body, sloped
  front, cradle, number plate and finger stop, all colliders); ``wheel`` (the finger wheel) on the revolute
  ``dial`` about the dial face's normal. The wheel's pockets are the gaps between radial webs, a hub and a rim (its
  colliders, hidden); what is drawn is a disc with ten round holes over the numbered plate.
- :class:`Handset`: the handset lying off its cradle on the desk (static colliders; its cord is not simulated).
- :class:`Pencil`: a plain free box (yam_arena's own) for the pencil, with its grip friction set and drawn as a
  hexagonal pencil with ferrule and eraser.

Before every physics step (240 Hz, via a wrapper on ``InteractiveScene.write_data_to_sim``) the kit:

1. applies the dial's return spring (``dialmodel.spring_torque``) as a joint effort and, while the wheel runs back,
   the governor: a brake proportional to speed that holds the run-back at ``GOV_SPEED`` (the real governor only
   acts on the run-back: a one-way clutch);
2. feeds the wheel angle to ``dialmodel.Dialer``, which records each pull and the digit it dials;
3. latches the fouls: the phone slid 3 cm, tipped 10 degrees or lifted 1 cm; a gripper finger or the gripper
   itself on the dial (within 15 mm of the face, inside the wheel's radius + 4 mm).

``object_states`` reports a :class:`PhoneState`. Importing the module also hooks ``yam_arena.scripted`` so that a
task file's ``controller(bases, dt)`` is played by ``yam_scripted``.
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
import dialmodel as M  # noqa: E402

ARMS = ("left_robot", "right_robot")
TCP_OFFSET = 0.134
IVORY, IVORY_DARK, PLATE_C, INK = (0.88, 0.84, 0.72), (0.74, 0.70, 0.60), (0.97, 0.97, 0.95), (0.08, 0.08, 0.08)
CHROME = (0.75, 0.76, 0.78)
DEBUG = bool(os.environ.get("DIAL_DEBUG"))

# seven-segment glyphs: segments a b c d e f g
SEGS = {"a": ((0, 1), 0), "b": ((0.5, 0.5), 1), "c": ((0.5, -0.5), 1), "d": ((0, -1), 0), "e": ((-0.5, -0.5), 1),
        "f": ((-0.5, 0.5), 1), "g": ((0, 0), 0)}
GLYPH = {0: "abcdef", 1: "bc", 2: "abged", 3: "abgcd", 4: "fgbc", 5: "afgcd", 6: "afgedc", 7: "abc", 8: "abcdefg",
         9: "abcfgd"}


@dataclass(frozen=True)
class Phone(Box):
    """``pos``: the body's underside centre (world); ``yaw`` its turn about the vertical (0: front faces the arms)."""

    friction: float = 0.8  # rubber feet on the desk


@dataclass(frozen=True)
class Handset(Box):
    """``pos``: the centre of the handset lying on the desk; its length along ``yaw``."""


@dataclass(frozen=True)
class Pencil(Box):
    """A free hexagonal pencil: ``size`` its box (length along x, lying; or along z, standing); eraser at the -end.
    Collided as its across-flats square prism (the pads seat on two flats). Its friction is ``grip_mu`` with the
    "max" combine: the simulated gripper squeezes a 7 mm part with only about 17.5 N per finger (its finger drive
    saturates at fully closed), the real YAM with about 40 N; 2.0 x 17.5 N gives the same friction capacity as the
    real rubber pads (about 0.9 x 40 N). Without it the pencil twists between the pads under about 1 N at its end."""

    grip_mu: float = 2.0


@dataclass(frozen=True)
class PhoneState(ObjectState):
    theta_deg: float = 0.0  # finger wheel, from rest, clockwise
    pulls: tuple = ()  # (theta_max, pulses, held) per completed pull
    digits: tuple = ()
    fouls: tuple = ()


def quat_wxyz_from_matrix(R) -> tuple:
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return (w, x, y, z)


def rot_z(deg) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(rad) -> np.ndarray:
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def wheel_mesh(n_ang: int = 240, n_rad: int = 28):
    """Top face of the finger wheel: a disc of WHEEL_R with ten round finger holes (dial frame, z = 0)."""
    holes = [M.dial_point(M.hole_psi(n))[:2] for n in range(1, 11)]
    rs = np.linspace(M.HUB_R * 0.0 + 0.004, M.WHEEL_R, n_rad + 1)
    angs = np.linspace(0, 2 * math.pi, n_ang + 1)
    pts, faces = [], []
    idx = {}

    def vid(i, j):
        key = (i, j % n_ang)
        if key not in idx:
            idx[key] = len(pts)
            r, a = rs[i], angs[j % n_ang]
            pts.append((r * math.cos(a), r * math.sin(a), 0.0))
        return idx[key]

    for i in range(n_rad):
        for j in range(n_ang):
            rm, am = (rs[i] + rs[i + 1]) / 2, (angs[j] + angs[j + 1]) / 2
            c = np.array([rm * math.cos(am), rm * math.sin(am)])
            if any(np.linalg.norm(c - h) < M.HOLE_D / 2 for h in holes):
                continue
            faces.append((vid(i, j), vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)))
    return pts, faces


# -- spawning -------------------------------------------------------------------------------------------------------

_SPECS: dict[str, object] = {}
_PATHS: dict[str, str] = {}


def _tools(prim_path, stage):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    looks: dict = {}
    dbl = UsdGeom.XformOp.PrecisionDouble

    def material(name, mu, restitution=0.0) -> str:
        path = f"{prim_path}/Looks/{name}"
        cfg = sim_utils.RigidBodyMaterialCfg(static_friction=mu, dynamic_friction=mu, restitution=restitution)
        cfg.func(path, cfg)
        return path

    def look(color, rough=0.5, metal=0.0) -> str:
        key = (tuple(round(c, 4) for c in color), rough, metal)
        if key not in looks:
            looks[key] = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal)
            vis.func(looks[key], vis)
        return looks[key]

    def collide(prim, mat, offset=0.001):
        UsdPhysics.CollisionAPI.Apply(prim)
        col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        col.CreateContactOffsetAttr(offset)
        col.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), mat, stage=stage)

    def place(g, centre, R=None, scale=None):
        g.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*map(float, centre)))
        if R is not None:
            g.AddOrientOp(precision=dbl).Set(Gf.Quatd(*quat_wxyz_from_matrix(R)))
        if scale is not None:
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, scale)))

    def finish(g, color, mat, hidden, rough=0.5, metal=0.0):
        if mat:
            collide(g.GetPrim(), mat)
        if hidden:
            UsdGeom.Imageable(g.GetPrim()).MakeInvisible()
        else:
            bind_visual_material(str(g.GetPath()), look(color, rough, metal), stage=stage)

    def cube(path, centre, size, color=GREY_, mat=None, R=None, hidden=False, rough=0.5, metal=0.0):
        c = UsdGeom.Cube.Define(stage, path)
        c.CreateSizeAttr(1.0)
        place(c, centre, R, size)
        finish(c, color, mat, hidden, rough, metal)
        return c

    def cyl(path, centre, radius, height, color=GREY_, mat=None, R=None, axis="Z", hidden=False, rough=0.5, metal=0.0):
        c = UsdGeom.Cylinder.Define(stage, path)
        c.CreateAxisAttr(axis)
        c.CreateRadiusAttr(float(radius))
        c.CreateHeightAttr(float(height))
        place(c, centre, R)
        finish(c, color, mat, hidden, rough, metal)
        return c

    def mesh(path, pts, faces, color, centre, R=None):
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
        m.CreateFaceVertexCountsAttr([len(f) for f in faces])
        m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        m.CreateDoubleSidedAttr(True)
        place(m, centre, R)
        bind_visual_material(str(m.GetPath()), look(color), stage=stage)
        return m

    def body(path, pos, R, mass, inertia=None) -> str:
        xf = UsdGeom.Xform.Define(stage, path)
        place(xf, pos, R)
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mapi = UsdPhysics.MassAPI.Apply(prim)
        mapi.CreateMassAttr(mass)
        if inertia is not None:
            mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
        return path

    return material, cube, cyl, mesh, body


GREY_ = (0.6, 0.6, 0.6)


def _spawn_phone(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    s: Phone = _SPECS[cfg.rig_name]
    _PATHS[s.name] = prim_path
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=dbl).Set(Gf.Quatd(w, x, y, z))
    material, cube, cyl, mesh, body = _tools(prim_path, stage)
    plastic = material("plastic", 0.5)
    feet = material("feet", s.friction)
    eraser = material("plate", 0.6)

    D, Wd, H = M.BODY
    base = body(f"{prim_path}/base", (0, 0, 0), None, M.PHONE_MASS)
    cube(f"{base}/slab", (0, 0, M.FRONT_Z / 2), (D, Wd, M.FRONT_Z), IVORY, feet)
    top_x0 = M.FRONT_X + M.SLOPE_LEN * math.cos(M.TILT)
    top_z = M.FRONT_Z + M.SLOPE_LEN * math.sin(M.TILT)
    cube(f"{base}/top", ((top_x0 + D / 2) / 2, 0, (M.FRONT_Z + top_z + 0.012) / 2),
         (D / 2 - top_x0, Wd - 0.02, top_z + 0.012 - M.FRONT_Z), IVORY, plastic)
    Rs = rot_y(-M.TILT)  # slope direction: phone x turned up by TILT
    sdir = Rs[:, 0]
    nrm = Rs[:, 2]
    mid = np.array([M.FRONT_X, 0, M.FRONT_Z]) + sdir * M.SLOPE_LEN / 2 - nrm * 0.02
    cube(f"{base}/slope", mid, (M.SLOPE_LEN, Wd - 0.01, 0.04), IVORY, plastic, R=Rs)
    # cradle: two horns with their hook-switch plungers up (handset off), a saddle between
    for i, yy in enumerate((0.072, -0.072)):
        cube(f"{base}/horn_{i}", (0.045, yy, top_z + 0.012 + 0.017), (0.06, 0.045, 0.034), IVORY, plastic)
        cube(f"{base}/plunger_{i}", (0.045, yy * 0.72, top_z + 0.012 + 0.01), (0.012, 0.012, 0.02), INK)
    cube(f"{base}/saddle", (0.045, 0, top_z + 0.012 + 0.004), (0.06, 0.10, 0.008), IVORY_DARK, plastic)

    # dial frame in the phone frame
    RD = M.dial_axes(yaw=0.0)
    C = np.asarray(M.DIAL_C)

    def at(p):  # dial frame -> phone frame
        return C + RD @ np.asarray(p, float)

    cyl(f"{base}/number_plate", at((0, 0, M.PLATE_TOP / 2)), M.WHEEL_R - 0.001, M.PLATE_TOP, PLATE_C, eraser, R=RD)
    cyl(f"{base}/bezel", at((0, 0, M.PLATE_TOP / 2 - 0.0002)), M.WHEEL_R + 0.004, M.PLATE_TOP, CHROME, None, R=RD,
        metal=0.6, rough=0.3)
    for n in range(1, 11):  # the numbers, read through the holes
        d = n % 10
        c = M.dial_point(M.hole_psi(n))
        for sg in GLYPH[d]:
            (ox, oy), vert = SEGS[sg]
            hgt, wid = 0.0035, 0.0008
            p = c + np.array([ox * 0.0035, oy * 0.0035, M.PLATE_TOP + 0.0002])
            size = (wid, hgt, 0.0003) if vert else (hgt, wid, 0.0003)
            cube(f"{base}/n{d}_{sg}", at(p), size, INK, None, R=RD)
    # finger stop: its end stands over the hole circle at STOP_PSI; its arm reaches out over the bezel
    Rst = RD @ rot_z(M.STOP_PSI)
    zc = (M.STOP_Z0 + M.STOP_Z1) / 2
    cube(f"{base}/stop_end", at(M.dial_point(M.STOP_PSI, M.HOLE_R + 0.002, zc)),
         (0.016, M.STOP_T, M.STOP_Z1 - M.STOP_Z0), CHROME, plastic, R=Rst, metal=0.7, rough=0.3)
    cube(f"{base}/stop_arm", at(M.dial_point(M.STOP_PSI, M.WHEEL_R + 0.002, M.STOP_Z1 - 0.002)),
         (0.012, M.STOP_T, 0.004), CHROME, None, R=Rst, metal=0.7, rough=0.3)
    cube(f"{base}/stop_post", at(M.dial_point(M.STOP_PSI, M.WHEEL_R + 0.006, M.STOP_Z1 / 2)),
         (0.004, M.STOP_T, M.STOP_Z1), CHROME, None, R=Rst, metal=0.7, rough=0.3)

    # the finger wheel (its own body, frame = dial frame)
    wheel = body(f"{prim_path}/wheel", at((0, 0, 0)), RD, M.WHEEL_MASS, inertia=(3.5e-5, 3.5e-5, 6.5e-5))
    hz, hh = (M.WHEEL_Z0 + M.WHEEL_Z1) / 2, M.WHEEL_Z1 - M.WHEEL_Z0
    cyl(f"{wheel}/hub", (0, 0, hz), M.HUB_R, hh, IVORY, plastic, hidden=True)
    web_r = (M.HUB_R + M.WHEEL_R - 0.0025) / 2
    web_len = M.WHEEL_R - 0.0025 - M.HUB_R + 0.002
    for n in range(1, 10):
        a = M.hole_psi(n) + M.PITCH / 2
        cube(f"{wheel}/web_{n}", M.dial_point(a, web_r, hz), (web_len, M.WEB_T, hh), IVORY, plastic, R=rot_z(a),
             hidden=True)
    for i, a in enumerate(np.arange(M.hole_psi(10) + 25.0, M.hole_psi(1) + 360.0 - 14.0, 10.0)):  # the solid part
        cube(f"{wheel}/solid_{i}", M.dial_point(a, web_r, hz), (web_len, 2 * web_r * math.sin(math.radians(5.5)), hh),
             IVORY, plastic, R=rot_z(a), hidden=True)
    for i in range(24):
        a = i * 15.0
        cube(f"{wheel}/rim_{i}", M.dial_point(a, M.WHEEL_R - 0.00125, hz),
             (0.0025, 2 * M.WHEEL_R * math.sin(math.radians(7.6)), hh), IVORY, plastic, R=rot_z(a), hidden=True)
    pts, faces = wheel_mesh()
    mesh(f"{wheel}/face", pts, faces, (0.92, 0.90, 0.84), (0, 0, M.WHEEL_Z1))
    # the holes' walls, drawn: a short dark tube per hole
    for n in range(1, 11):
        cyl(f"{wheel}/hole_wall_{n}", M.dial_point(M.hole_psi(n), M.HOLE_R, M.WHEEL_Z1 - 0.0005), M.HOLE_D / 2 + 0.0006,
            0.001, IVORY_DARK)
    cyl(f"{wheel}/centre_card", (0, 0, M.WHEEL_Z1 + 0.0003), M.HUB_R - 0.008, 0.0006, PLATE_C)

    j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/dial")
    j.CreateAxisAttr("Z")
    j.CreateBody0Rel().SetTargets([Sdf.Path(base)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(wheel)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, C)))
    j.CreateLocalRot0Attr(Gf.Quatf(*quat_wxyz_from_matrix(RD)))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(-M.THETA_MAX)
    j.CreateUpperLimitAttr(0.0)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(0.0)
    d.CreateDampingAttr(0.0)

    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(base))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(base))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _spawn_handset(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdGeom

    s: Handset = _SPECS[cfg.rig_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    dbl = UsdGeom.XformOp.PrecisionDouble
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*s.pos))
    root.AddOrientOp(precision=dbl).Set(Gf.Quatd(math.cos(s.yaw / 2), 0, 0, math.sin(s.yaw / 2)))
    material, cube, cyl, mesh, body = _tools(prim_path, stage)
    plastic = material("plastic", 0.5)
    cube(f"{prim_path}/handle", (0, 0, -0.005), (0.13, 0.04, 0.03), IVORY, plastic)
    for i, xx in enumerate((-0.085, 0.085)):
        cyl(f"{prim_path}/cup_{i}", (xx, 0, 0.0), 0.03, 0.04, IVORY, plastic)
        cyl(f"{prim_path}/grille_{i}", (xx, 0, 0.0202), 0.022, 0.0005, IVORY_DARK)
    return root.GetPrim()


def _pencil_post(prim_path: str, s: Pencil) -> None:
    """After the plain box is spawned: the grip friction on its collider, the box hidden, a pencil drawn instead."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = get_current_stage()
    roots = [p for p in stage.Traverse() if p.GetName() == prim_path.rsplit("/", 1)[-1]
             and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    for root in roots:
        n_col = 0
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material

        mpath = f"{root.GetPath()}/Looks/pencil_grip"
        mcfg = sim_utils.RigidBodyMaterialCfg(static_friction=s.grip_mu, dynamic_friction=s.grip_mu,
                                              friction_combine_mode="max")
        mcfg.func(mpath, mcfg)
        for p in Usd.PrimRange(root):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                bind_physics_material(str(p.GetPath()), mpath, stage=stage)
                n_col += 1
            if p.IsA(UsdGeom.Gprim):
                UsdGeom.Imageable(p).MakeInvisible()
        path = str(root.GetPath())
        material, cube, cyl, mesh, body = _tools(path, stage)
        w, L = min(s.size), max(s.size)
        look = UsdGeom.Xform.Define(stage, f"{path}/look")  # drawn along its z; turned onto the long axis
        if s.size[0] == L:
            look.AddOrientOp().Set(Gf.Quatf(math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0))  # z -> x
        path = f"{path}/look"
        r = w / 2 / math.cos(math.pi / 6)  # hexagon corner radius
        z0, z1 = -L / 2 + 0.020, L / 2 - 0.0005  # yellow barrel from the ferrule to the (unsharpened) end
        pts, faces = [], []
        for k in range(6):
            a = k * math.pi / 3
            pts += [(r * math.cos(a), r * math.sin(a), z0), (r * math.cos(a), r * math.sin(a), z1)]
        for k in range(6):
            i, j = 2 * k, 2 * ((k + 1) % 6)
            faces.append((i, j, j + 1, i + 1))
        faces += [tuple(2 * k for k in range(6)), tuple(2 * k + 1 for k in range(6))]
        mesh(f"{path}/barrel", pts, faces, s.color, (0, 0, 0))
        cyl(f"{path}/eraser", (0, 0, -L / 2 + 0.006), w / 2 - 0.0002, 0.012, (0.93, 0.55, 0.60))
        cyl(f"{path}/ferrule", (0, 0, -L / 2 + 0.016), w / 2 + 0.0002, 0.008, (0.80, 0.80, 0.78), metal=0.8, rough=0.3)
        print(f"[dialkit] pencil drawn at {path}; friction {s.grip_mu} on {n_col} colliders",
              flush=True)


def _arena_object(spec):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    if isinstance(spec, Handset):
        @configclass
        class _HandsetCfg(SpawnerCfg):
            func: object = clone(_spawn_handset)
            rig_name: str = ""

        return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=_HandsetCfg(rig_name=spec.name),
                      initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))

    @configclass
    class _PhoneCfg(SpawnerCfg):
        func: object = clone(_spawn_phone)
        rig_name: str = ""

    class PhoneObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {"dial": ImplicitActuatorCfg(joint_names_expr=["dial"], stiffness=0.0, damping=2e-5,
                                                     armature=2e-4)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return PhoneObject(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=_PhoneCfg(rig_name=spec.name),
                       initial_pose=Pose(position_xyz=tuple(spec.pos),
                                         rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))


# -- runtime --------------------------------------------------------------------------------------------------------


def _arr(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


@dataclass
class Snapshot:
    """The latest physics step, for the scripted demo (simulator state, which a scripted demo may read)."""

    t: float = 0.0
    theta: float = 0.0  # deg
    omega: float = 0.0  # deg/s, + = pulled further
    dial_origin: np.ndarray = field(default_factory=lambda: np.zeros(3))  # world, env-relative
    dial_R: np.ndarray = field(default_factory=lambda: np.eye(3))
    pencil: tuple = ()  # (pos, R) env-relative
    tcp: dict = field(default_factory=dict)  # arm -> (pos, R)
    hand_clear: float = 1.0  # m, the closest gripper point's margin outside the dial's foul zone
    pulls: int = 0
    digits: tuple = ()


SNAP = Snapshot()


class _Runtime:
    def __init__(self, base, phone: Phone):
        self.base, self.phone = base, phone
        self.asset = base.scene[phone.name]
        self.jid = list(self.asset.joint_names).index("dial")
        self.dt = float(base.physics_dt)
        self.origin = _arr(base.scene.env_origins)[0]
        self.pad_ids = {arm: [base.scene[arm].find_bodies(b)[0][0] for b in ("tip_left", "tip_right", "gripper")]
                        for arm in ARMS}
        self.pencil = base.scene["pencil"] if "pencil" in base.scene.keys() else None
        self.reset_latches()

    def reset_latches(self):
        self.dialer = M.Dialer()
        self.fouls: set[str] = set()
        self.n = 0
        self.root0 = None
        self.worst = {"slide_mm": 0.0, "tilt_deg": 0.0, "lift_mm": 0.0}

    def step(self):
        import torch

        self.n += 1
        t = self.n * self.dt
        q = float(_arr(self.asset.data.joint_pos)[0][self.jid])
        qd = float(_arr(self.asset.data.joint_vel)[0][self.jid])
        theta = max(-q, 0.0)
        tau = M.spring_torque(theta)  # + q is back toward rest
        if qd > 0:  # running back: the governor brakes it to GOV_SPEED
            tau -= tau / M.GOV_SPEED * qd
        self.asset.actuators.target_command.set_effort_index(
            value=torch.tensor([[tau]], dtype=torch.float32, device=self.base.device), joint_ids=[self.jid])
        th_deg = math.degrees(theta)
        self.dialer.step(t, th_deg)

        root = _arr(self.asset.data.root_link_pose_w)[0]
        Rr = quat_to_matrix(root[3:7])
        pos = root[:3] - self.origin
        if self.root0 is None:
            self.root0 = pos.copy()
        slide = float(np.linalg.norm(pos[:2] - self.root0[:2])) * 1000
        tilt = math.degrees(math.acos(float(np.clip(Rr[2, 2], -1, 1))))
        lift = (pos[2] - self.root0[2]) * 1000
        for k, v in (("slide_mm", slide), ("tilt_deg", tilt), ("lift_mm", lift)):
            self.worst[k] = max(self.worst[k], v)
        if slide > M.MOVE_MM:
            self.fouls.add("phone moved")
        if tilt > M.TIP_DEG:
            self.fouls.add("phone tipped")
        if lift > M.LIFT_MM:
            self.fouls.add("phone lifted")

        RD = Rr @ M.dial_axes(yaw=0.0)
        oD = pos + Rr @ np.asarray(M.DIAL_C)
        clear = 1.0
        SNAP.tcp = {}
        for arm in ARMS:
            poses = _arr(self.base.scene[arm].data.body_link_pose_w)[0]
            ids = self.pad_ids[arm]
            g = poses[ids[2]]
            Rg = quat_to_matrix(g[3:7])
            tcp = g[:3] - self.origin + Rg[:, 2] * TCP_OFFSET
            SNAP.tcp[arm.split("_")[0]] = (tcp, Rg)
            for p in [poses[i, :3] - self.origin for i in ids[:2]] + [tcp]:
                b = RD.T @ (p - oD)
                margin = max(math.hypot(b[0], b[1]) - (M.WHEEL_R + 0.004), b[2] - M.HAND_H)
                clear = min(clear, margin)
                if margin < 0:
                    self.fouls.add(f"{arm.split('_')[0]} gripper on the dial")
        SNAP.t, SNAP.theta, SNAP.omega = t, th_deg, -math.degrees(qd)
        SNAP.dial_origin, SNAP.dial_R, SNAP.hand_clear = oD, RD, clear
        SNAP.pulls, SNAP.digits = len(self.dialer.pulls), tuple(self.dialer.digits)
        if self.pencil is not None:
            pp = _arr(self.pencil.data.root_link_pose_w)[0]
            SNAP.pencil = (pp[:3] - self.origin, quat_to_matrix(pp[3:7]))
        if DEBUG and self.n % 24 == 0:
            print(f"[dialkit] t {t:6.2f} theta {th_deg:6.1f} omega {SNAP.omega:7.1f} pulls {len(self.dialer.pulls)} "
                  f"digits {self.dialer.digits} clear {clear * 1000:.0f} mm fouls {sorted(self.fouls)} "
                  f"worst {dict((k, round(v, 1)) for k, v in self.worst.items())}", flush=True)

    def state(self) -> PhoneState:
        root = _arr(self.asset.data.root_link_pose_w)[0]
        pos = root[:3] - self.origin
        half = np.asarray(M.BODY) / 2
        return PhoneState(pos=pos, quat_xyzw=root[3:7], lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                          low=pos - half * (1, 1, 0), high=pos + half * (1, 1, 2), theta_deg=SNAP.theta,
                          pulls=tuple((round(p.theta_max, 1), p.pulses, p.held) for p in self.dialer.pulls),
                          digits=tuple(self.dialer.digits), fouls=tuple(sorted(self.fouls)))


_RT: dict[str, _Runtime] = {}


def _runtime(base) -> _Runtime | None:
    if "rt" not in _RT:
        phones = [s for s in _SPECS.values() if isinstance(s, Phone)]
        if not phones:
            return None
        _RT["rt"] = _Runtime(base, phones[0])
    return _RT["rt"]


# -- hooks ----------------------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_dialkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, Pencil):
            obj = task_object(Box(spec.name, pos=spec.pos, color=spec.color, mass=spec.mass, yaw=spec.yaw,
                                  size=spec.size))
            func = obj.spawner_cfg.func

            def spawn(prim_path, cfg, *a, **k):
                prim = func(prim_path, cfg, *a, **k)
                _pencil_post(prim_path, spec)
                return prim

            obj.spawner_cfg.func = spawn
            obj.object_cfg.spawn.func = spawn  # the cfg the scene spawns from (built when the Object was made)
            return obj
        if isinstance(spec, (Phone, Handset)):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        rt = _runtime(environment.unwrapped)
        for n in names:
            if n in _SPECS:
                s = _SPECS[n]
                if isinstance(s, Phone):
                    out[n] = rt.state()
                else:
                    p = np.asarray(s.pos, float)
                    out[n] = ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                                         ang_vel=np.zeros(3), low=p - 0.05, high=p + 0.05)
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._dialkit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_dialkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        rt = _runtime(base)
        if rt is not None:
            rt.step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._dialkit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_dialkit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def _ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
        return out

    def _reset(self, scene, *, seed=None):
        self._ensure_sim(None)
        rt = _runtime(self._base)
        obs = reset(self, scene, seed=seed)
        if rt is not None:
            rt.reset_latches()  # the settle steps are not part of the trial
        return obs

    cls._ensure_sim, cls.reset, cls._dialkit = _ensure_sim, _reset, True


def _patch_scripted(module) -> None:
    """``yam_scripted`` plays the task file's closed-loop ``controller`` when it has one."""
    cls = module.YamScriptedPolicy
    if getattr(cls, "_dialkit", False):
        return
    reset, act = cls.reset, cls.act

    def _reset(self, scene):
        task = self._task
        make = getattr(task.module, "controller", None) if task is not None else None
        self._ctl = make(dict(self._bases), self._dt) if make is not None else None
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

    cls.reset, cls.act, cls._dialkit = _reset, _act, True


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
