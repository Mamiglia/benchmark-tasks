"""liquid: a reduced-order model of two layered liquids in an open test tube, driven by the tube's motion.

Pure NumPy (no Isaac Sim), so the tests run it directly. :class:`TubeLiquid` is stepped at the physics rate with the
pose of the tube and gives, at every step, how high the liquid stands at the wall, whether it has reached the mouth
(a spill, latched), and how far the two layers have mixed.

The model, in the tube's frame (axis from the bottom to the mouth):

- **Apparent gravity.** The liquid feels the specific force ``f = a + g z`` (``a``: the acceleration of the liquid's
  centre, from the tube's measured motion, low-passed at ``lpf_hz``). Its axial part ``f_ax`` presses the liquid onto
  the bottom; its sideways part ``f_lat`` tips the free surface.
- **Slosh.** The free surface's first sloshing mode in a circular tube is a damped oscillator ``x'' + 2 zeta w x' +
  w^2 x = -f_lat`` with ``w^2 = 1.841 f_ax / R tanh(1.841 h / R)`` (the standard first antisymmetric mode, Abramson,
  NASA SP-106; about 8 Hz here). Held still, ``w^2 x = -f_lat``: the surface tilts by ``tan phi = |f_lat| / f_ax``,
  the quasi-static result. The mode's amplitude (displacement and velocity) gives the dynamic surface slope
  ``tan phi = w^2 A / f_ax``, so a sharp jerk or shaking near 8 Hz raises the liquid on the wall well above the
  quasi-static height.
- **Wall height.** A plane at that slope through the liquid's volume reaches ``wall_top(tan phi)`` up the wall:
  ``h + R tan phi`` while the bottom stays covered, then a wedge (computed exactly; see :func:`wall_top_table`). As
  the tube nears horizontal, ``tan phi`` grows without bound and the liquid runs to the mouth, as a real tube does.
- **Column lift.** When ``f_ax`` turns negative (the tube pulled down faster than gravity, or turned over), the
  liquid leaves the bottom and moves as a free body along the tube (``s'' = -f_ax``) until it lands again. A stroke
  that throws it to the mouth spills it; one that drops it back onto the bottom from 5 mm or more churns the layers.
- **Spill.** Latched the first time the liquid reaches the mouth: ``wall_top + s >= depth``.
- **Mixing.** The dense lower layer is turned over into the upper one each time the free surface swings past
  ``mix_tan`` (30 degrees from level) toward the other side from the last stroke, and each time the column drops
  back from a lift of 5 mm or more. Each such stroke leaves ``1 - mix_per_stroke`` of the remaining contrast between
  the two layers. ``contrast`` runs from 1 (two clear layers) to 0 (one even colour); mixed below ``mixed_contrast``.
  The real-trial check is a measurement of the same contrast (README): the top and the bottom of the column differ
  by less than 10% in the red channel.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.81


def wall_top_table(radius: float, fill: float, n: int = 400, t_max: float = 400.0) -> tuple[np.ndarray, np.ndarray]:
    """(tan phi grid, height the liquid reaches up the wall) for a flat-bottomed tube of ``radius`` holding liquid to
    ``fill`` when level, with the free surface at slope ``tan phi`` to the tube's cross-section."""
    ts = np.r_[0.0, np.geomspace(1e-3, t_max, n - 1)]
    xs = np.linspace(-radius, radius, 801)
    chord = 2.0 * np.sqrt(np.maximum(radius**2 - xs**2, 0.0))
    dx = xs[1] - xs[0]
    vol = math.pi * radius**2 * fill
    tops = []
    for t in ts:
        lo, hi = -t * radius, fill + t * radius + 1e-9  # z0: the surface's height on the tube's axis
        for _ in range(60):
            z0 = 0.5 * (lo + hi)
            v = float(np.sum(chord * np.maximum(z0 + t * xs, 0.0)) * dx)
            lo, hi = (z0, hi) if v < vol else (lo, z0)
        tops.append(0.5 * (lo + hi) + t * radius)
    return ts, np.array(tops)


@dataclass
class LiquidParams:
    radius: float = 0.007  # bore, m (16 x 100 mm tube, 1 mm wall)
    depth: float = 0.098  # inside depth from the bottom to the mouth, m
    lower: float = 0.013  # depth of the dense lower layer (2 mL), m
    upper: float = 0.013  # depth of the water layered on it (2 mL), m
    zeta: float = 0.10  # slosh damping ratio
    lpf_hz: float = 25.0  # low-pass on the measured acceleration (contact jitter)
    mix_tan: float = math.tan(math.radians(30.0))  # a stroke: the surface swings past 30 degrees ...
    mix_lift: float = 0.005  # ... or the column drops back from 5 mm
    mix_per_stroke: float = 0.25
    mixed_contrast: float = 0.10
    min_fax: float = 0.5  # m/s^2: below this (near free fall) the surface slope is held at its last value

    @property
    def fill(self) -> float:
        return self.lower + self.upper


@dataclass
class TubeLiquid:
    p: LiquidParams = field(default_factory=LiquidParams)

    def __post_init__(self):
        self._ts, self._tops = wall_top_table(self.p.radius, self.p.fill)
        self.reset()

    def reset(self) -> None:
        self.x = np.zeros(2)  # slosh mode displacement (m/s^2 / w^2 units: metres of equivalent mass travel)
        self.xd = np.zeros(2)
        self.s = self.sd = 0.0  # column lift off the bottom and its speed
        self.s_peak = 0.0
        self.prev_p = self.prev_v = None
        self.a = np.zeros(3)
        self.contrast = 1.0
        self.strokes = 0
        self.last_dir = None  # which way the surface was tipped at the last stroke
        self.spilled = False
        self.spill_t = None
        self.peak_wall = 0.0  # highest the liquid stood on the wall (m from the bottom)
        self.wall = self.p.fill
        self.tan_phi = 0.0
        self.t = 0.0

    @property
    def mixed(self) -> bool:
        return self.contrast <= self.p.mixed_contrast

    def wall_top(self, tan_phi: float) -> float:
        if tan_phi >= self._ts[-1]:
            return math.inf
        return float(np.interp(tan_phi, self._ts, self._tops))

    def _stroke(self) -> None:
        self.contrast *= 1.0 - self.p.mix_per_stroke
        self.strokes += 1

    def step(self, bottom, rot, dt: float) -> None:
        """Advance by ``dt`` with the tube's bottom centre at ``bottom`` (world, m) and its rotation matrix ``rot``
        (columns: the tube's axes in the world; column 2 points from the bottom to the mouth)."""
        p = self.p
        self.t += dt
        rot = np.asarray(rot, float)
        centre = np.asarray(bottom, float) + rot[:, 2] * (p.fill / 2)
        if self.prev_p is None:
            self.prev_p = centre
            return
        v = (centre - self.prev_p) / dt
        self.prev_p = centre
        if self.prev_v is None:
            self.prev_v = v
            return
        a_raw = (v - self.prev_v) / dt
        self.prev_v = v
        k = 1.0 - math.exp(-2 * math.pi * p.lpf_hz * dt)
        self.a += k * (a_raw - self.a)
        f = rot.T @ (self.a + np.array([0.0, 0.0, G]))  # specific force in the tube frame
        f_ax, f_lat = float(f[2]), f[:2]
        # column lift
        if self.s > 0.0 or f_ax < 0.0:
            self.sd += -f_ax * dt
            self.s += self.sd * dt
            self.s_peak = max(self.s_peak, self.s)
            if self.s <= 0.0:
                if self.s_peak >= p.mix_lift:
                    self._stroke()
                self.s = self.sd = self.s_peak = 0.0
        # slosh mode
        fax = max(f_ax, p.min_fax)
        w2 = 1.841 * fax / p.radius * math.tanh(1.841 * p.fill / p.radius)
        w = math.sqrt(w2)
        self.xd += (-f_lat - 2 * p.zeta * w * self.xd - w2 * self.x) * dt
        self.x += self.xd * dt
        amp = math.sqrt(float(self.x @ self.x) + float(self.xd @ self.xd) / w2)
        if f_ax > p.min_fax:  # in near free fall the surface keeps its last slope; the column lift carries it
            self.tan_phi = w2 * amp / f_ax
            slope = w2 * self.x / f_ax  # the surface's slope now, signed (which side is high)
            n = float(np.linalg.norm(slope))
            if n >= p.mix_tan and (self.last_dir is None or float(slope @ self.last_dir) < 0.0):
                self._stroke()  # the surface swung past 30 degrees, to the other side from the last stroke
                self.last_dir = slope / n
        self.wall = self.wall_top(self.tan_phi) + self.s
        self.peak_wall = max(self.peak_wall, self.wall)
        if self.wall >= p.depth and not self.spilled:
            self.spilled, self.spill_t = True, self.t
