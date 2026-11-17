"""titration: a reduced-order model of a burette stopcock, its drops, and an acid-base titration in a swirled flask.

Pure NumPy (no Isaac Sim), so the tests run it directly. :class:`Titration` is stepped at the physics rate with the
stopcock key's angle, the burette tip's position and the flask's pose, and gives at every step how much titrant
has left the burette (the burette reading), where it went, and what colour the flask shows.

- **Stopcock.** The PTFE key's bore (diameter ``bore``) and the barrel's channel line up when the key's handle
  stands along the burette (90 degrees from closed, either way). Turned ``delta`` off that, the bore's end slides
  ``s = plug_r * delta`` along the channel's end; the open area is the overlap of two circles of diameter ``bore``
  whose centres are ``s`` apart, and the flow is ``q_max`` times the open fraction (an orifice-limited outflow).
  Closed from ``delta >= bore / plug_r`` (17 degrees here); a drip (under ``q_stream``) only in the last ~7 degrees
  before that.
- **Drops.** Below ``q_stream`` the titrant gathers at the tip and falls as drops of ``v_drop`` (0.05 mL); above it,
  it runs as a stream. A drop or stream lands in the flask when the tip is over the flask's mouth, otherwise it is
  lost (latched).
- **Mixing.** Titrant that lands is first unmixed (``u``, mL); mixing moves it into the bulk at a rate
  ``k = k_still + k_swirl * min(1, swirl / a_ref)``: ``swirl`` is the RMS horizontal acceleration of the flask (a
  hand swirling it in 5 mm circles at 2 Hz gives about 0.8 m/s^2). Where it lands it shows a pink patch (``w``):
  while the bulk is still acid the patch is neutralised as fast as it mixes, but near the equivalence point there is
  little acid left, so it lingers (its fading rate is ``k`` scaled by ``(V_e - V_bulk) / v_slow``, floored at
  ``g_min``): the cue titrators slow down on.
- **Colour.** The bulk turns pink once it holds ``pink_excess`` (0.02 mL, under half a drop) more titrant than the
  equivalence volume ``V_e``; deeper pink the further past (phenolphthalein's colour, saturating by ~1 mL excess).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.81


def open_fraction(key_deg: float, bore: float, plug_r: float) -> float:
    """Open area of the stopcock as a fraction of fully open, for a key turned ``key_deg`` from closed."""
    a = abs(((key_deg + 90.0) % 180.0) - 90.0)  # 0 closed (handle across the burette), 90 open (along it)
    s = plug_r * math.radians(90.0 - a)
    x = s / bore
    if x >= 1.0:
        return 0.0
    return (2.0 / math.pi) * (math.acos(x) - x * math.sqrt(1.0 - x * x))


@dataclass
class TitrationParams:
    v_e: float = 10.0  # equivalence volume, mL (hidden: set per trial)
    capacity: float = 25.0  # burette, mL (filled to the 0.00 mark)
    q_max: float = 0.40  # mL/s fully open
    bore: float = 0.0012  # key bore and channel, m
    plug_r: float = 0.0040  # key plug radius, m
    v_drop: float = 0.05  # mL per drop
    q_stream: float = 0.25  # mL/s: above this the titrant runs as a stream
    k_still: float = 0.02  # 1/s: mixing of an unswirled flask (diffusion and slow convection)
    k_swirl: float = 1.0  # 1/s: added at full swirl (a swirled flask mixes in about a second)
    a_ref: float = 0.40  # m/s^2 RMS horizontal acceleration for full swirl
    swirl_tau: float = 0.5  # s: averaging time of the swirl measure
    v_slow: float = 1.50  # mL: the patch lingers over the last v_slow before equivalence ...
    g_min: float = 0.08  # ... mixing at no less than this fraction of the usual rate
    pink_excess: float = 0.02  # mL past V_e: the bulk shows pink
    tip_r: float = 0.003  # burette tip outer radius, m
    mouth_r_in: float = 0.0105  # flask neck bore radius, m
    mouth_r_out: float = 0.0145  # flask neck outer radius, m
    rim_h: float = 0.114  # flask's rim above its bottom, m
    neck_h: float = 0.030  # length of the flask's neck, m
    spill_tilt_deg: float = 45.0


@dataclass
class Titration:
    p: TitrationParams = field(default_factory=TitrationParams)

    def __post_init__(self):
        self.reset()

    def reset(self, v_e: float | None = None) -> None:
        if v_e is not None:
            self.p.v_e = float(v_e)
        self.t = 0.0
        self.key_deg = 0.0
        self.q = 0.0
        self.out = 0.0  # mL that has left the burette (the burette reading)
        self.hanging = 0.0  # mL in the drop forming at the tip
        self.landed = 0.0  # mL that reached the flask
        self.lost = 0.0  # mL that missed the flask
        self.u = 0.0  # landed but not yet mixed into the bulk, mL
        self.w = 0.0  # the pink patch, as mL of titrant not yet neutralised (lingers near equivalence)
        self.bulk = 0.0  # mixed into the bulk, mL
        self.drops = 0
        self.drop_t = -1.0  # time the last drop fell
        self.streaming = False
        self.swirl2 = 0.0
        self.prev_c = self.prev_v = None
        self.a = np.zeros(3)
        self.struck = False  # the tip touched the flask's glass (latched)
        self.spilled = False  # the flask tipped past spill_tilt_deg (latched)
        self.ran_dry = False  # the burette ran below its last mark (latched)
        self.max_tilt = 0.0
        self.tip_in_flask = None  # tip in the flask frame, last step

    # -- derived ------------------------------------------------------------------------------------------------
    @property
    def reading(self) -> float:
        return self.out + self.hanging

    @property
    def swirl(self) -> float:
        return math.sqrt(self.swirl2)

    @property
    def excess(self) -> float:
        """Titrant in the bulk past equivalence (mL); negative while still acid."""
        return self.bulk - self.p.v_e

    @property
    def final_excess(self) -> float:
        """Past equivalence once everything that landed is mixed (mL)."""
        return self.landed - self.p.v_e

    @property
    def pink(self) -> bool:
        return self.excess >= self.p.pink_excess

    @property
    def final_pink(self) -> bool:
        return self.final_excess >= self.p.pink_excess

    def bulk_pinkness(self) -> float:
        """0 colourless .. 1 deep pink."""
        e = self.excess
        return 0.0 if e < self.p.pink_excess else min(1.0, 0.15 + 0.85 * (e - self.p.pink_excess) / 1.0)

    # -- step -----------------------------------------------------------------------------------------------------
    def _mouth(self, tip_f) -> str:
        """Where the tip is relative to the flask (flask frame, origin at the bottom centre): 'in', 'glass', 'out'."""
        p = self.p
        r = math.hypot(tip_f[0], tip_f[1])
        z = tip_f[2]
        in_neck = p.rim_h - p.neck_h <= z <= p.rim_h + 0.002
        if in_neck and p.mouth_r_in - p.tip_r < r < p.mouth_r_out + p.tip_r:
            return "glass"
        if z < 0.004:
            return "glass"  # on the flask's bottom
        if z <= p.rim_h + 0.30 and r <= p.mouth_r_in - p.tip_r:
            return "in"  # over or inside the mouth
        return "out"

    def step(self, key_deg: float, tip, flask_bottom, flask_rot, dt: float) -> None:
        """Advance by ``dt``: the key turned ``key_deg`` from closed, the tip at ``tip`` (world, m), the flask's
        bottom centre at ``flask_bottom`` and its rotation matrix ``flask_rot`` (column 2: its axis, up)."""
        p = self.p
        self.t += dt
        rot = np.asarray(flask_rot, float)
        base = np.asarray(flask_bottom, float)
        tip_f = rot.T @ (np.asarray(tip, float) - base)
        self.tip_in_flask = tip_f
        where = self._mouth(tip_f)
        self.struck |= where == "glass"
        tilt = math.degrees(math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))))
        self.max_tilt = max(self.max_tilt, tilt)
        self.spilled |= tilt > p.spill_tilt_deg

        # swirl: RMS horizontal acceleration of the liquid's centre, averaged over swirl_tau
        c = base + rot[:, 2] * 0.008
        if self.prev_c is not None:
            v = (c - self.prev_c) / dt
            if self.prev_v is not None:
                a = (v - self.prev_v) / dt
                k = 1.0 - math.exp(-2 * math.pi * 6.0 * dt)  # low-pass 6 Hz (grip and contact jitter)
                self.a += k * (a - self.a)
            self.prev_v = v
        self.prev_c = c
        self.swirl2 += (1.0 - math.exp(-dt / p.swirl_tau)) * (float(self.a[0] ** 2 + self.a[1] ** 2) - self.swirl2)

        # stopcock and drops
        self.key_deg = key_deg
        left = p.capacity - self.out - self.hanging
        q = p.q_max * open_fraction(key_deg, p.bore, p.plug_r) if left > 0 else 0.0
        if left <= 0:
            self.ran_dry = True
        self.q = q
        self.streaming = q > p.q_stream
        fell = 0.0
        if self.streaming:
            fell = q * dt + self.hanging
            self.hanging = 0.0
        else:
            self.hanging += q * dt
            if self.hanging >= p.v_drop:
                fell, self.hanging = self.hanging, 0.0
                self.drops += 1
                self.drop_t = self.t
        if fell > 0:
            self.out += fell
            if where == "in":
                self.landed += fell
                self.u += fell
                self.w += fell
            else:
                self.lost += fell

        # mixing
        k = p.k_still + p.k_swirl * min(1.0, self.swirl / p.a_ref)
        g = 1.0 if self.bulk >= p.v_e else min(1.0, max(p.g_min, (p.v_e - self.bulk) / p.v_slow))
        moved = self.u * (1.0 - math.exp(-k * dt))
        self.u -= moved
        self.bulk += moved
        self.w *= math.exp(-k * g * dt)
