"""The wedge-smear rule: what the drop does under the spreader, step by step. Pure Python (no Isaac), unit-tested.

Inputs per physics step, all in the base slide's frame (x along the slide from its frosted end, toward its far end;
y across it; m): the spreader's lower edge (its centre), whether it is touching the base slide (PhysX contact force),
and its angle to the slide on the trailing side (deg; over 90 means it leans forward, the wrong way).

1. Wick. While the edge touches the slide on the drop (within ``drop_r`` of its centre), having come onto it from the
   front or been set straight down on it, the drop runs out along the edge: the wetted width grows as
   ``w_max * (1 - exp(-t / tau_wick))``. An edge that slides onto the drop from behind (or leans forward) shoves the
   drop ahead of it instead: ploughed (latched).
2. Spread. The first forward move in contact after wicking starts the smear. The blood trails in the wedge behind the
   edge and is laid down as a film: thickness ``h = h_ref * (v / v_ref)**(2/3) * (tan a / tan a_ref) * sqrt(V / V0)``
   (Landau-Levich speed law; a steeper spreader holds a taller meniscus; the film thins as the reservoir ``V`` runs
   down). With speed and angle held, the film tapers linearly to nothing at ``L = 2 V0 / (W h0)``: a feathered end.
3. Faults (latched for the trial). Edge off the slide for more than ``gap_m`` of travel (or ``lift_s``) before the
   blood is used up: the smear ends blunt; touching down again with blood left is a restart. Moving back more than
   ``back_m`` while spreading: a restart. Stopping (under ``still_v``) for ``pause_s`` in contact mid-smear: a
   hesitation ridge. The edge passing the slide's far end with blood left: run off (no feathered end).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class Law:
    """Constants (SI). All are estimates; the README says how each is measured on real smears."""

    volume: float = 5e-9  # 5 uL drop
    drop_r: float = 0.0018  # wetted radius of the drop on the slide
    w_max: float = 0.020  # width the drop reaches along the 25 mm edge
    tau_wick: float = 0.5  # s
    h_ref: float = 12.5e-6  # film thickness at the start of a smear at v_ref and a_ref (gives L = 40 mm at w_max)
    v_ref: float = 0.040  # m/s
    a_ref: float = 35.0  # deg
    gap_m: float = 0.001
    lift_s: float = 0.15
    back_m: float = 0.0015
    still_v: float = 0.003  # m/s
    pause_s: float = 0.3
    start_m: float = 0.001  # forward travel in contact that starts the smear
    bin_m: float = 0.0005  # profile resolution

    def h0(self, v: float, angle_deg: float) -> float:
        a = min(max(angle_deg, 5.0), 85.0)
        return (self.h_ref * (max(v, 1e-4) / self.v_ref) ** (2.0 / 3.0)
                * math.tan(math.radians(a)) / math.tan(math.radians(self.a_ref)))


@dataclass
class Smear:
    """The drop, the film and the latches for one base slide (length ``slide_l``, width ``slide_w``)."""

    law: Law = field(default_factory=Law)
    slide_l: float = 0.075
    slide_w: float = 0.025
    drop_x: float = 0.024
    drop_y: float = 0.0
    spreader_w: float = 0.025

    def __post_init__(self):
        self.reset()

    def reset(self) -> None:
        n = int(round(self.slide_l / self.law.bin_m))
        self.h = np.zeros(n)  # film thickness per bin
        self.state = "drop"  # drop | wicking | carrying | spreading | feathered | blunt | runoff
        self.width = 0.0  # wetted width along the edge (then the smear's width)
        self.t_wick = 0.0
        self.vol = self.law.volume
        self.ploughed = self.restart = self.ridge = False
        self.starts = 0
        self.start_x = None
        self.end_x = None
        self.x_max = None
        self.last_x = None  # last deposit point
        self.prev_x = None
        self.prev_contact = False
        self.off_t = self.off_m = 0.0
        self.still_t = 0.0
        self.v = 0.0
        self.angles: list[float] = []
        self.speeds: list[float] = []

    # -- the rule ----------------------------------------------------------------------------------------------------

    def _on_drop(self, x: float) -> bool:
        return abs(x - self.drop_x) <= self.law.drop_r

    def _wet_width(self, y: float) -> float:
        """Width wetted along the edge, clipped by the spreader's and the slide's sides."""
        lo = max(self.drop_y - self.width / 2, y - self.spreader_w / 2, -self.slide_w / 2)
        hi = min(self.drop_y + self.width / 2, y + self.spreader_w / 2, self.slide_w / 2)
        return max(0.0, hi - lo)

    def _deposit(self, x0: float, x1: float, angle: float) -> None:
        """Lay film from x0 to x1 (> x0) at the current speed and angle, using up the reservoir."""
        law, b = self.law, self.law.bin_m
        i0, i1 = int(x0 / b), int(x1 / b)
        h0 = law.h0(self.v, angle)
        for i in range(i0, i1 + 1):
            if i >= len(self.h):
                break
            lo, hi = max(x0, i * b), min(x1, (i + 1) * b)
            if hi <= lo:
                continue
            h = h0 * math.sqrt(max(self.vol, 0.0) / law.volume)
            dv = h * self.width * (hi - lo)
            if dv >= self.vol or h < 0.02 * h0:
                self.h[i] = max(self.h[i], h)
                self.vol = 0.0
                self.end_x = hi
                self.state = "feathered"
                return
            self.h[i] = max(self.h[i], h)
            self.vol -= dv

    def step(self, dt: float, x: float, y: float, contact: bool, angle: float) -> str:
        """Advance one physics step; returns a message when something happens."""
        law = self.law
        msg = ""
        on_slide = 0.0 <= x <= self.slide_l
        contact = contact and on_slide
        dx = 0.0 if self.prev_x is None else x - self.prev_x
        self.v += (dx / dt - self.v) * min(1.0, dt / 0.04)  # 40 ms smoothing
        if self.state == "drop" and contact and self._on_drop(x):
            behind = self.prev_contact and self.prev_x < self.drop_x - law.drop_r
            if angle >= 90.0 or behind:
                self.ploughed = True
                self.state, self.width, self.x_max = "carrying", law.w_max * 0.5, x
                msg = "PLOUGHED: the drop is pushed ahead of the spreader"
            else:
                self.state, self.x_max = "wicking", x
                msg = "the drop touches the edge and starts to run along it"
        if self.state == "wicking":
            if contact and x > self.x_max + law.start_m:
                msg = "; ".join(m for m in (msg, self._start(x, y)) if m)
            elif contact and self._on_drop(x):
                self.t_wick += dt
                self.width = law.w_max * (1.0 - math.exp(-self.t_wick / law.tau_wick))
                self.x_max = min(self.x_max, x)
            elif not contact:
                self.state = "carrying"  # lifted with blood on the edge
                msg = f"spreader lifted off the drop with {self.width * 1000:.1f} mm of blood on its edge"
        elif self.state == "carrying":
            if contact and dx > 0:
                self.x_max = x
                msg = "; ".join(m for m in (msg, self._start(x, y)) if m)
        elif self.state == "spreading":
            self.angles.append(angle)
            if contact:
                self.off_t = self.off_m = 0.0
                if x > self.last_x:
                    self.speeds.append(self.v)
                    self._deposit(self.last_x, x, angle)
                    if self.state == "spreading":
                        self.last_x = x
                    else:
                        msg = f"feathered end at {self.end_x * 1000:.1f} mm"
                self.x_max = max(self.x_max, x)
                if self.state == "spreading" and self.x_max - x > law.back_m:
                    self.restart, self.state, self.end_x = True, "blunt", self.last_x
                    msg = "RESTART: the spreader went back over the smear"
                if self.state == "spreading" and abs(self.v) < law.still_v:
                    self.still_t += dt
                    if self.still_t >= law.pause_s and not self.ridge:
                        self.ridge = True
                        msg = f"RIDGE: the spreader stopped mid-smear at {x * 1000:.1f} mm"
                else:
                    self.still_t = 0.0
            else:
                self.off_t += dt
                self.off_m += max(dx, 0.0)
                if self.off_m > law.gap_m or self.off_t > law.lift_s:
                    self.state, self.end_x = "blunt", self.last_x
                    msg = f"smear ends blunt at {self.end_x * 1000:.1f} mm: the spreader left the slide"
            if self.state == "spreading" and x > self.slide_l:
                self.state, self.end_x = "runoff", self.slide_l
                msg = "RUN OFF: the smear reached the slide's far end with blood left"
        elif self.state == "blunt" and contact and self.vol > 0 and not self.restart:
            self.restart = True
            msg = "RESTART: the spreader came down again with blood on it"
        self.prev_x, self.prev_contact = x, contact
        return msg

    def _start(self, x: float, y: float) -> str:
        self.starts += 1
        if self.starts > 1:
            self.restart = True
        self.width = self._wet_width(y)
        self.state = "spreading"
        self.start_x = self.drop_x if self._on_drop(self.x_max) else self.x_max
        self.last_x = max(self.start_x, self.x_max)
        self.x_max = x
        self.still_t = 0.0
        return f"smear starts at {self.start_x * 1000:.1f} mm, {self.width * 1000:.1f} mm wide"

    # -- what a ruler on the photo reads ------------------------------------------------------------------------------

    @property
    def length(self) -> float:
        nz = np.nonzero(self.h > 0)[0]
        if len(nz) == 0:
            return 0.0
        return (nz[-1] + 1) * self.law.bin_m - (self.start_x if self.start_x is not None else nz[0] * self.law.bin_m)

    @property
    def far_end(self) -> float | None:
        nz = np.nonzero(self.h > 0)[0]
        return None if len(nz) == 0 else (nz[-1] + 1) * self.law.bin_m

    @property
    def continuous(self) -> bool:
        """No unfilmed gap longer than ``gap_m`` inside the film."""
        nz = np.nonzero(self.h > 0)[0]
        if len(nz) == 0:
            return False
        return bool(np.max(np.diff(nz), initial=1) * self.law.bin_m <= self.law.gap_m + 1e-9)

    def summary(self) -> dict:
        a = [x for x in self.angles if x == x]
        return dict(state=self.state, length_mm=round(self.length * 1000, 1), width_mm=round(self.width * 1000, 1),
                    start_mm=None if self.start_x is None else round(self.start_x * 1000, 1),
                    end_mm=None if self.far_end is None else round(self.far_end * 1000, 1),
                    left_ul=round(self.vol * 1e9, 2), ploughed=self.ploughed, restart=self.restart, ridge=self.ridge,
                    angle_deg=round(float(np.median(a)), 1) if a else None,
                    speed_mm_s=round(float(np.median(self.speeds)) * 1000, 1) if self.speeds else None)


# -- the score (task.py calls this) -------------------------------------------------------------------------------


def smear_score(s: Smear, moved_m: float, full_len=(0.030, 0.045), part_len=(0.020, 0.055), min_film=0.015,
                min_width=0.015, start_tol=0.005, end_margin=0.005, moved_cap=0.003, moved_zero=0.010) -> float:
    """0 no film 15 mm long, a restart, or the base slide moved over 10 mm. 0.3 any film 15 mm or longer (or the
    base slide moved 3-10 mm, or the drop was ploughed). 0.6 one continuous film from within 5 mm of the drop, 20-55
    mm long, ending in a feathered edge on the slide. 1.0 that, 30-45 mm long, 15 mm or more wide, no hesitation
    ridge, the feathered end 5 mm or more before the slide's far end."""
    if s.restart or moved_m > moved_zero or s.length < min_film:
        return 0.0
    if s.ploughed or moved_m > moved_cap:
        return 0.3
    good = (s.state == "feathered" and s.continuous and s.start_x is not None
            and abs(s.start_x - s.drop_x) <= start_tol)
    if not (good and part_len[0] <= s.length <= part_len[1]):
        return 0.3
    if (full_len[0] <= s.length <= full_len[1] and s.width >= min_width and not s.ridge
            and s.far_end <= s.slide_l - end_margin):
        return 1.0
    return 0.6
