# Make a wedge blood smear with a spreader slide

A glass microscope slide lies on white bench paper in front of the right arm, frosted end toward the arms, with a
5 uL drop of blood (in real trials, a blood stand-in) 24 mm from that end. A second, clean slide, the spreader,
stands upright in a slotted stand to the right.

**Instruction:** `Make a wedge smear of the drop on the slide with the spreader slide: let the drop run along the spreader's edge, then push forward in one smooth stroke. The smear should be 30-45 mm long and end in a feathered edge before the slide does.`

## Changes since the last review

- TK-6SNFEJ: "Please judge hesitation and restarts from the finished slide (a visible ridge or a second film edge in
  the photo) rather than from timing or distances in the video, so a person can score the trial from the photo."
  Changed: ridge and restart are now defined, in the scoring table, the real-trial checks, task.json and the
  simulated scorer, by what the finished film shows (a transverse band over 1.5x the film around it; a second film
  edge or a gap band over 1 mm); the video is no longer used for them, and the smear rule now lays the band and the
  second edge into its film record, which the scorer reads.

## Why this task

- **What it is.** The wedge (two-slide) smear is how a peripheral blood film is made by hand for a manual
  differential count, the standard method of CLSI H20 and of every haematology textbook. Hospital haematology
  labs make them every day (for every flagged automated count), and research labs make them for blood, bone
  marrow and cell suspensions. The technique has not changed: the spreader is set on the slide in front of the
  drop at 30-45 degrees, drawn back into the drop until the blood runs along its edge, then pushed forward in one
  smooth, steady stroke. A good film is 2/3 to 3/4 of the slide long and ends in a thin, feathered edge, where the
  cells lie in a single layer for counting.
- **The difficulty is in the motion.** The film's length and thickness are set by the spreader's angle and speed
  during one stroke, and nothing else holds them: steeper or faster gives a short, thick film; shallower or slower
  gives a long, thin film that runs off the end of the slide (no feathered edge). The robot has to hold a 1 mm
  glass edge on glass at a fixed angle with light, unbroken contact, move it at a steady speed, and time it: push
  before the drop has run along the edge and the film is a narrow streak; lift off or go back and the film has a
  blunt end or a second start; stop mid-stroke and it leaves a ridge. All of this is precise position and force
  control of a tool against a surface, with no fixture to set any of it.
- **The score is the outcome.** It scores the film a person would check on the slide (length, width, start,
  feathered end, breaks, ridge), not the robot's angles or speeds.

### Distinct from

- **"Draw a continuous laboratory film with a fixed-gap applicator, then lift it clear" (pending, another team).**
  There, a film applicator with a machined 0.1 mm gap rides flat on a plate and the gap sets the film's thickness;
  the skill is covering an 80 x 200 mm area without dry stripes. Here no gap is fixed: the robot holds a loose
  glass slide at an angle it chooses, and the angle and speed of its one stroke decide the film's length. Success is
  a film of a given length ending in a feathered edge, not full coverage, and it is a two-slide hand technique.
- **"Run a glucose meter's control check ... touching only its tip to a drop" (pending).** A strip touched to a
  drop: a brief light touch, no stroke.
- **Wiping and cleaning tasks** in household suites (BEHAVIOR-1K wiping/cleaning tasks, RoboCasa wiping) score
  surface coverage with a soft cloth, where pressure and angle hardly matter. No lab suite we know of (LIBERO,
  RoboCasa, BEHAVIOR-1K, SimplerEnv) has a smear.

Category: **Other** (clinical and research laboratory work: a standard manual specimen preparation, done the way
labs do it; medical laboratory technologists and technicians number about 0.34 M in the US, under the jobs bar,
so we do not claim Impact on jobs).

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, `score()` / `success()`, and `controller()` (the demo `yam_scripted` plays) |
| `smearmodel.py` | The smear rule (wicking, film deposition, faults) and the score; pure Python |
| `smearkit.py` | The slides, stand and paper, the 240 Hz update of the smear rule, the drawn film, the hooks into yam_arena |
| `smeardemo.py` | The scripted demo (closed loop on the measured joint positions) |
| `test_wedge_blood_smear.py` | Unit tests of the rule and the score (no Isaac Sim needed) |
| `spreader_stand.stl` | The spreader stand (3D-print), dimensions below |

`smearkit.py` hooks yam_arena without modifying it: it patches `yam_arena.scene.task_object` and `object_states`, the
embodiment's `_ensure_sim` and `reset`, and `yam_arena.scripted.YamScriptedPolicy` (when a task file defines
`controller()`, `yam_scripted` asks it for each action). It wraps `InteractiveScene.write_data_to_sim` to step the
smear rule before every physics step. The task runs with the standard `yam_arena` embodiment, `yam_scripted` policy
and `task_score` scorer.

## Setup

Positions in cm from the midpoint between the two arm bases on the table top (x forward, y left); arms start in the
ready pose; the layout is the same in every trial.

- **Bench paper:** white, 30 x 30 cm, taped flat, centred at x 30, y -12.
- **Base slide:** plain glass microscope slide with a 20 mm frosted end, 75 x 25 x 1.0 mm, about 4.7 g (Fisherbrand
  frosted microscope slides, Fisher 12-550-400 or any equivalent 75 x 25 x 1 mm frosted slide), lying flat on the
  paper with its long side along x, centred at x 27.75, y -8, frosted end toward the arms (x 24.0 to 26.0). Its
  four corners are marked on the paper with a fine pencil.
- **Drop:** 5 uL of the blood stand-in, placed by the operator with a fixed-volume pipette just before the trial,
  centred 24 mm from the frosted end (4 mm in front of the frosted band), on the slide's centre line. It wets a
  circle about 3.5 mm across. Placing the drop is not part of the task.
- **Spreader:** a second slide of the same kind, clean, standing upright on its short end, frosted end up, in the
  stand's slot, its faces normal to x.
- **Spreader stand:** a 3D-printed block, 24 (x) x 40 (y) x 15 mm, with one slot 1.6 mm wide and 12 mm deep
  running through its 40 mm length (`spreader_stand.stl`, PLA), slot centred at x 30, y -20, taped down. The
  spreader's top end is 78 mm above the table.
- Nothing else is on the table. The other hand may hold the base slide's frosted end, but need not.

### Real-trial blood stand-in

No blood is used. The stand-in is 40% (w/w) glycerol in water with red food colouring (1 drop per 10 mL): its
viscosity is about 3.7 mPa s at 20 C, close to whole blood at the shear rates of a smear (3-4 mPa s), and it is
nonhazardous, does not clot, dry or spoil, and is mixed in a minute (40 g glycerol, USP grade, plus 60 g tap water;
keep capped, use within a month). Its surface tension (about 70 mN/m) is higher than blood's (about 55 mN/m), so it
wicks a little more slowly; the film it leaves stays wet (glycerol does not dry), which makes the photo easy. This is
a stand-in matched on viscosity, not a product sold for smear training.

## Run a trial

```
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/wedge-blood-smear/task.py -E object_jitter=0 -E control_hz=20 \
  --scorer task_score \
  --instruction "Make a wedge smear of the drop on the slide with the spreader slide: let the drop run along the spreader's edge, then push forward in one smooth stroke. The smear should be 30-45 mm long and end in a feathered edge before the slide does." \
  --max-steps 1200 --seed 0 --store-frames --log-dir ~/yam-eval/wedge-smear-dream
```

Run limit: 1,200 steps (60 s at 20 Hz). The scripted demo finishes in 364 steps (18.2 s).

## Scoring

Latched over the whole trial; the trial's score is the score at the end. Success (which ends the episode early) is
1.0 with the spreader taken away (its edge 3 cm or more above the table and 10 cm or more from the base slide's
centre), held 1 s.

| Score | When |
|---|---|
| 0 | no film 15 mm or longer; or a restart (a second film edge across the film, or a clear gap band over 1 mm across it); or the base slide moved more than 10 mm |
| 0.3 | any film 15 mm or longer; or the drop was ploughed (the edge slid into it from behind, or the spreader leaned forward, so the blood was pushed ahead instead of trailing); or the base slide moved 3-10 mm |
| 0.6 | one continuous film starting within 5 mm of the drop's centre, 20-55 mm long, ending in a feathered edge on the slide (the blood ran out under the spreader, not lifted off or run off the end) |
| 1.0 | as 0.6, and 30-45 mm long, 15 mm or more wide, no hesitation ridge (a transverse band across the film, before the feathered end, more than 1.5 times as thick as the film 1-4 mm either side of it), the feathered end 5 mm or more before the slide's far end |

### Real trial

Lay the slide on 1 mm graph paper and photograph it from above with a ruler in view, right after the trial.

- **Film length and start:** from the drop's position (marked on the paper under the slide before the trial) to
  the last visible colour, along the slide; start within 5 mm of the drop mark.
- **Width:** the film's widest point across the slide.
- **Feathered end:** the film thins out gradually to nothing (no straight, thick line of colour across its end) and
  ends on the slide. A film that reaches the far end, or ends in a thick line where the spreader was lifted, is
  not feathered.
- **Restart:** a second film edge (a line of colour across the film, or a second film start, behind or ahead of
  the first film), or a clear, uncoloured band over 1 mm wide across the film.
- **Ridge:** a visible transverse thick band across the film before the feathered end: a darker stripe clearly
  thicker than the film on either side of it (in simulation, over 1.5 times the film 1-4 mm before and after).

Length, width, start, end, gaps, restarts and ridges are all read from the photo alone; the video is not needed to
score them.
- **Ploughed:** in the video, the spreader touches down behind the drop and is pushed through it, or leans forward
  (its top ahead of its edge).
- **Base slide moved:** its corners against the pencil marks on the paper, measured with a ruler.

## Scripted demo

```
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/wedge-blood-smear/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --scorer task_score --instruction "..." --max-steps 1200 --seed 0 --store-frames --log-dir ~/yam-eval/wedge-smear-demo
```

`smeardemo.py` sees only the measured joint positions (forward kinematics gives the tool point) and the fixed layout
above; it does not read the simulator. The right arm grips the spreader across its thickness 8 mm below its top end,
lifts it out of the stand, pitches it to lean back over the drop at 35 degrees, and holds its edge 6 mm in front of
the drop and 4 mm above the glass. There it measures its own sag and lag (forward kinematics of the measured joints
against the command) and aims off by them, then lowers the edge over 1 s to 1 mm below the glass: the arm's joints are
springs, so the edge rests on the glass with about 0.2-0.4 N. It backs the edge into the drop at 5 mm/s to 1.5 mm past
its centre (it stops about 1 mm short of that: the glass drags the edge), takes out the arm's x lag there, waits
1.5 s while the drop runs along the edge, and pushes forward 48 mm at a constant 50 mm/s (0.25 s ramps); the blood runs out after about 40 mm. Then it lifts clear and turns away from
the top camera's view of the slide, still holding the spreader.

### Checked in simulation

| Run | What it does | Score |
|---|---|---|
| Demo, seeds 0, 4, 10 | as above | 1.0 each: film from 24.0 to 61.0 mm (37.0 mm), 19.9 mm wide, feathered end 14 mm before the far end, edge force 0.2-0.5 N, angle 34.7 deg |
| `DEMO_CHEAT=nowait` (naive: push at once) | backs into the drop and pushes as soon as the edge reaches it | 0.3 (only 11 mm wetted: a narrow film still running at the end of the 48 mm stroke, no feathered end) |
| `DEMO_CHEAT=plough` (loophole: skip the back-in) | comes down 6 mm behind the drop and pushes straight through it | 0.3 (ploughed; 10 mm wide film to the end of the stroke) |
| `DEMO_CHEAT=slow` | the demo at 15 mm/s | 0.3 (film still running at the end of the 48 mm stroke: no feathered end) |
| `DEMO_CHEAT=lift` | lifts the spreader 4 mm for 8 mm of travel mid-stroke, then finishes the stroke | 0 (blunt end, then a restart) |
| Dream-1 (`--policy dream`, seed 0, 1,200 steps) | | 0.0: the spreader never touched the drop (no film) |

The demo's runs are identical across seeds (`object_jitter=0`, nothing random in the task). Contrast runs are
started with `DEMO_CHEAT=<name>` in the environment of the scripted run.

## Smear rule and simulation fidelity

The simulator cannot wet glass or lay down a liquid film, so a rule (`smearmodel.py`) stands in for the liquid. It
runs at 240 Hz from what PhysX gives: the spreader's pose relative to the base slide and the contact force between
the two slides. The robot must still do the real physical work: grip and pitch the spreader, put its edge on the
glass (contact force 0.02 N or more, about half the spreader's weight; PhysX reports a light sliding contact on most
240 Hz substeps but not all, so a push within the last 25 ms counts), and move it.

- **Wick.** While the edge touches the slide on the drop (within 1.8 mm of its centre), having come onto it from the
  front or been set straight down on it, the wetted width along the edge grows as 20 mm x (1 - exp(-t / 0.5 s)).
- **Film.** From the first forward move in contact after wicking, the film thickness is
  `h = 12.5 um x (v / 40 mm/s)^(2/3) x (tan a / tan 35 deg) x sqrt(V / V0)`, with `v` the edge's speed along the
  slide, `a` its angle to the slide on the trailing side, `V` the blood left on the edge (V0 = 5 uL). The 2/3 power
  is the Landau-Levich law for a film drawn by a moving meniscus; the tan term says a steeper spreader holds a taller
  meniscus; the square root makes the film taper to a feathered end as the blood runs out. At a steady speed and
  angle the film is `2 V0 / (W h0)` long: 37 mm at 45 mm/s and 35 degrees, 30 to 45 mm between about 30 and 41
  degrees at 45 mm/s, or between about 35 and 62 mm/s at 35 degrees.
- **Faults.** Out of contact for more than 1 mm of travel or 0.15 s while blood is left: blunt end; contact again
  lays a line of blood (a second film edge) where the edge lands. Moving back more than 1.5 mm while spreading:
  a second film edge where it turned back. Standing still (under 3 mm/s) in contact mid-smear drains the wedge into
  a band at the edge, growing from the film's thickness to twice it over 0.3 s. The edge past the slide's far end
  with blood left: run off.
- **Ridge and restart are judged from the film, as on the photo.** The rule keeps the film thickness in 0.5 mm bins
  along the slide and where each film edge was laid. A ridge is a band over 1.5 times the median thickness of the
  film 1-4 mm before and after it; a restart is a second film edge or an unfilmed gap over 1 mm inside the film.
  The scorer never reads the spreader's timing or distances for these two.
- **Estimates and how to measure them.** All constants are estimates from the textbook description of the
  technique (30-45 degrees, a film of 2/3 to 3/4 of the slide), not measurements. To calibrate them with the
  stand-in: make 9 smears by hand or with the robot at 30, 35 and 40 degrees (a printed protractor card behind the
  slide) and three speeds (timed from 60 fps phone video against a ruler), measure each film's length and width on
  graph paper, and fit `h_ref` and the angle and speed exponents; time the wicking (drop touch to full width) from
  the same videos for `tau_wick` and `w_max`.
- **Bodies.** Both slides are PhysX rigid boxes (75 x 25 x 1 mm, 4.7 g); the spreader's top 20 mm collides as
  3 mm thick, because the simulated finger pads stop about 3 mm apart when closed (real rubber pads close on the
  1 mm glass); the look is 1 mm. Friction: glass on glass 0.15 (lubricated by the liquid), glass on paper about
  0.5, finger pads on the spreader 1.5 (the pad's rubber, not averaged with the glass). The film is drawn as 1.5 mm
  segments in three shades of red by thickness; its colour is not used for scoring.

## Runtime

About 5-6 min of wall time per trial on one GPU (Isaac Sim start included); 60 s of simulated time at the
run limit.

## Tests

```
pytest test_wedge_blood_smear.py      # no GPU or Isaac Sim needed (under a second)
```

## Real-world build and safety

| Item | Example product | Size, mass | Approx. price |
|---|---|---|---|
| Frosted microscope slides (base and spreader; one new pair per trial) | Fisherbrand frosted microscope slides, Fisher 12-550-400 (144 per pack) | 75 x 25 x 1.0 mm, 4.7 g | $30 per 144 |
| Fixed-volume pipette, 5 uL, and tips | any 5 uL fixed or 2-20 uL adjustable pipette | | $60 |
| Glycerol, USP, 500 mL | any pharmacy or lab supplier | | $12 |
| Red food colouring | any brand | | $3 |
| White bench paper, ruler, 1 mm graph paper, pencil, tape | | | $15 |
| Spreader stand | 3D-printed from `spreader_stand.stl` (PLA, about 10 g) | 24 x 40 x 15 mm | $1 |
| Nitrile finger covers (optional) | any | | $5 |

Total about $125 (beyond the robot).

- **Force.** The spreader weighs 4.7 g; it is pressed on the glass with a fraction of a newton, and gripped with a
  few newtons across its 1 mm thickness (the YAM's fingers give up to about 40 N). Payload is negligible.
- **Liquid.** 5 uL of a nonhazardous, food-grade liquid, which stays on the two slides; the fingers hold the
  spreader's frosted top 20 mm, 50 mm from the wet edge. Nitrile finger covers can be worn. Wipe any drop off the
  paper with a damp towel.
- **Glass.** Slides have ground edges. If a slide breaks or is dropped, stop the trial, and collect the pieces with
  tweezers into a glass-waste box; the trial scores 0 if the base slide broke (no film can be measured) or as
  measured otherwise.
- **Reset.** New base slide and spreader each trial (used slides are rinsed in water and can be reused for practice
  only after drying). Place the base slide on its pencil marks, the spreader in the stand, pipette a new drop.
  About 1 minute.

## Assets and licences

No third-party meshes, scans or textures. The slides, stand, paper and film are primitives defined in `smearkit.py`;
`spreader_stand.stl` is generated from the same dimensions (made for this task, MIT like the rest of this task's code). The
robot, room and cameras are the standard yam_arena embodiment: the YAM robot model (URDF and meshes in
`yam_arena/assets/yam/`) comes from i2rt (github.com/i2rt-robotics/i2rt) under the MIT licence; the yam_arena
station and Isaac Lab Arena are used as provided by the program.
