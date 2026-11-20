# Titrate to the first permanent pink: stopcock to a drip, swirl the flask, shut it as the colour holds

A 25 mL glass burette is clamped on a retort stand. It is filled to 0.00 mL with 0.05 M sodium hydroxide. Its PTFE
stopcock key faces the left arm. Under the tip, on a white tile, stands a 125 mL Erlenmeyer flask. The flask holds an
unknown amount (8 to 12 mL) of 0.05 M acetic acid, made up to about 30 mL with water, and 3 drops of
phenolphthalein. The burette tip is 1 cm inside the flask's neck.

**Instruction:** `Titrate the acid in the flask with the burette to the first permanent pink: swirl the flask while
you add the titrant, and close the stopcock as soon as the pink colour stays.`

## Why this task

- **The real procedure.** A manual acid-base titration with a glass burette is a standard wet-lab assay. Chemistry
  teaching labs use it, and so do research and QC labs (acidity of food and water, standardising a solution). It is
  done the way it is done here: one hand works the stopcock, the other swirls the flask under the tip, and the
  stopcock is shut at the first pink that does not fade. The equipment is the standard equipment: a class B
  burette with a PTFE key, an Erlenmeyer flask, a white tile and a retort stand. Category **Other**, as
  laboratory research automation (the review guide names common wet-lab steps that people do by hand).
- **The difficulty is physical.**
  - *Fine valve control.* The key is closed when its handle lies across the burette. It must be turned about
    73 degrees before any titrant flows, and at 86 degrees the flow is already a stream. A controlled drip
    (1 to 5 drops a second) is a window of about 10 degrees. The key has real friction (about 0.05 N m). The
    gripper must hold it at an angle inside that window, by rolling the wrist about the key's axis.
  - *Bimanual swirling.* At the same time the other arm holds the flask by its neck and swirls it in small
    circles. The tip is inside the neck, with 7.5 mm of radial clearance. A swirl that is too wide strikes the glass
    tip. A swirl that is too weak does not mix, so the pink flash where each drop lands does not clear.
  - *Reactive shut-off.* Near the end point the flashes last longer. The titrator then slows to single drops and
    shuts the key within about one drop of the colour holding. Each drop is 0.05 mL. Full credit allows 0.30 mL
    (6 drops) past the equivalence volume. The equivalence volume is different in each trial (8 to 12 mL) and the
    robot is not told it.
- **Perception guides the motion; it is not the task.** The colour tells the robot when to slow down and when to
  stop. The score depends on what the hands did: the angle the key was held at, how well the flask was swirled,
  and how fast the key was shut. In a real trial a pH meter reads the result, so no one judges the colour by eye.

### Distinct from

- **Mixing two layered liquids in open test tubes (TK-AA2GWL, pending).** There the skill is to agitate
  an open tube hard enough to mix it, without spilling. Here nothing is shaken to its limit. The skill is to meter
  liquid through a valve, drop by drop, with one hand while the other swirls the flask, and to stop at an end point.
  The objects (burette, stopcock, flask), the action (a wrist roll that holds a valve angle) and the success
  condition (a burette reading against a titre) are different.
- **Decanting a beaker while keeping the stir bar in it (TK-LBRPTL).** That is pouring: tilt
  control of a vessel so the liquid leaves and the bar stays. Here no vessel is tipped. The volume comes from a
  valve, and the vessel that receives it is swirled.
- **Drawing a 10 mL dose into a syringe (TK-UDCFTR, accepted).** That measures a volume by plunger travel, with no
  valve and no end point. **Pouring a 10 mL dose of syrup (TK-GSEUPC)** is a pour into a cup to a known volume.
  **Loading a vacuum desiccator (TK-Y4ACC5)** has a stopcock, but it only needs to be shut.
- **External benchmarks.** BEHAVIOR-1K, LIBERO and RoboCasa have faucet and knob tasks (turn a sink faucet on or
  off, turn a stove knob). Those are on/off turns with no flow control, no second hand and no end point. We found no
  titration task in the public suites.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, `score()` / `success()`, and `controller()` (the demo that `yam_scripted` plays) |
| `titrkit.py` | The stand, burette, key and flask bodies, the key's friction, the drawn drops and colours, the 240 Hz model update and the hooks into yam_arena |
| `titration.py` | The reduced-order model (plain NumPy): stopcock flow, drops, mixing, colour, the latches |
| `titrdemo.py` | The closed-loop scripted demo |
| `test_burette_titration.py` | Tests: the stopcock curve, drops, the latches, swirl, the lingering flash, every scoring outcome, the seeds, and careful and naive titrations on the model |
| `task.json` | Submission details |

`titrkit.py` hooks yam_arena without modifying it. It patches `yam_arena.scene.task_object` and `object_states`,
the embodiment's `_ensure_sim` and `reset`, and `yam_arena.scripted.YamScriptedPolicy` (when a task file defines
`controller()`, `yam_scripted` asks it for each action). It wraps `scene.write_data_to_sim`, so the model is stepped
at 240 Hz. The task runs with the standard `yam_arena` embodiment, `yam_scripted` policy and `task_score` scorer.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left, z up). The arms
start in the ready pose. The layout is the same in every trial; only the acid aliquot changes.

| Object | Description |
|---|---|
| Retort stand | Cast-iron base plate 25 x 16 x 1 cm, centred at x 34.5, y 0; 12.7 mm steel rod, 70 cm tall, at x 44, y 0 (for example United Scientific RS12, about $35). The plate is clamped to the bench |
| Burette clamp | Single burette clamp with rubber-lined jaws on a boss head, 45 cm above the table, reach 14 cm (about $15) |
| Burette | 25 mL class B borosilicate burette, 0.1 mL graduations, PTFE stopcock key, 620 mm long (for example DWK Pyrex 329504 or Kimble 17026F-25, about $40). Vertical, its axis at x 30, y 0; tip 12.0 cm above the table; stopcock axis 17.5 cm above the table. The burette is turned in its clamp so that the key's axis points 35 degrees from the x axis toward -y (from the handle into the barrel): the key's white winged handle (32 mm span, 5 mm thick) faces the left arm, 2.2 cm from the burette's axis. Closed: the handle lies across the burette (horizontal). Filled to 0.00 mL with 0.05 M sodium hydroxide, tip full, no bubble |
| Tile | White glazed ceramic tile 152 x 152 x 6 mm, on the stand's base plate, centred under the tip |
| Flask | 125 mL narrow-mouth borosilicate Erlenmeyer flask (Pyrex 4980-125: 67 mm base, 114 mm tall, neck about 29 mm outside and 21 mm bore; about $5), standing on the tile centred under the tip. The tip is 1 cm inside the neck |
| In the flask | The trial's acid aliquot (8.00 to 12.00 mL of 0.05 M acetic acid, from the seed's table), water to about 30 mL, 3 drops of 1% phenolphthalein. Colourless |

Nothing else is on the table. The screenshot's inset (top right) is a zoom of the boxed area: the white key
handle, the burette tip and the flask on the tile.

### Reagents and their hazards

- **Titrant: 0.05 M sodium hydroxide** (0.2% w/v). Made from a commercial 0.1 M volumetric solution diluted 1:1 with
  water. At 0.2% it is below the 0.5% concentration limit at which EU CLP classifies sodium hydroxide solutions as
  skin or eye irritants; treat it as nonhazardous, with ordinary lab glasses for the technician.
- **Acid: 0.05 M acetic acid**, white vinegar (5% acetic acid, about 0.83 M) diluted 1 in 16.6 with water. Not
  hazardous. The equivalence volume is then equal to the aliquot volume (both 0.05 M).
- **Indicator: phenolphthalein**, 1% in ethanol. The stock is flammable and classified as a carcinogen, so only
  the technician handles it, with gloves, adding it from a dropper bottle. In the flask it is 3 drops in 30 mL
  (about 0.005%), far below the 0.1% classification limit.

All the liquids are aqueous, dilute and nonhazardous at the concentrations in the scene.

### Real-world build and safety

- **Liquid stays in clamped glassware.** The burette is clamped, the flask is held by its dry neck, and the grippers
  touch only the dry PTFE handle and the outside of the neck. The pads wear nitrile finger cots (size L), changed
  if wet.
- **Drip tray.** The stand stands in a 40 x 30 cm polypropylene lab tray lined with white absorbent bench paper.
  A lost drop shows on the tile or the paper. After any spill: e-stop, wipe the robot and the rig with a damp
  cloth, dry them, and fit new cots.
- **Glass.** If the tip is struck hard enough to chip it, the trial stops and scores 0; the burette is replaced. A
  plastic (PMP) burette with a PTFE key of the same size is a direct substitute if the lab prefers no glass near
  the robot.
- **Forces.** The flask with its liquid weighs about 115 g; it is held by friction on the neck (a few newtons of
  grip). The key needs about 0.05 N m to turn: measure it with a 0-5 N spring gauge hooked on the end of a handle
  wing (1.6 cm from the axis): it should start to turn at 2 to 5 N. Tighten or loosen the key's tension nut to
  match. Both are far inside the YAM's limits (about 40 N per finger, 1.5 kg per arm).
- **Reach and view.** The stopcock is 17.5 cm above the table (the top camera sees up to about 20 cm). The left
  gripper reaches the handle along the key's axis, horizontal, with its fingers closing vertically. The right
  gripper reaches the neck horizontally from the right. We checked both grips and the key's roll from 0 to
  90 degrees with the arm's inverse kinematics; the wrist roll stays inside its limits.

### Per-trial acid aliquot

The seed picks the aliquot: `ve_for(seed)` in `titrkit.py` (uniform 8.00 to 12.00 mL, to 0.05 mL). With `--seed` 0, 4 and 10 the
aliquots are 9.60, 11.05 and 11.55 mL (inspect-robots derives the episode seed from `--seed`; the log prints the
aliquot as `[titrkit] seed ...: equivalence at ... mL`). In a real trial, the technician dispenses the aliquot from a second burette or weighs it on a
0.01 g balance (1.00 g/mL), and records it. Before a session, the technician standardises the batch: three manual
titrations of a 10.00 mL aliquot give the ratio of titrant to acid. The trial's equivalence volume V_e is the
aliquot times that ratio. The robot is not told the aliquot.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Titrate the acid in the flask with the burette to the first permanent pink: swirl the flask while you add the titrant, and close the stopcock as soon as the pink colour stays." \
  --scorer task_score --max-steps 3600 --seed 0 --store-frames --log-dir ~/yam-eval/burette-dream
```

`--policy yam_scripted` plays the scripted demo instead. `task_score` gives 1.0 on success, otherwise `score()` at the
end of the episode. `TITR_DEBUG=1` prints the model's state every second (key angle, flow, burette reading,
patch, excess over equivalence, swirl, tip position in the flask). The seed sets the aliquot (`--seed`; if the
runner gives none, the environment variable `TITRATION_SEED`, default 0). Runtime per trial on one shared RTX-class GPU: about 6 to 8 minutes for the scripted demo (stops at success after
about 50 s of simulated time), about 28 minutes for a Dream-1 run of the full 3,600 steps.

## Scoring

The model is updated every physics step (240 Hz). The score at the end of the episode counts
(`SCORE_OVER = "end"`). T is the burette reading at the end; V_e is the trial's equivalence volume.

| Condition | Score | Real-trial check |
|---|---|---|
| At any time: the flask's glass touched the burette tip | 0 | Video (the tip knocks, the burette moves), and the tip inspected for chips |
| At any time: the flask tilted more than 45 degrees | 0 | Video |
| At any time: titrant fell outside the flask | 0 | A drop on the tile, tray or bench paper (photo) |
| The burette ran empty (T reached 25 mL) | 0 | Burette reading |
| At the end, the stopcock still lets titrant through | 0 | Video: a drop forms or falls after the end; or a drop falls within 30 s |
| Past the end point once mixed, and T - V_e <= 0.30 mL | **1.0** | pH meter: 8.4 <= pH <= 10.6 |
| Past, 0.30 < T - V_e <= 1.00 mL | 0.6 | 10.6 < pH <= 11.1 |
| Past, 1.00 < T - V_e <= 3.00 mL | 0.3 | 11.1 < pH <= 11.55 |
| Past, further over | 0 | pH > 11.55 |
| Short once mixed, T >= V_e - 1.00 mL (stopped on a fading flash) | 0.3 | 5.7 <= pH < 8.4 |
| Short, T >= V_e / 2 | 0.1 | 4.8 <= pH < 5.7 |
| Anything less | 0 | pH < 4.8 |

"Past the end point once mixed" is computed in simulation from all the titrant that landed: the bulk turns pink at
0.02 mL (under half a drop) past V_e. The robot uses the indicator's colour to know when to stop. The real trial
judges the end point with a pH meter, not by eye.

### Real-trial pH reading

- **Meter.** A benchtop pH meter with 0.01 pH resolution, for example the Mettler Toledo FiveEasy Plus FP20 with
  its LE438 combination electrode (12 mm shaft) and electrode arm (about $600 as a kit). A pocket meter with
  0.01 pH resolution (Hanna HI98103 Checker, about $40) is a cheaper alternative.
- **Calibration.** At the start of each session, a three-point calibration with pH 4.01, 7.00 and 10.01 buffers at
  room temperature (slope 95 to 105%, offset within 30 mV, or recalibrate). Check the pH 10.01 buffer again every
  10 trials; recalibrate if it reads more than 0.05 pH off.
- **Reading.** After the trial, the technician reads and records the burette (to 0.05 mL, a cross-check) and
  raises the burette 3 cm in its clamp so the tip clears the neck. The liquid in the Erlenmeyer is only about 1.5 cm
  deep, which is too shallow for the electrode's junction. So the technician pours all of it into a 50 mL beaker,
  swirls the beaker 10 s, and lowers the electrode on the electrode arm into it: the electrode is clamped upright,
  off-centre and 5 mm above the floor, clear of the swirl. The meter's auto-endpoint gives a stable reading (within
  about 30 s). The technician records the pH to 0.01 and the temperature, then rinses the electrode with water.
  The reading is taken within 2 minutes of the end of the trial, because air's CO2 slowly lowers the pH of an
  alkaline solution.
- **The indicator does not shift the pH.** 3 drops of 1% phenolphthalein in about 40 mL is far too little to
  buffer the solution.

### From burette volume to pH

The flask starts as 0.05 M acetic acid (pKa 4.76, aliquot V_e of 8 to 12 mL) made up to 30 mL. After T mL of
0.05 M NaOH the volume is 30 + T mL. The pH comes from the charge balance
[Na+] + [H+] = [acetate] + [OH-], with [acetate] = C_acid Ka / (Ka + [H+]). Three cases give simple formulas:

- **Short (T < V_e).** An acetate buffer: pH = 4.76 + log10(T / (V_e - T)). At T = V_e / 2: pH 4.76. At
  T = V_e - 1.00 mL with V_e 10: pH = 4.76 + log10(9) = 5.71.
- **At equivalence (T = V_e).** 0.5 mmol acetate in 40 mL (0.0125 M): [OH-] = sqrt(Kw / Ka x 0.0125)
  = 2.7e-6 M, so pH 8.43.
- **Past (T > V_e).** Excess hydroxide: [OH-] = 0.05 (T - V_e) / (30 + T). At +0.30 mL with V_e 10:
  0.015 mmol / 40.3 mL = 3.7e-4 M, pH 10.57. At +1.00 mL: 1.2e-3 M, pH 11.09. At +3.00 mL: 3.5e-3 M, pH 11.54.

The full charge balance, solved over the whole range of aliquots, gives:

| Sim threshold (T - V_e) | pH, V_e 8 mL | pH, V_e 10 mL | pH, V_e 12 mL | Real bound used |
|---|---|---|---|---|
| -V_e / 2 | 4.76 | 4.76 | 4.76 | 4.8 |
| -1.00 mL | 5.61 | 5.71 | 5.80 | 5.7 |
| 0 (equivalence) | 8.39 | 8.43 | 8.46 | 8.4 |
| +0.02 mL (sim colour threshold) | 9.42 | 9.40 | 9.38 | (8.4 used) |
| +0.30 mL | 10.59 | 10.57 | 10.55 | 10.6 |
| +1.00 mL | 11.11 | 11.09 | 11.07 | 11.1 |
| +3.00 mL | 11.56 | 11.54 | 11.52 | 11.55 |

The overshoot side is the steep part of the curve, and the aliquot barely moves it (0.04 pH). One more 0.05 mL drop
near +0.30 mL raises the pH by about 0.07, so a reading to 0.01 pH places the end point to well under a drop. On the
short side the aliquot moves the -1.00 mL bound by up to 0.1 pH, which is 0.2 mL. That shifts only the 0.3 / 0.1
boundary, not full credit. One gap is less than half a drop wide: between equivalence and +0.02 mL (pH 8.4 to 9.4)
the simulation counts the flask as not yet pink (0.3), while the pH meter gives 1.0. A drop is 0.05 mL, so a
titration rarely ends there. We kept the simulation's colour threshold as it is.

The 0.30 mL window is 6 drops: a titrator working drop by drop near the end
point stays well inside it, while a stream (0.4 mL/s) shut on the first lasting pink overshoots it.

## Changes since the last review

- TK-688YJC: "Can the scoring be done with a ph meter instead?" Yes. The real trial now judges the end point with a
  calibrated pH meter (0.01 pH, pH 4/7/10 buffers) instead of a photo of the flask's colour. Each sim volume
  threshold is mapped to a pH window, with the calculation, under Scoring. The simulation is unchanged.

## Reset

Close the key. Lower the burette back to its clamp mark if it was raised for the pH reading. Empty and rinse the flask, then dry its neck. Refill the burette to 0.00 mL (run a little through
the tip to clear bubbles). Dispense the next seed's aliquot into the flask, add water to about 30 mL and 3 drops
of indicator, and stand the flask on the tile under the tip (tip 1 cm inside the neck: mark the tile with the
flask's outline). Turn the burette in its clamp so the handle faces the left arm at 35 degrees (a tape mark on
the clamp). About 2 minutes per trial.

## Scripted demo

`titrdemo.py`, played by `yam_scripted`. It sees only the top camera's image and the arms' joint positions. It knows
the layout from the setup (where the key and the flask are), not the aliquot and not the simulator's state.

1. The right arm reaches in from the right and grips the flask's neck 1.6 cm below the rim, fingers either side of
   it (the tip inside the neck is not touched), and lifts it 4 mm. The left arm reaches the key along its axis and
   grips the handle wings between its pads.
2. The right arm swirls the flask for the rest of the titration: the neck goes round 4 mm circles twice a second.
3. **Coarse.** The left wrist rolls the key to 86 degrees from closed (a fast drip, about 5 drops a second). The top
   camera watches the flask's liquid. Each drop leaves a strong pink flash that the swirling clears. Near the end
   point the flash lingers and grows. When strong pink covers 10% of the pale pixels over the liquid for 4 frames
   in a row, the key goes back to 79 degrees (about 2 drops a second).
4. **Fine.** It keeps swirling and dripping until the whole liquid turns faint pink (30% of the pale pixels for
   4 frames), then rolls the key closed at once. If a flash grows very large first, it closes the key and adds
   single drops instead (open 1 s, closed, swirl until the flash fades or the pink holds). The demo runs never
   needed this.
5. It keeps swirling 2 s, sets the flask down and lets go of both. The episode stops at success, which comes
   during this step.

Results:

| `--seed` | V_e (mL) | Switch to fine | Key shut | Burette reading (mL) | Over (mL) | Score |
|---|---|---|---|---|---|---|
| 0 | 9.60 | 44.3 s | 48.2 s | 9.81 | +0.21 | 1.0 |
| 4 | 11.05 | 50.1 s | 53.8 s | 11.26 | +0.21 | 1.0 |
| 10 | 11.55 | 52.0 s | 55.8 s | 11.74 | +0.19 | 1.0 |

The demo needs about 1,000 control steps. The run limit (3,600 steps, 180 s) leaves room for a learned policy's
slower grasps and for single-drop work near the end point.

Contrasts (`DEMO_CHEAT`):

| Contrast | What it does | Result | Score |
|---|---|---|---|
| `noswirl` | The demo's valve logic, but the flask is held still | The flash never clears, so the demo never sees the end point; still dripping at the end, 13.8 mL in for V_e 9.60 | 0 |
| `fullopen` | Key fully open (a stream), shut on the first whole-liquid pink, with swirling | Reading 10.15 for V_e 9.60 (+0.55 mL: the stream outruns the mixing) | 0.6 |

The unit tests also cover running titrant past the flask (moved away from the tip: 0), striking the tip (0),
tipping the flask (0), an unswirled careful titration (below 1.0) and every scoring branch.

Dream-1 (`--seed 0`, 3,600 steps): the arms move toward the table but never reach the key or the flask; the key
stays closed and nothing is added. Score 0.

## Fidelity

- **Bodies.** The stand, burette and tile are a fixed (kinematic) body. The flask is a PhysX rigid body; its
  colliders are a stack of five cylinders for the conical body and the neck. The key is a rigid body on a revolute
  joint along the barrel's axis; its handle wings are the collider the fingers grip. The key's friction is an
  elasto-plastic stick torque (0.05 N m, 0.36 degrees of give) applied every physics step. To keep that torque
  stable at 240 Hz the key's moment of inertia is raised to 2e-3 kg m^2 (a real key's is about 1e-6).
- **The tip has no collider.** The model latches a strike whenever the flask's glass (its neck or rim) comes
  within the tip's radius (3 mm). A real tip would be pushed instead, and could chip; both score 0.
- **No fluid simulation.** The liquid is a reduced-order model (`titration.py`, documented in its header):
  stopcock flow from the overlap of the key's bore with the barrel's channel; drops of 0.05 mL below 0.25 mL/s and
  a stream above; a two-pool mixing model (unmixed titrant mixes into the bulk at a rate that depends on how hard
  the flask is swirled, measured as the RMS horizontal acceleration of the flask); a pink patch where the drops land
  that fades as it mixes, more slowly over the last 1.5 mL; and the bulk turning pink 0.02 mL past
  equivalence. The flow curve's bore (1.2 mm) and plug radius (4 mm), and the mixing rates, are chosen values,
  not measured on a real key; a real key is what it is, and the real trial measures the outcome (the flask's pH,
  and the burette reading as a cross-check) directly. The liquid's level does not rise in the drawing, and the liquid is not drawn swirling.
- **Colour.** The flask's liquid is drawn in four colour stages (colourless, faint pink, pink, magenta) and the patch
  in four sizes, switched each control step.

## Assets and licences

No third-party meshes, textures or scans. Every object is built from USD primitives (boxes, cylinders, spheres) in
`titrkit.py`. The robot is the YAM arm from the yam_arena embodiment (I2RT YAM URDF and meshes, MIT licence,
as distributed with yam_arena; the arm colour parts are Dream Labs' MIT-licensed split of the same meshes). Isaac
Sim / Isaac Lab and Isaac Lab-Arena are used under their own licences (NVIDIA Omniverse licence; BSD-3-Clause).

## Real hardware list

| Item | Example | Approx. price |
|---|---|---|
| 25 mL class B glass burette, PTFE key | DWK Pyrex 329504 or Kimble 17026F-25 | $40 |
| Retort stand, 25 x 16 cm cast-iron base, 70 cm rod | United Scientific RS12 or similar | $35 |
| Burette clamp with boss head | Any single burette clamp, rubber-lined jaws | $15 |
| 125 mL Erlenmeyer flask, narrow mouth (x 3, to rotate) | Corning Pyrex 4980-125 | $15 |
| White ceramic tile 152 x 152 mm | Any glazed wall tile | $2 |
| 0.1 M NaOH volumetric solution, 1 L | Any supplier | $15 |
| White vinegar 5%, 1 L | Grocery | $2 |
| Phenolphthalein 1% in ethanol, 30 mL dropper bottle | Any supplier | $8 |
| Polypropylene lab tray 40 x 30 cm, bench paper | Any | $20 |
| Nitrile finger cots, size L (box) | Any | $8 |
| 0-5 N spring gauge (key torque check) | Any | $10 |
| Benchtop pH meter, 0.01 pH, with electrode and electrode arm | Mettler Toledo FiveEasy Plus FP20 kit with LE438 | $600 |
| pH 4.01, 7.00 and 10.01 buffers, 250 mL each | Any supplier | $30 |
| 50 mL glass beakers (x 3) | Any | $10 |

Total about $810 (about $250 with a pocket meter such as the Hanna HI98103).
