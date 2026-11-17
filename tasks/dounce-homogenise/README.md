# Homogenise a sample in a Dounce: ten full strokes of the tight pestle, head kept under the liquid, mortar held still

A 7 mL glass Dounce tissue grinder stands in a tube rack on a lab bench. Its tight pestle rests in the mortar with
the head on the floor of the bore, under 3.0 to 3.4 mL of sample. The robot homogenises the sample.

**Instruction:** `Homogenise the sample in the Dounce with ten full strokes of the pestle: each stroke down to the
bottom and back up, keeping the pestle's head under the liquid, and hold the mortar still.`

## Why this task

- **The real procedure.** Dounce homogenisation is the standard hand method for breaking cells open and isolating
  nuclei, mitochondria and other organelles in biochemistry and cell-biology labs. Protocols specify it exactly
  like this: "10 strokes with the tight pestle", the mortar held in one hand (normally on ice), the pestle worked
  with the other, slowly, without lifting the head out of the liquid, because air drawn in foams the sample and
  denatures proteins. The tool is the standard tool: a borosilicate Dounce with a loose (A) and a tight (B) pestle.
  Category **Other**, as laboratory research automation (common wet-lab sample preparation done by hand).
- **The difficulty is physical.**
  - *Force along a tight fit.* The tight pestle has 0.025 to 0.076 mm radial clearance in a 10 mm bore. The sample
    can only pass the head through that gap, so the head works like a piston: about 8 N of drag at 50 mm/s (see
    Forces), pushing on the way down and sucking on the way up. The hand must drive the knob straight along the
    bore's axis: the tube is held only by the other hand and a loose rack hole, so a sideways push tips it.
  - *A position limit on every upstroke.* Each stroke must reach the floor and come back up at least 25 mm, but the
    head must stay under the liquid: once all the sample is below the head (38 to 43 mm up, depending on the
    trial's volume) a further pull draws air past the head and the sample foams. The usable window is 13 to 18 mm
    on every one of ten strokes. Pulling faster than about 50 mm/s cavitates (suction above one atmosphere) and
    also makes bubbles.
  - *Bimanual hold against the reaction.* The suction on every upstroke pulls the 35 g mortar up with the pestle.
    The other hand must hold it down in the rack for the whole trial; a mortar lifted more than 5 mm scores 0.
- **Not a covered ability.** It is not a hold-and-turn, a snap-fit or a pick-and-place: it is repeated
  reciprocation of a close-fitting piston under load, with a position limit per stroke, while the second hand
  holds the reaction.

### Distinct from

- **Our burette titration task (wet lab, Other, already submitted).** That task meters a liquid through a valve: a
  wrist roll that holds a stopcock angle inside a 10-degree window while the other hand swirls a flask, stopped
  at a colour end point. Nothing is pushed or pulled there. Here the skill is stroke force and travel: driving a
  tight-fit pestle up and down along its axis against drag and suction, ten times, with a per-stroke travel
  window, while the other hand holds the reaction. Different objects, motion and success condition.
- **French press plunge (TK-ZUWPVL, pending).** One slow downward plunge of a loose mesh filter, then a lid turn.
  No upstroke, no suction on the vessel, no travel limit, no repetition.
- **Syringe tasks (TK-YHWYSG purge air, TK-UDCFTR draw a dose; accepted).** A plunger moved once to a volume mark.
  Here the piston is the whole pestle, reciprocated ten times bottom to a limit, with the vessel held against the
  reaction.
- **External benchmarks.** We found no homogeniser, pestle or grinding task in BEHAVIOR-1K, LIBERO, RoboCasa,
  SimplerEnv or the others the review lists. The nearest are pushing or pulling drawers and plungers
  (RoboCasa open/close drawer, BEHAVIOR-1K "press" tasks): single strokes against a guide with no reaction to hold
  and no travel limit.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, `score()` / `success()`, and `controller()` (the demo that `yam_scripted` plays) |
| `dkit.py` | The Dounce as a floating articulation (mortar root, pestle on a prismatic joint with damping), the drawn sample and foam, the hooks into yam_arena |
| `dounce.py` | The liquid and stroke model (plain NumPy): liquid level, strokes, air, cavitation, foam, latches, the score |
| `ddemo.py` | The scripted demo (and the contrasting attempts, `DEMO_CHEAT`) |
| `test_dounce_homogenise.py` | Offline tests: geometry, the stroke counter, air and cavitation, every scoring rule |
| `task.json` | Submission details |

`dkit.py` hooks yam_arena without modifying it: `yam_arena.scene.task_object` and `object_states`, the embodiment's
`reset`, and `yam_arena.scripted.YamScriptedPolicy`. The task runs with the standard `yam_arena` embodiment,
`yam_scripted` policy and `task_score` scorer.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left, z up). The arms
start in the ready pose. The layout is the same in every trial; only the sample volume changes (3.0 to 3.4 mL, set
by the seed).

| Object | Description |
|---|---|
| Tube rack | Nalgene Unwire test-tube rack for 13 mm tubes, 5970-0213 (yellow acetal, 200 x 102 x 57 mm, 72 places, about $30), its long side across the robot (along y), centred at x 30, y 0. The mortar stands in the hole nearest the rack's centre; the sim models the rack as its base, two end panels and a top tier with one 15 mm hole |
| Dounce tissue grinder | DWK Wheaton 357542, 7 mL, borosilicate, with loose (A) and tight (B) glass pestles (about $150 a set; one set of the two-pack). Mortar: grinding tube 13 mm OD, 10 mm bore, 82 mm long, with a 20 mm reservoir on top; standing vertical in the rack, its axis at x 30, y 0, bottom on the rack's base. Only the tight pestle (B) is used: 10 mm head, 6 mm shaft, knob on top, 175 mm long, resting in the mortar with the head on the floor |
| Sample | 3.0 to 3.4 mL (from the seed) of tap water with 0.1% (v/v) dish detergent, dispensed with a 5 mL pipette. Fills the bore to 7 to 8 cm with the pestle down |
| Bench | Grey silicone bench mat (40 x 30 cm) under the rack |

Nothing else is on the table.

### The medium

The skill is the stroke, not the biology, so the sample is a standard, repeatable stand-in for a cell suspension:
water with 0.1% dish detergent. It behaves like a lysis buffer for the stroke (aqueous, about 1 mPa s), and it
makes the real failure visible: any air drawn past the head foams it, and the foam stays for minutes. It is made
fresh in seconds (1 mL of detergent in 1 L of water), costs nothing and is not perishable.

### Real-world build and safety

- **Glass on the rig.** The mortar never leaves the rack: it stands in the rack's hole, 5.7 cm deep, and the rack
  stands on a silicone mat. If it is ever lifted more than 5 mm the trial is over (score 0). The grippers touch only
  the dry outside of the reservoir and the dry knob; nitrile finger cots on the pads. The glass is borosilicate,
  1.5 mm wall; a drop from the rack's height onto the silicone mat is unlikely to break it. If the lab prefers no
  glass near the robot, a Dounce with the same bore and a PTFE or polypropylene pestle may be substituted; the
  clearance must be the tight grade (under 0.08 mm) for the drag below to hold.
- **Liquid.** About 3 mL of soapy water, held inside a 10 mm bore under a pestle. A spill needs the mortar to tip,
  which ends the trial. Wipe up with a paper towel; nothing harmful.
- **Forces.** The drag on the tight pestle with water is a few newtons: for a concentric head the annular-gap
  formula gives about 19 N at 50 mm/s with 0.05 mm clearance, and an off-centre head about 2.5 times less; we
  use 8 N at 50 mm/s (160 N s/m). The demo's strokes run at up to 25 mm/s (about 4 N). Measure it on the real set
  by standing the mortar on a kitchen scale (0.1 g) and pushing the pestle down at about one stroke per two seconds:
  the reading rises by 300 to 900 g. Both well inside the YAM's limits (about 40 N per finger, 1.5 kg per arm).
- **Reach and view.** The knob is 17 to 21 cm above the table during the strokes. The right gripper points
  straight down the bore's axis and grips the knob between its pads (14 mm across, 94 mm opening). The left
  gripper reaches the reservoir horizontally from the left, 9 to 11 cm above the table, with its fingers closing
  front to back. We checked both poses through the full stroke with the arm's inverse kinematics.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Homogenise the sample in the Dounce with ten full strokes of the pestle: each stroke down to the bottom and back up, keeping the pestle's head under the liquid, and hold the mortar still." \
  --scorer task_score --max-steps 2400 --seed 0 --store-frames --log-dir ~/yam-eval/dounce-dream
```

`--policy yam_scripted` plays the scripted demo; `DEMO_CHEAT=nohold|high|short` plays the contrasting attempts.
`DOUNCE_DEBUG=1` prints the model's state every half second (head height and speed, mortar lift and tilt, strokes,
air, cavitation). The seed sets the sample volume: `volume_for(seed)` in `dounce.py`. RUNTIME_LINE

## Scoring

The model is stepped after every control step (20 Hz). The score at the end of the episode counts
(`SCORE_OVER = "end"`). z is the height of the pestle head's bottom face above the bore's floor.

| Condition | Score | Real-trial check |
|---|---|---|
| At any time the mortar rose more than 5 mm off the rack's base | 0 (latched) | Video against a tape mark on the reservoir level with the rack's top tier |
| At any time the mortar tipped past 20 degrees | 0 (latched) | Video |
| At any time the head was pulled up out of the grinding tube into the reservoir (z > 70 mm) | 0 (latched) | Video: the head is visible through the glass |
| Each full stroke: the head within 3 mm of the floor, then at least 25 mm above it, then back to the floor | +0.08, up to 10 strokes (0.8) | Video: tape marks on the tube at 3 mm and 25 mm above the floor |
| Foam on the sample at the end under 2 mm | +0.2 | Photo of the tube against a ruler after the trial |
| Each upstroke on which the head rose above the liquid (all of the sample below the head) and drew air in | -0.05 each | Video: the head's bottom face above the meniscus |

The total is floored at 0 and capped at 1. Success = 1.0. In the model, an upstroke that draws air leaves 2 mm of
foam, and one faster than the cavitation speed (about 50 mm/s) leaves 0.5 mm; so any air, or four or more fast
pulls, lose the foam bonus. Every outcome is covered: a trial that does nothing scores 0.2 (no strokes, no foam);
the stroke count needs both the floor and the 25 mm rise; the latches cover lifting, tipping and pulling out.

| Run | Score |
|---|---|
RESULTS_TABLE

## Reset

Lift the pestle out, pour the sample away, rinse the mortar and pestle with water, shake dry. Pipette the next
seed's volume of detergent water into the mortar, slide the tight pestle in slowly until the head rests on the
floor, wipe the outside dry, and stand the mortar in the marked hole. About 1 minute.

## Fidelity

Approximate. The rack is fixed. The Dounce is a floating articulation: the mortar (35 g) is the root with tube and
reservoir colliders; the pestle (15 g) slides on a prismatic joint along the bore's axis, which stands in for the
tight fit (no tilt, no turn; the real 0.05 mm clearance allows neither to any useful degree). The sample's
resistance is the joint's damping, 160 N s/m, acting equally on pestle and mortar (so a pull lifts an unheld
mortar); it is the same whether the head is under the liquid or not. There is no fluid simulation: liquid level,
air drawn in, cavitation and foam come from the model in `dounce.py` (quasi-static level from the volume and the
pestle's displacement). Masses are real; the constants are estimated, not measured.

## Assets and licences

No third-party meshes, textures or scans. Every object is built from USD primitives (boxes, cylinders) in
`dkit.py` and `task.py`. The robot is the YAM arm from the yam_arena embodiment (I2RT YAM URDF and meshes, MIT
licence, as distributed with yam_arena; the arm colour parts are Dream Labs' MIT-licensed split of the same
meshes). Isaac Sim / Isaac Lab and Isaac Lab-Arena are used under their own licences (NVIDIA Omniverse licence;
BSD-3-Clause).

## Real hardware list

| Item | Example | Approx. price |
|---|---|---|
| 7 mL Dounce tissue grinder, glass, loose and tight pestles | DWK Wheaton 357542 (sold in pairs, about $300 a pair; one set used) | $150 |
| Test-tube rack for 13 mm tubes | Nalgene Unwire 5970-0213 | $30 |
| Silicone bench mat 40 x 30 cm | any | $10 |
| Dish detergent, tap water, 5 mL pipette | any | $5 |
| Nitrile finger cots, kitchen scale 0.1 g (force check) | any | $10 |
| **Total** | | **about $205** |
