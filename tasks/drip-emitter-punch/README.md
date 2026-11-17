# Install a drip emitter: punch the 1/2 in line at the mark, push the barbed dripper in square

A planting bed (plywood under brown landscape fabric) lies on the table. Across it runs a 1/2 in poly drip line,
held down by two wire tubing stakes. A white paint-pen ring on the tube marks where the shrub standing just behind
the line gets its dripper. On the bed to the right, a drilled pine block holds the emitter punch (point down in its
hole) and one barbed pressure-compensating dripper (barb down in its hole).

**Instruction:** `Install the dripper in the drip line at the white mark, then put the punch back in its block.`

## Why this task

- **It is routine landscaping work.** Landscaping and groundskeeping workers (O*NET 37-3011, about 1.0 million US
  workers) list installing and maintaining irrigation systems among their tasks. On drip systems, adding a dripper
  for a new plant, or replacing a clogged one, is done the same way every time: punch a hole in the 1/2 in line with
  an emitter punch, push the dripper's barb into it until it pops in to its collar. A crew sets dozens per bed.
- **The difficulty is two force-staged pushes at a point the robot cannot see when it matters.** The punch has to go
  through the tube wall on the mark, square to the tube's surface, with about 25 N; pushed at an angle on a round
  tube it slips off and tears a slit that leaks. The dripper's 4 mm barb then has to find the 3 mm hole, which its
  30 mm body hides from above, and be pushed in square with about 15 N; pushed in crooked it tears the hole. A
  second hole leaks too. Nothing in the scene does this work for the robot: the line lies as staked, the hole exists
  only once the robot punches it.

### Distinct from

- **Board tasks.** Pushing a coolant hose onto a rigid barb and clamping it (the hose goes over a fixed barb; there
  is no hole to make and no hidden target); punching a card's filing holes on a desk punch (a lever machine with a
  fixed die); drilling at a punch mark on a drill press. Pressing a part into a snap-fit until it clicks is a covered
  ability; here the robot first makes the hole with a hand tool at the right spot and angle and force, then has to
  hit that 3 mm hole blind with the barb.
- **External benchmarks.** No BEHAVIOR-1K, LIBERO, RoboCasa, DataCenterBench or RoboDojo task uses irrigation
  tubing, a punch or a barbed emitter. The nearest are peg-in-hole insertion tasks (for example the insertion tasks
  of the assembly suites): the hole is given and rigid. Here the robot makes the hole itself with a tool, with a
  minimum force and angle, on a round tube, and the hole is then hidden under the part.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, `score()` / `success()`, and `controller()` (the demo `yam_scripted` plays) |
| `dripkit.py` | The bodies (bed, line, stakes, pot, block, punch, dripper), the 240 Hz punch-and-seat rule, and the hooks into yam_arena |
| `dripdemo.py` | The scripted demo |
| `test_drip_emitter_punch.py` | Unit tests of the rule and every scoring outcome (no Isaac Sim) |
| `block_drawing.svg` | Dimensioned drawing of the drilled pine block |
| `task.json` | Submission details |

`dripkit.py` hooks yam_arena without modifying it (as our other tasks' kits do): it patches
`yam_arena.scene.task_object` and `object_states`, the embodiment's `_ensure_sim` and `reset`, and
`yam_arena.scripted.YamScriptedPolicy` (when a task file defines `controller()`, `yam_scripted` asks it for each
action), and wraps `scene.write_data_to_sim`, called once per physics step.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left, z up). Arms start in
the ready pose. The layout is the same in every trial; mark it with tape.

| Object | Description |
|---|---|
| Bed | 12 mm plywood, 30 (x) x 64 (y) cm, centred at x 33, y -2, covered with brown woven landscape fabric stapled underneath |
| Drip line | 1/2 in poly drip distribution tubing, 0.700 in (17.8 mm) OD, about 0.9 mm wall, brown/black (e.g. Rain Bird T63-100S or any 0.700 in OD tubing; 100 ft about $20). A 62 cm length lying on the bed along y at x 30, from y -33 to +29, both ends closed with figure-8 end closures. A fresh length (or an unpunched stretch) each trial |
| Tubing stakes (2) | Rain Bird TS-025 1/2 in tubing stakes (wire hoops, about $0.30 each), over the line at y -20.5 and +4.5, pushed through the fabric into 3 mm holes in the plywood |
| Mark | A white paint-pen ring, 3 mm wide, round the tube at y -8 (seeded in simulation: -9.5 to -6.5; in a real trial draw it anywhere in that range and measure it) |
| Shrub | An artificial boxwood about 14 cm across in a black 1 gal nursery pot (16.5 cm across, 17 cm tall), the pot standing on the bed at x 45, y -8, just behind the line at the mark |
| Tool block | Pine 2x4 offcut, 89 (x) x 120 (y) x 38 mm, lying flat on the bed centred at x 40, y -25 (`block_drawing.svg`): a 6 mm hole for the punch at y -28.8 and a 5 mm hole for the dripper at y -21.2 (each 22 mm from its end), both on its centre line, 34 mm deep |
| Emitter punch | Netafim punch tool, 3 mm (part 682901033, about $6): yellow plastic handle about 28 mm across and 87 mm long, 3 mm steel point 15 mm long below an 8 mm stop shoulder, about 40 g. Standing point down in its block hole, the shoulder on the block |
| Dripper | Netafim Woodpecker pressure-compensating dripper with barbed base (e.g. #01WP2, 0.5 GPH; about $0.40), about 30 mm across and 21 mm tall above a 4 mm, 7 mm long barb, 2 g. Standing barb down in its block hole |
| Ruler, 3 mm drill bit (as a gauge), phone camera | For scoring only, off the table during the trial |

Nothing else is on the table. The left arm is not needed (it may be used, for instance to steady the line).

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Install the dripper in the drip line at the white mark, then put the punch back in its block." \
  --scorer task_score --max-steps 1600 --seed 0 --store-frames --log-dir ~/yam-eval/drip-emitter-punch
```

The scripted demo: the same with `--policy yam_scripted`. It reads the simulator's state (where the mark and the hole
are, and where the held punch or dripper sits in the fingers) to aim and to keep the part upright while pushing,
and it moves everything only through the arm; the pushes are ramps of the arm's command until the wall gives. Contrasts (`DEMO_CHEAT=`): `nopunch` (pushes the dripper
onto the mark without punching), `tilt` (punches with the tool leaning 18 degrees), `light` (a third of the push),
`twice` (a second hole 15 mm along). `DRIP_DEBUG=1` prints the tools and pushes every 0.1 s. Unit tests:
`python -m pytest test_drip_emitter_punch.py`.

Seeds: `DRIP_SEED=<n>` (environment variable; default 0) sets the mark's place along the line (-8 +- 1.5 cm) and the push the tube wall takes (25 N +-10%,
as tubing walls differ).

Runtime: the scripted demo finishes in 830 to 960 steps (42 to 48 s simulated at 20 Hz); one trial takes about 4
minutes of wall-clock time on one GPU, Isaac Sim start-up included. The run limit is 2,000 steps (100 s).

## Scoring

From the line, the punch and the trial video, at the end of the trial (the simulation scores the last step):

- **0.4** a round hole punched within 10 mm of the mark (along the line);
- **+0.5** the dripper seated in that hole (barb through the wall, collar down on the tube);
- **+0.1** then the punch standing back in its block hole, let go.
- **Cap 0.2** if the line has a torn hole (punched more than 15 degrees off square, or the dripper forced in more
  than 15 degrees off square), or a second hole anywhere: the line leaks however the rest ends.

Success is 1.0, held 1 s. A dripper seated anywhere but the mark's hole scores 0 for it and the punch.

| Sim condition | Real-trial check |
|---|---|
| Hole made (punch pushed on the line with the wall's force for 0.1 s, its point on the tube) | Inspect the line after the trial: a hole through the wall (the 3 mm drill bit's shank enters it) |
| Round, not torn (punch within 15 degrees of the surface normal) | Close photo with a ruler: the hole is round, no slit or tear longer than 4 mm |
| Within 10 mm of the mark | Ruler from the mark ring's centre to the hole's (or the seated dripper's) centre, along the tube |
| Dripper seated (barb on the hole within 1.5 mm, pushed with 15 N for 0.1 s, within 15 degrees of square) | Photo: the dripper's base touches the tube all round (no gap); it does not come off with a firm hand pull |
| Dripper forced in crooked: torn | Pull the dripper out after the trial: the hole is torn (slit longer than 4 mm) |
| Second hole | Inspect the whole tube: any other hole through the wall |
| Punch back in its block hole, let go | Photo at the end: the point down in its hole, the shoulder on the block, no finger on it |

## Fidelity

Approximate. The simulator cannot pierce plastic, so a rule (in `dripkit.py`, unit-tested) stands in for the wall,
and the robot has to do the real work to trigger it: the same grip on the punch, its point on the tube, the real
push held for a moment, and the barb on the hole it made, pushed in with the real force.

- **The punch.** A hole is made where the punch's point is (within 2 mm of the tube's surface) once the punch has
  pressed on the line with at least the wall's force (25 N +-10% per seed) for 0.1 s (pushes are read through a
  0.03 s filter, so a contact spike does not count). It is round when the punch was
  within 15 degrees of the surface normal there, torn otherwise. Pressing again within 4 mm of a hole is the same
  hole. The point's plunge through the wall (to its stop) is not drawn; the hole appears as a dark 3.4 mm disc.
- **The dripper.** It seats once its barb's tip is on a hole (within 1.5 mm across, at the surface) and it has
  pressed on the line with at least 15 N for 0.1 s. Square (within 15 degrees of the hole's normal) it is set to the
  seated pose (barb through the wall, base on the tube) and held there for the rest of the trial; crooked, it tears
  the hole (latched). A barb pressed into a hole's mouth (2 N or more) stays in it while it is pressed, up to 4 mm
  across: a real hole holds the barb's point, the rigid round tube in the simulation would let it skid off. It cannot be pulled out again (real ones hold far more than a gripper's accidental tug).
- **Where the numbers come from.** We found no published force for punching 1/2 in drip tubing or for pushing in an
  emitter. Users and installers describe a firm push with the palm for the punch and a push until the barb "pops" for
  the emitter. Our values are estimates for a 0.9 mm LDPE wall and a 3 mm point. To measure them: lay the tube on a
  bathroom or kitchen scale (5 kg, 1 g) on a hard floor, tare it, punch and push in an emitter by hand, and film the
  display: the peak reading at the moment the point goes through (g x 0.0098 N) is the punch force, the peak as the
  barb pops in is the seat force. Set the thresholds from 10 such punches if they differ. The YAM can give these: 25
  N at the tool is about 7.5 N m at the shoulder joints (27 N m peak), and a 40 N grip on a plastic handle holds well
  over 25 N along it.
- **Simplified.** The line is fixed in place (staked on a firm bed a 1/2 in line does not move much under a
  downward push; in a real trial the left hand may steady it). Line, stakes, pot, shrub and block are rigid. The
  dripper weighs 20 g in the solver (real 2 g) for contact stability, and both it and the punch handle have square
  colliders (22 mm) inside their drawn round shapes, so they cannot roll in the fingers. The scorer reads holes and seating from the
  rule; the real trial checks the same outcomes on the tube (above).

## Real-world build and safety

Hardware, about $55 in all: 100 ft of 1/2 in drip tubing (about $20, many trials), two tubing stakes ($1), figure-8
end closures ($2), a bag of barbed drippers (about $10 for 25), a Netafim 3 mm punch ($6), a plywood offcut with
landscape fabric ($10), a pine 2x4 offcut, an artificial boxwood in a 1 gal pot ($15), a white paint pen.

- **The punch point is sharp** (3 mm steel). It lives point down in its block hole; the gripper pads only touch the
  plastic handle. The operator watches with the e-stop in hand and stops the trial if the point is pushed toward a
  gripper pad, cable or camera.
- **No water.** The line is dry and capped at both ends; nothing wet or greasy touches the rig.
- **Reset:** pull the dripper out, cut out the punched stretch or slide the tube along to an unpunched stretch
  (about 3 cm per trial), draw a new mark, stand the punch and a new dripper in their holes.

## Results

| Run | Score | What happened |
|---|---|---|
| Scripted demo, seed 0 | 1.0 | Round hole 0.1 mm from the mark (5.1 deg off square, 24.9 N); dripper seated at 16.1 N; punch back in its block; 903 steps |
| Scripted demo, seed 4 | 1.0 | Round hole 0.3 mm from the mark (6.7 deg, 26.3 N); dripper seated at 15.8 N; 958 steps |
| Scripted demo, seed 10 | 1.0 | Round hole 0.1 mm from the mark (8.7 deg, 24.6 N); dripper seated at 16.0 N; 831 steps |
| `DEMO_CHEAT=nopunch` (dripper pushed onto the mark without punching) | 0.0 | No hole: the dripper never seats |
| `DEMO_CHEAT=light` (punch pushed with at most about a third of the arm's command) | 0.0 | The wall does not give; no hole, no seat |
| `DEMO_CHEAT=twice` (second hole 15 mm along, then the dripper in the first) | 0.2 | Seated, but the line leaks: capped at 0.2 |
| `DEMO_CHEAT=tilt` (punch held leaning 18 deg) | 0.0 | The leaning punch twisted out of the fingers on the tube; no hole. The torn-hole rule itself is covered by the unit tests |
| Dream-1, seed 0 | 0.0 | Ran the full 2,000 steps (the policy service asked to reconnect 12 times); reached toward the tool block but never grasped the punch; no hole |


## Third-party assets and licences

- The YAM arm models, the station scene, its cameras and the `yam_arena` embodiment come from the Robocurve
  simulation walkthrough's Isaac Lab-Arena environment (yam-isaac-lab-arena; the YAM description from i2rt), used
  under the licences that environment ships with (i2rt's YAM description: MIT; Isaac Lab and Isaac Lab-Arena:
  BSD-3-Clause; Isaac Sim: NVIDIA Isaac Sim additional software and materials licence). No other third-party meshes,
  scans or textures are used: everything else is built from primitives in `dripkit.py`, and `block_drawing.svg` is
  ours (same licence as this repository).
