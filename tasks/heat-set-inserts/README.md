# Set four M3 heat-set inserts flush into a printed robot bracket with a soldering iron and a flat block

A blue silicone soldering mat lies on the table, just right of centre. On it lies an orange PETG bracket (80 x 40 x
12 mm), printed with four 4.0 mm holes. A brass M3 heat-set insert stands loosely in each hole, about 4.5 mm proud of
the top face. Beside the bracket a small aluminium block (19 x 19 x 50 mm) stands on its end. A soldering station's
iron stand is clamped to the table to the right; the iron rests in its holder, switched on at 245 C, fitted with an
M3 heat-set insert tip.

**Instruction:** `Set the four inserts flush and square into the bracket with the soldering iron and the flat block, then put the iron back in its stand.`

## Changes since the last review

- TK-GM9TQL: "Typically, to seat a heat-set insert flat, a flat piece of metal is laid on top to seat the last little
  bit while it's still hot." Changed: the method now follows this. A flat aluminium block (19 x 19 x 50 mm, 49 g) is
  in the scene; the iron sinks each insert most of the way, and within 3 s of the iron leaving (while the plastic is
  still soft) the block, held in the left hand, presses the insert flush and square. Full credit per insert now needs
  flush and pressed flat with the block; flush by the iron alone, or the block too late, scores 0.10. The bracket
  moved 10 cm toward the centre so that both arms reach it. Demo, cheats, tests, Dream-1 run and media redone.

## Why this task

- **It is how printed robot parts get their threads.** Gripper fingers, joint housings, camera and sensor brackets
  of open-source arms and of most lab prototypes are 3D-printed, and their screws go into brass heat-set inserts.
  The standard way to set them, in labs and in the printer makers' own guides (CNC Kitchen, Prusa, Adafruit), is
  this one: put the insert on its hole, press it in with a soldering iron fitted with an insert tip, stop just short
  of flush, lift the iron and at once press a flat piece of metal on it, which brings it flush and square while the
  plastic is still soft.
  Robots that build robots have to do it many times per part. Category: **Impact on AI timelines** (robot
  building).
- **The difficulty is force and depth control with a hot tool.** The insert moves only while the plastic around
  it is soft: the robot must hold the tip on it with light, steady pressure for a few seconds before anything
  happens, then let it sink at the rate the plastic allows, keep the iron square to the hole, and stop at flush.
  Pushing hard drives it in faster than the plastic flows (it ends loose), leaning sets it crooked (a screw then
  runs in at an angle), going on past flush presses the hot shoulder into the face (a melt mark) and finally
  bottoms the insert. The pin must find the insert's 2.5 mm bore. Then the two hands work in turn on one spot against
  the clock: the right lifts the iron off and clears it, the left brings the flat block down square on the insert
  within 3 s, before the plastic sets, and presses the last millimetre in. The tool is hot: only its grip may touch the
  robot, it must not be dropped or left lying, and it goes back into a sloping holder tube at the end.
- **Nothing in the scene does this work for the robot.** The inserts sit loose on their holes, as a person sets
  them before pressing; the bracket lies flat on the mat; the iron is the ordinary station iron.

### Distinct from

- **Board tasks.** Sweating a copper pipe joint (heat until solder flows: nothing is pressed to a depth); shrinking
  heat-shrink tubing with a heat gun (heat moved along a sleeve); seating a humanoid leg's side plate flush in its
  pocket (flush seating, but cold, rigid and no tool); putting a circuit board down where a hot iron lies (a safety
  choice; the iron is not used). Pressing a part into a snap-fit until it clicks is a covered ability; here nothing
  clicks: the robot must wait for the plastic, regulate a few newtons over seconds and stop at a depth by itself.
- **External benchmarks.** No BEHAVIOR-1K, LIBERO, RoboCasa, DataCenterBench or RoboDojo task uses a soldering iron
  or a heat-set insert. The nearest are peg-in-hole insertion tasks (e.g. Factory/IndustReal peg insertion, FurnitureBench
  assembly): a peg slides in under its own clearance as soon as it is aligned. Here alignment is
  only the start; the insert sinks only while hot plastic yields, and the stop depth is the score.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, `score()` / `success()`, and `controller()` (the demo `yam_scripted` plays) |
| `insertkit.py` | The bodies (bench, bracket, inserts, stand, iron, flat block), the 240 Hz plastic rule and latches, and the hooks into yam_arena |
| `insertdemo.py` | The closed-loop scripted demo |
| `test_heat_set_inserts.py` | Unit tests of the plastic rule and every scoring outcome (no Isaac Sim) |
| `bracket.stl`, `bracket_drawing.svg` | The bracket to print, and its dimensioned drawing |
| `make_bracket.py` | Writes both (the STL needs `pip install manifold3d numpy`) |
| `task.json` | Submission details |

`insertkit.py` hooks yam_arena without modifying it (as other task kits in this repository do): it patches
`yam_arena.scene.task_object` and `object_states`, the embodiment's `_ensure_sim` and `reset`, and
`yam_arena.scripted.YamScriptedPolicy` (when a task file defines `controller()`, `yam_scripted` asks it for each
action and passes it the observation), and wraps `scene.write_data_to_sim`, called once per physics step.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left, z up). Arms start in
the ready pose. The layout is the same in every trial; mark it with tape.

| Object | Description |
|---|---|
| Silicone soldering mat | Heat-resistant silicone repair mat (e.g. Kaisi S-160, 45 x 30 cm, cut or folded to 20 x 16 cm; about $12), 2 mm, blue, centred at x 27, y -4.5 |
| Bracket | PETG, 80 x 40 x 12 mm (`bracket.stl`, `bracket_drawing.svg`): four 4.0 mm holes 7.0 mm deep in its top face, 56 mm apart along its length and 20 mm across. Lies flat on the mat, long side along y, centred at x 28, y -8. Printed flat, 0.2 mm layers, 4 perimeters, 30% infill: about 40 g, 25 min, $0.40 of filament. A fresh one each trial |
| Inserts (4) | Brass M3 heat-set inserts, CNC Kitchen "M3 standard" (M3 x 5.7 mm long, 4.6 mm across the knurls, 3.9 mm lead-in; $0.08 each in packs of 100). Each stands in its hole, dropped in by hand: its lead-in rests in the hole and its top is about 4.5 mm above the face (4.2 to 4.8 mm). Their tops are at (27, -10.8), (29, -10.8), (27, -5.2), (29, -5.2) |
| Flat block | A 50 mm offcut of 6061 aluminium square bar, 19 x 19 mm (3/4 in; any metal supplier, about $5 for 30 cm), both ends faced flat on a lathe or a sheet of sandpaper on glass: 49 g. Stands on one end on the mat at x 22, y 0.5 |
| Soldering station | Hakko FX-888D (about $120), set to 245 C (PETG prints at 230 to 250 C; the insert makers' guides set the iron 10 to 20 C above the print temperature). Its iron (FX-8801): about 19 cm long without the cord, 45 g; grip about 20 mm across from 8 to 17.5 cm behind the tip. Fitted with an M3 heat-set insert tip for Hakko T18 handles (Adafruit 4239: 2.1 mm pin into the insert, 6.5 mm flat shoulder, about $10; any M3 insert tip for T18 handles works). Switched on and hot before the trial starts |
| Iron stand | The FX-888D's holder (Hakko FH-800 or similar): a weighted base, about 11 x 9 cm, with a holder tube about 30 mm across inside and 9 cm long sloping down at about 50 degrees, pointing away from the arms; its closed end at x 42, y -27, 3 cm above the table. Clamped to the table (a small C-clamp over the base's foot, or VHB tape). The iron rests in it, tip down, grip up toward the arms. The station unit itself stands off the table's working area, the cord led along the table edge and taped down |
| M3 x 12 screw, try square, steel rule, 0.5 mm feeler gauge, calipers | For scoring only, off the table during the trial |

Nothing else is on the table.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the four inserts flush and square into the bracket with the soldering iron and the flat block, then put the iron back in its stand." \
  --scorer task_score --max-steps 2400 --seed 0 --store-frames --log-dir ~/yam-eval/heat-set-inserts
```

The scripted demo: the same with `--policy yam_scripted`. Contrasts: `DEMO_CHEAT=hard` (presses hard and fast),
`DEMO_CHEAT=tilt` (presses with the iron leaning 8 degrees), `DEMO_CHEAT=keep` (never puts the iron back),
`DEMO_CHEAT=noflat` (the iron alone, to flush; no block), `DEMO_CHEAT=late` (the block 3.5 s later than the demo, after
the plastic has set).
`HSI_DEBUG=1` prints the iron and the inserts every 0.1 s. Unit tests: `python -m pytest test_heat_set_inserts.py`.

Seeds (`HSI_SEED`, default 0): each seed sets each insert's start height (4.5 +- 0.3 mm proud) and the bracket's softness (the sinking rate,
+-25%, as prints differ). The layout is the same in every seed. Set the seed for a trial with `HSI_SEED=<n>` (the scripted demo needs no other input). A trial is at most 2,400 control steps (2 minutes simulated); on one shared GPU it takes about 8 to 10 minutes of wall time.

## Scoring

The plastic rule and the latches run every physics step (240 Hz); the score every control step. The score at the end
of the episode counts (`SCORE_OVER = "end"`).

| Condition | Score |
|---|---|
| At any time: a hot part of the iron (tip to the end of the barrel, 75 mm) touched the robot; or the iron was out of its stand and in no grip for 2 s (dropped, laid down, left balanced on the bracket) | 0 for the trial |
| At the end: the iron is not resting in its stand, or a gripper still holds it | 0 for the trial |
| Otherwise, per insert: top within 0.5 mm of the bracket's face (either way), and pressed flat with the block while still soft | 0.25 |
| Per insert: seated but not that (top 0.5 to 2.5 mm proud, or 0.5 to 1 mm below the face, or flush with the lip the iron leaves, as the block came too late or not at all) | 0.10 |
| Per insert: top more than 2.5 mm proud (barely or not sunk), or crooked (iron or block leaning), forced (iron or block), or with a melt mark beside it | 0 |
| All four flush, the iron resting in its stand and released: success once this has held 1 s | 1.0 |

Every outcome is covered: each insert ends in one of the three bands; the iron ends in the stand released, or not.
A crooked or forced insert cannot be repaired later (latched), and pressing a forced insert further does nothing.
Pushing an insert with anything but the iron (a finger, the block) before the iron has heated it does not sink it:
only the iron's heat softens the plastic. The block cannot do the iron's work: it moves only an insert within 1.5 mm
of flush, and only within 3 s of the iron leaving it. Leaving the iron in the stand and doing nothing scores 0 (no
insert sunk).

## Fidelity

Approximate. The simulator cannot melt plastic, so a rule (in `insertkit.py`, unit-tested) stands in for it, and the
robot has to do the real work to trigger it: the same grip on the iron, the pin centred on the insert (within 1.5 mm: its tapered point finds the bore),
a push of a few newtons held along the hole, for the real time.

- **Heating.** An insert sinks only after 2.5 s of the iron pressing on it (0.5 N or more, pin in the bore); off the
  iron for 3 s the plastic round it has set again.
- **Sinking.** Once hot it sinks at 0.3 mm/s per newton above 1.5 N (at most 3 mm/s), so about 3 s to flush at 6 N
  (the bracket's softness varies by +-25% with the seed, as prints differ). Below 1.5 N it does not move.
- **Too hard, crooked, too far.** Pushed harder than 15 N for 0.25 s while sinking: forced (latched, scores 0).
  Sinking while the iron leans more than 5 degrees: crooked (latched; the insert is drawn leaning). It stops at the
  hole's bottom, 1 mm below the face. Any part of the iron pressing on the bracket for 0.5 s in all: melt mark,
  charged to the insert nearest the tip.
- **The flat block.** The iron's 6.5 mm shoulder presses only the insert, so the plastic it displaces stays as a
  raised lip round it and its top is not registered to the face (in the rule: a *lip*). Pressed on the insert within
  3 s of the iron leaving, the block sinks the last 1.5 mm at the same rate and stops at flush, where it lands on the
  face; pressed level (within 5 degrees) with 1.5 N or more it flattens the lip. Leaning more than 5 degrees while it
  sinks the insert: crooked; over 15 N for 0.25 s: forced. The block is cold: touching the robot or the bracket with
  it costs nothing.
- **Where the numbers come from.** We found no published force for setting an M3 insert. The insert makers' guides
  (CNC Kitchen, Prusa, Adafruit, LulzBot) say: light, steady pressure, let the heat do the work, a few seconds per
  insert, never force it (forced, the plastic folds instead of flowing round the knurls and the insert ends loose),
  and keep the iron square. The values above are estimates within that guidance. To measure them on the real
  setup: stand a bracket on a kitchen scale (5 kg, 1 g) and tare it; press inserts in by hand with the iron at
  245 C; film the scale and the insert together. The scale gives the push (g x 0.0098 N) and the video the sinking
  time; the lowest push that sinks an insert within 10 s gives the 1.5 N threshold, the time to flush at a steady
  push the rate, and the push above which inserts come out loose (they turn with the screw) the 15 N limit. The 3 s
  window is measured the same way: lift the iron off an insert left 1 mm proud, wait a timed delay, press the block
  on it with about 5 N; the longest delay after which it still goes flush is the window.
- **Simplified.** The bracket is fixed in place (on silicone a 60 g bracket pressed down does not slide; in a real
  trial the left hand may steady it). The inserts are moved only by the rule (they cannot be knocked off their
  holes). The iron's grip is a 19 mm square collider inside the drawn round grip, so it cannot roll in the
  fingers; its cord is not simulated. The iron is always hot. The scorer reads the inserts' heights and latches
  from the rule; the real trial measures the same outcomes with a rule and gauge (below).

## Real-world build and safety

Hardware, about $165 in all: Hakko FX-888D station with its holder (about $120), M3 heat-set tip (about $10),
silicone mat ($12), 100 CNC Kitchen M3 standard inserts (about $8), 1 kg PETG ($20, about 25 brackets), a C-clamp,
an M3 x 12 screw, a feeler gauge set (0.1 and 0.5 mm blades), a small try square and the aluminium block (about $5). The bracket prints from `bracket.stl`.

The iron is hot (245 C at the tip). The trial is set up so that only its grip can reach the robot, and an operator
watches with the e-stop in hand.

- **The stand is clamped** to the table, so the iron cannot be knocked over with it; the station unit and its cord
  are outside the arms' reach, the cord taped along the table edge.
- **Only the grip is held.** The grip (8 to 17.5 cm behind the tip) is rubber, and stays near room temperature: its
  front is 8 cm from the hot tip. The fingers close on it; nothing else of the robot needs to come near the barrel.
  The real trial stops (e-stop) and scores 0 if the tip or barrel comes within 2 cm of any part of the robot, its
  cables or the camera stand; the simulation scores 0 if they touch the robot.
- **Nothing flammable** is on the table; everything under the work is silicone or PETG. Heated PETG gives off little
  fume; a small fume extractor or an open window is enough.
- **Reset** with heat-resistant gloves: put the iron back in its stand if needed, let the bracket cool 30 s, remove
  it, lay a fresh bracket, drop four new inserts in its holes.

## Real-trial scoring

Each check is made after the trial, on the bracket (cooled 30 s) and from the trial video.

| Sim condition | Real-trial check |
|---|---|
| Insert top within 0.5 mm of the face, pressed flat with the block | A steel rule (straightedge) laid across the insert, along and then across the bracket: no 0.5 mm feeler gauge fits under the rule beside the insert (proud by less than 0.5 mm), and the insert's top is not lower than 0.5 mm below the face (calipers' depth rod); and the rule lies flat on the face on both sides, with no 0.1 mm feeler gauge fitting under it next to the insert (no raised lip). The trial video shows the block pressed on the insert within 3 s of the iron leaving it |
| Lip: flush by the iron alone, or the block too late | The 0.1 mm feeler gauge fits under the rule beside the insert (the rule rocks on a raised ring), or the video shows no block, or the block later than 3 s |
| Seated: top 0.5 to 2.5 mm proud, or 0.5 to 1 mm down | Calipers' depth rod from the face to the insert's top |
| Crooked: sunk with the iron or the block leaning over 5 degrees | An M3 x 12 screw threaded in by hand to its head: against a try square standing on the face, the screw's head is more than 1 mm off square (5 degrees over 12 mm) |
| Forced: pushed over 15 N while sinking (iron or block) | The insert turns or comes out when the M3 screw is tightened by hand (finger-tight on a small screwdriver) |
| Melt mark | A glossy ring or dent in the face around the insert, in a close photo |
| Iron's hot part touched the robot | Video (operator stops the trial within 2 cm) |
| Dropped: out of the stand, in no grip, 2 s | Video: the iron lies anywhere but in the holder, or falls |
| Iron in its stand, released, at the end | Video / photo of the end |

## Results

| Run | Score |
|---|---|
| Scripted demo (`yam_scripted`), `HSI_SEED` 0, 4, 10 | 1.0, 1.0, 1.0 (about 88 s, 1,760 control steps each; in the logged runs inserts ended 0.07 to 0.24 mm proud; pressed flat with the block 1.3 to 1.6 s after the iron left) |
| `DEMO_CHEAT=noflat`: the iron alone, each insert to flush (about 0.15 mm proud); no block | 0.4 (four flush inserts with their lip: 0.10 each) |
| `DEMO_CHEAT=late`: the demo with the block 3.5 s later, after the plastic has set | 0.4 (the block does not move the inserts, left 0.8 mm proud with their lip: 0.10 each) |
| `DEMO_CHEAT=hard`: presses at 10 mm/s and feeds at 12 mm/s | 0.0 (the pin, pushed hard and fast, skids off every insert's bore: none sinks; a melt mark by insert 2) |
| `DEMO_CHEAT=tilt`: the same demo with the iron leaning 8 degrees | 0.0 (every insert crooked; pressing it flat with the block afterwards does not repair it) |
| `DEMO_CHEAT=keep`: four inserts set and pressed flat, the iron never put back | 0.0 |
| Dream-1, seed 0 (2,400 steps) | 0.0: both arms moved near their bases for the whole run; neither touched the iron or the block, no insert was pressed |

The demo (`insertdemo.py`) sees only the joint positions (the camera images are unused). It knows the fixed layout,
grips the iron along its axis in the holder, turns it upright, and per insert: lowers the pin onto the insert in
0.1 s steps, correcting its aim sideways by the tip position it measures through forward kinematics (under load
the arm's joints lag by a millimetre or two); commands the shoulder 3 mm into the insert for 3 s while it heats
(about 2 to 4 N); then lowers the command at 1 mm/s and stops when the measured shoulder is 1.2 mm above the face
(it reads about 0.4 mm high under load), leaving the insert about 0.8 mm proud. Meanwhile the left arm has taken the
block off the mat; before each insert it holds the block 10 mm over it, measures where it really is by forward
kinematics (the arm sags 1 to 4 mm) and keeps the correction, then waits 15 cm away. When the iron stops, the right
arm lifts it 12 mm and swings it 10 cm away, and the left brings the block down on the insert 1.3 to 1.6 s after the
iron left, commanding it 24 mm into the face for 1.3 s (the arm is soft: about 5 to 7 N while the insert sinks;
the first touch gives a spike lasting under 0.1 s, shorter than the 0.25 s that counts as forced): the insert
sinks the last 0.8 mm and stops flush. At the end the block goes back on the mat, and the iron back down the
tube's axis; let go, it settles leaning across the tube as at the start.

The run limit is 2,400 steps: the demo needs about 1,760.

## Third-party assets and licences

- The YAM arm models, the station scene, its cameras and the `yam_arena` embodiment come from the Robocurve
  simulation walkthrough's Isaac Lab-Arena environment (yam-isaac-lab-arena; the YAM description from i2rt), used
  under the licences that environment ships with (i2rt's YAM description: MIT; Isaac Lab and Isaac Lab-Arena:
  BSD-3-Clause; Isaac Sim: NVIDIA Isaac Sim additional software and materials licence). No other third-party meshes,
  scans or textures are used: the bracket, inserts, iron, stand, block and mat are built from primitives in
  `insertkit.py`, and `bracket.stl` is generated by `make_bracket.py` (ours, same licence as this repository).
