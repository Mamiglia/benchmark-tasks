# Move four sealed cups of unknown weight without crushing or dropping them

Four identical sealed cups stand on the table, two in front of each arm. Each holds a different, hidden mass of damp
sand, from 20 g to 400 g. Between them is a low tray. The robot must put all four cups on the tray, upright.

**Instruction:** `Put all four cups on the tray upright.`

## Why this task

- **The skill is grip-force regulation against an unknown load.** The cups look the same, and the fill cannot be
  seen. A thin wall gives under the fingers and buckles if squeezed too hard; a light grip lets a heavy cup slide out.
  A full cup's contents brace its wall, so it takes more squeeze to buckle than a near-empty one, but it also needs
  more grip to lift. With every seed, no single grip force works for all four cups (see Seeds): the robot has to feel
  each cup's weight as it lifts it and set its grip from that.
- **Real work.** Food service (lidded takeaway cups, portion pots, sealed soup or drinks), labs (sealed sample cups,
  specimen pots) and pick-and-pack of light packaging all handle thin-walled containers whose contents are hidden.
  Crushing one spills or ruins it; dropping one does the same.
- **Not visible from an image.** Nothing in a camera image tells the cups apart. The difficulty is in the
  manipulation: closing gently, lifting, sensing that the cup is not coming up, and tightening without overshooting.

### Distinct from

| Task | Its skill | Here |
|---|---|---|
| #1645 Calibrate gripper force with a spring reference | A procedure: zero, load a known spring to a marked compression, span, verify | No reference and no procedure: an unknown load per object, grip set from what the lift feels like; too much force breaks the object |
| #803 Weigh and reject the heavy block (not accepted) | Read a scale, then sort | No scale and no sorting: every cup goes to the same place; the weight matters only for how hard to grip |
| #1735 Take out the prop that carries nothing | Feel which of four props is loaded, without disturbing a slab | Here every object is lifted; the hidden quantity sets the grip force, and both too little and too much fail |
| #1430 Pot placement with a fragile egg (not accepted) | Clear an occupied trivet; contact force between objects | Here the fragile object is the one gripped, and the gripper's own squeeze is what can break it |

It is also not ordinary pick-and-place (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR): those objects are rigid and any
firm grip works.

## Design

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left).

| Object | Size | Mass | Notes |
|---|---|---|---|
| 4 cups | 7 cm across (octagonal, 8 wall panels), 9 cm tall, straight sides | 26 g empty + fill | White thin-walled plastic with a red seal 6 mm below the rim. Start upright at (34, 21), (22, 21), (34, -21), (22, -21) |
| Fill | Damp sand inside, not visible | 20 to 400 g, per seed | Sets the weight and how much squeeze the wall takes |
| Tray | 21 x 28 cm outside, 5 mm floor, 1 cm rims 2 cm tall | fixed | Dark grey, centred at (30, 0), long side along y. Inside 19 x 26 cm |

**Cup wall.** Each of the 8 panels sits on a radial slide with a spring (200 N/m). A flat finger pad bears on one
panel and on the edges of its two neighbours, so the wall under a pad is about 1,000 N/m stiff: a 2 N grip presses it
in 2 mm, a 9 N grip 9 mm, which shows in the video. The kit computes the force each pad applies to the wall from the
panel deflections, every physics step (240 Hz).

**Buckling.** If the force on the wall under one pad stays above the cup's buckling force for 3 physics steps
(12.5 ms), the wall buckles there: the three panels under that pad take a new rest position 16 mm in, so the wall
stays dented and the fingers close through it. The buckling force is 1.5 N + 34 N/kg x fill: 2.2 N at 20 g, 15.1 N at
400 g. (The spring's hard stop is 18 mm in, 18 N, so every cup buckles before it.)

**Holding.** The materials are set to friction 0.5; the effective pad-on-wall friction in the simulation is about 0.33
(from the grips at which cups came off the table in the runs below). Lifting a cup straight up then needs about
m g / (2 x 0.33) = 15 N/kg x mass per pad: 0.7 N for the lightest cup, 6.3 N for the heaviest.

## Seeds

The fills come from the environment variable `CUPS_SEED` (default 0). Each seed draws one cup from each band, 20-60 g,
100-180 g, 200-280 g, 320-400 g (rounded to 5 g), and shuffles them over the four places. So every seed has a cup that
buckles at 3.5 N or less and one that needs 5.1 N or more per pad to lift: no fixed grip moves all four intact
(`test_no_single_grip_force_lifts_every_cup_of_any_seed` checks 200 seeds).

| Seed | far left (34, 21) | near left (22, 21) | far right (34, -21) | near right (22, -21) |
|---|---|---|---|---|
| 0 (default) | 120 g | 205 g | 45 g | 320 g |
| 1 | 395 g | 210 g | 40 g | 175 g |
| 2 | 30 g | 265 g | 325 g | 125 g |

`python -c "import task; print(task.fills_for(7))"` prints any seed's fills (kg).

## Scoring

Evaluated after every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). Each cup
earns 0.25 if all of the following hold; 1.0 (success, held 1 s) when all four do.

| Per cup, at any time during the trial (any one fails the cup for good) | Real-world check |
|---|---|
| The wall buckled (wall force under a pad above the buckling force for 12.5 ms) | A crease or dent in the wall (photo) |
| A robot link touched the base or the seal (over 0.3 N for 12.5 ms) | Video |
| The cup's base was more than 1 cm above the table or tray under it without a wall grip (dropped, tossed, scooped from below, carried on the fingers or the seal, stacked) | Video against a 1 cm scale |
| It slid more than 1 cm along the gripper while held (cup lags the fingers) | Video: 1 cm bands on the cup wall against the finger tips |
| It tilted more than 30 degrees (knocked over) | Video |
| One gripper held it together with another cup, both off their supports | Video |
| It fell off the table | Video |

A **wall grip** is two opposite sides of the wall pressed with at least 0.3 N each, by a gripper whose tool point is
within 5 cm of the cup's axis and between 1 cm above its base and 1 cm below its rim.

| Per cup, at the end | Real-world check |
|---|---|
| It was lifted at least 2 cm in a wall grip at some point | Video |
| The last wall grip ended with the cup on the tray (pushing a cup onto the tray, or setting it on the table and sliding it in later, does not count) | Video |
| Its base is inside the tray's rim (3 mm overhang allowed) and resting on the tray floor | Photo |
| It is within 10 degrees of upright | Photo |
| No gripper holds it | Photo |

Unintended routes and what they score:
- **Squeezing hard** so no cup slips: the wall buckles (every cup buckles under the gripper's 50 N).
- **Pushing or dragging cups** onto the tray: never lifted in a wall grip, so 0 for that cup.
- **Scooping** from under the base, **pinching** at the seal or base, or **pressing down on the seal**: the
  touch rule, and any lift without a wall grip is the off-support rule.
- **Two cups in one hand:** both fail.
- **Recovering** a dropped or knocked-over cup: the failure is latched.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder. The layout is fixed: always pass
`-E object_jitter=0`.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put all four cups on the tray upright." --scorer task_score \
  --max-steps 2000 --store-frames --log-dir ~/yam-eval/hidden-fill-cups-dream
```

Prefix `CUPS_SEED=1` (or any integer) for other fills. `task_score` prints the trial's score. `CUPS_DEBUG=1` prints
each cup's state five times a second (deflections, peak wall force against its buckling force, latches) and the demo's
lift tests.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put all four cups on the tray upright." --scorer task_score \
  --max-steps 2000 --store-frames --log-dir ~/yam-eval/hidden-fill-cups-demo
```

`yam_scripted` runs `grip_demo.Controller` closed loop (`cupkit` hooks it in when the task module defines
`demo_controller`). **It uses only proprioception:** the joint positions in each observation (including each
gripper's measured opening) and its own commands, plus the arm's drive gains. It knows the start layout, as every
`yam_scripted` demo does. It does not read the fills, any simulator state, or the cameras (the wrist cameras see only
their own fingers during a grasp, and the top camera looks down onto the gripper). See the docstring of `grip_demo.py`.
In short, each arm closes on a cup at 0.8 N and lifts its hand 4 to 7 mm, reading the load on its joints. While the
pads slide up a cup that stays on the table, the load is the sliding friction and grows with the grip; once the cup
comes up, the load is its weight and stops growing. So it sets the hand back down and grips 1.4 times harder until the
load rises by less than 15 %, then carries the cup at 1.2 times that grip and sets it on the tray.

Loophole checks: `DEMO_CHEAT=squeeze` (grip fully closed), `DEMO_CHEAT=push` (shove the cups onto the tray with closed
fingers) and `DEMO_CHEAT=seal` (press down on the seals and drag), with the same command.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, seeds, `score()` / `success()`, `demo_controller` |
| `cupkit.py` | The cup (articulated wall, buckling), the scoring monitor, and the hooks into yam_arena |
| `grip_demo.py` | The closed-loop scripted demo |
| `test_hidden_fill_cups.py` | Tests: buckling, every scoring rule on handcrafted trajectories, seeds, demo reach |
| `task.json` | Submission details |

`cupkit.py` hooks yam_arena without modifying it: `yam_arena.scene.task_object` and `object_states`, the embodiment's
`_ensure_sim` and `reset`, `yam_arena.scripted.YamScriptedPolicy.reset` / `act`, and the environment's
`scene.write_data_to_sim` (called once per physics substep).

No third-party assets: every object is built from primitives in `cupkit.py` and `task.py`. The robot and the station
come with yam_arena.

## Tests

```bash
python -m pytest -q      # in the yam_arena environment; no GPU needed
```

## Checked in simulation

Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1) on a shared RTX workstation, with the commands above (`CUPS_DEBUG=1`,
`--max-steps 2000`). Frames of every run listed were checked (not blank).

| Run | `task_score` | What the log shows |
|---|---|---|
| Scripted demo, seed 0 | **1.0** | Lift-test grips at which the load stopped rising: 1.1 N (45 g), 1.6 N (120 g), 3.1 N (205 g), 6.0 N (320 g); highest wall force 5.1 N against 12.4 N for the 320 g cup. All four on the tray, success at 57 s (1,148 steps) |
| Scripted demo, seed 1 | **1.0** | 40 g, 175 g, 210 g and 395 g (carried at a 10.1 N grip; highest wall force 9.1 N against 14.9 N). Success at 75 s |
| Scripted demo, seed 2 | **1.0** | 30 g, 125 g, 265 g, 325 g (highest wall force 5.1 N against 12.6 N). Success at 69 s |
| `DEMO_CHEAT=squeeze` (fingers fully closed, 50 N) | **0** | Every cup's wall buckled on the first squeeze |
| `DEMO_CHEAT=push` | **0** | No cup was lifted in a wall grip; the shoved cups tipped over the rim |
| `DEMO_CHEAT=seal` | **0** | Fingertips on every seal: touch rule |
| Dream-1, seed 0 | **0** | 2,000 steps. Both arms reached down beside the cups and toward the tray but never closed on a cup; nothing moved |

A scoop from under the base is not reachable with this gripper on a cup standing flat (the base sits on the table,
3 mm inside the wall); the rule that catches it (off its support without a wall grip) is covered by the unit tests,
as are a set-down-then-push, a drop, two cups in one hand and a slip.

## Runtime and seeds

A scripted trial takes 6.5 to 8 minutes of wall time (Isaac Sim start, reset, 1,150 to 1,500 control steps) on a
shared RTX workstation. A 2,000-step policy run takes longer, depending on the policy's latency. Seeds: see above;
the default (seed 0) is reproducible with `-E object_jitter=0`.

## Real-world build

- **Cups:** 8 identical thin-walled PP or PET cups with straight sides, about 7 cm across and 9 cm tall (specimen
  pots or deli cups), sealed with a red foil lid recessed 6 mm. Draw 1 cm bands on the wall for the slip check. Fill
  with damp sand to the seed's masses (kitchen scale), seal, and keep a spare set: a buckled cup is used up.
- **Calibrate the buckling law** per batch with a force gauge and two flat 2 cm pads: squeeze an empty cup and a 400 g
  one at 3 cm above the base until the wall creases. Choose a cup wall (or fill height) that gives about 1.5 N empty
  and 15 N at 400 g; the scoring only needs the crease to show.
- **Tray:** any low tray, 19 x 26 cm inside with 2 cm rims, taped to the table.
- **Safety:** no hazards beyond the arms themselves; damp sand is the only spill.

## Simplifications

- **How the wall works.** The side wall is not a shell: it is 8 rigid panels, each on a radial slide with a spring
  (PhysX joint drives). The fingers really press the panels in, the reaction they feel is the springs' force, and
  friction between pad and panel is what carries the cup. Nothing about holding or slipping is scripted.
- **Why the buckling latch is a fair model.** A thin cup wall squeezed between two pads behaves like a spring up to a
  limit load, then snaps through (buckles) and loses most of its stiffness, and the crease stays: an empty plastic
  cup does not spring back. The kit reproduces exactly that load path. Every physics step it computes the force each
  pad applies to the wall from the measured panel deflections; once that force has stayed above the cup's buckling
  load for 12.5 ms, the three panels under the pad get a new spring rest position 16 mm in. The wall's resistance
  then drops, the fingers close through it in the video, and the dent remains after release. What is simplified is
  the snap itself (an instant change of rest position instead of a shell's post-buckling curve) and the geometry
  (flat panels, not a thin curved shell). PhysX rigid bodies cannot buckle, and Isaac Lab's FEM deformables are
  elastic only (no snap-through with permanent set), so a buckling shell is not available on this stack. The
  threshold is a measurable property of a real cup (force gauge, README "Real-world build"), not a score rule.
- **The buckling force is a calibrated linear law in the fill mass** (contents bracing the wall). Real cups get stiffer
  as well as stronger when full; here the stiffness is the same for every fill, so squeezing alone does not reveal the
  fill (the conservative case for the policy).
- **The fill is a rigid mass** fixed in the cup (no sloshing). Pad friction is a single coefficient (set 0.5; about 0.33 effective).
- **The tray is fixed** to the table.
