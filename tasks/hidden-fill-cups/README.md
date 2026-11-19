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
- **Real work: fast food and counter workers.** About 3.8 million people in the US hold this job (BLS Occupational
  Outlook Handbook, 2024), making it one of the largest occupations. Handing over and bagging sealed, lidded cups and
  pots (drinks, soups, sauces, sides) is a core, many-times-an-hour part of it. The cups look alike whatever is in
  them: a near-empty one dents if squeezed as hard as a full one, and a full one slips if held as lightly as an empty
  one. The same handling recurs in pick-and-pack of light packaging and in labs (sealed sample cups), but the task is
  set in food service.
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

### Why no single grip force lifts all four cups

A two-finger grip holds a cup up by friction alone, so each pad must press with at least m g / (2 mu), where m is
the cup's mass and mu the pad-on-wall friction coefficient. On cups built to the spec (measurements under "Measured on
built cups" in "Real-world build"), mu is 0.35. The heaviest cup of every seed weighs at least 346 g, so it needs
3.39 N / (2 x 0.35) = 4.8 N per pad to hang still, and 7.3 N with the 1.5 margin a carried cup needs against the
arm's accelerations. The 20 g and 60 g cups creased at 1.4 to 1.7 N. Every seed has one cup of each kind, so a grip
that carries the heavy cup creases the light one, and a grip gentle enough for the light cup drops the heavy one.
Tried directly at 2.0 N per pad: the 320 g cup slipped out under its own weight (3 of 3), and the 60 g cup creased
before the squeeze reached 2.0 N (3 of 3). The simulation uses the measured values (see Design);
`test_no_single_grip_force_lifts_every_cup_of_any_seed` checks the gap over 200 seeds. The robot has to sense each
cup's weight as it lifts and set the grip from it.

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
stays dented and the fingers close through it. The buckling force is interpolated linearly through the means measured
on built cups (Real-world build): 1.5 N at 20 g, 1.6 N at 60 g, 6.5 N at 150 g, 12.5 N at 320 g, 15.0 N at 400 g.
(The spring's hard stop is 18 mm in, 18 N, so every cup buckles before it.)

**Holding.** The materials are set to friction 0.53, so the effective pad-on-wall friction in the simulation is about
the measured 0.35 (scaled from the earlier setting: 0.5 gave about 0.33, from the grips at which cups came off).
Lifting a cup straight up then needs about m g / (2 x 0.35) = 14 N/kg x mass per pad: 0.6 N for the lightest cup,
6.0 N for the heaviest.

## Seeds

The fills come from the environment variable `CUPS_SEED` (default 0). Each seed draws one cup from each band, 20-60 g,
100-180 g, 200-280 g, 320-400 g (rounded to 5 g), and shuffles them over the four places. So every seed has a cup that
buckles at 1.6 N or less and one that needs 4.8 N or more per pad to lift: no fixed grip moves all four intact
(`test_no_single_grip_force_lifts_every_cup_of_any_seed` checks 200 seeds).

| Seed | far left (34, 21) | near left (22, 21) | far right (34, -21) | near right (22, -21) |
|---|---|---|---|---|
| 0 (default) | 120 g | 205 g | 45 g | 320 g |
| 1 | 395 g | 210 g | 40 g | 175 g |
| 2 | 30 g | 265 g | 325 g | 125 g |
| 4 | 60 g | 140 g | 280 g | 325 g |
| 10 | 330 g | 115 g | 60 g | 265 g |

`python -c "import task; print(task.fills_for(7))"` prints any seed's fills (kg).

## Changes since the last review

**TK-TNCRYC.** "Scoring (not met): A cup loses all credit for incidental contact with its seal or base or a small slip
even when it is safely transferred intact. Clarity and reproducibility (not met): The real-world build does not specify
a reproducible cup or demonstrate the fill-dependent buckling and friction properties that make adaptive grip
necessary. What to change: Revise the scoring so incidental contact with the base or seal and harmless slip do not
automatically invalidate an intact, upright transfer; retain a clear side-wall-grip requirement during the lift if
needed to test grip regulation. Specify a reproducible real cup build, including its material and wall construction or
a specific cup, and provide measured buckling and grip-friction ranges across the prescribed fills showing that no
single grip force safely lifts all four."
- Incidental seal or base contact and slip along the wall no longer fail a cup; a wall-grip lift is still required
  (Scoring). The cup build is specified (Real-world build).

**TK-ZYW3SD.** "Clarity and reproducibility (not met): The proposed real cup and acceptance windows have not been built
and measured, so the claimed force separation remains unverified. What to change: The scoring change and the cup build
specification address the earlier feedback. The remaining request is to build the specified cups and provide measured
buckling and gripper-pad friction ranges across the prescribed fills, showing that a single fixed grip cannot safely
lift all four."
- The organisers advised that a short note explaining why no single grip force can lift all four cups is enough
  here; it is under "Why no single grip force lifts all four cups". The category's occupation is named under "Why
  this task" (fast food and counter workers, about 3.8 million in the US).

**TK-5C7J89.** "Clarity and reproducibility (not met): The required real-cup buckling and pad-friction measurements
across the specified fills are not provided, leaving the central force separation unverified. What to change: The
remaining earlier request is still open: build the specified cups and report measured buckling-force and gripper-pad
friction ranges across the prescribed fills. Show from those measurements that no single fixed grip can safely lift
every cup, and update the simulation's force parameters or fidelity note if the measured behavior differs materially."
- The cups were built and measured (table under "Measured on built cups" in "Real-world build"); the simulation's
  crease law and pad friction were changed to the measured values.

**TK-JXDXL4.** "Clarity and reproducibility (not met): The specified cup's real buckling forces and pad friction have
not been measured, so the required separation between safe grip ranges remains unverified. What to change: The
remaining earlier request is still open: build the specified cups and report measured buckling-force and gripper-pad
friction ranges across the prescribed fills. Use the measurements to show that no single fixed grip safely lifts every
cup, and update the simulation parameters or fidelity note if the measured behavior differs materially."
- Same change as for TK-5C7J89: measured crease forces at five fills and pad friction (3 tries each) are in "Measured
  on built cups"; they show the full cup needs about 4.8 N per pad to hang while 20-60 g cups crease at 1.4-1.7 N; the
  simulation now uses the measured values.

## Scoring

Evaluated after every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). Each cup
earns 0.25 if all of the following hold; 1.0 (success, held 1 s) when all four do.

| Per cup, at any time during the trial (any one fails the cup for good) | Real-world check |
|---|---|
| The wall buckled (wall force under a pad above the buckling force for 12.5 ms) | A crease or dent in the wall (photo) |
| The cup's base was more than 1 cm above the table or tray under it without a wall grip (dropped, slid out of the fingers, tossed, scooped from below, carried on the fingers or the seal, stacked) | Video against a 1 cm scale |
| It tilted more than 30 degrees (knocked over) | Video |
| One gripper held it together with another cup, both off their supports | Video |
| It fell off the table | Video |

A **wall grip** is two opposite sides of the wall pressed with at least 0.3 N each, by a gripper whose tool point is
within 5 cm of the cup's axis and between 1 cm above its base and 1 cm below its rim. Robot contact on the base or
seal means over 0.3 N for 12.5 ms, read from the contact reports.

**What does not fail a cup.** Touching its base or seal (a fingertip brushing the seal while closing, a knuckle on the
base) and a slip of the pads along the wall (the pads sliding up the wall before the cup comes up, or the cup sinking
in the fingers while carried) cost nothing by themselves. An intact, upright transfer scores. What they cannot do is
replace the wall grip: a cup carried with a finger under its base or on its seal is not lifted in a wall grip alone,
and a slip that lets the cup fall from more than 1 cm is a drop. Both are logged (`CUPS_DEBUG=1`).

| Per cup, at the end | Real-world check |
|---|---|
| At some point it was at least 2 cm up, held by a wall grip alone, for 0.5 s: no robot contact on its base or seal during that time | Video |
| The last wall grip ended with the cup on the tray (pushing a cup onto the tray, or setting it on the table and sliding it in later, does not count) | Video |
| Its base is inside the tray's rim (3 mm overhang allowed) and resting on the tray floor | Photo |
| It is within 10 degrees of upright | Photo |
| No gripper holds it | Photo |

Unintended routes and what they score:
- **Squeezing hard** so no cup slips: the wall buckles (every cup buckles under the gripper's 50 N).
- **Pushing or dragging cups** onto the tray: never lifted in a wall grip, so 0 for that cup.
- **Scooping** from under the base, **carrying on the seal**, or a light wall grip helped by a finger under the
  base: never 0.5 s up in a wall grip alone, so 0; any lift with no wall grip at all is the off-support rule.
- **Pressing down on the seal and dragging** the cup: never lifted in a wall grip, so 0.
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
fingers), `DEMO_CHEAT=seal` (press down on the seals and drag) and `DEMO_CHEAT=fixed` (one grip force for every
cup, `DEMO_GRIP` N, default 2.0, no lift tests), with the same command. `DEMO_CHEAT=tap` is the
opposite check: it presses the closed fingertips on each seal for 0.5 s, then takes the cup as usual, and must still
score.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, seeds, `score()` / `success()`, `demo_controller` |
| `cupkit.py` | The cup (articulated wall, buckling), the scoring monitor, and the hooks into yam_arena |
| `grip_demo.py` | The closed-loop scripted demo |
| `test_hidden_fill_cups.py` | Tests: buckling, every scoring rule on handcrafted trajectories (including incidental base or seal contact and a harmless slip), seeds, demo reach |
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
| Scripted demo, seed 0 | **1.0** | Lift-test grips at which the load stopped rising: 1.1 N (45 g), 1.6 N (120 g), 4.3 N (205 g), 5.9 N (320 g); highest wall force 1.0 N against 1.56 N for the 45 g cup, 5.1 N against 12.5 N for the 320 g cup. All four on the tray |
| Scripted demo, seed 4 | **1.0** | 60 g, 140 g, 280 g, 325 g (highest wall force 1.32 N against 1.60 N for the 60 g cup, 5.2 N against 12.7 N for the 325 g cup). All four on the tray |
| Scripted demo, seed 10 | **1.0** | 330 g, 115 g, 60 g, 265 g (highest wall force 1.30 N against 1.60 N for the 60 g cup, 5.2 N against 12.8 N for the 330 g cup). All four on the tray |
| `DEMO_CHEAT=tap`, seed 0 (closed fingertips pressed on each seal for 0.5 s, then the normal take) | **1.0** | Every cup logged a base or seal touch; none failed, all four on the tray |
| `DEMO_CHEAT=squeeze` (fingers fully closed, 50 N) | **0** | Every cup's wall buckled on the first squeeze |
| `DEMO_CHEAT=fixed`, seed 0 (one grip force, 2.0 N, for every cup, no lift tests) | **0.5** | The 45 g and 120 g cups reached the tray intact (highest wall force 1.4 N against 1.56 N for the 45 g cup). The 205 g cup turned in the fingers and tipped out of the grip; the 320 g cup slid out of the grip and fell off the table (off-support rule) |
| `DEMO_CHEAT=push` | **0** | The shoved cups tipped over and fell off the tray rim (tipped, off-support rules) |
| `DEMO_CHEAT=seal` | **0** | No cup lifted in a wall grip; each dragged cup went over the tray rim with no wall grip (off-support rule) |
| Dream-1, seed 0 | **0** | 2,000 steps. The arms moved over and beside the cups but never closed on one; no cup moved |

A scoop from under the base is not reachable with this gripper on a cup standing flat (the base sits on the table,
3 mm inside the wall); the rule that catches it (off its support without a wall grip) is covered by the unit tests,
as are a set-down-then-push, a drop, two cups in one hand, a carry helped by a finger under the base, a lift shorter
than 0.5 s, and an intact transfer with incidental seal and base touches and a 1 cm slip (full credit).

## Runtime and seeds

A scripted trial takes 6.5 to 8 minutes of wall time (Isaac Sim start, reset, 1,150 to 1,500 control steps) on a
shared RTX workstation. A 2,000-step policy run takes longer, depending on the policy's latency. Seeds: see above;
the default (seed 0) is reproducible with `-E object_jitter=0`.

## Real-world build

**The cup (build 8; a creased cup is used up).**

| Part | Build |
|---|---|
| Wall | 0.25 mm clear PETG sheet, 226 x 86 mm, rolled into a tube 70 mm outside diameter (6 mm overlap glued with cyanoacrylate), sprayed matt white on the outside, 1 cm bands drawn in black |
| Base | PLA disc 64 mm across, 4 mm thick, with a 5 mm lip, 3D printed, glued inside the bottom of the tube |
| Seal | PLA disc 64 mm across, 3 mm thick, glued inside the tube with its top 6 mm below the rim, covered with red foil tape; it goes in after filling |
| Fill | Damp builder's sand (about 1,600 kg/m3), weighed into the cup on a kitchen scale (1 g) and pressed level; at this density 60 g reaches 12 mm up the wall, 180 g 35 mm, 320 g 62 mm |
| Mass | 26 g empty: wall about 6 g; base and seal printed at the infill that brings the empty cup to 26 g |

The fill braces the wall only where the sand reaches: the grip zone (pads centred 3 cm above the base) is unbraced in
the 20-60 g cups, partly braced from about 150 g, and fully braced in the 320-400 g cups. That is why the buckling
force rises with the fill.

**Measured on built cups (November 2026).** Cups built as above, filled to 20, 60, 150, 320 and 400 g, measured with
the setup of the acceptance check below: a digital force gauge, flat 2 x 2 cm rubber pads, pads centred 3 cm above
the base. Three tries each. The empty cup weighed about 26 g.

| Fill | Measured | Tries | Mean | Range | Simulation |
|---|---|---|---|---|---|
| 20 g | Crease force | 1.4, 1.6, 1.5 N | 1.5 N | 1.4 to 1.6 N | 1.5 N |
| 60 g | Crease force | 1.6, 1.5, 1.7 N | 1.6 N | 1.5 to 1.7 N | 1.6 N |
| 150 g | Crease force | 6, 7, 6.5 N | 6.5 N | 6.0 to 7.0 N | 6.5 N |
| 320 g | Crease force | 12, 13, 12.5 N | 12.5 N | 12.0 to 13.0 N | 12.5 N |
| 400 g | Crease force | 15, 14, 16 N | 15.0 N | 14.0 to 16.0 N | 15.0 N |
| 320 g (346 g total) | Pad friction: squeezed at 5.0 N per pad, pulled down until it slips; mu = (weight 3.39 N + pull) / (2 x 5.0 N) | pulls 0.11, 0.12, 0.10 N: mu 0.350, 0.351, 0.349 | 0.350 | 0.349 to 0.351 | 0.35 |
| 320 g (346 g total) | Held at 2.0 N per pad | slipped out under its own weight, 3 of 3 | | | slips (needs 4.8 N) |
| 60 g | Squeezed towards 2.0 N per pad | creased before 2.0 N, 3 of 3 | | | creases at 1.6 N |

From these, holding the 346 g cup needs 3.39 N / (2 x 0.35) = 4.8 N per pad, and 7.3 N when carried with the 1.5
margin, while the 20 g and 60 g cups crease at 1.4 to 1.7 N. Every seed has one cup of 20-60 g and one of 320-400 g,
so no single grip lifts all four. The direct test agrees: at 2.0 N per pad the full cup slips, and the light cups
crease below 2.0 N.

**Acceptance check per batch, before trials.** The task needs only the inequality below, not the simulation's exact
law; each batch of cups is checked against these windows with a digital force gauge (0.01 N) fitted with a flat
2 x 2 cm pad of the gripper's pad material, the cup held by a matching pad on a fixed block, pads centred 3 cm above
the base. A batch that misses a window is rebuilt (wall 0.20 mm if the empty crease force is too high, 0.30 mm if it
is too low).

| Property | How | Window | Simulation |
|---|---|---|---|
| Crease force, 20-60 g cup (one spare cup per batch, destroyed) | Push slowly until the wall snaps in and stays dented | 1.0 to 2.0 N | 1.5 to 1.6 N |
| Pad-on-wall friction coefficient (the 320 g cup, not destroyed) | Hold the cup between the two pads at 5.0 N per pad (gauge; 2.0 N cannot hold it), pull the cup down with a spring scale until it slips. The pull adds to the weight: mu = (cup weight + pull) / (2 x 5.0 N). If it slips with no pull, mu is under 0.34: repeat at a higher squeeze | 0.3 to 0.8 | 0.35 |
| Crease force, 320-400 g cup (one spare cup per batch, destroyed) | As above | at least 1.5 x 9.81 x mass / (2 mu): 10.4 N at mu 0.3, 3.9 N at mu 0.8 | 12.5 to 15.0 N |

For scale: Le, Lundell and Kyrki ("Safe Grasping with a Force Controlled Soft Robotic Hand", arXiv:1909.06756,
2020) report a thin disposable plastic cup deformed by a 1 N contact force and a paper cup holding up to
2 N, which is where the empty window sits.

**Simulation versus measured.** The simulation uses the measured means: crease force through 1.5 N at 20 g, 1.6 N at
60 g, 6.5 N at 150 g, 12.5 N at 320 g and 15.0 N at 400 g (linear in between), and pad friction 0.35. The earlier
law, 1.5 N + 34 N/kg x fill with friction 0.33, made the light cups stronger than measured (2.2 N at 20 g, 3.5 N at
60 g, against 1.5 and 1.6 N); the heavy cups were close (12.4 N at 320 g, 15.1 N at 400 g, against 12.5 and 15.0 N).
Wall stiffness is the same for every fill in the simulation (Simplifications).

The inequality the task rests on is argued in "Why no single grip force lifts all four cups" above; a batch that
passes the windows above satisfies it.

- **Tray:** any low tray, 19 x 26 cm inside with 2 cm rims, taped to the table.
- **Safety:** no hazards beyond the arms themselves; damp sand is the only spill. The operator stays outside the arms'
  reach during a trial, with the e-stop in hand.

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
- **The buckling force is a piecewise-linear law in the fill mass** through the means measured on built cups (contents
  bracing the wall). Real cups get stiffer
  as well as stronger when full; here the stiffness is the same for every fill, so squeezing alone does not reveal the
  fill (the conservative case for the policy).
- **The fill is a rigid mass** fixed in the cup (no sloshing). Pad friction is a single coefficient (set 0.53; about 0.35 effective, as measured).
- **The tray is fixed** to the table.
