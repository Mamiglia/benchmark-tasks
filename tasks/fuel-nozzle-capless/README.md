# Refuel a car through its capless filler: nozzle in past the sprung flap, trigger clipped open, then back in the boot

Instruction: *"Fill the car: put the nozzle in the filler, squeeze the trigger and set the hold-open clip, then release
the trigger and hang the nozzle back up."*

Category: **Societal relatability**. Filling a car at the pump is one of the most familiar adult chores; about 230
million licensed US drivers do it themselves, most of them every week or two. A ten-second clip of a robot taking
the nozzle off the pump and pushing it into a car needs no explanation. No fuel is used in any trial.

## Why this task

The robot does the whole job the way a driver does, with the real tools: a dry automatic pump nozzle on a real hose,
the pump's boot, and a car's capless (Ford Easy Fuel type) filler set in a body panel at fender height. What makes
it hard is physical:

- **A heavy, off-balance tool on a dragging hose.** The nozzle weighs 1.3 kg, most of it ahead of the handle, and
  1 m of hose pulls on its back end. That is at or over what one YAM arm carries (about 1.5 kg), so the robot must
  carry it in two hands, as many people do: one on the handle, one under the valve housing.
- **Blind insertion past a sprung flap.** The filler's bore is 24 mm across for a 21 mm spout, tilted 30 degrees down
  into the car. A few centimetres in, a sprung flap the robot cannot see closes the bore; the spout must push it open
  and go on to 8 cm.
- **Bimanual trigger and clip.** With the spout in, one hand squeezes the lever up against its spring while the other
  flips the small hold-open clip under it. Then the lever must stay open with no hand on it for 3 s.
- **A constrained order.** The lever may only open with the spout in the filler. To finish, the lever is squeezed past
  the clip's notch so the clip drops, let return, and only then is the nozzle drawn out and hung back in its boot.

### Distinct from

- Board tasks: the nearest are a trailer gladhand coupling (a coupling pushed and turned, no trigger or clip), a hair
  dryer stowed in a wall holster (one holster insertion, light tool, no hose), and a coolant hose pushed onto a barb
  (a hose end, no tool to carry). None has a heavy two-handed tool on a dragging hose, a blind sprung flap, or a
  trigger held by a clip set with the other hand.
- External benchmarks: no refuelling task in BEHAVIOR-1K, LIBERO, RoboCasa or the other listed suites. The nearest
  are tool-use and insertion tasks (for example peg-in-hole insertion in robosuite/FurnitureBench), which insert a light
  part with one hand and have no trigger, latch or hose.

## Design

Everything is at real size.

- **Car body.** A painted panel 50 cm wide and 42 cm tall standing on the table across the far left (its face 56 cm
  out from the arms). The fuel door stands open on its left. Behind it, a pocket 10 x 10 cm and 3.5 cm deep holds the
  filler head: an aluminium face 66 mm across with a 40 mm opening that tapers over 12 mm into a 24 mm bore (the real
  head's restrictor is 23.6 mm), which runs 17 cm into the car, 30 degrees below horizontal. Its centre is 17 cm above
  the table (92 cm above the floor at the station's 75 cm table), a usual filler height for a crossover. A sprung flap
  hinged at the top of the bore, 25 mm in, closes it; opening it takes about 4 N, rising to about 10 N as it swings
  up.
- **Pump.** A post at the panel's right holds the boot: a sleeve 36 mm square inside and 17.5 cm deep, tilted so the
  nozzle hangs spout down (55 degrees below horizontal), handle toward the arms, its handle about 27 cm above the table.
  The hose leaves the nozzle's swivel and runs 1.04 m to the pump's hose outlet on a mast at the right front corner of
  the table (52 cm up). Simulated as 8 rigid links 13 cm long on ball joints with a bending spring and damping.
- **Nozzle.** A dry automatic unleaded nozzle: valve housing, handle tube, swivel, steel spout 20 cm long bent 30
  degrees down, a trigger lever under the handle inside a guard, and a red hold-open clip on the guard.
  Mass 1.3 kg. The lever pivots at the housing; its return spring needs 8 N at the finger position to start and 20 N
  at full travel (21 degrees). The valve would open at a third of the travel. The clip is a small sprung lever pivoted on
  the guard bar: its tab hangs under the bar, and pushing the tab back swings its arm up under the lever's tail. With the clip up and
  the lever squeezed past half travel, the clip catches the lever and holds it at the middle notch. Squeezing the lever
  past the notch again lets the clip drop and the lever spring back.

The layout is fixed (always pass `-E object_jitter=0`); the seed changes nothing in the scene.

## Scoring

Score 0 if at any time in the trial:
- the lever was opened (a third of its travel) while the spout tip was less than 4 cm into the filler (in the real
  world this sprays fuel), or
- the nozzle fell (moving down faster than 0.7 m/s, or its housing within 4 cm of the table).

Otherwise:

| Score | Condition | Real-trial check |
|---|---|---|
| 0.15 | the spout tip went 8 cm or more down the filler | tape mark on the spout 8 cm from its tip went inside the filler face (video/photo) |
| 0.5 | and, with it that far in, the lever stayed held open by the clip for 3 s in a row with no gripper on the lever | video: lever up, clip under it, both grippers off the lever, 3 s |
| 0.7 | and after that the clip released and the lever was back at rest | video: clip down, lever down |
| 1.0 | and now the nozzle hangs in its boot (spout tip 13 cm or more in), lever at rest, no gripper holding it | photo: tape mark 13 cm from the tip inside the boot; grippers clear |

Each stage needs the ones before it, in order. "No gripper on the lever": no gripper closed to under 80 mm with its
tool point within 15 mm of the point between the handle top and the lever. "Holding": closed under 80 mm with its tool
point within 5 cm of the housing or the handle. A trial that holds the lever open by hand never reaches 0.5; one that
squeezes the lever before the spout is in, or pulls the spout out with the lever latched, scores 0.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fill the car: put the nozzle in the filler, squeeze the trigger and set the hold-open clip, then release the trigger and hang the nozzle back up." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/fuel-nozzle-capless-dream
```

`-E control_hz=20` is required: the task's latch and score bookkeeping runs once per control step and assumes 20 Hz.
`FUEL_DEBUG=1` prints the nozzle state (lever, clip, latch, depth in the filler and in the boot, flap) five times a
second.

## Scripted demo

Same command with `--policy yam_scripted`. `yam_scripted` runs `nozzledemo.Controller` closed loop (`nozzlekit` hooks
it in when the task module defines `demo_controller`). It reads the simulator state the task keeps (nozzle pose,
lever and clip angles, latch state), as scripted demos may; it does not move objects. Under the 1.3 kg nozzle the
simulated arms sag 6-8 cm (their joint gains are I2RT's), so the demo corrects the planned nozzle pose from the
measured one (a rate-limited integral term) and lines the spout up on the bore 6 cm out before pushing it in. Sequence:
both hands take the
carry grips (left round the handle tube with its lower finger in the gap above the lever, so the lever is not touched;
right on the valve housing); the nozzle is drawn out of the boot, turned level and pushed 9.5 cm down the bore, the
planned pose corrected for the arms' sag from the measured one; the left hand moves its lower finger under the lever
and squeezes; the right hand pushes the clip's tab forward; the left hand opens and both stay off for 3.6 s; the left
hand squeezes past the notch (the clip drops) and opens; both take the carry grips again and the nozzle goes back
down the boot.

Loophole checks, same command with `DEMO_CHEAT=hold` (holds the lever open by hand for 4 s, never sets the clip, then
hangs the nozzle up) and `DEMO_CHEAT=shallow` (pushes the spout only 3 cm into the filler, then squeezes and clips the lever there).

## Checked in simulation

| Run | Score | What happened |
|---|---|---|
| Scripted demo, seeds 0, 4, 10 | 1.0, 1.0, 1.0 | spout 98 mm in (flap open), lever latched and hands off 3.05 s, released, nozzle back in the boot 174 mm deep; done at 57 s (about 1,150 steps) |
| `DEMO_CHEAT=hold` (lever held open by hand, clip never set) | 0.15 | inserted, never latched; then hung up |
| `DEMO_CHEAT=shallow` (spout 3 cm in, lever squeezed and clipped there) | 0.0 | spout 33 mm in; the squeezed lever opened with the spout under 4 cm in (latched fail) |
| Dream-1, seed 0, 3,000 steps | 0.0 | moved both arms around the table and toward the nozzle late in the run; never lifted it |

Also seen while building: pulling the spout out with the lever still latched scores 0 (lever open with the spout out of
the filler).

## Runtime and seeds

The scripted demo needs about 1,150 control steps (57 s at 20 Hz); the run limit is 3,000. A trial takes about 6 to 8
minutes of wall time on one shared data-centre GPU. The layout is fixed: the seed changes nothing (seeds 0, 4 and 10
give the same demo result).

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, run-time latch hook, `score()` / `success()`, `demo_controller` |
| `nozzlemodel.py` | Geometry, the hose's starting shape, and the latch and score bookkeeping (NumPy only) |
| `nozzlekit.py` | Multi-box parts, sprung child joints, D6 and ball joints between parts, and the hooks into yam_arena |
| `nozzledemo.py` | The closed-loop scripted demo |
| `test_fuel_nozzle_capless.py` | Tests: every scoring rule on handcrafted trajectories, latch and release, geometry |
| `task.json` | Submission details |

`nozzlekit.py` hooks yam_arena without modifying it: `yam_arena.scene.task_object` and `object_states`, the
embodiment's `_ensure_sim`, `reset` and `_task_step`, and `yam_arena.scripted.YamScriptedPolicy.reset` / `act`.

## Tests

```bash
python -m pytest -q test_fuel_nozzle_capless.py   # NumPy only; no GPU needed
```

## Real-world build

All parts new and dry: the nozzle and hose have never carried fuel, and no fuel or other liquid is used.

| Item | Product | Size / mass | Approx. price |
|---|---|---|---|
| Nozzle | Husky 1A automatic unleaded nozzle, 3/4 in NPT, three-notch hold-open clip (for example Husky 159464N), die-cast aluminium | spout 21.3 mm OD (the SAE J285 unleaded size), about 35 cm long, 1.3 kg (Husky lists about 3 lb shipping weight) | $90-120 |
| Hose | 3/4 in UL 330 dispenser hose whip, male NPT both ends, 4 ft (1.2 m; 1.04 m between fittings) | about 0.6 kg/m | $35-50 |
| Boot | universal nozzle boot for 3/4 in automatic nozzles, or a 50 mm PVC pipe 18 cm long with an end cap, screwed to the post at 55 degrees below horizontal | 36-50 mm bore | $10-25 |
| Filler | Ford Easy Fuel capless filler head (the upper filler pipe with its capless head; OE number by VIN, for example a 2013-2019 Escape or Fusion), pipe cut 17 cm below the face | 66 mm face, 23.6 mm restrictor | $30 used, $80-150 new |
| Car body | 12 mm plywood (or a scrap steel quarter panel) 50 x 42 cm, painted, 10 x 10 cm opening with a pocket box 3.5 cm deep behind it, the head bolted in its back at 30 degrees down; a hinged plate 10 x 10 cm as the open fuel door; two L-brackets to stand it on the table | | $20 |
| Pump post and mast | 80 x 120 mm timber post 52 cm tall clamped at the panel's right; 30 x 30 mm post 52 cm tall at the right front corner with a 3/4 in NPT floor flange near its top for the hose | | $15 |

Total about $200-250. Positions are in the Design section and the screenshot (world frame: x forward from the arms'
midpoint, y left, z up): filler lip centre (59.5, 15, 17 above the table), boot mouth (43.7, -5, 16), hose outlet
(56, -40, 52).

**Forces.** Real nozzle levers and Easy Fuel flaps vary; measure before the first trial. Lever: hook a luggage scale
at the lever 65 mm behind its pivot and pull up: the dry lever works against its return springs only (no fuel
pressure); it must take under 30 N at full travel to leave margin under the YAM's roughly 40 N per finger (the sim
uses 8-20 N). Flap: push a 21 mm dowel into the head on a kitchen scale (the sim takes 4-10 N; it must stay under
about 15 N, which the wrist can push). Clip: about 2 N on its tab. Weight: 1.3 kg nozzle plus up to about 0.3 kg of
hose on its end, about 0.8 kg per arm in a two-handed carry. Choose a nozzle whose lever works without fuel pressure
(no "no-pressure/no-flow" feature, or one that can be disabled) and check that the clip holds the dry lever.

**Safety.** No fuel, vapour or liquid anywhere, and the nozzle and hose are new, so nothing can drip onto the rig. The
spout end is rounded (no sharp edge); tape the panel's opening edges. The nozzle can fall at most to the table: put a
rubber mat under the boot and the filler. The heaviest load is the 1.3 kg nozzle, carried 15-35 cm above the table.

## Simplifications

- The spout is a 19 mm square bar and the bore a 24 mm square channel (PhysX boxes); the real ones are round (21.3 mm
  spout in a 23.6 mm restrictor). The square keeps the clearance similar and stops the spout rolling in the bore.
- The real Easy Fuel head has two flaps and two latch arms that only release for a spout of the right size; the sim
  has one sprung flap at the mouth, behind a square tapered lead-in.
- The clip's catch on the lever is a rule, not contact: when the clip's arm is up past 0.5 rad (pushed there by a
  gripper against its spring) and the lever has been squeezed past half travel, the lever's spring target moves to the
  middle notch and the clip is held up; squeezing the lever past 0.29 rad after it has come down onto the notch
  releases both, and the clip must fall back before it can catch again. The robot does the real motions with the real
  forces to trigger either change.
- The hose is 8 rigid links on ball joints with a bending spring (0.25 N m/rad per joint) and damping; a real
  dispenser hose is a continuous, somewhat stiffer tube.
- The fuel door, panel, post, boot and filler are fixed; nothing flows.

## Assets and licences

No third-party assets in the task: every object is built from boxes in `task.py` / `nozzlekit.py`. The robot is the
standard yam_arena YAM (I2RT YAM arm URDF and meshes, MIT licence, `yam_arena/assets/yam/LICENSE.i2rt`); the room and
cameras are the standard station scene.
