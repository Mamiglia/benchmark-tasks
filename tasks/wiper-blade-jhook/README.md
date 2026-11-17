# Replace a worn wiper blade on a spring-loaded J-hook arm

A windshield panel slopes up away from the arms at 27 degrees. A J-hook wiper arm lies across its lower part, pivot on
the right, with a worn 18-inch (45 cm) conventional blade on its hook. The arm is sprung against the glass and does
not stay up by itself. A new blade of the same model, its J-hook adapter fitted, stands in a cardboard tray on the
table to the left.

**Instruction:** `Replace the wiper blade with the new one and lay the arm back on the windshield.`

The instruction states the ordinary goal only. It does not say how to release the blade, that the arm must be
held up, or that it must not be dropped.

## Why this task

- **Everyone knows it.** Most drivers have changed a wiper blade, or watched it done at an auto-parts counter. A
  ten-second clip of a robot lifting a wiper arm and swapping the blade needs no explanation.
- **The arm fights back.** A wiper arm presses its blade on the glass with about 8 N (service manuals give 7.5 to
  9.5 N at the arm tip for a car arm, measured with a spring scale at the tip). It does not stay up: released, it
  snaps back, and a bare hook or a blade slammed onto the glass can chip or crack the windshield. One hand must hold
  the arm up, against its spring, for the whole swap, while the other hand works on the blade. At the end the arm
  must be lowered by hand onto the glass, not let go in the air.
- **The order is fixed by the hook.** The J-hook is the arm's end bent back on itself. The blade's adapter sits in
  the bend, held by a sprung latch. To remove the blade: press the release tab, slide the blade toward the pivot
  until the adapter leaves the bend, then draw the blade away from the arm. Pulling without the tab does not release
  it (forced, the hook bends). To fit the new one: bring the adapter up under the hook behind the bend, lined up
  with the arm, then pull the blade toward the bend until the latch clicks. Threading it from the tip side, or turned
  across the arm, does not engage.
- **The difficulty is physical.** Bimanual: one arm holds a sprung load steady while the other does fine work
  near it; a constrained threading order under a lifted arm; the push needed to click the latch (about 7 N); and a
  controlled lowering of a spring-loaded arm at the end.

### Distinct from

- **Mount a phone in a spring-clamp car holder (TK-DJYB5S, accepted).** That task holds two sprung arms open while a
  phone goes in, then lets go: the sprung part closes on the object by design. Here the sprung part is the load
  the hand must carry for the whole task and then put down gently; the inserted part must be threaded on a hook in a
  fixed order and latched, and the old one released first.
- **Refit a car headrest (TK-T5MAQE, accepted).** Two posts into sleeves, hold a button and push down. No sprung
  load to hold, no threading order.
- **Covered abilities.** It contains a click (the latch), but the click is the last 4 mm of a threading path under
  an arm that one hand must hold up against its spring; the task is not "press until it clicks".
- **External benchmarks.** The nearest public tasks are hanging or hooking objects (for example "hang the mug on
  the hook" in RLBench-style suites, BEHAVIOR's hanging tasks): a free object onto a fixed hook. Here the hook is on
  a sprung arm that must be held up and put down, and the old part must be released and removed first.

Category: **Societal relatability**.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, the 60 Hz latch and arm rules, `score()` / `success()`, the demo hook |
| `hookmodel.py` | Geometry (glass, arm, seat frames) and the latch state machine and arm rules, plain NumPy |
| `wiperkit.py` | Multi-box parts in any orientation, the fixed arm articulation with its spring, switchable D6 joints, hooks into yam_arena |
| `wiperdemo.py` | The closed-loop scripted demo |
| `test_wiper_blade_jhook.py` | Unit tests of the latch state machine, the threading order and the arm rules |
| `task.json` | Submission details |

`wiperkit.py` hooks yam_arena without modifying it (it patches `yam_arena.scene.task_object` / `object_states`, the
embodiment's `_ensure_sim`, `_task_step` and `reset`, and `yam_arena.scripted.YamScriptedPolicy` so `yam_scripted`
plays `demo_controller`). The task runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left, z up). Arms start in
the ready pose. The layout is the same in every trial.

| Object | Description |
|---|---|
| Windshield panel | Clear acrylic sheet, 5 mm thick, 68 cm (along y) x 36 cm (up the slope), sloping up away from the arms at 27 degrees. Its lower edge's top surface is at x 20, z 3, centred on y -2; its upper edge at x 52, z 19. Held by a black front rail (4 x 68 cm, 2.6 cm tall, under the lower edge) and a rear block under the upper edge, both fixed to the table |
| Wiper arm | Steel J-hook wiper arm, black, 300 g, pivot on a post on the glass 11 cm up the slope from the lower edge, at y -32; hinge axis 4.9 cm above the glass, running up the slope. Shaft 12 mm wide, 10 mm deep (4.4 to 5.4 cm above the glass); hook 36 cm from the pivot (inside of the bend), at y +4. J-hook (9 x 4 size): 9 mm wide, 2.5 mm thick strip bent back on itself round the adapter's nose, return leg 2 cm long. Spring: about 8 N on the glass at the blade's centre (2.9 N m about the hinge with the blade on), rising to about 9 N at 14 degrees up. A stop limits the lift to 45 degrees; with no blade it rests on the glass on the bare hook (3.8 degrees below its blade-on angle). It does not stay up anywhere in its travel |
| Old blade | Worn 18-inch (45 cm) conventional blade, dull grey steel frame (spine 10 mm wide, centre bridge 20 cm long), rubber 6 mm wide, 110 g, total height 2.4 cm to the bridge top. On its centre, a black J-hook adapter (30 x 11 x 10 mm) latched in the hook. Release tab: a 10 x 16 mm lever on the adapter's side facing the arms, 3 mm thick; pressed 1.5 mm (about 5 N) it frees the latch |
| New blade | Same model, black frame, adapter fitted in the J-hook position; standing on its rubber in the tray slot, centre at x 33, y 45, running along x with its adapter's nose toward the arms and its release tab facing the windshield |
| Tray | Corrugated cardboard, 48 x 4 cm, 3 mm floor, two walls 12 mm tall forming a 12 mm slot, at x 33, y 45 |

Nothing else is on the table.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Replace the wiper blade with the new one and lay the arm back on the windshield." \
  --scorer task_score --max-steps 3000 --seed 0 --store-frames --log-dir ~/yam-eval/wiper-dream
```

Scripted demo: the same with `--policy yam_scripted` (seeds 0, 4 and 10 all score 1.0; the layout does not depend on
the seed). `DEMO_CHEAT=drop` (let go of the lifted arm once the new blade is on) and `DEMO_CHEAT=yank` (pull the old
blade off without pressing the tab) play the two loopholes. `WIPER_DEBUG=1` prints the arm angle and each blade's
latch state, offset from the seat and tab travel every 0.1 s. Unit tests (no simulator): `pytest
test_wiper_blade_jhook.py` with `yam_arena` importable.

Runtime per trial: the scripted demo succeeds at about 62 s of simulated time (about 1,240 control steps), about 4 minutes of wall time on one Quadro-class GPU; a full 3,000-step model run takes about 10-15 minutes.

## Scoring

The latch and arm rules run at 60 Hz; the score every control step. The score at the end counts
(`SCORE_OVER = "end"`).

| Condition | Score | Real-trial check |
|---|---|---|
| At any time: the arm reached the glass with the hook moving faster than 0.75 m/s (the speed of a fall of about 3 cm on the spring alone) | 0 for the rest of the trial | Video: the arm was let go with the hook 3 cm or more above where it came to rest, or snapped down |
| At any time: the arm, with no blade on its hook, came down onto the glass (bare hook on the glass) | 0 | Video: the bare hook touched the glass |
| At any time: a latched blade was forced off the hook without its tab (real trial only: in simulation the weld does not break, so forcing gets nothing) | 0 | Photo: hook bent or adapter broken, or the blade came off without the tab pressed (video) |
| At any time: a blade fell off the table | 0 | Video |
| Old blade off the hook and lying on the table (lowest point within 1.5 cm of the table, not on the windshield or its stand), not held | 0.25 | Photo |
| And the new blade latched on the hook | 0.5 | After the trial, a 2 kg pull along the arm on the blade does not move it (luggage scale); the adapter is in the bend |
| And the arm resting on the glass on the new blade (hinge within -0.6 to +1 degree of the rest angle, still) | 0.75 | Photo: the rubber is on the glass along its length |
| And every gripper tool point 3 cm or more from the arm and both blades; held 1 s (success) | 1.0 | Video |

Doing nothing scores 0. Re-latching the old blade, or leaving it on the windshield, scores 0.

**Unintended strategies.** The old blade cannot be pulled off without the tab: in simulation the weld holds
whatever the pull (a breakable weld also broke under normal tab presses, from PhysX joint-force spikes), and in a
real trial forcing it bends the hook, which scores 0. The new blade cannot be latched from the tip side or turned
across the arm: the latch engages only from behind the bend, lined up within 3 mm and 6 degrees. Letting go of the
arm in the air fails on the landing speed, and putting it down without a blade fails on the bare hook.

## Simulation and fidelity

- **Arm.** One revolute joint about the hinge axis, an implicit spring drive (1.2 N m/rad toward 1.73 rad below the
  glass: 2.1 N m at rest, plus 0.8 N m from the arm's and blade's weight on the slope), 300 g arm, stop at 45
  degrees. Real arms use a coil spring on a lever with a similar rise.
- **Hook and latch.** The hook is drawn but has no collider; its engagement is a state machine (`hookmodel.py`)
  that switches two D6 joints between the blade and the arm: a weld (latched) and a slide along the arm (threaded,
  16 mm of travel, 15 N s/m of sliding friction). The latch tab is a real prismatic part on the blade (3 N preload,
  about 5 N to press it 1.5 mm); the click is a drive that resists the last 4 mm with up to 7 N. The robot does the
  real work for every transition: it must press the tab with its fingers, push the blade 13 mm or more along the
  arm, hold it to draw it off, place the new adapter within 3 mm and 6 degrees behind the bend, and push it 12 mm
  home against the click.
- **Rigid blade.** The blade is rigid; a real blade's rubber and frame flex to the glass's curve. The panel is
  flat, so a rigid blade lies along it.
- **Speeds.** The landing speed is the hinge speed times 36 cm, read at 60 Hz.

## Real-world build and safety

| Item | Example | Approx. price |
|---|---|---|
| J-hook wiper arm (9 x 4 hook, about 36 cm pivot to hook) | Dorman 42631 or any aftermarket J-hook front arm of that length | $20-30 |
| Two 18-inch conventional blades with J-hook adapter | ANCO 31-Series 31-18 (one worn, one new) | 2 x $10 |
| Acrylic sheet, 5 mm, 68 x 36 cm | Hardware store (cut to size) | $25 |
| Pivot base: a 25 mm square steel post with the arm's pivot stud, bolted through the sheet, and a stop bolt at 45 degrees | M8 bolt, nut, washers, angle bracket | $5 |
| Stand: front rail 4 x 2.6 cm and rear block, plywood, screwed or clamped to the table | Scrap plywood | $5 |
| Cardboard tray | Cut from the blade's box | 0 |

Total about $75-85. Set the stand so the sheet slopes at 27 degrees. Check the arm's force with a luggage scale at
the blade's centre (lift until the blade leaves the glass): 0.75-0.95 kg; adjust the spring tension (or choose an
arm) to match. A real blade weighs about 110 g; the arm's spring force at the hand (about 13 N at 22 cm from the
pivot) is within the YAM's payload, and pressing the tab (about 5 N) and clicking the latch (about 7 N) are well
within its grip.

Safety: the acrylic panel stands in for glass, so nothing can shatter. Wiper blades have steel frames with folded
edges (no sharp edges); cover the J-hook's end with a strip of tape if it has a burr. No liquids are used. Keep
hands clear of the arm while the robot holds it up: a released arm snaps down with about 8 N.

## Assets and licences

All geometry is built from boxes in `task.py`; no third-party meshes, textures or images. The robot and the station
come with yam_arena: the YAM robot model (meshes and URDF) is from i2rt (github.com/i2rt-robotics/i2rt) under the
MIT licence; the station scene is the one yam_arena ships for this program.
