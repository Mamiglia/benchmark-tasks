# Keep a machine loader's infeed nest stocked while it works on its own cycle

Two robot arms share a machine-tending cell. The left arm is the machine's loader and runs its own program: every
so often it reaches into an infeed nest, takes the aluminium blank standing there and drops it into the machine's
infeed chute. The right arm is the one under test. It must keep that nest stocked from a tray of five blanks, one
blank at a time, so that the loader finds a blank every time it comes for one. The loader does not wait, and how long
it stays away changes from cycle to cycle and from trial to trial.

**Instruction:** `Keep the infeed nest stocked with one blank at a time from your tray, so the other arm finds a blank there every time it comes for one.`

**Category:** Impact on jobs.

## Why this task

- **Real work.** Machine feeders, machine tenders and CNC operators keep the infeed of a machine or a loading robot
  supplied with raw parts. Sawn bar blanks fed to a lathe loader through a locating nest is a standard setup. The
  supplier must keep up with a consumer whose cycle varies (it waits for the machine), must not put a part where one
  already sits, and must keep its hand out of the loader's path.
- **The skill is supplying a consumer that you have to watch.** The loader's visits come at varying intervals of 10
  to 16 s, set by the trial seed. There are six visits and five blanks. The first visit takes the blank that is
  already in the nest, and every visit after that must find a supplied blank. A missed visit is lost, and a second
  blank set on an occupied nest faults the cell. So the robot can't follow a fixed clock and can't wait things out.
  It has to see when the nest has been emptied, see where the loader's arm is, and fit a pick, a precise set-down
  and a retreat into the time before the next visit.
- **The difficulty is in the motion, not in recognising something.** The blank has to go into a 40 mm pocket
  (5 mm clearance round a 30 mm blank) and stand up in it. The hand has to arrive while the loader's arm is just
  leaving the same spot and be gone again before the loader returns. The robot also has to fetch the next blank in
  time. To do this reliably, it holds the next blank ready near the nest. A policy that sets a blank down blindly
  either faults the cell or loses visits. A policy that rushes in as soon as the nest looks empty risks hitting the
  loader's arm, and any arm-to-arm contact scores 0.

### Distinct from

| Task | Its skill | Here |
|---|---|---|
| TK-8RX6NT Unload fruit from a bin another robot arm is also unloading (accepted) | Finish your own pick-and-place around a second arm that works in the same bin. The other arm needs nothing from you | The other arm depends on you. Each of its visits must find exactly one part, and the timing of the visits is observed, not fixed |
| TK-56TUV2 Unload a carousel another robot arm keeps loading; TK-WAGG2Y Unload a part from a cycling shuttle table | Remove parts from a source that moves on a fixed timetable | Supply rather than remove. There is no fixed timetable: the consumer's interval varies by visit and by seed, so the timing must be observed |
| TK-RNF7BR Load four blanks into a gravity magazine; TK-KK4Y8H Load a raw blank into a machine nest | Load parts into a fixture, at your own pace | The same kind of fixture placement, but synchronised with a consumer that empties it on its own varying cycle |
| Conveyor picking (RoboDojo "match and pick from conveyor", abc-sim Conveyor Pick, RoboCasa) | Pick items that move past on a belt | Nothing moves past. The robot places parts for a second robot and must fit its motions round that robot's |

## Design

World frame: x forward from the arms, y to the left, from the midpoint between the two arm bases, on the table top.
The station is Dream's calibrated YAM station, with arm bases 61 cm apart.

| Object | Size | Mass | Where | Notes |
|---|---|---|---|---|
| Blanks, 6 | 30 mm diameter x 40 mm, 6061-T6 aluminium bar sawn to length and deburred | 76 g each | One stands in the nest. Five stand on the tray at (26, -33), (33, -33), (40, -33), (29.5, -42), (36.5, -42) cm | Upright on their flat ends |
| Infeed nest | Black acetal block, 70 x 70 mm, 12 mm tall, with a 40 x 40 mm square pocket through to the table | fixed | Centred at (32, 0) cm | Holds one blank upright, located to within 5 mm |
| Blank tray | Blue-grey ESD plastic tray, 24 x 20 cm outside, 5 mm floor, 1 cm rims 12 mm tall | fixed | Centred at (33, -37.5) cm | Taped to the table |
| Infeed chute | Steel box with an open top, 12 x 12 cm inside, 1 cm walls, 10 cm tall | fixed | Centred at (27, 40) cm | Stands in for the machine's load station. Only the left arm can reach it |
| Left arm | I2RT YAM, the cell's loader | - | Base at (0, 30.5) cm | Runs its own program. The policy's left-arm commands are ignored |
| Right arm | I2RT YAM, the arm under test | - | Base at (0, -30.5) cm | |

**The loader's program.** It waits at a pose above its own side, at (20, 28) cm and 15 cm up. At each visit it goes
over the nest (1.6 s), comes down to grip height (1.0 s) and closes (0.6 s). It then lifts (0.8 s), carries what it
holds to 16 cm over the chute (1.6 s), lowers into it to 8.5 cm (0.5 s), opens (0.4 s), rises (0.5 s) and returns to its waiting pose (1.2 s).
It closes six times per trial. The first close is 5.0 to 6.5 s after the start. Each later close comes 10 to 16 s
after the one before, drawn per visit from the trial seed. For example:

| Seed (`--seed`) | Close times (s) |
|---|---|
| 0 (episode seed 1415158438) | 6.4, 17.6, 28.6, 44.4, 55.2, 70.7 |
| 4 (episode seed 3490944092) | 6.1, 22.0, 34.9, 48.5, 60.3, 75.2 |
| 10 (episode seed 3996681712) | 6.4, 19.4, 32.1, 42.2, 57.4, 67.5 |

`CELL_DEBUG=1` prints the close times for any seed. inspect-robots turns `--seed` into an episode seed, and that is
what is printed. A real trial plays the same table on the real left arm.

## Scoring

The score is 1/5 for each of the five tray blanks that meets both conditions:

1. At one of the loader's visits, at the moment its fingers had closed, the blank stood in the nest's pocket, and
   the right gripper's tool point was at least 8 cm (horizontally) from the nest's centre.
2. At the end of the trial, the blank is in the chute: its centre is inside the chute's inner walls and less than 14 cm
   above the table (resting on the chute floor or on other blanks).

The blank that starts in the nest never counts. The score is **0** if the two arms touched at any time in the trial,
or if two blanks ever lay in the nest's pocket at once. Both conditions are latched, so recovering does not undo
them. The trial succeeds, and stops after 1 s, once all five count.

| Outcome | Score |
|---|---|
| The robot does nothing | 0. The loader takes the preloaded blank at its first visit and finds the nest empty at the next five |
| Every visit after the first finds one supplied blank, and the arms never touch | 1.0 |
| A visit finds the nest empty, or the right hand still at the nest | That visit is lost. Blanks are capped at the remaining visits |
| A blank is set on an occupied nest | 0 |
| The arms touch at any time | 0 |
| A blank is put into the chute some other way | Nothing. It was never taken from the nest by the loader |

Measured on a real cell with simple tools, from the top camera video (or a phone video from above):

| Check | Tool |
|---|---|
| At each visit, when the loader's fingers close: one supplied blank in the pocket; the right gripper's tip 8 cm or more from the nest centre | Video frame, and a tape mark 8 cm round the nest |
| Never two blanks in the pocket at once | Video |
| The arms never touched | Video. Both arms log joint currents, and a contact shows as a spike |
| Blanks in the chute at the end (resting inside its walls, on the floor or on other blanks) | Count them |

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, run from this folder. The layout is fixed, so
always pass `-E object_jitter=0`. The seed sets the loader's timing.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Keep the infeed nest stocked with one blank at a time from your tray, so the other arm finds a blank there every time it comes for one." \
  --scorer task_score --max-steps 2000 --seed 0 --store-frames --log-dir ~/yam-eval/restock-robot-station
```

The run limit is 2,000 steps (100 s at 20 Hz). The last close of any seed is at most 86.5 s, so the last blank is in
the chute by 90 s. A trial takes about 7 min (scripted) to 9 min (Dream-1) of wall time on a shared GPU, Isaac start-up included.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "..." --scorer task_score --max-steps 2000 --seed 0 --store-frames
```

`yam_scripted` runs `feed_demo.Controller` closed loop. It reads only the observation:

- **Top camera:** a 3 x 3 pixel window over the nest's pocket, projected from the station's calibrated top camera.
  A blank standing in the pocket reads about 225 (grey level). The empty pocket reads about 30, and the loader's
  black gripper above it about 65.
- **Joint positions:** the left arm's six joints, through the URDF kinematics, give where the loader's gripper is.

It knows the start layout, as every `yam_scripted` demo does. It never knows the loader's timing and never reads
simulator state. For each blank, it takes the next blank from the tray and waits with it 15 cm to the right of the
nest. When the nest has looked empty for 0.3 s and the loader's gripper is more than 15 cm from the nest, it sets the
blank down in the pocket. On the way down it steers its measured tool point, from its own joint readings, onto the
pocket centre: the compliant arm otherwise lands up to 2 cm off, and the blank catches on the pocket wall. It then
lets go and leaves.

| Run | Score |
|---|---|
| Demo, seed 0 | **1.0** (success at 74 s; all five visits after the first found a supplied blank) |
| Demo, seed 4 | **1.0** (success at 79 s) |
| Demo, seed 10 | **1.0** (success at 71 s) |
| `DEMO_CHEAT=idle`: never moves (seed 0) | **0**: the loader took the preloaded blank at its first visit and found the nest empty at the other five |
| `DEMO_CHEAT=timed`: sets a blank down every 13 s (the mean interval), without looking (seed 4) | **0**: at 22 s it reached in while the loader was at the nest and the arms touched |
| `DEMO_CHEAT=rush`: watches the nest but not the loader (seed 0) | **0**: it saw the loader's black gripper over the nest as "empty" and the arms touched at 6 s |
| Dream-1, seed 0 | **0**: 2,000 steps. The right arm hovered over the blank tray and never closed on a blank. The loader took the preloaded blank and then found the nest empty five times |

Unit tests (no Isaac Sim): `pytest test_restock_robot_station.py`. They cover the schedule, holding still (0), the
preloaded blank (never counts), a missed visit, double loading, blanks put straight in the chute, the hand left at
the nest, and arm contact.

## Fidelity

Approximate:

- The blanks are rigid cylinders, and the pad friction is rubber-like (static 1.2). The gripper pinches with up to
  50 N, which is far more than a 76 g blank needs.
- The nest, tray and chute are fixed boxes. The real nest's pocket is milled into one block, while here it is four
  walls on the table. Both are the same shape.
- Arm links collide as convex hulls, which are slightly larger than the real links, so a near miss of under about
  1 cm can count as a touch. Contact between any right-arm link and any left-arm link is read from PhysX at every
  physics step (240 Hz).
- The loader follows its program by inverse kinematics on the URDF, open loop. It does not sense whether it holds a
  blank, so it closes and goes to the chute even when the nest is empty, as a simple real loader without a part
  sensor would.
- The controller uses the manufacturer's joint gains, gravity compensation and motor torque and speed limits
  (i2rt yam_v1.yml).

## Real-world trial

Use two I2RT YAM arms on one table, 61 cm apart, with the objects listed above. Play the loader's program for the
trial's seed on the left arm. No person is in the cell. An operator stands outside both arms' reach, with the
e-stop of each arm in hand. Each YAM's joint torque is low: 27 N m peak at the shoulder, and 50 N at the gripper.
Run both with the joint current limit set to stop on contact. The blanks are deburred and weigh 76 g, so a dropped
blank cannot hurt anyone outside the reach envelope. Reset: put the five blanks back on their tray marks, put one
blank in the nest and empty the chute.

## Files

| File | What |
|---|---|
| `task.py` | The scene, the instruction, the scorer hooks and the demo hook |
| `cellkit.py` | The loader's program (seeded schedule, waypoints), the cell monitor (scoring), and the hooks into yam_arena: the left-arm override, arm-to-arm contact at 240 Hz, and the closed-loop demo |
| `feed_demo.py` | The closed-loop scripted demo and its loophole variants |
| `test_restock_robot_station.py` | Unit tests for the scorer and the schedule |

No third-party assets: every object is built from primitives in `task.py`. The robot, the station and the cameras
come from yam_arena (YAM URDF and meshes from I2RT, MIT).

## Credit

The idea for this task was first proposed in robocurve/benchmark-tasks#441 (TK-TT776S). The code, models and text here are an independent implementation.
