# Steer the steel ball through a BRIO Labyrinth with its two knobs

A BRIO Labyrinth tilting maze stands in front of the arms. Each arm grips one of its two knobs. Turning a knob tilts
the playing board about one axis. The robot must roll the steel ball from START along the numbered path, past the
holes, to FINISH.

**Instruction:** `Turn the labyrinth's two knobs to roll the steel ball from START along the numbered path, past every hole to FINISH, without letting it drop into a hole.`

## Why this task

- **Recognisable at a glance.** The wooden labyrinth (BRIO, sold since 1946, more than 3 million sold) is one of
  the best-known table games. Most people have played it or watched someone play it. A ten-second clip of two arms
  on the knobs and a ball rolling between holes needs no explanation. People know from their own play why it is
  hard.
- **The skill is closed-loop control of a moving object through two coupled axes, with both hands at once.** About
  1 degree of tilt sets the ball rolling. At 25 degrees of knob per degree of tilt, a correction is a wrist turn of
  10 to 50 degrees, made while the ball moves at a few cm/s. The ball has to be slowed before each turn and steered
  past holes on either side of the line. Each knob controls one axis of the ball's acceleration, so in every turn
  and between holes the two hands correct together. Nothing is held still: the knobs are steered continuously, not
  set to a position. The ball is underactuated: the robot changes only its acceleration, so it must anticipate.
- **Not a toy-only skill.** The same control (steering a free rolling or sliding load by tilting its support with
  two hands) appears when people level a tray of drinks, a pan with an egg, or a board with a marble on it. The
  labyrinth turns it into a repeatable, numerically scored test with an off-the-shelf part. The board is the real
  game at full size, not a model of something larger; it is also an established control benchmark (ETH Zurich's
  "CyberRunner", 2023, played the same game with motors on the knob shafts).
- **The difficulty is physical.** Seeing the ball is easy; the hard part is turning two knobs by the right small
  amounts at the right time.

Category: **Societal relatability.**

### Distinct from

- **Board tasks.** No task on the board steers a rolling ball. The nearest found: "Send a tote down a roller
  conveyor to the far station without slamming the end stop" (one push sets a load moving; here the load is steered
  all the way, in two dimensions), "Tilt a long board with both arms to roll a ball into a cup" (not accepted: one
  tilt axis, no holes or path, a foam ball that could be grasped directly; here the board is a known product steered
  through knobs and the ball cannot be reached), and knob tasks such as "Dispense one serving of cereal from a
  twist-knob dispenser" (a knob turned to a stop, not steered against feedback).
- **External benchmarks.** RoboCasa's knob tasks (turn a stove burner on or off) set a knob to a fixed position.
  LIBERO and BEHAVIOR have no tilt-maze or ball-steering task. Ball-on-plate and labyrinth control (CyberRunner) are
  motor-in-the-loop research rigs with the knobs removed, not manipulation tasks: here the robot's own hands turn
  the knobs.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: layout, instruction, scorer, the demo hook |
| `labmodel.py` | Pure Python: the board's geometry, the path and its holes, the knob gearing, the progress tracker and the score |
| `labkit.py` | The Isaac Sim side: the labyrinth articulation and the ball, the per-physics-step gearing, hole and foul checks |
| `labdemo.py` | The closed-loop scripted demo |
| `test_labyrinth_two_knobs.py` | Unit tests (no Isaac Sim needed) |

## Setup

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left, z up). One fixed
layout; no randomisation.

| Object | Size, material | Where |
|---|---|---|
| Labyrinth | BRIO Labyrinth (article 34000): wooden box 35 x 30.5 x 9.4 cm, 1.25 kg; inside it a gimbal frame and a playing board about 29 x 24.5 cm with a wooden-walled path and holes, a steel ball. Two knobs (35 mm, plastic) on shafts, one centred on a long side, one centred on a short end, axes 5.5 cm above the box's underside | Box centred at (42, 0), turned 45 degrees: the long-side knob faces the left arm, the end knob faces the right arm |
| Riser | Plywood box riser, 37 x 32 cm top, 9.5 cm high (drawing below), or any rigid flat box of that height | Under the labyrinth, same centre and turn. Taped to the table |
| Ball | Steel ball 12.7 mm, 8.4 g (the game's own) | At START |

The riser puts the knob axes 15 cm above the table, where the YAM can hold its gripper level along a knob's shaft
(at the table's own height, 5.5 cm, it cannot: the wrist runs out of pitch). It changes nothing else about the
game; people also play it on a table in front of them, with the knobs at hand height.

**Knob gearing (as modelled; measure on the delivered game).** In the BRIO labyrinth each knob shaft winds a string
that pulls the board's edge against a spring. The simulation uses 1 degree of board tilt per 25 degrees of knob
turn (a 8 mm shaft pulling about 10 cm from the tilt axis) and +-4 degrees of board travel. To measure on the real
game: put a phone inclinometer app on the board, turn a knob 90 degrees from level and read the tilt; the ratio is
90 / tilt. The demo's controller reads its gain from this ratio; set `RATIO` in `labmodel.py` to the measured value.

**Path.** The simulated board is a simplified version of the real one: the same size, wall height, hole size and
ball, but a shorter path: a serpentine of four lanes (5.45 cm between walls) with 20 holes, five per lane, 3.5 cm
apart along the lane and 12 mm either side of its centre line, alternately. The real BRIO board has 60 numbered holes
along a longer, winding path. The real trial uses the real board (score below out of 60).

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Turn the labyrinth's two knobs to roll the steel ball from START along the numbered path, past every hole to FINISH, without letting it drop into a hole." \
  --scorer task_score --max-steps RUNLIMIT --seed 0 --store-frames --log-dir ~/yam-eval/labyrinth-dream
```

`--policy yam_scripted` plays the scripted demo. `LAB_DEBUG=1` prints the ball's position, both knob angles and
tilts, the holes passed and any foul every 0.2 s.

## Scoring

- **Foul (latched for the whole trial): 0.** Any gripper finger or tool point inside the box (within its walls,
  below its rim + 1 cm) or within 1 cm of the ball; the box lifted 1 cm, tipped 5 degrees or slid 5 cm on the riser.
  The ball must be moved only by tilting the board through the knobs.
- **Otherwise: holes passed / 20.** A hole is passed once the ball's centre has crossed the line across the path
  through that hole (simulation: the ball's furthest point along the path's centre line beyond the hole's
  point). It is latched: rolling back loses nothing. When the ball drops into a hole (its centre within the hole's
  9 mm radius), progress stops there; the score keeps the holes passed before it. Past hole 20 the ball is at FINISH:
  1.0.

### Real trial

Run on the real 60-hole board: score = highest-numbered hole the ball has rolled past / 60, read from the trial
video (top camera) against the numbers printed beside the holes; 1.0 at the board's finish. A ball that drops ends
the trial. Fouls from the video: a gripper reaches inside the box or touches the ball, the box is lifted off the
riser (any gap visible), tipped or slid more than 5 cm (tape marks on the riser at its corners).

## Scripted demo

`labdemo.py`, played by `yam_scripted`:

DEMOTEXT

## Simulation fidelity

FIDELITY

## Runtime

RUNTIME

## Tests

```bash
pytest test_labyrinth_two_knobs.py      # in the yam_arena environment; no GPU needed (about a second)
```

## Real-world build and safety

HARDWARE

## Assets and licences

No third-party meshes or scans: the labyrinth, its board, knobs, riser and ball are primitives defined in
`labkit.py` and `task.py`. The robot, room and cameras are the program's yam_arena embodiment (yam-isaac-lab-arena,
the program's SDK, not part of this folder); its YAM arm and gripper model comes from i2rt
(github.com/i2rt-robotics/i2rt) under the MIT licence. "BRIO" and "Labyrinth" name the real product used in the
trial; no BRIO artwork, logo or design files are included.
