# Steer the steel ball through a BRIO Labyrinth with its two knobs

A BRIO Labyrinth tilting maze stands in front of the arms. Each arm grips one of its two knobs. Turning a knob tilts
the playing board about one axis. The board is BRIO's 8-hole practice board, sold as BRIO Labyrinth Boards (34030)
and in BRIO Labyrinth with practice boards (34020). The robot must roll the steel ball from START along the numbered
path, past the holes, to FINISH.

## Changes since the last review

- TK-TP38XF: "Specify a layout of BRIO that is commercially available". The custom 20-hole serpentine is gone. The
  board is now BRIO's 8-hole practice board (BRIO Labyrinth Boards, article 34030, the simpler of its two boards;
  also in set 34020). Its partitions, holes and printed path are traced from BRIO's product photo (Board layout,
  below). The score is now (holes passed + 1 at FINISH) / 9, and the demo follows the printed path.

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
| Labyrinth | BRIO Labyrinth (article 34000): wooden box 35 x 30.5 x 9.4 cm, 1.25 kg; inside it a gimbal frame, a steel ball and a playing board about 29 x 24.5 cm. The board used is BRIO's 8-hole practice board (below), not the 60-hole board that ships with 34000. Two knobs (35 mm, plastic) on shafts, one centred on a long side, one centred on a short end, axes 5.5 cm above the box's underside | Box centred at (42, 0), turned 45 degrees: the long-side knob faces the left arm, the end knob faces the right arm |
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

**Board layout.** BRIO Labyrinth Boards (article 34030, "compatible with 34000") holds two boards that replace the
board of the 34000 game. The one used is the simpler board: 8 numbered holes, black plastic partitions and a printed
path from an arrow (START, top middle) to a star (FINISH, bottom middle). The same two boards come in BRIO
Labyrinth with practice boards (34020). Put it in the box printed side up, with the BRIO logo in the corner nearest
the side knob (left arm) and furthest from the end knob (right arm).

Where the numbers come from: the simulated layout (`labmodel.py`: 31 partitions, 8 holes, the path as 63 points)
is traced from BRIO's own product photo of 34030 (the pack shot of the two boards on retailer pages). The photo was
rectified to the board's outline, taken as 290 x 245 mm (the 34000 board's size, which 34030 replaces). Hole
diameter as measured: 14.4 mm. Partitions are modelled as 6 mm wide and 10 mm high. Tolerance: about +-3 mm on
positions (photo resolution about 0.8 mm per pixel, and the partitions' height seen at an angle), plus any error in
the 290 x 245 mm outline. Measure the delivered board before the first trial. The path runs 1.39 m, passes every
hole 12 mm or more away, and leaves the ball at least 0.8 mm clear of every partition (unit test).

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Turn the labyrinth's two knobs to roll the steel ball from START along the numbered path, past every hole to FINISH, without letting it drop into a hole." \
  --scorer task_score --max-steps 2400 --seed 0 --store-frames --log-dir ~/yam-eval/labyrinth-dream
```

`--policy yam_scripted` plays the scripted demo. `LAB_DEBUG=1` prints the ball's position, both knob angles and
tilts, the holes passed and any foul every 0.2 s.

## Scoring

- **Foul (latched for the whole trial): 0.** Any gripper finger or tool point inside the box (within its walls,
  below its rim + 1 cm) or within 1 cm of the ball; the box lifted 1 cm, tipped 5 degrees or slid 5 cm on the riser.
  The ball must be moved only by tilting the board through the knobs.
- **Otherwise: (holes passed + 1 at FINISH) / 9.** A hole is passed once the ball's centre has crossed the line
  across the path through that hole (simulation: the ball's furthest point along the printed path's centre line
  beyond the hole's point; the ball's place on the path is followed only within 4 cm of its last place, so it
  never jumps across a partition). It is latched: rolling back loses nothing. When the ball drops into a hole (its
  centre within the hole's 7.2 mm radius), progress stops there; the score keeps the holes passed before it. FINISH
  is the star: the ball within 1 cm of the path's end there. 1.0 = all 8 holes passed and at FINISH.

### Real trial

Run on the same BRIO 34030 8-hole board: score = (highest-numbered hole the ball has rolled past + 1 if it reaches
the star) / 9, read from the trial video (top camera) against the numbers printed beside the holes. A ball that drops ends
the trial. Fouls from the video: a gripper reaches inside the box or touches the ball, the box is lifted off the
riser (any gap visible), tipped or slid more than 5 cm (tape marks on the riser at its corners).

## Scripted demo

`labdemo.py`, played by `yam_scripted`:

1. **Grip the knobs.** Each arm brings its gripper level and pointing along its knob's shaft, 7 cm out, moves in
   along the shaft and closes on the knob's rim (about 15 N). The right arm holds the end knob, the left arm the
   side knob. From then on only each wrist's roll joint moves: rolling the wrist turns the knob.
2. **Steer.** Every control step (20 Hz) it reads the ball's position and velocity on the board and both knob
   angles (simulator state; a scripted demo may read it, a person reads the same by eye) and computes the ball
   acceleration that pulls it toward a point 25 mm ahead on its planned line at 4 cm/s:
   `a = 3 (target - p) + 2.5 (v_line - v)`, at most 0.25 m/s^2. The planned line is the printed path, which passes each
   hole 12 mm or more away. Each tilt is the one that gives that acceleration
   (`a = 5/7 g sin(tilt)` for a rolling ball), each knob angle is 25 times its tilt, and each wrist roll is set to
   turn its knob there, plus an integral term that removes the knob's lag behind the wrist. Both hands correct at
   the same time throughout; in each turn of the path both knobs move.
3. After FINISH it levels the board and opens both hands.

The demo is closed loop: nothing in it is timed to the layout except the approach to the knobs. It never touches
the ball or the board.

### Checked in simulation

| Run | Policy | Result | Score |
|---|---|---|---|
| Demo, seeds 0, 4, 10 | `yam_scripted` | Grips both knobs at 5.2 s, steers the ball along the printed path past all 8 holes, at FINISH at 28.2 s (about 575 steps) | 1.0, 1.0, 1.0 |
| Contrast `DEMO_CHEAT=none` | grips the knobs, never turns them | The ball stays in the START channel (it drifts 5 mm in 80 s) | 0.0 |
| Contrast `DEMO_CHEAT=press` | right hand on its knob, left fingers press the board's corner down inside the box | Left hand inside the box on the board (foul); the ball rolls back into the dead end beside START and passes no hole | 0.0 |
| Dream-1, seed 0 | `dream`, 2,400 steps | Raises both arms toward the box but never reaches or grips a knob; the board stays level and the ball drifts 27 mm along the START channel, short of hole 1 | 0.0 |

The demo video shows the top camera (rendered at 1280 x 704) with, on the right, a 2x crop of the same frame
around the labyrinth, so the 12.7 mm ball can be seen. Seeds only matter through object jitter, which is 0 for this task (one fixed layout), so the three demo seeds run
the same scene.

## Simulation fidelity

- **Mechanism.** The box, gimbal frame, board and both knobs are one PhysX articulation standing loose on the riser.
  Every physics step (240 Hz) each knob's measured angle sets its tilt joint's position target (1 degree per 25
  degrees, +-4 degrees); the tilt joints are stiff position drives (400 N m/rad). The real string-and-spring
  linkage also has some slip, delay and backlash (Gaber et al., arXiv:2406.08650); these are not modelled, which
  makes the simulated game somewhat easier than the real one.
- **Knobs.** Each knob turns on its shaft against a stick-slip friction of 0.04 N m (chosen, not measured: a
  fingertip turn). The gripper turns it by friction on its 35 mm rim.
- **Ball.** A 12.7 mm, 8.4 g steel sphere with friction 0.3 on wood, light rolling resistance (angular damping
  0.15, chosen). It rolls at 5/7 g sin(tilt), as a real ball does.
- **Holes.** The board is a solid collider; holes are a rule. When the ball's centre comes within a hole's 7.2 mm
  radius it has dropped: it is moved to the box floor under that hole and progress stops.
- **Board.** BRIO 34030's 8-hole board, traced from a photo (+-3 mm, Board layout above). Partitions are boxes;
  the board's edge is a 6 mm raised frame just outside the board. The printed path is drawn as dots; the hole
  numbers, arrow and star are not drawn (green START and red FINISH marks stand for the arrow and the star).

## Runtime

On the shared RTX workstation used, a scripted demo trial (Isaac Sim start, reset, about 500 control steps)
takes about 5 minutes of wall time; a 2,400-step Dream-1 run takes about an hour. The demo reaches FINISH 23 s
after gripping (about 575 steps in all), well inside the 2,400-step limit.

## Tests

```bash
pytest test_labyrinth_two_knobs.py      # in the yam_arena environment; no GPU needed (about a second)
```

## Real-world build and safety

| Item | Part | Approx. price |
|---|---|---|
| Labyrinth | BRIO Labyrinth, article 34000 (35 x 30.5 x 9.4 cm, 1.25 kg, two steel balls included) | $55 |
| Board | BRIO Labyrinth Boards, article 34030 (two boards for the 34000; the 8-hole one is used). Or buy BRIO Labyrinth with practice boards, 34020, in place of 34000 and 34030 | $30 |
| Riser | 18 mm plywood: a top 37 x 32 cm and two side walls 37 x 7.7 cm under its long edges, glued and screwed (95 mm high in all); or any rigid flat box of the same height | $15 |
| Tape, phone | Masking tape for corner marks; any phone inclinometer app | - |

Total about $100.

**Before the first trial** measure on the delivered game: the knob-axis height above the box's underside (make the
riser 15 cm minus it); the gearing (put the phone on the board, turn one knob 90 degrees from level, read the tilt:
ratio = 90 / tilt); the board's travel each way. Enter the ratio as `RATIO` in `labmodel.py` if it differs from 25.

**Forces.** Turning a labyrinth knob takes a light fingertip torque. Measure it with a string wound twice round
the knob and a luggage scale: we expect under 0.1 N m (under 6 N on the 17.5 mm rim). The YAM's wrist roll gives
7 N m, and a 15 N squeeze with rubber pads holds the knob's rim with a large margin. The game weighs 1.25 kg and is
never lifted.

**Safety.** Nothing is sharp, hot, liquid or heavy. The ball stays inside the box (a dropped ball comes out at the
ball return). People stay outside the arms' reach during the trial, as in any YAM trial; the game is reset by hand
between trials with the arms stopped.

## Assets and licences

No third-party meshes or scans: the labyrinth, its board, knobs, riser and ball are primitives defined in
`labkit.py` and `task.py`. The robot, room and cameras are the program's yam_arena embodiment (yam-isaac-lab-arena,
the program's SDK, not part of this folder); its YAM arm and gripper model comes from i2rt
(github.com/i2rt-robotics/i2rt) under the MIT licence. "BRIO" and "Labyrinth" name the real product used in the
trial; no BRIO artwork, logo or design files are included.
