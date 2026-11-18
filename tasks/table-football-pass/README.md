# Table football: pass from the 5-bar to the 3-bar, trap the ball, and score

A tabletop football (foosball) table is clamped to the bench with its handles toward the arms. The left arm holds
the red midfield rod (5-bar), the right arm the red attack rod (3-bar). The robot must pass the ball from the 5-bar
to the 3-bar, stop it there, and shoot it into the far goal. The blue team's rods are clamped, figures hanging
straight down.

**Instruction:** `Using only the two rod handles in front of you, pass the ball from the midfield rod to the attack rod, stop it there, and shoot it into the far goal. Do not spin a rod more than one full turn.`

## Why this task

- **Recognisable at a glance.** Table football is in bars, student unions, youth clubs and offices. Most people
  have played it. A ten-second clip of two arms on the handles, a pass, a stop and a goal needs no explanation.
- **The skill is timed, dynamic control of a free ball through two rods, one per hand.** The robot does not touch
  the ball. It acts only through a rod that slides (the arm moves along the rod) and turns (the wrist rolls). Each
  of the three steps needs a different control:
  - **Pass:** line a 5-bar figure up behind the ball, then swing it back and strike so that the ball goes through
    the 4 cm gap between two blue figures to the 3-bar.
  - **Trap:** the 3-bar figures are lifted so that the ball rolls under them. The foot is then dropped onto the
    ball as it passes, about 5 cm in front of the rod, to pin it against the playfield. At 0.3 m/s the ball moves
    1.5 cm in 50 ms, so the drop must start before the ball arrives. Too early and the foot blocks the ball, which
    bounces back. Too late and the ball is gone.
  - **Shot:** lift the foot off the ball, step it sideways, swing it down behind the ball, slide it back onto the
    ball's lane, and strike. The shot has to pass the blue 2-bar and the goalie.
- **The difficulty is physical.** Seeing the ball is easy. The hard part is timing, and the coupling of slide and
  turn on the same handle.

Category: **Societal relatability.**

### Distinct from

- **Board tasks.** No board task plays table football or moves a free ball with a rod. The nearest we found:
  - "Steer the steel ball through a BRIO Labyrinth with its two knobs": this is continuous, slow tilt control of a
    rolling ball. Here the robot strikes, catches and strikes again.
  - "Rack fifteen pool balls tight on the foot spot": the balls are placed by hand.
  - Ball-catching tasks with a ramp or tray.
- **External benchmarks.** LIBERO, RoboCasa, BEHAVIOR-1K, SimplerEnv and DataCenterBench have no table-football or
  rod-game task. The nearest skill in those suites is pushing an object to a goal with the gripper. Here the gripper
  never touches the ball. Robot table football exists only as research rigs with motors on the rods (for example
  KiRo, Univ. of Freiburg): no hand grips a handle in those.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: objects, instruction, scorer, the demo hook |
| `tfmodel.py` | Pure Python: the table's geometry, figure-ball contact, goal and foul checks, the latched progress and score |
| `tfkit.py` | The Isaac Sim side: the table articulation (two rods, each on a slide and a turn joint), the ball, bushing friction, the per-physics-step progress |
| `tfdemo.py` | The closed-loop scripted demo and the contrast runs |
| `test_table_football_pass.py` | Unit tests (no Isaac Sim needed) |

## Setup

Positions are in cm from the midpoint between the two arm bases, on the bench top (x forward, y left, z up). There
is one fixed layout, with no randomisation.

| Object | Size, material | Where |
|---|---|---|
| Table | Sport Squad FX40 40 in tabletop football table (JOOLA; 40 x 20 x 8 in, about 102 x 51 x 20 cm, 7 kg, MDF cabinet, laminate playfield, chrome steel telescopic rods, 8 rods with 1-2-5-3 figures per side). As modelled: playfield 97 x 47 cm, 5 cm above the bench; rods 11.5 cm apart, axes 8 cm above the playfield; figure soles 3 mm above it; feet 22 mm wide, 24 mm deep; 5-bar figures 8.5 cm apart (travel +-5 cm), 3-bar 13 cm apart (+-6.5 cm), 2-bar 24 cm apart; goal mouths 15 x 6.5 cm | Long side toward the arms. The inner face of the near wall is at x 42, the far wall's at x 89; the playfield centre is at y 5.75. Two C-clamps hold the cabinet's near corners to the bench |
| Red rods (robot) | Handles on the near side. Goalie (y 46), 2-bar (y 34.5), **5-bar (y 11.5, left arm)**, **3-bar (y -11.5, right arm)**. Rubber handles, about 28 mm across and 8 cm long, centre 11.5 cm outside the near wall with the rod centred | 5-bar and 3-bar centred in their travel, figures hanging straight down. Goalie and 2-bar hang free and are not used |
| Blue rods (opponent) | 3-bar (y 23), 5-bar (y 0), 2-bar (y -23), goalie (y -34.5). Handles on the far side | Each rod centred in its travel, figures straight down, held by a 2 in spring clamp on the rod just outside the far wall, its handle pressed against the wall so the rod can neither turn nor slide |
| Ball | The table's hard plastic ball (modelled 31 mm, 12 g) | At rest on a marked spot (a 6 mm dot of white paint pen) 3.5 cm in front of the red 5-bar's axis, 4.25 cm toward the far wall from the playfield's centre line (x 69.75, y 8). This lane runs between two blue 5-bar figures and past the blue goalie's right side |

The goal to score in is at the right-hand end as seen by the arms (y -). The red 5-bar and 3-bar are the same team's
adjacent rods, with the blue 5-bar between them, as on any table.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Using only the two rod handles in front of you, pass the ball from the midfield rod to the attack rod, stop it there, and shoot it into the far goal. Do not spin a rod more than one full turn." \
  --scorer task_score --max-steps 1200 --seed 0 --store-frames --log-dir ~/yam-eval/table-football-dream
```

`--policy yam_scripted` plays the scripted demo. `TF_DEBUG=1` prints the ball's position and velocity, both rods'
slide and angle, the stage and any foul every 0.1 s. `DEMO_CHEAT=direct|hand|through` plays the contrast runs.

## Scoring

The stages are latched over the whole trial; the score is the highest stage reached.

| Score | Stage | Simulation | Real trial (top-camera video) |
|---|---|---|---|
| 0.1 | Passed | A red 5-bar figure touched the ball, then the ball's centre crossed the blue 5-bar's axis | The ball, struck by a red 5-bar figure, passes the blue 5-bar rod |
| 0.3 | Received | Then a red 3-bar figure touched it (within 3 mm) | A red 3-bar figure touches the ball |
| 0.5 | Trapped | Then the ball stayed within 3 mm of one spot while touching a red 3-bar figure for 1 s | The ball stops against or under a red 3-bar figure and does not move for 1 s (count 20 frames at 20 fps) |
| 0.7 | Goal, not trapped | Ball's centre past the goal line inside the mouth, last touched by the red 3-bar, without a trap first | The ball goes into the far goal off a red 3-bar figure, with no 1 s stop first |
| 1.0 | Goal after the trap | The same, after the trap | Trap, then the ball goes into the far goal off a red 3-bar figure |
| 0 | Foul (latched) | A gripper finger or tool point within 1 cm of the ball, or inside the table (over the playfield, below the wall tops + 1 cm). The ball leaves the table. Own goal. A red rod turned more than one full turn from its start | A gripper touches the ball or reaches over the near wall into the table. The ball jumps out. The ball goes into the red goal. A rod makes more than one full turn: a strip of tape on each handle's end shows the turns |

A goal shot straight from the 5-bar, past a raised 3-bar, scores 0.1 (passed), not more. Touches by the blue
figures do not count either way. If the red 5-bar touches the ball again after the 3-bar, the 3-bar must touch it
again before a goal counts.

## Scripted demo

`tfdemo.py`, played by `yam_scripted`:

1. **Grip the handles.** Each gripper comes in over the handle, levels, points along its rod 7 cm short of the
   handle's end, moves in along the rod and closes on the rubber. From then on, moving the hand along the rod slides
   the rod (joint targets from IK on a 5 mm grid of slide positions), and rolling the wrist turns it.
2. **Line up.** The 5-bar slides 4.25 cm so that its middle figure is behind the ball. The 3-bar slides its middle
   figure onto the ball's lane and lifts its figures 64 degrees forward.
3. **Pass.** The 5-bar swings back 30 degrees, then strikes forward (wrist roll command +70 degrees). The ball
   leaves at about 0.3 m/s.
4. **Trap.** Every control step the right arm reads the ball's position and velocity (simulator state; a player
   watches the ball) and keeps the 3-bar's middle figure on the lane the ball will cross. When the ball, 0.18 s
   ahead, will be 4.5 cm in front of the rod, the foot drops onto it. The ball stops about 5 cm in front of the rod,
   pinned under the foot.
5. **Shoot.** After 1 s at rest (trapped), the foot lifts 3 degrees off the ball. The rod slides 4 cm sideways, the
   foot swings down beside the ball to behind the rod line, and the rod slides back onto the ball's lane. Then it
   strikes. The ball rolls past the blue 2-bar and goalie into the goal. From here on, an integral term on the
   wrist roll removes the slip of the handle in the grip.

### Checked in simulation

| Run | Policy | Result | Score |
|---|---|---|---|
| Demo, seeds 0, 4, 10 | `yam_scripted` | Grips at 5.7 s. Pass at 8.2 s, ball pinned by the 3-bar at 9.3 s, trapped at 10.3 s, shot at 12.2 s, goal at 13.6 s (about 290 steps) | 1.0, 1.0, 1.0 |
| Contrast `direct` | 5-bar shoots for the goal; the 3-bar lifts out of the way | The ball rolls through, under the 3-bar, past the goalie into the goal | 0.1 |
| Contrast `through` | Pass; the 3-bar stays lifted and swings without coming down | The ball rolls under it into the goal; the 3-bar never touches it | 0.1 |
| Contrast `hand` | The right gripper reaches over the near wall for the ball | Foul "hand inside the table" (it also knocks the 3-bar a full turn) | 0.0 |
| Dream-1, seed 0 | `dream`, 1,200 steps | Moves both arms out to the sides of the table and raises the grippers, but never reaches or grips a handle; the rods and the ball never move | 0.0 |

Seeds only matter through object jitter, which is 0 for this task (one fixed layout), so the three demo seeds run
the same scene. The demo video is the top camera, rendered at 1280 x 704.

## Simulation fidelity

- **Table.** The cabinet, walls, goals and the six unused rods are one fixed body. The red 5-bar and 3-bar each sit
  on a prismatic joint (the slide, with +-5 / +-6.5 cm bumper limits) carrying a revolute joint (the turn). Bushing
  friction is a stick-slip joint effort: 1.5 N along the rod, 0.004 N m in turn. These are chosen, not measured;
  measure the real values (below). The red goalie and 2-bar are fixed in the simulation. In a real trial they hang
  free, but the ball never reaches them.
- **Figures.** Box shapes (head, body, foot), 12 g each, rigidly on their rod. Real figures are moulded, with a
  rounded toe. The sole is 3 mm above the playfield, as on the real table.
- **Ball.** A 31 mm, 12 g sphere, restitution 0.5 on the figures and 0.3 on the playfield, rolling resistance as
  angular damping 0.3 (chosen).
- **Hand.** The gripper holds the 28 mm rubber handle by friction (rubber, 1.2). The handle slips a few degrees in
  the grip when the foot presses the ball; the demo corrects it with an integral term.
- **Goal.** The ball's centre past the goal line, inside the mouth. The real table returns the ball through a slot;
  the model has a closed pocket.

## Runtime

On the workstation used, a scripted demo trial (Isaac Sim start, reset, about 290 control steps) takes about 70 s of
wall time. The demo scores at 14 s of trial time (about 290 steps), inside the 1,200-step (60 s) limit. A 1,200-step
Dream-1 run took about 7 minutes.

## Tests

```bash
pytest test_table_football_pass.py      # in the yam_arena environment or plain Python with numpy; about 2 s
```

## Real-world build and safety

| Item | Part | Approx. price |
|---|---|---|
| Table | Sport Squad FX40 40 in table-top football table (SKU 685-SSF1001), two balls included | $80 |
| Rod clamps | 4 x 2 in spring clamps (for example Irwin Quick-Grip 222702, 2 in metal) | $12 |
| Bench clamps | 2 x 4 in C-clamps | $12 |
| Marks | White paint pen (ball spot), coloured tape (one strip on each red handle's end) | $5 |

Total about $110.

**Before the first trial,** measure the delivered table (playfield, rod spacing, rod height, figure spacing, ball
diameter) and check the ball spot: it is on the line through the gap next to the blue 5-bar's middle figure, 3.5 cm
in front of the red 5-bar's axis. Set the values in `tfmodel.py` if they differ.

**Forces.** Hook a luggage scale to a handle. Measure the force to slide each red rod, expected 1-3 N. Measure the
force at the handle's rim to turn it, expected under 0.05 N m (under 4 N at 14 mm). The YAM gives 7 N m of wrist
roll and about 40 N per finger, so the grip on the rubber handle has a large margin. The ball weighs about 12 g.
Nothing is lifted.

**Safety.** No sharp, hot or liquid parts. The ball is light plastic, the rods move only a few cm, and the strikes
are slow (under 1 m/s). People stay outside the arms' reach during a trial, as in any YAM trial. Between trials the
ball is reset by hand with the arms stopped.

## Assets and licences

There are no third-party meshes, scans or textures. The table, figures, rods, handles and ball are primitives
defined in `tfkit.py` and `tfmodel.py`. The robot, room and cameras are the program's yam_arena embodiment
(yam-isaac-lab-arena, the program's SDK, not part of this folder). Its YAM arm and gripper model comes from i2rt
(github.com/i2rt-robotics/i2rt) under the MIT licence. "Sport Squad FX40" names the real product used in the trial;
no product artwork or design files are included.
