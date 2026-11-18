# Pot the object ball in the taped corner pocket: one hand is the bridge, the other strokes the cue

A tabletop pool table stands in front of the robot. On the cloth are the white cue ball and a yellow object ball. The
near-left corner pocket has white tape on its rails, and a 36 in cue lies on the near rail. The robot must pot the
yellow ball in the taped pocket with one strike of the cue on the white ball, as a pool player does: one hand makes the
bridge the cue slides through, and the other strokes it.

**Instruction:** `Pot the yellow ball in the corner pocket marked with white tape, striking only the white ball, once, with the cue.`

**Category:** Societal relatability. A pool shot is known at a glance from a ten-second clip. Most people have played
one in a bar, a games room or a student union, and know how hard it is to pot a ball cleanly.

## Why this task

- **The skill is a guided, bimanual tool strike.** The cue is long and is held at its balance point. The bridge hand
  must hold the shaft on the aim line while it slides. The stroking hand must drive the cue straight through the
  bridge at a set speed and hit the cue ball within a few millimetres of its centre, once. In the simulation, a tip
  4 mm off centre sends the cue ball 4 to 6 degrees off line, and the object ball then misses the pocket.
- **Aim is physical, not a reading.** For the object ball to go into the pocket, the cue ball must reach the
  ghost-ball point: one ball diameter behind the object ball, on the line from the pocket. Here that means a 32 degree
  cut. The cue must be laid on the line from the cue ball's centre through that point.
- **Pace is part of the task.** Strike too softly and the object ball stops short. Strike too hard and the cue ball
  follows into a pocket (a scratch). Following through too far brings the tip back onto the cue ball (a double hit).
  In the simulation, the demo pots the ball with cue speeds of 0.32 and 0.41 m/s at contact; at 0.65 m/s the
  cue ball follows into the far-left pocket. With an earlier, slower-tracking stroke, a 5 cm follow-through caught the
  cue ball again (two strikes, score 0).
- **It is not a push.** Pushing the cue ball with the tip in contact (a push shot) is a foul in pool and scores 0
  here. So does moving a ball by hand or touching the object ball with the cue.

### Distinct from

| Task | Its skill | Here |
|---|---|---|
| TK-AG3K3H, rack fifteen balls in a triangle (not accepted) | Pack balls in a frame, lift it off without disturbing them | A single impulsive strike through a bridge, with aim and pace |
| Tool-push tasks on the board (e.g. raking a strap or pushing a tote) | A slow push, held in contact | A strike: one contact of a few milliseconds; a held push is a foul |
| LIBERO, RoboCasa, BEHAVIOR-1K "push the object to X" | Quasi-static pushing with the gripper or a tool | Ball-to-ball transfer: the cue ball must be sent along a computed line at a set speed |

No public suite we know of (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR-1K, DataCenterBench, RoboDojo) has a cue sport.
Robot billiards exists only as research systems with a dedicated cue actuator, not as a two-handed task with a bridge.

## Scene

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left).

| Object | Size | Mass | Notes |
|---|---|---|---|
| Pool table, Hathaway Breakout 40 in (BG5026) | 102 x 55 cm outside, cloth 13 cm above the table top; playing field 91.6 x 44.6 cm | fixed (clamped) | Long side along y, centred at y = 0; playing field from x = 14.0 to 58.6. Blue cloth, 2.5 cm cushions, 3.5 cm black rails. Six pockets: 6 x 6 cm corner holes, 7 cm side holes |
| Cue ball | 38 mm (1.5 in) | 50 g | White, on a spot sticker at (25.0, 15.0) |
| Object ball | 38 mm | 50 g | Yellow (the 1 ball), on a spot sticker at (23.0, 32.5) |
| Cue | 36 in (914 mm): 11 mm tip, 13 mm shaft, 25 mm butt | 250 g | Lies on the near rail (rail centre x = 10.75), tip toward +y, its balance point (59.5 cm from the tip) at y = -25 |
| White tape | 19 mm tape, 4 cm on each rail | - | Either side of the near-left corner pocket (the target) |

The object ball is 18 cm from the cue ball, and the pocket is 12 cm beyond it. The cut is 32 degrees, so the cue ball
leaves straight along +y and meets the object ball off centre.

## Scoring

Latched over the whole trial, from the trial video:

| Score | Condition | Real-trial check |
|---|---|---|
| 1.0 | The yellow ball drops into the taped pocket, the white ball stays on the table, and the cue touched the white ball in exactly one strike | Video |
| 0.5 | The yellow ball drops into another pocket, or comes to rest with its centre within 10 cm of the taped pocket's corner (where the two cushion lines meet) | Video; ruler from the corner |
| 0.25 | The white ball touched the yellow ball | Video |
| cap 0.25 | Scratch: the white ball goes into a pocket or off the table | Video |
| 0 | No strike; two or more strikes (contacts more than 30 ms apart); a push (one cue contact lasting over 70 ms, i.e. the tip visibly stays on the ball for 2 or more frames at 30 fps); a gripper or arm touched either ball; the cue touched the yellow ball; the white ball never touched the yellow ball | Video |

`task_score` gives the score at the end of the episode; a 1.0 shot that holds 1 s with the cue ball at rest ends it
early. The simulation reads contacts at 240 Hz (PhysX contact reports between each ball and the cue, the other ball,
and every link of both arms), and a ball counts as potted once its centre is below the cloth inside a pocket hole.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder. The layout is fixed: always
pass `-E object_jitter=0`.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pot the yellow ball in the corner pocket marked with white tape, striking only the white ball, once, with the cue." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/pool-bridge-shot-dream
```

The scripted demo: `--policy yam_scripted` (same command), about 430 control steps. `POOL_DEBUG=1` prints the demo's
state every 0.1 s; `POOL_LOG=file.npy` saves a 240 Hz log (balls, cue, contact forces, gripper tool points).
`DEMO_MODE=push|hand` plays the loophole checks below; `POOL_V`/`POOL_FOLLOW` the pace checks. `POOL_V` sets the demo's stroke speed (m/s, default 0.38).

**Run limit:** 1,500 steps (75 s at 20 Hz), about three times the scripted demo.

**Runtime:** about 3 minutes of wall time for a scripted trial on a shared RTX workstation (Isaac Sim start, reset,
520 steps); a 1,500-step policy run takes longer, depending on the policy's latency. Seeds do not change the layout
(it is fixed); seeds 0, 4 and 10 give the same demo result.

## Results

| Run | Score |
|---|---|
| Scripted demo, seeds 0, 4, 10 (427 steps; cue 0.41 m/s at contact) | 1.0, 1.0, 1.0 |
| Check `DEMO_MODE=push`: tip placed on the cue ball and rolled forward at 0.15 m/s | 0 (the cue went on to touch the yellow ball: foul; the 70 ms push rule itself is checked in the unit tests) |
| Check `DEMO_MODE=hand`: the left gripper sweeps the yellow ball into the taped pocket | 0 (yellow potted, but a robot touched it) |
| Check `POOL_V=0.6`: a hard stroke | 0.25 (yellow potted, the white ball scratched in the far-left pocket) |
| Check `POOL_V=0.5 POOL_FOLLOW=0.06`: a long, fast follow-through | 0.25 (yellow potted, white ball scratched) |
| Dream-1, seed 0, 1,500 steps | 0.0 (it moved both grippers over the table and toward the balls but never picked up the cue; the cue was knocked off the rail; no strike) |

## Simulation and fidelity

- **Table** (`poolkit.TableGeom`): one fixed body of box colliders. The slate leaves six pocket holes 6 cm deep; the
  cushions are boxes with restitution 0.75 and a 2.5 cm nose height; the rails are wood. Real cushions have an angled
  nose and pocket jaws; here they are square.
- **Balls**: solid spheres (inertia 2/5 m r^2), restitution 0.93 ball on ball, cloth friction 0.2 (ball on ball 0.04,
  by PhysX "multiply" combining). Cloth rolling resistance is modelled as body damping (0.45 1/s, linear and angular):
  a ball rolled at 0.5 m/s stops after about 1 m (v / 0.45). Measure your table (roll a ball off a 5 cm ramp, measure how far it
  goes) and adjust `BallParams` if it differs much.
- **Cue**: one rigid body: a 5.5 mm sphere tip (friction 3.0 x 0.2 = 0.6 on the ball, restitution 0.75), a 13 mm
  shaft (friction 0.35, so it slides through a bridge) and a 25 mm butt (friction 1.0, for the grip). The real cue
  tapers smoothly; here it is two steps. The grip uses an 8 mm torsional patch radius: a rubber pad on a round cue
  grips over a patch, not at a point.
- **Arms**: the YAM's own PD gains (i2rt), as yam_arena sets them. The achieved cue speed at contact in the demo is
  0.41 m/s.
- **What is not simulated**: the cloth's nap, ball spin effects beyond rolling friction (no swerve), chalk wear.

## Real-world build and trial

| Item | Product | Price |
|---|---|---|
| Table | Hathaway Breakout 40 in tabletop pool table, model BG5026, 40 x 21.5 in, 17 lb, MDF with cloth; comes with 16 balls, two 36 in cues, chalk, triangle | about $100 (Walmart) |
| Balls (if the table's set is not 1.5 in) | RACK Essential 1.5 in billiard ball set | about $18 |
| Spot stickers, white masking tape, 2 C-clamps | any | about $10 |

Total about $130. Clamp the table's legs to the robot table so it cannot slide. Measure the playing field and the
cloth height and set them in `TableGeom` if they differ from the values above; put the spot stickers at the positions
in the Scene table (measured from the midpoint between the two arm bases). Weigh one ball and set `BallParams.mass`.

**Forces.** The grip on the 25 mm butt is about 35 N per finger (the demo's command), under the YAM's roughly 40 N.
The bridge is a light pinch of about 3 N per finger. The stroke needs a cue speed of about 0.3 to 0.45 m/s at contact; the
YAM's shoulder joints (DM4340, about 5.5 rad/s) can move the hand well over 1 m/s.

**Safety.** The balls weigh 50 g and leave the cue at under 0.5 m/s; the cue tip is a rounded leather tip. Nothing is
sharp, wet or messy. Chalk the tip before the session, not during a trial, so no chalk dust reaches the grippers.
Keep people out of the cue's swing (the butt reaches about 30 cm behind the right hand).

**Reset.** Put both balls back on their spot stickers and the cue on the near rail, tip to the left, its balance point
(mark it with a pen) 25 cm right of the table's centre line.

## Third-party assets and licences

None: the table, balls and cue are built from primitives in `poolkit.py` and `task.py`. The YAM robot model and the
station scene come with yam_arena (the YAM description from I2RT, distributed with yam_arena under its own licence
terms; nothing from it is copied into this folder).

## Files

| File | What it does |
|---|---|
| `task.py` | The scene, the instruction, `score`/`success`, the demo hook |
| `poolkit.py` | Table, ball and cue bodies; the 240 Hz shot monitor; `shot_score`; hooks into yam_arena |
| `shot_demo.py` | The scripted demo (closed loop on the cue's pose) and the checks |
| `test_pool_bridge_shot.py` | Unit tests of the scoring rules and geometry: `python -m pytest -q` (no GPU) |
