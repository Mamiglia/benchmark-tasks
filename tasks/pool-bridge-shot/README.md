# Pot the object ball in the taped corner pocket: one hand is the bridge, the other strokes the cue

The robot stands at the near-left corner of a 7 ft bar pool table, at full size: regulation 2 1/4 in balls, real
cushions and rails, and a 48 in "shorty" cue of the kind bars keep for shots near a wall. On the cloth are the white
cue ball and a yellow object ball. The near-left corner pocket has white tape on its rails, and the cue lies on the
near rail. The robot must pot the yellow ball in the taped pocket with one strike of the cue on the white ball, as a
player does: one hand makes the bridge the cue slides through, and the other strokes it.

**Instruction:** `Pot the yellow ball in the corner pocket marked with white tape, striking only the white ball, once, with the cue.`

**Category:** Societal relatability. A pool shot is known at a glance from a ten-second clip. Most people have played
one in a bar, a games room or a student union, and know how hard it is to pot a ball cleanly.

## Changes since the last review

- TK-GEWR8P: "Scoring (not met): The stated 30 fps video check cannot reliably distinguish a contact over 70 ms or separate strikes more than 30 ms apart. What to change: For real-trial scoring, specify an unobstructed high-speed view of the cue tip and a frame-counting procedure that can resolve both the 30 ms strike-separation and 70 ms push thresholds. The current two-frame check at 30 fps does not measure a 70 ms contact reliably."
  Changed: the real-trial check is now a 240 fps side view of the tip and cue ball (named phone, placement, scale mark, light) with a frame count: a push is 17+ consecutive contact frames, two strikes are contact runs 8+ frames apart, matching the simulation's 240 Hz count. The scorer is unchanged.

## Why this task

- **The skill is a guided, bimanual tool strike.** The cue is 1.2 m long and weighs 510 g. It is held at its balance
  point, and the bridge hand must hold the shaft on the aim line while it slides. The stroking hand must drive it
  straight through the bridge at a set speed and hit the cue ball near its centre, once.
- **Aim is physical, not a reading.** For the object ball to go into the pocket, the cue ball must reach the
  ghost-ball point: one ball diameter behind the object ball, on the line from the pocket. Here that is a 35 degree
  cut, and the cue must be laid on the line from the cue ball's centre through that point.
- **Pace is part of the task.** Strike too softly and the object ball stops short. Strike too hard and the cue ball
  can run on into a pocket (a scratch). A cue kept on the ball while it rolls is a push.
- **Fouls are fouls.** Pushing the ball with the cue, touching a ball with a gripper or arm, or hitting the yellow
  ball with the cue scores 0.
- **Full size.** The table, balls and cue are the real ones, so the ball mass (170 g), the cushion and rail heights,
  the pocket size and the cue's length and weight are those a player meets.

### Distinct from

| Task | Its skill | Here |
|---|---|---|
| TK-AG3K3H, rack fifteen balls in a triangle (not accepted) | Pack balls in a frame, lift it off without disturbing them | A single impulsive strike through a bridge, with aim and pace |
| Tool-push tasks on the board (e.g. raking a strap or pushing a tote) | A slow push, held in contact | A strike: one contact of a few milliseconds; a held push is a foul |
| LIBERO, RoboCasa, BEHAVIOR-1K "push the object to X" | Quasi-static pushing with the gripper or a tool | Ball-to-ball transfer: the cue ball must be sent along a computed line at a set speed |

No public suite we know of (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR-1K, DataCenterBench, RoboDojo) has a cue sport.
Robot billiards exists only as research systems with a dedicated cue actuator, not as a two-handed task with a bridge.

## Scene

Positions in cm from the midpoint between the two arm bases, at the arm bases' mounting height (x forward, y left).

| Object | Size | Mass | Notes |
|---|---|---|---|
| Pool table, 7 ft bar table (39 x 78 in playing field) | Cloth 7 cm above the arm bases' mounting plane; 2 in rubber cushions, nose 36 mm above the cloth; 4 in wooden rails, 44 mm above the cloth; corner pocket opening 85 mm square at each field corner (a 4 3/4 in mouth across the jaws) | fixed | Long side along y. Near cushion nose at x = 29, left cushion nose at y = 65 (the near-left corner is the target pocket); the field runs to x = 128 and y = -133, beyond the cameras. Green cloth, brown rails |
| Cue ball | 57.15 mm (2 1/4 in) | 170 g | White, on a spot at (42.3, 35.9) |
| Object ball | 57.15 mm | 170 g | Yellow (the 1 ball), on a spot at (39.0, 52.6) |
| Cue, Action ACTO48 one-piece shorty | 48 in (1,219 mm), 12 mm tip, 13.6 mm shaft, 29 mm butt | 510 g (18 oz) | Lies on the near rail (rail centre x = 19), tip toward +y, its balance point (80 cm from the tip) at y = -55 |
| White tape | 6 cm on each rail | - | Either side of the near-left corner pocket (the target) |

The cue ball travels 12 cm to the ghost-ball point; the object ball is 10 cm from the middle of the pocket opening. The cut is 35 degrees.

## Scoring

Latched over the whole trial, from the trial video:

| Score | Condition | Real-trial check |
|---|---|---|
| 1.0 | The yellow ball drops into the taped pocket, the white ball stays on the table, and the cue touched the white ball in exactly one strike | Video |
| 0.5 | The yellow ball drops into another pocket, or comes to rest with its centre within 10 cm of the taped pocket's corner (where the two cushion lines meet) | Video; ruler from the corner |
| 0.25 | The white ball touched the yellow ball | Video |
| cap 0.25 | Scratch: the white ball goes into a pocket or off the table | Video |
| 0 | No strike; two or more strikes (contacts 30 ms or more apart); a push (one cue contact lasting over 70 ms); a gripper or arm touched either ball; the cue touched the yellow ball; the white ball never touched the yellow ball | Strikes and push: frame count in the 240 fps side view (below); the rest: video |

**Real-trial strike and push check.** A 30 fps video cannot time a 70 ms contact or a 30 ms gap, so the strike is
filmed at 240 fps (one frame = 4.17 ms):

- **Camera:** an iPhone 13 or later in Slo-mo at 1080p 240 fps (or any phone or camera with a true 240 fps mode;
  check with `ffprobe` that the file is 240 fps), on a mini tripod on the cloth at (67, 36): 25 cm from the cue ball,
  square to the cue line (the line runs along +y through x = 42.3), on the side away from the arms and out of both
  balls' paths. Lens 3 cm above the cloth (the balls' centre height), main 1x lens, focus and exposure locked on the
  cue ball. The view must show the tip and the cue ball against the cloth with nothing in between.
- **Scale:** a 10 mm band of white tape on the ferrule, just behind the tip. It must span at least 30 px in the
  frame (3 px per mm or better); move the camera closer if not.
- **Light:** a DC- or battery-powered LED panel (for example a Neewer 660) on the cue ball from the camera side.
  Mains lighting flickers at 100 or 120 Hz, which shows as banding at 240 fps.
- **Contact frame:** a frame with no cloth pixel visible between the tip's leather and the cue ball along the cue
  axis (a gap of 0 px, about 0.3 mm at the stated scale).
- **Count:** extract the frames (`ffmpeg -i shot.mov f/%04d.png`) and step through them from 10 frames before the
  cue ball moves to 10 frames after the tip stops. One run of consecutive contact frames is one contact.
  - **Push:** a run of 17 or more contact frames (17 x 4.17 = 71 ms, over 70 ms).
  - **Two strikes:** two runs separated by 8 or more non-contact frames (8 x 4.17 = 33 ms, at least 30 ms). Runs
    separated by 1 to 7 non-contact frames are one strike.
  - A clean strike lasts a few milliseconds, so it shows as 0 to 2 contact frames; with 0, the strike is the frame
    where the cue ball starts to move with the tip at it.

  This is the count the simulation makes: it reads contacts at the same 240 Hz, with the same 70 ms and 30 ms limits.

`task_score` gives the score at the end of the episode; a 1.0 shot that holds 1 s with the cue ball at rest ends it
early. The simulation reads contacts at 240 Hz (PhysX contact reports between each ball and the cue, the other ball,
and every link of both arms). A ball counts as potted once its centre has sunk a quarter of its radius below the
cloth level inside a pocket opening (it may come to rest on another ball in the pocket).

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder. The layout is fixed: always
pass `-E object_jitter=0`.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pot the yellow ball in the corner pocket marked with white tape, striking only the white ball, once, with the cue." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/pool-bridge-shot-dream
```

The scripted demo: `--policy yam_scripted` (same command). `POOL_DEBUG=1` prints the demo's state every 0.1 s;
`POOL_LOG=file.npy` saves a 240 Hz log (balls, cue, contact forces, gripper tool points). `DEMO_MODE=push|hand`
plays the loophole checks below; `POOL_V` sets the demo's stroke speed command (m/s, default 0.38).

**Run limit:** 1,500 steps (75 s at 20 Hz), about three times the scripted demo.

**Runtime:** about 3 minutes of wall time for a scripted trial on a shared RTX workstation (Isaac Sim start, reset,
about 540 control steps); a 1,500-step policy run takes longer, depending on the policy's latency. Seeds do not change the
layout (it is fixed); seeds 0, 4 and 10 give the same demo result.

## Results

| Run | Score |
|---|---|
| Scripted demo, seeds 0, 4, 10 (541 steps; stroke command 0.38 m/s, 0.48 m/s at contact) | 1.0, 1.0, 1.0 |
| Demo stroke commands 0.32 and 0.45 m/s | 1.0, 1.0 |
| Check `DEMO_MODE=push`: tip placed on the cue ball and rolled forward at 0.15 m/s | 0 (three cue contacts, and the cue reached the yellow ball: foul) |
| Check `DEMO_MODE=hand`: the left gripper sweeps the yellow ball into the taped pocket | 0 (yellow potted, but a robot touched it) |
| Hard stroke, `POOL_V=0.7` | 1.0 (at this short range a firm stroke still pots and the 35 degree cut keeps the white ball out of the pocket) |
| Dream-1, seed 0, 1,500 steps | 0.0 (the right gripper went to the cue on the rail and hovered over it, but never lifted it; no strike) |

`setup.png` is the top camera with the left arm lifted out of its view of the taped pocket (`DEMO_MODE=show`); trials
start with both arms in the ready pose.

## Simulation and fidelity

- **Table** (`poolkit.TableGeom`): one fixed body of box colliders for the corner region in reach and the rest of
  the 39 x 78 in field. The bed (slate and cloth) is a 7 cm slab on the station table with six square pocket
  openings that go down to the table top; the cushions are boxes with restitution 0.75 and a 36 mm nose; the rails are
  wood. Real cushions have an angled nose and pocket jaws, and real pockets have a liner and a ball return; here they
  are square openings.
- **Balls**: solid spheres (inertia 2/5 m r^2), 170 g, restitution 0.93 ball on ball, cloth friction 0.2 (ball on
  ball 0.04, by PhysX "multiply" combining). Cloth rolling resistance is modelled as body damping (0.45 1/s, linear
  and angular): a ball rolled at 0.5 m/s stops after about 1 m (v / 0.45).
- **Cue**: one rigid body, centre of mass at its balance point: a 6 mm sphere tip (friction 3.0 x 0.2 = 0.6 on the
  ball, restitution 0.75), a 13.6 mm shaft (friction 0.35, so it slides through a bridge) and a 29 mm butt (friction
  1.0, for the grip). The real cue tapers smoothly; here it is two steps. The grip uses a 10 mm torsional patch
  radius: a rubber pad on a round cue grips over a patch, not at a point.
- **Arms**: the YAM's own PD gains (i2rt), as yam_arena sets them.
- **Impact speeds**: at these slow speeds PhysX makes the tip impact nearly inelastic: the cue ball leaves at 0.6 to
  0.8 times the cue's speed, where a real centre hit gives about 1.3 times. So the simulated stroke is faster than a
  real one (demo commands 0.32 to 0.45 m/s, 0.40 to 0.57 m/s at contact, all pot the ball), and it stops 1 cm past
  the contact point so the cue does not stay on the slower ball (which the scorer would call a push). Pace on the
  real table differs and is part of what the real trial tests.
- **What is not simulated**: the cloth's nap, swerve, chalk wear.

## Feasibility: stroke speed and forces

The shot is short (12 cm to the ghost-ball point, 10 cm on to the pocket), so it needs a cue ball speed of only
about 0.25 to 0.4 m/s. A real cue ball leaves a centre hit at about 1.3 times the cue's speed, so a real cue needs
about 0.2 to 0.3 m/s at contact. The simulation needs more (0.40 to 0.57 m/s, see below), and the simulated arm,
with the YAM's own PD gains and the 510 g cue, reached it; the real arm's shoulder motors (DM4340, about 5.5 rad/s)
allow well over 1 m/s at the hand. Stroke speed is not the limit; control of it is. The grip on the
29 mm butt is about 37 N per finger, under the YAM's roughly 40 N; the cue weighs 510 g, under the 1.5 kg payload,
and is held at its balance point. The bridge is a light pinch of about 3 N per finger.

## Real-world trial

| Item | Product | Price |
|---|---|---|
| Table | A 7 ft bar or home pool table with a 39 x 78 in field. On site: a coin-op bar table (for example Valley Panther ZD-11, quote only). Cheapest new full-size option we found: Hathaway Maverick 7 ft (comes with 2 1/4 in balls) | about $750 (Hathaway Maverick); a bar's table costs nothing to use |
| Balls | The table's 2 1/4 in set, or Aramith Blue Dot cue ball plus any 1 ball | about $13 each |
| Cue | Action ACTO48 one-piece 48 in shorty cue, 18 oz, 12 mm tip | about $30 |
| Tape, spot stickers | any | about $5 |
| Strike camera | iPhone 13 or later (Slo-mo, 1080p 240 fps; any phone already owned with a true 240 fps mode), a mini tripod, a battery LED panel (for example Neewer 660) | about $40 for tripod and light if a phone is to hand |

This is the one part of the task over the usual ~$200: a full-size table. The honest options are to run the trial at
a bar or club table, or to buy a home 7 ft table. A tabletop toy table would make it a miniature.

**Placing the robot.** The arms must stand at the table's near rail as a player does: arm bases 7 cm below the
cloth, 32 cm back from the near cushion's nose, with the near-left corner pocket in front of the left arm (positions
above). A bar table's cloth is 29 to 31 in high, so the two arms go on a stand of that height minus 7 cm at the rail,
not on the usual rig table (which cannot go under the table). Measure the cloth height and pocket opening of the table
used and set `TableGeom` if they differ.

**Safety.** The balls leave the cue at under 0.5 m/s; the cue tip is a rounded leather tip. Nothing is sharp, wet or
messy. Chalk the tip before the session, not during a trial, so no chalk dust reaches the grippers. Keep people out of
the cue's butt (it reaches about 40 cm behind the right hand).

**Filming.** Besides the usual trial video, film every strike with the 240 fps side camera described under Scoring.

**Reset.** Take both balls out of the pocket or off the cloth, put them back on their spot stickers, and lay the cue on
the near rail, tip to the left, its balance point (mark it with a pen) at the marked place.

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
