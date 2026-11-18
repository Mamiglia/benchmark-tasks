# Couple a trailer's 7-way electrical plug: lid held up, keyed plug pushed home, lid let down to catch it

Instruction: *"Plug the trailer cable into the 7-way socket on the trailer front and leave it latched. Then take both
grippers away."*

Category: **Impact on jobs**. Heavy and tractor-trailer truck drivers (about 2.1 million in the US) couple trailers
as routine work. O*NET 53-3032.00 lists the task "Couple or uncouple trailers by changing trailer jack positions,
connecting or disconnecting air or electrical lines, or manipulating fifth-wheel locks". The electrical line is the
SAE J560 7-way cable. Drivers plug it into the socket on the trailer's front wall at every coupling, by hand, as done
here.

## Why this task

The robot does the job the way a driver does it, with the real connector pair: a zinc J560 socket with a sprung lid
and split pins, and the matching plug on a length of 7-way cable. The difficulty is physical:

- **Two hands on one coupling.** The lid is sprung shut over the socket mouth and opens outward, so the plug cannot
  push it open. One hand must lift the lid by its thumb tab and hold it about 100-115 degrees open, clear of the plug's
  path, while the other hand brings the plug in under it.
- **A keyed, blind push against the pins.** The plug lies on the table rolled 15-40 degrees off 12 o'clock (seeded).
  The robot must turn it until its key rib lines up with the slot at the top of the barrel. With the plug at the mouth,
  the robot cannot see the slot behind the plug body. Off by more than about 7 degrees, the rib hits the barrel rim.
  Lined up, the insert must go 30 mm home against the split pins' grip (20 N in the sim, rising over the last 15 mm).
- **Let the lid down in the right state.** Released, the lid's spring swings it down onto the plug. Its catch drops
  into the notch behind the plug's top ridge only if the plug is fully home (within 3 mm). If the plug is short, the
  catch stays on the ridge and the plug is not locked. If the lid is let go before the plug is in, it shuts on the mouth.

### Distinct from

- Board tasks: the nearest by setting is the trailer gladhand coupling (air couplings put face to face at 90 degrees,
  then turned to lock; no lid, no keyed push). The nearest by motion are a push-to-connect air fitting, a ferrule
  pushed into a push-in terminal, and a night light pushed through a sliding outlet cover (one hand, the blade tips
  carry the cover across). None holds a sprung lid back with one hand while the other makes a keyed push, and then
  uses the lid as the lock.
- External benchmarks: no trailer or 7-way task in BEHAVIOR-1K, LIBERO, RoboCasa or DataCenterBench. The nearest are
  generic plug and peg insertions (for example the RLBench and FurnitureBench insertion tasks). Those are one-handed,
  with no sprung lid and no catch.

## Design

All parts are full size. Positions are in metres from the midpoint between the two arm bases, on the table top
(x forward, y left, z up).

- **Trailer front wall.** An upright 18 mm plywood plate 30 x 30 cm, painted grey, face at x 0.536, centred on
  y 0.02, on a 30 x 10 cm foot. It stands for the trailer's front wall at the nose box.
- **Socket** (Phillips 15-720 class: SAE J560, zinc die-cast, split pins, sprung lid). It is bolted to the plate with
  its axis horizontal and pointing at the robot, 13 cm above the table, at (x 0.500 barrel mouth, y 0.02). The barrel
  is 30 mm deep to the pin face, with a 44 mm bore and the key slot at 12 o'clock. The lid is 64 x 75 mm. It is hinged
  on a pin 40 mm above the axis and 5 mm in front of the mouth, and its torsion spring holds it shut (0.12 N m shut,
  0.36 N m at 90 degrees: about 4-5 N at the thumb tab). Its stop is at 120 degrees. On its inner face near the lower
  edge is the catch lug. Its thumb tab sticks out 25 mm at the lower edge.
- **Plug** (Phillips 15-710 class, zinc die-cast). The insert is 42 mm across and 31 mm long, with a key rib on top.
  The body is 54 mm across and 16 cm long overall. On top of the body is a catch ridge from 35 to 100 mm behind the
  insert face. With the plug home, the lid's catch drops just behind the ridge's end. Mass 0.35 kg. A 1.2 m length of
  7-way cable runs from its rear to a hook on a post at (0.20, -0.45), 0.40 m up.
- **Start.** The plug lies in a pine V-block at (0.30, -0.13), with its insert toward the trailer. It is rolled about
  its axis so the key is 15-40 degrees toward the robot's left of 12 o'clock (whole degrees, from the seed:
  `PLUG_SEED`, default 0: 31 degrees; seeds 4 and 10: 39 degrees). The lid is shut. The arms start in the ready pose.

## Scoring

Score 0 if, at any time after the plug left the V-block, it fell back onto the table.

Otherwise, the highest that applies:

| Score | Condition | Real-trial check |
|---|---|---|
| 1.0 | at the end the plug is seated (insert within 3 mm of the pin face), the lid is down in the notch behind the plug's ridge (lid within 2.3 degrees of where it sits with its catch in the notch, i.e. below 88.5 degrees open), and no gripper holds the plug or the lid; held 1 s | paint mark on the plug body level with the barrel rim (ruler, within 3 mm); side photo shows the lid down with its catch behind the ridge; after the trial a 20 N pull on the cable (luggage scale) does not move the plug; grippers clear (video) |
| 0.6 | at the end the plug is seated and released, but the lid is not caught (held open, or resting on the ridge) | as above, lid not in the notch |
| 0.3 | at some point the insert went 15 mm or more into the barrel (key rib in its slot) | tape mark 15 mm from the insert face went inside the barrel rim (video) |
| 0.1 | at some point the lid was held 100 degrees or more open while the plug's insert face was on the socket axis within 1 cm of the mouth | video, side view |
| 0 | none of these | |

"Holds": a gripper closed to under 80 mm with its tool point within 6 cm of the plug body's grip zone, or within 4 cm
of the lid's thumb tab. A trial that keeps the lid held open never gets more than 0.6. A plug left short of home cannot
reach 1.0, because the lid then rests on the ridge.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
PLUG_SEED=0 inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Plug the trailer cable into the 7-way socket on the trailer front and leave it latched. Then take both grippers away." \
  --scorer task_score --max-steps 1800 --seed 0 --store-frames --log-dir ~/yam-eval/trailer-7way-plug-dream
```

`-E control_hz=20` is required: the score bookkeeping runs once per control step and assumes 20 Hz. `PLUG_SEED`
sets the plug's start roll (`plugmodel.roll_for`). `PLUG_DEBUG=1` prints the plug and lid state five times a second.

## Scripted demo

Same command with `--policy yam_scripted`. `yam_scripted` runs `plugdemo.Controller` closed loop (`partkit` hooks it
in when the task module defines `demo_controller`). It reads the simulator state the task keeps (plug pose, lid
angle), as scripted demos may; it does not move objects. The sequence:

1. The left hand pinches the lid's thumb tab from the left and swings the lid up (it reaches about 105 degrees
   against the spring). At the same time the right hand takes the plug from the V-block from above and lifts it.
2. The right hand turns the plug about its axis until the key is at 12 o'clock and brings it onto the socket axis
   4 cm in front of the mouth. It lines the plug up from the measured pose, correcting for the arm's sag with a
   rate-limited integral term, then pushes it home.
3. The left hand lets go of the tab; the lid swings down onto the plug and its catch drops into the notch. Both hands
   open and draw back.

Loophole checks, same command with `DEMO_CHEAT=naive` (never lifts the lid; pushes the plug at the shut lid),
`DEMO_CHEAT=holdlid` (seats the plug but keeps holding the lid open to the end) and `DEMO_CHEAT=early` (lifts the lid,
then lets it go before the plug is in).

## Checked in simulation

| Run | Score | What happened |
|---|---|---|
| Scripted demo, seeds 0, 4, 10 (key rolled 31, 39, 39 degrees) | 1.0, 1.0, 1.0 | lid up to about 105 degrees; plug turned to within 2 degrees of 12 o'clock, lined up and pushed home (insert 30 mm in); lid let go, it came down to 84-86 degrees with its catch in the notch; the plug moved back about 1 mm onto the catch when released; success held 1 s at about 415 steps |
| `DEMO_CHEAT=naive` (lid never lifted) | 0.0 | the plug stopped against the shut lid; the lid cannot be pushed open from the front |
| `DEMO_CHEAT=holdlid` (plug seated, lid kept held open) | 0.6 | seated and released, lid never caught |
| `DEMO_CHEAT=early` (lid let go before the plug was in) | 0.0 | the lid shut on the mouth and the plug stopped against it |
| Dream-1, seed 0, 1,800 steps | 0.0 | took the plug out of the V-block and moved it about near the block, never brought it to the socket or touched the lid, and set it down on the table (the plug-dropped rule) |

## Runtime and seeds

The scripted demo needs about 415 control steps (21 s at 20 Hz). The run limit is 1,800 steps (90 s).
A trial takes about 3 minutes of wall time on one data-centre GPU. Seeds change only the plug's start roll.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, the per-physics-step pin and cable forces, `score()` / `success()`, `demo_controller` |
| `plugmodel.py` | Geometry, the pin rule, the lid's catch angles and the score bookkeeping (NumPy only) |
| `partkit.py` | Multi-box parts, sprung child joints, and the hooks into yam_arena (our kit from earlier tasks) |
| `plugdemo.py` | The closed-loop scripted demo |
| `test_trailer_7way_plug.py` | Tests: pin rule, lid catch geometry, every scoring rule on handcrafted trajectories |
| `task.json` | Submission details |

`partkit.py` hooks yam_arena without modifying it: `yam_arena.scene.task_object` and `object_states`, the
embodiment's `_ensure_sim`, `reset` and `_task_step`, and `yam_arena.scripted.YamScriptedPolicy.reset` / `act`.

## Tests

```bash
python -m pytest -q test_trailer_7way_plug.py   # NumPy only; no GPU needed
```

## Real-world build

| Item | Product | Size / mass | Approx. price |
|---|---|---|---|
| Socket | Phillips Industries 15-720, 7-way trailer receptacle socket, SAE J560, reinforced zinc die-cast, split pins, spring-loaded lid, 2-hole mount (2-7/8 in centres) | about 80 x 80 mm flange, 2 in mounting hole | $15-30 |
| Plug | Phillips Industries 15-710, 7-way plug, SAE J560, zinc die-cast (or 15-730, the same plug with a spring cable guard) | about 54 mm across, 0.3 kg | $15-25 |
| Cable | 1.2 m (4 ft) of 7-conductor trailer cable (6/14 + 1/12 AWG), clamped in the plug's cable entry; no wiring needed | about 0.2 kg/m | $10-15 |
| Trailer wall | 18 mm plywood 300 x 300 mm, painted grey, 2 in (51 mm) hole saw hole with its centre 130 mm above the table and the socket bolted over it (two M6 bolts); a 300 x 100 mm plywood foot screwed to its back edge; two 75 mm steel corner braces; clamped to the table with its face 536 mm from the arms' midpoint | | $15 |
| V-block | pine 120 x 100 x 37 mm with a 90-degree V-groove 22 mm deep along its 120 mm length (groove bottom 15 mm above its base), taped to the table at (0.30, -0.13) | | $5 |
| Cable hook | 30 x 30 mm timber post 0.42 m tall clamped to the table at (0.20, -0.45), a cup hook 0.40 m up | | $5 |

Total about $65-95. Before the first trial, check the plug's catch notch. With the plug fully home, mark the body
level with the barrel rim, and put a 15 mm tape mark on the insert.

**Forces.** New split-pin J560 pairs are stiff for the first few couplings and ease after that. Couple and uncouple
the pair by hand 20 times before the trials, then measure the push: stand the trailer-wall plate on a kitchen or
bathroom scale and push the plug home along its axis. The sim uses 20 N. The trial is in range up to about 30 N. The
YAM grips the plug body with its two fingers (about 40 N each, so about 40 N of axial hold at a friction coefficient
of 0.5) and pushes along the arm. Lid: hook a luggage scale on the thumb tab; about 4-5 N (sim) is typical. Plug and
cable weigh about 0.35 kg in the hand.

**Safety.** The cable is not connected to any power, and nothing is wet, greased or sharp. The die-cast edges are
deburred. Put a rubber mat under the V-block and the socket in case the plug is dropped (it falls at most 15 cm).

## Simplifications

- The barrel and plug are 12-sided prisms (PhysX boxes), not round. The 1 mm radial clearance and the key rib (6 mm
  wide) in its slot (about 10 mm wide at the bore) allow about 7 degrees of roll. Sim sizes follow the J560 pattern;
  the real pair's exact dimensions are as bought.
- The split pins are not modelled as contacts. Once the insert is 15 mm in, a Coulomb friction force acts along the
  axis. It is capped at 0 rising to 20 N at full depth, and it opposes any axial motion. To overcome it the robot must
  push as hard as on the real pins. The pin face is a solid stop.
- The cable is not drawn. Its pull is a constant 1.5 N force toward the hook at (0.20, -0.45, 0.40 m up), applied at
  the plug's centre of mass.
- The lid is a rigid plate on a sprung hinge. Its catch, the plug's ridge and the notch are real colliders, so the
  lid locks the plug by contact. The 20 N pull check of the real trial is not simulated; the score uses the lid's
  angle (catch in the notch).
- The wall, socket, V-block and hook post are fixed.

## Assets and licences

No third-party assets in the task: every object is built from boxes in `task.py`. The robot is the standard yam_arena
YAM (I2RT YAM arm URDF and meshes, MIT licence, `yam_arena/assets/yam/LICENSE.i2rt`); the room and cameras are the
standard station scene.
