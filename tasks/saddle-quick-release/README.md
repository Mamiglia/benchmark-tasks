# Raise a child's bike saddle with its quick-release

A red 12-inch children's balance bike stands on the table, front wheel in a floor stand, its left side toward the
robot's left arm. The saddle is low and turned 12 degrees out of line. A band of blue tape is wrapped round the seat
post just above the seat clamp, a quick-release collar whose red lever is folded down along the seat tube.

Instruction: `Raise the saddle until the bottom of the blue tape on the seat post is 4 cm above the seat clamp, with the saddle straight, and close the quick-release lever.`

## Why this task

Parents raise a child's saddle every few months as the child grows; bike shops, rental desks and school cycling
schemes do it many times a day. Everyone who has done it knows what makes it awkward:

- **The post is free in two directions at once.** Once the lever is open the post slides and turns in the frame,
  and with nothing holding it, it sinks back down under its own weight. One hand must hold the saddle at the right
  height and turn it in line with the bike, and keep it there.
- **The other hand closes an over-centre cam.** The lever gets stiff over its last stretch (about 30 N at the paddle
  here) and only stays shut once it is pushed past its dead point. Released short of it, the cam pushes it back open
  and the saddle drops. So the closing push has to be strong and complete while the first hand still holds the post
  steady against that push.
- **Two coupled hands, two precise targets.** Height to +-5 mm and alignment to +-5 degrees are set by the holding
  hand, and must survive the closing hand's push.

### Nearest existing tasks and why the ability differs

- Board, accepted: closing a swing-top bottle (TK-FFD5AU) and extending a suitcase handle (TK-28LP2A). The bottle's
  over-centre lever closes on a fixed seat (the stopper only has to be on the mouth); the hard part is the lever.
  Here the over-centre close fixes a part that is free in height and rotation and is being held in place by the
  other hand: the skill is setting and holding a free 2-degree-of-freedom part to a target while the other hand
  locks it, which neither task asks for. The suitcase handle locks by itself at fixed detents.
- Board, accepted: releasing a camera from a cam-clamped quick-release mount (TK-XDB5VZ). That task only opens a cam
  and lifts a part out; nothing is positioned or re-clamped.
- Board, not accepted: turning an angle grinder's guard and clamping it (TK-4VTXPU). Closest in spirit (open a clamp,
  set an angle, close it), but the guard stays where it is put and a single hand can do it; here the post sinks and
  turns the moment it is let go.
- External: no BEHAVIOR-1K, LIBERO, RoboCasa or RoboDojo task adjusts a clamped post. The nearest are articulated
  "open/close" lever tasks (RoboCasa's appliance levers, LIBERO "turn on the stove"), which move one joint to a stop
  with one arm.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the bike, the clamp's run-time rules, `score()` / `success()`, `demo_controller()` |
| `qrmodel.py` | Pure-Python geometry, the clamp model (`ClampParams`, `Clamp`, `paddle_force`) and the score (`score_state`) |
| `bikekit.py` | Multi-shape parts, the fixed-base articulation with chained joints, per-step joint efforts, and the hooks into yam_arena (adapted from an earlier task's kit) |
| `saddledemo.py` | The closed-loop scripted demo that `yam_scripted` plays |
| `test_saddle_quick_release.py` | Unit tests of the geometry, the clamp model and every scoring outcome (no Isaac Sim needed) |
| `task.json` | Submission details |

## Setup

Centimetres from the midpoint between the arm bases, x forward, y left, heights above the table top.

- **Bike:** Strider 12 Sport balance bike (red; 3.0 kg; 12 in wheels, tyres 30 cm across and 5.7 cm wide; seat
  height range 28-48 cm), about 54 cm wheelbase, seat tube leaning back 15 degrees. It stands along x with the front
  wheel away from the arms: rear hub (0, 0, 15), front hub (56, 0, 15), handlebar 42 up at x 49. Its left side (the
  lever side) faces +y.
- **Stand:** a slot-type floor bike stand round the bottom of the front wheel (30 x 16 cm base, slot walls 11 cm
  tall and 6.8 cm apart), on the table.
- **Seat clamp:** the bike's stock aluminium quick-release collar, top face centred at (25, 0, 24). Red lever on the
  bike's left side, cam pivot 2 cm behind the tube; 4.8 cm from the pivot to the middle of its paddle (2.0 x 1.6 cm).
  Closed (start): folded down along the seat tube. Open: swung back and up toward the arms (stop at 120 degrees).
- **Seat post and saddle:** the stock 22.2 mm steel post with its moulded saddle, 0.30 kg together; saddle 20 cm
  long, 12.5 cm wide at the rear, 5 cm wide at the nose, 3 cm thick. At the start 4.5 cm of post shows above the
  collar, the saddle top is 31.8 cm up (seat height 32 cm; 36 cm when raised), and the saddle is turned 12 degrees nose-left of the frame's line.
- **Tape:** blue painter's tape, a 12 mm band round the post, its lower edge level with the collar's top. The post's
  MIN INSERT line is 7.5 cm below that edge.
- **Robot:** arms in the ready pose, grippers open.

## Mechanisms (what the simulation does each 240 Hz physics step)

PhysX cannot model a cam clamping a tube, so `Clamp.step` in `qrmodel.py` turns the joint angles into joint efforts:

- **Lever stroke.** Free above 90 degrees. From 90 down to 22 degrees the cam pushes the paddle open with a force
  rising to 30 N; from 22 to 12 degrees it falls back to 0 (the dead point); below 12 degrees it pulls the lever shut
  with 5 N. So closing needs a push of up to 30 N at the paddle, and a lever let go above 12 degrees springs open.
- **Collar grip.** Below 30 degrees the collar grips the post where it is: a 15,000 N/m spring along the tube and a
  60 N m/rad spring about it (a 20 N push on the saddle moves it 1.3 mm). Above 30 degrees the post is loose:
  1.5 N sliding friction and 0.03 N m turning friction, so it sinks under its own 2.9 N weight along the tube.
- The post is a prismatic joint along the seat tube and a revolute joint about it, chained; the lever is a revolute
  joint. The bike is fixed.

## Scoring

Read at the end of the run (`SCORE_OVER = "end"`), with a ruler, a photo from above and by hand:

| Condition | Score |
|---|---|
| The post was pulled up past its MIN INSERT line (7.5 cm) at any time | 0 |
| Lever not folded within 12 degrees of the frame (short of its dead point), or a gripper within 3 cm of the post, saddle or lever, or the tape less than 10 mm above the collar | 0 |
| Otherwise | 0.3 |
| ... and the tape's lower edge 40 +- 5 mm above the collar's top (ruler) | +0.5 |
| ... and the saddle's centre line within 5 degrees of the frame's (photo from above, protractor) | +0.2 |

1.0 is success, held for 1 s. Real trial, after the run: push straight down on the saddle with 20 N (a kitchen scale
under the hand reading 2 kg). If the tape moves 2 mm or more the clamp is not holding: score 0. In simulation a
closed lever always holds (the spring above), so this check is implied.

## Unintended strategies and what stops them

- **Lift the saddle without opening the lever.** The collar holds the post: it does not move, score 0
  (`DEMO_CHEAT=naive`).
- **Close the lever part way and let go.** Above 12 degrees the cam springs it open, the post sinks, score 0
  (`DEMO_CHEAT=short`).
- **Keep holding the saddle or the lever at the end.** Hands-clear check: 0.
- **Only straighten the saddle.** No credit unless the post is at least 1 cm up.
- **Yank the post out and drop it back.** Past MIN INSERT at any time: 0 for the whole trial.

## Run a trial

From a yam_arena install (Isaac Lab-Arena, `inspect-robots`):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E device=cuda:0 \
      -E task=tasks/saddle-quick-release/task.py -E object_jitter=0 -E control_hz=20 \
      --scorer task_score --max-steps 2400 --seed 0 \
      --instruction "Raise the saddle until the bottom of the blue tape on the seat post is 4 cm above the seat clamp, with the saddle straight, and close the quick-release lever."

Use `--policy dream` for Dream-1. `DEMO_CHEAT=naive` or `DEMO_CHEAT=short` plays the loopholes; `SADDLE_DEBUG=1`
prints the state. The layout is fixed; the seed only changes the policy's randomness (seeds 0, 4, 10 tested).

## Scripted demo

`saddledemo.py`, closed loop at 20 Hz. It reads the post's rise and turn and the lever angle from the task's physics
hook. Right hand: pinches the saddle's right edge (top and bottom of the shell). Left hand: pinches the lever's
paddle across its width and swings it open to 105 degrees. Right hand: raises the post 40 mm along the seat tube and
turns it straight, servoing the measured rise and turn (the arm sags under the load, so it keeps a correction), and
holds. Left hand: pushes the lever shut through its dead point, then lets go. Right hand lets go; both arms back away.

## Verified in simulation

Run with the command above (`object_jitter=0`, 20 Hz, 320 x 176 cameras):

| Run | Seed | Score | Notes |
|---|---|---|---|
| Scripted demo | 0 | 1.0 | success after 417 steps (21 s) |
| Scripted demo | 4 | 1.0 | |
| Scripted demo | 10 | 1.0 | |
| `DEMO_CHEAT=naive` (lift without opening the lever) | 0 | 0.0 | post held by the collar |
| `DEMO_CHEAT=short` (lever shut only to 20 degrees, let go) | 0 | 0.0 | lever springs open, post sinks |
| Dream-1 | 0 | 0.0 | full 2,400 steps; reaches toward the saddle, never scores |

The demo's video is 640 x 352 (seed 0, 1.0).

## Runtime

About 2-4 minutes per trial in simulation on one GPU (Isaac Sim start-up included); the demo itself is 21 s of
simulated time. A real trial takes about 5 minutes including the reset.

## Tests

    pytest tasks/saddle-quick-release/test_saddle_quick_release.py

## Real-world build

| Item | What | Approx. price |
|---|---|---|
| Strider 12 Sport balance bike | 3.0 kg, aluminium quick-release seat collar and standard seat post included | $150 |
| Slot-type floor bike stand | any wheel-slot stand that takes a 12 in, 2.25 in wide tyre (e.g. a steel kids' bike floor stand) | $20 |
| Blue painter's tape | 12 mm band round the post; a second strip on the collar for reset marks | $5 |
| Kitchen scale, ruler, protractor or phone | scoring | (lab) |

Total about $175. No custom parts.

**Setting the cam (once per session, 5 minutes).** With the lever open, turn the adjusting nut on the right side so
that closing the lever takes about 30 N at the paddle (hook a luggage scale on the paddle and pull it shut; the peak
reading should be 2.5-3.5 kg), and so that, with the lever open and the saddle let go, the post slides down by
itself. Then close the lever and check that a 20 N push on the saddle does not move the tape. Adult bikes are set
harder (80-120 N at the lever in maker manuals); a child's balance bike needs much less to hold, and 30 N keeps the
push within a YAM finger's ~40 N.

**Real-trial safety.** Nothing sharp, hot or wet: the bike's parts are painted steel, aluminium and plastic. The
lever can pinch: the paddle is pushed with the gripper's pads, never with a person's hand in the way. The bike
stands in its stand on the table; if it tips, it falls away from the arms (front wheel in the stand, away from
the robot).

## Third-party assets

None. Every shape (bike, stand, post, saddle, lever) is built from boxes and cylinders in `task.py`. The YAM arms,
grippers and station are the program's yam_arena embodiment (its own licence terms, unchanged by this task).

## What is approximate

- The bike is fixed in place. On the real table it stands in a wheel stand; a robot that shoves it can move it, which
  the simulation does not show. If the bike moves off its taped footprint by more than 2 cm in a real trial, the
  trial scores 0.
- The cam, the clamp and the post's friction are rules (above), not contacts; the post and saddle collide with the
  grippers, the frame does not collide with the post.
- Real friction of a loose post varies; the cam-nut setting above brings it into range.
