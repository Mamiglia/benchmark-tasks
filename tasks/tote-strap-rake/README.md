# Take a tote off a shelf when its strap is caught on a hand rake

A blue storage tote stands on a steel shelf in front of the arms. An orange hanging strap is sewn to its back wall and
ends in a steel ring. Left of the tote, a small hand rake lies on its back on the shelf, tines up, and the strap's ring
has dropped over its middle tine. The rake weighs 0.22 kg and slides on the steel deck under a pull of about 1 N. A
carry that leaves the ring on the tine either drags the rake or leaves the tote on the table still tethered to it.

**Instruction:** `Take the tote down from the shelf and put it on the table.`

The instruction states the ordinary goal. It does not mention the strap or the rake.

## Why this task

- **The everyday situation.** Totes, bags and baskets on garage, shed and pantry shelves carry straps, loops and
  rings, and the shelves hold tools with hooks and tines. A strap that has dropped over a tool is a common snag. The
  careless move is to pull the container and let the strap drag the tool off the shelf; the careful one is to lift the
  strap off first. This task tests whether a robot frees an entangled attachment before it moves the object it is
  attached to, and whether it can then carry the object down without disturbing what stays on the shelf.
- **The difficulty is physical.** The ring is 45 mm across with a 31 mm opening; the tine stands 4.5 cm above the
  rake's head. Freeing it means pinching a 25 mm strap 2 cm above the deck and lifting the ring straight up by more
  than the tine's height,
  without pulling the rake sideways. The rake is light enough to follow any sideways pull, so the lift has to be
  nearly vertical. Then two hands must take the tote by its end ledges, lift it level, draw it off the shelf and
  set it down on the table 15 cm lower.
- **Both parts are checked over the whole trial.** Full credit needs the ring off every tine, the rake never dragged
  and never off the shelf, and the tote standing on the table top in front of the shelf.

### Distinct from

- **"Open a drawer held shut by a bar" (accepted).** There a rigid bar is slid out of the way before a pull. Here the
  impediment is a flexible strap hooked on a loose tool: any motion of the tote moves the tool, so the robot has to
  disengage the strap from the tine (lift the ring vertically, keep the rake still) rather than remove a fixed obstacle.
- **"Fish a charger out from behind a box by its cord" (accepted).** There the robot pulls on a cord to retrieve an
  object while a neighbour must not move. Here the strap must not be pulled at all while it is hooked; it has to be
  lifted off a hook, and the container is then carried with both hands.
- **"Release trapped clothing and open a drawer" (not accepted).** Its trapped cloth was a rigid proxy on a vertical
  slide. Here the strap is a jointed chain of five webbing links ending in a free ring, which hangs, bends and drags
  the rake as a real strap does.
- **External task sets.** RoboCasa, BEHAVIOR and LIBERO include taking objects off shelves and out of cabinets, with
  clear access. None has an attachment caught on a second loose object that must be freed first; DataCenterBench's
  cable tasks route or unplug cables, not unhook a strap from a tool.

Category: **Societal relatability** (an everyday household and shed chore; no person is placed at risk in the
scene).

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: instruction, `score()` / `success()`, and `controller()` (the demo `yam_scripted` plays) |
| `totekit.py` | The shelf, tote, strap, ring and rake bodies and joints, the seeded shift, the ring-on-tine and deck-edge checks, the latches, and the hooks into yam_arena |
| `totedemo.py` | The closed-loop scripted demo: camera-based location of the tote and the strap's caught end, the strap lift and the two-handed carry; the contrast runs |
| `test_tote_strap_rake.py` | Tests: every scoring outcome, the ring-on-tine and edge checks, the seed shift, the demo's camera geometry and vision on synthetic images |
| `task.json` | Submission details |

`totekit.py` hooks yam_arena without modifying it, as our other kits do: it patches `yam_arena.scene.task_object` and
`object_states`, the embodiment's `_ensure_sim` and `reset`, and `yam_arena.scripted.YamScriptedPolicy` (when a task
file defines `controller()`, `yam_scripted` asks it for each action and passes it the observation). It wraps the
environment's `scene.write_data_to_sim`, which runs once per physics substep, so the latches are checked at 240 Hz.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left, z up). Arms start in
the ready pose. All sizes are outside dimensions.

| Object | Description |
|---|---|
| Shelf | Steel shelf (a bench riser): a painted steel deck 30 cm deep (x 36 to 66) and 76 cm wide (y -32 to +44), 1.5 cm thick, its top 15 cm above the table, on four 25 mm square steel posts at its corners. Fixed (clamped) to the table |
| Tote | Blue polypropylene storage tote, 26 cm long (along y) x 18 cm deep (x) x 10 cm tall, 5 mm walls, open top, 0.40 kg, empty. Along the top of each end wall a hand-hold ledge sticks out 3.5 cm, 10 cm long, 6 mm thick. It stands on the deck, its footprint centred at x 47, y -6 |
| Strap | Orange 25 mm polypropylene webbing, 20 cm long (taut), sewn to the outside of the tote's back wall 1 cm below the rim and 1 cm from its left end (x 56, y +6, 9 cm above the deck); its other end is sewn round one side of the ring |
| Ring | Steel ring (square-cornered D-ring), 45 mm outside, 6 mm bar, 31 mm opening, 30 g. It lies flat on the rake's head, round the middle tine |
| Hand rake | Three-tine steel hand rake with an ash handle, 0.22 kg, lying on its back on the deck, tines up: a head 8 cm (along x) x 2 cm x 2.5 cm tall with three round tines 8 mm thick rising 4.5 cm above it, 3 cm apart, at x 42, 45 and 48, y +23; a 3 cm neck, then a 14 cm handle 2.4 cm square running to the left (+y), its end at y +41 |

The trial seed moves the tote, strap, ring and rake together by up to 2 cm in x and 3 cm in y (below); the shelf
stays. Nothing else is on the table. Mark the tote's and the rake's start outlines on the deck with tape.

## Seeds

`inspect-robots run --seed N` makes a trial reproducible (the trial seed is derived from N, the scene and the epoch).
The shift is uniform within +-2 cm (x) and +-3 cm (y); the kit prints it at every reset (`[totekit] seed ...`). Always
pass `-E object_jitter=0`: only the seeded shift moves things.

| `--seed` | Trial seed | Shift x | Shift y |
|---|---|---|---|
| 0 (the submitted trial) | 1415158438 | +0.8 cm | -2.8 cm |
| 4 | 3490944092 | +0.0 cm | -0.2 cm |
| 10 | 3996681712 | -1.7 cm | +2.8 cm |

For a real trial, shift the taped outlines by the table's values.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the tote down from the shelf and put it on the table." --scorer task_score \
  --max-steps 1200 --seed 0 --store-frames --log-dir ~/yam-eval/tote-strap-rake-dream
```

`task_score` gives 1.0 on success, otherwise `score()` at the end of the episode. `TOTE_DEBUG=1` prints the rig's
state every half second (tote pose and height, ring position, ring on a tine or not, how far the rake moved, latches).

## Scripted demo

Same command with `--policy yam_scripted` (it plays `controller()` in `task.py`). The demo is closed loop. It uses only
what the robot observes, the top camera image and the arms' measured joint positions, and reads no simulator state.

1. Both arms turn 70 degrees outward at the base, out of the top camera's view.
2. **Locate.** It finds the blue tote and the orange strap by colour. Through the camera's calibration (pose and
   intrinsics), the tote's pixels are put on the plane of its rim and the strap's on a plane 2 cm above the deck. The
   tote reading minus the same reading at seed shift 0 (calibrated once from a logged run, `REF_TOTE` in
   `totedemo.py`) gives how far the tote has moved. The strap's left end is where it is caught: the ring's centre lies
   5 cm beyond the centroid of the strap's last fifth, along the strap's main direction (`RING_PAST_END`,
   calibrated once).
3. **Free the strap.** The left hand comes down from the front, pitched 60 degrees (so its wrist stays clear of the
   deck), the fingers 44 mm apart either side of the 25 mm strap, and pinches the strap 3.6 cm short of the ring's
   centre. It lifts 11.5 cm straight up, turning the grip up to 20 degrees on the way (the ring then hangs from the
   strap, its lowest point about 2 cm over the tine's tip), raises it to 17 cm over the deck, so it hangs clear of the
   tote's rim, carries it over the tote and lets it drop inside: the strap then travels with the tote. The arm parks
   and the demo locates the tote again (the strap's pull can shift it by a centimetre or two).
4. **Carry.** Both hands come in from the front, pitched 30 degrees down, the fingers above and below each end ledge,
   and grip. They lift the tote 3 cm, draw it toward the arms, past the front of the shelf (its centre commanded to
   x 23; the loaded arms lag, and it lands near x 26), lower it until its bottom is 4 mm above the table, let go and
   move 8 cm up and 4 cm back.

Contrasts (same command, an environment variable): `DEMO=naive` skips step 3; `DEMO=short` skips step 3 and sets the
tote down just in front of the shelf, the shortest carry that reaches the table; `DEMO=yank` skips step 3 and lifts
the tote 6 cm before the carry, so the strap pulls up on the ring; `DEMO=hold` frees the strap and sets
the tote down but keeps holding it.

### Checked in simulation

Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1), the commands above with `--max-steps 900`, `TOTE_DEBUG=1`. Camera frames
were checked (normal renders).

| Run | `task_score` | What happened |
|---|---|---|
| Demo, `--seed 0` | **1.0** | Ring lifted off the tine and dropped into the tote; rake moved 1.2 cm (by the strap's lift, ring already free); tote on the table at x 26, released; success at 31 s |
| Demo, `--seed 4` | **1.0** | Same; success at 31 s |
| Demo, `--seed 10` | **1.0** | Same; the dropped ring caught on the tote's rim and hung outside it, which does not matter; success at 31 s |
| `DEMO=naive`, seed 0 | **0** | Tote carried down and set on the table; the strap, still on the tine, pulled the rake 1.4 cm (under the 3 cm drag latch). Ring still round the tine at the end |
| `DEMO=short`, seed 0 | **0** | Tote set down just in front of the shelf; the strap went taut, dragged the rake 3.1 cm (latched) and jerked the tote down at 0.61 m/s (fell, latched) |
| `DEMO=yank`, seed 0 | **0** | 6 cm lift before the carry: the ring stayed on the tine; rake moved 2.4 cm |
| `DEMO=hold`, seed 0 | **0.25** | Strap freed, tote set down but held: tilted in the grips, one end over 1 cm above the table |

One Dream-1 run (`--policy dream`, the command under "Run a trial", seed 0, 1,200 steps, frames checked) scored 0: it
raised both arms near their bases, closed the grippers and never touched the tote, strap or rake.

The naive carry shows why the scorer asks for the ring itself: in this layout a careful carry can reach the table
without moving the rake 3 cm, and the tote is then still tethered to the rake; any later move drags it.

## Scoring

Evaluated every control step; the latches every physics step (240 Hz). The score at the end of the episode counts
(`SCORE_OVER = "end"`).

| Condition | Score |
|---|---|
| The rake went off the shelf: a point of it (the bottom corners of its head, neck and handle, the tines' tips) more than 1 cm past any edge of the deck (front, back, left or right), or more than 1 cm below the deck top, at any time | 0 |
| The rake was dragged: its head moved 3 cm or more (horizontally) from its start while the ring was round a tine, at any time | 0 |
| The tote fell (moved down faster than 0.6 m/s, a free fall of about 2 cm) or went below the table top, at any time | 0 |
| At the end, the ring is round a tine (any of the three) | 0 |
| At the end, the rake is not resting on the deck (lowest point more than 1 cm above the deck top, or moving) | 0 |
| Otherwise (ring free, rake resting on the deck): the tote stands on the table (below), released (both grippers at least 80% open, every finger pad tip and tool point 3 cm or more from the tote, ledges included) and still (under 1 cm/s and 0.05 rad/s): success once this has held **1 s** | 1.0 |
| The same, not yet held 1 s when the episode ends | 0.75 |
| The tote stands on the table but is held or moving | 0.5 |
| The tote is anywhere else (still on the shelf, held in the air) | 0.25 |

**On the table:** all four bottom corners of the tote's outer box are between 0.3 cm below and 1 cm above the table
top, inside the table's footprint, and in front of the deck's front edge (x 36 or less: not under the shelf), with the
tote's up axis within 5 degrees of vertical. A tote held above the table, resting on the shelf's edge, tipped, or
pushed under the shelf is not on the table.

**Ring round a tine:** some tine's axis passes through the ring's 31 mm opening (the axis crosses the ring's
mid-plane inside the opening).

In a real trial, all of this is read from the video and a tape measure:
- **Ring:** after the trial, is the ring round a tine? Any time the rake moves 3 cm (tape marks along its outline)
  while the ring is still round a tine is a drag.
- **Rake off the shelf:** any part of it more than 1 cm past a deck edge (a 1 cm strip of tape along each edge), or
  falling.
- **Tote on the table:** standing flat on the table top in front of the shelf's front edge (a tape line under the
  edge); a falling tote (a visible drop) scores 0.
- **Released and 3 cm clear, 1 s still:** the video.

Ways around the method get nothing:
- **Carrying without freeing the strap**, however short or careful (`naive`, `short`): the ring stays round the tine:
  0, even if the rake happens to stay on the shelf.
- **Yanking the ring off** with a high lift or a hard pull (`yank`): if the strap drags the rake 3 cm before the ring
  comes free, the drag has latched: 0. In the `yank` run the ring stayed on the tine anyway: 0.
- **Moving the rake by hand while it is still caught** counts the same as dragging it: 0. The way to free it is to
  lift the ring off the tine.
- **Pushing the tote off the shelf** to fall onto the table: it falls faster than 0.6 m/s: 0.
- **Holding it on the table**: 0.5 at most (`hold` got 0.25: the held tote was tilted, one end still above the
  table). **A fixed schedule cannot be waited out:** every condition is a latch over
  the whole trial or a state that must hold for 1 s at the end.

## Simulation fidelity

Approximate. The tote is one rigid body of boxes (friction 0.8); the ring is one rigid body of four bars (a square ring
standing in for a round one, same opening); the rake is one rigid body (boxes and three cylinders, friction 0.4 on the
steel deck). The strap is five rigid webbing links, 3.95 cm x 25 mm x 4 mm, joined to each other, the tote and the ring
by spherical joints (100-degree cone limits): a coarse model of a limp strap. Each link is 5 g (real webbing of that
size weighs about 1 g) to keep PhysX's joint solver stable. The shelf is static. Contacts are PhysX's; the ring-on-tine,
deck-edge, table and release checks are geometric, from body poses and the robot's link poses.

## Runtime

On the RTX workstation used (one GPU, shared), a scripted demo trial (Isaac Sim start, reset, about 620 control
steps, episode end) takes about 7 minutes of wall time. A 1,200-step policy run takes longer, depending on the policy's
latency.

## Tests

```bash
pytest test_tote_strap_rake.py      # in the yam_arena environment; no GPU needed (under a second)
```

## Real-world build and safety

All parts are stock: a steel bench-riser shelf (or a 76 x 30 cm steel shelf panel on four 15 cm posts) clamped to the
table; a 4 L polypropylene storage tote with end hand-holds; 25 mm webbing and a 45 mm steel D-ring, sewn; a three-tine
hand rake (blunt the tine tips with tape or rubber caps for the trial). The shelf is a low bench riser rather than a wall shelf because the YAM reaches the
tote's ledges and the strap only up to about 26 to 30 cm above the table. The heaviest load is the 0.40 kg tote, shared
by two arms (each YAM arm is rated for about 1.5 kg); the strap and rake need well under 1 N. If the rake is dragged off
the shelf it falls 15 cm onto the table: no hazard to people. Keep people out of the arms' reach as usual.

## Assets and licences

No third-party assets. The shelf, tote, strap, ring and rake are primitives defined in `totekit.py`. The robot, room and
cameras are the standard yam_arena embodiment.

## Credit

The idea for this task was first proposed in robocurve/benchmark-tasks#295 (TK-UVZ464). The code, models and text here are an independent implementation.
