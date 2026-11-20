# Lock a monitor with a keyed cable lock

Instruction: *"Lock the monitor with the cable lock and take the key out."*

Category: **Societal relatability**. Offices, schools, libraries, computer labs and shops lock their monitors and
laptops to the desk with this kind of lock, and most office workers have seen one fitted or have fitted one. Anyone
watching a clip sees at once what the robot is doing.

## Why this task

The robot fits a real lock (Kensington MicroSaver 2.0) to a real office monitor (Dell E2422H), the way people do it.
The lock head goes into the monitor's security slot, and the key is turned a quarter turn while the head is held
flat against the monitor. The difficulty is physical:

- **A small, square fit.** The slot is a 7 x 3 mm hole in the monitor's back. The lock's T-bar (a 6.0 x 2.2 mm
  crossbar on a stem, with two fixed side pins) goes in only level, square to the back and within about 0.4 mm of the
  slot's centre. Skewed or off by more, it stops on the wall.
- **Hold it home while the other hand turns.** The crossbar can turn only when it is fully behind the 1.5 mm wall,
  which leaves 0.7 mm of play: the head must be pressed flat on the monitor while the key turns. If the head backs
  off, the crossbar binds in the slot; if it is out, the bar turns outside the slot and the lock cannot go in. The
  steel cable's weight pulls the head down and out of the slot (the pins hold only 2 mm deep), so a head that is let
  go before the key is turned falls out. One hand must keep the head square and home while the other hand turns the
  key. Then the key is pulled straight out along the lock's axis; it comes out only in the locked position.
- **A free monitor.** The monitor (3.9 kg on a light stand) stands free on the desk. Pressing too hard or
  levering on it slides or tips it, which scores 0.

### Distinct from

- Board tasks: tightening a drill bit with a chuck key (the key's pinion is kept in mesh while it turns), adding a
  padlock to a lockout hasp, locking a bayonet cap or a two-pin lamp tube by turning the part itself, threading a key
  onto a split ring. None puts a T-bar through a slot and holds it home while a second hand turns a key in it.
  Plugging an HDMI plug into the downward-facing port bay of a monitor (another monitor-port task) is a one-hand push;
  here the slot is on the back and the task is the two-hand hold-and-turn.
- External benchmarks: no cable-lock or security-slot task in BEHAVIOR-1K, LIBERO, RoboCasa or DataCenterBench. The
  nearest are Meta-World "door-lock" (turn a lock knob on a fixed lock) and RLBench "insert onto square peg". Neither
  needs a part held seated by one hand while the other turns a key in it.

## Design

All parts are full size. Positions are in metres from the midpoint between the two arm bases, on the table top
(x forward, y left, z up).

- **Monitor** (Dell E2422H, 23.8 in, on its fixed-height tilt stand), its screen facing away from the robot, so the
  robot works at its back, as when locking a monitor from behind a desk. Panel 552.6 mm wide, 331.6 mm tall, 49.7 mm
  deep, its lower edge 88.7 mm above the desk, no tilt (Dell user guide, "Physical characteristics"). Centre line at
  y -0.070. The lower rear housing's back face is at x 0.420; the screen face at x 0.470. Stand: a neck behind the
  housing and a 222 x 171 mm base plate (x 0.300 to 0.471). Mass 3.9 kg (guide: 4.03 kg with stand and cables).
  The monitor is not clamped.
- **Security slot** (Kensington Security Slot, 7 x 3 mm, long side level) in the lower rear housing, 115 mm left of
  the centre line as seen from behind and 45 mm above the panel's lower edge (guide, "Back view", item 1): centre at
  (0.420, 0.045), 134 mm above the desk. Wall 1.5 mm; a 12 x 12 x 10 mm pocket behind it.
- **Lock** (Kensington MicroSaver 2.0 keyed laptop lock, K65020WW): head with a 10 mm front nose, an 18 mm barrel
  (34 mm long in all), a cable collar at the back; the T-bar (2.0 mm stem, 6.0 x 2.2 x 1.8 mm crossbar) and two fixed
  side pins stick out 2.2-4.0 mm from the nose. The tubular key is in the back of the head: 7 mm shank, 22 x 22 x 3 mm
  bow. The T-bar turns with the key, 90 degrees, with light detents at both ends. Head about 50 g, key about 8 g.
  The 1.8 m steel cable is already looped round the desk's left front leg and comes up over the desk's left edge to
  the head.
- **Start.** The lock lies on the desk in front of the monitor's base, T-bar toward the monitor, key in and in the
  unlocked position (as it is left after being taken off a device), bow flat. Near (0.24, -0.02); the seed
  (`LOCK_SEED`, default 0) shifts it by up to 2 cm in x and y and turns it by -20 to +20 degrees (seed 0: (0.245,
  -0.029), T-bar turned 18 degrees to the robot's right; seed 4: (0.258, -0.020), 19 degrees to the left; seed 10:
  (0.258, -0.032), 13 degrees to the left). The arms
  start in the ready pose.

## Scoring

| Score | Condition at the end (unless latched) | Real-trial check |
|---|---|---|
| 0 (latched) | at any time the monitor slid more than 3 cm, tipped more than 10 degrees or left the desk | tape marks at the base's corners, ruler; video |
| 1.0 | locked (head flush on the monitor's back, T-bar through the slot and turned crosswise), key out of the lock, both grippers 4 cm or more from the head; held 2 s | operator pulls the head straight back with a luggage scale to 20 N: it stays; no 1 mm gap at the nose (1 mm card); the key is not in the lock; video |
| 0.8 | locked and key out, but a gripper is still at the head | as above; video |
| 0.6 | locked, key still in the lock | tug test holds; key in the lock |
| 0.3 | head flush with the T-bar through the slot, not turned | the head comes out under the tug |
| 0.1 | the T-bar went through the slot at some point, but it is not there at the end | video |
| 0 | none of these (including a key turned with the head outside the slot) | |

"Locked" in the simulation: the T-bar's crossbar is behind the slot's wall (head face at most 0.3 mm off the monitor's
back, within 2 mm of the slot's centre, the lock's axis within 8 degrees of square) and turned 75 degrees or more.
Turned, the crossbar cannot pass back through the slot: the colliders hold it. "Key out": the key has been pulled out
of the lock (3 N or more straight back, in the locked position). Real trial: the operator tugs the head after the trial, then turns the key back (or uses the
second key) to unlock it for the reset.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
LOCK_SEED=0 inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lock the monitor with the cable lock and take the key out." \
  --scorer task_score --max-steps 2400 --seed 0 --store-frames --log-dir ~/yam-eval/monitor-cable-lock-dream
```

`-E control_hz=20` is required: the score bookkeeping runs once per control step. `LOCK_SEED` sets the lock's start
position and turn (`lockmodel.start_pose`). `LOCK_DEBUG=1` prints the lock's state five times a second.

## Scripted demo

Same command with `--policy yam_scripted`. `yam_scripted` runs `lockdemo.Controller` closed loop (`partkit` hooks it
in when the task module defines `demo_controller`). It reads the lock's and key's poses and the T-bar's angle from the
simulator state the task keeps, as scripted demos may; it does not move objects. The sequence:

1. The right hand takes the head off the desk from above and behind, across its barrel, and lifts it.
2. It brings the head level, square to the monitor's back, 4 cm out from the slot, then to 6 mm out; it lines the
   T-bar up with the slot from the measured pose (a rate-limited integral term corrects for the arm's sag and the
   grasp), and pushes the head flat onto the monitor, pressing about 6 mm past the face (a few newtons).
3. Keeping the head pressed home, it waits while the left hand comes in from behind along the lock's axis, pinches
   the key's bow across its 3 mm thickness, turns it a quarter turn with the wrist and pulls it 35 mm straight back.
4. The left hand lays the key on the desk. The right hand stops pressing, opens and backs away; both arms go home.

Loophole checks, same command with `DEMO_CHEAT=onehand` (the head is pushed home and let go before the key is turned),
`DEMO_CHEAT=early` (the key is turned while the head is still in the air, then the head is pushed at the slot),
`DEMO_CHEAT=keyin` (locks but leaves the key in) and `DEMO_CHEAT=tug` (the full demo, but before letting go the right
hand pulls the locked head 2 cm straight back, like the real trial's tug test).

## Checked in simulation

| Run | Score | What happened |
|---|---|---|
| Scripted demo, seeds 0, 4, 10 | 1.0, 1.0, 1.0 | head taken and brought square to the slot; on each seed the first push stopped with the crossbar on the wall (0.6-0.9 mm off centre), so the hand backed off, lined up again and pushed home; key turned 90 degrees with the head held, key pulled out and laid on the desk; head let go: it sagged about 0.5 mm and 4-5 degrees on the cable's pull but stayed locked; the monitor moved at most 4 mm (seed 0); success held 2 s at 1,160, 1,004 and 1,052 steps |
| `DEMO_CHEAT=onehand` (head pushed home and let go before the key is turned) | 0.1 | the cable's pull tipped the head out of the slot within a second; it fell to the desk; the key turned on nothing |
| `DEMO_CHEAT=early` (key turned with the head in the air) | 0.0 | the crosswise T-bar stopped on the wall on every one of four tries; never in |
| `DEMO_CHEAT=keyin` (locks, leaves the key in) | 0.6 | locked, key still in |
| `DEMO_CHEAT=tug` (full demo, then a 2 cm pull on the locked head before letting go) | 1.0 | the hand pulled 2 cm past the head (a firm pull through the compliant arm); the turned crossbar held on the back of the wall and the head moved 0.6 mm; then let go as in the demo |
| Dream-1, seed 0, 2,400 steps | 0.0 | the arms made small moves near their ready pose for the whole run and never reached the lock |

While building the demo, earlier versions of the scene had the key slide out when the head was let go (it was held
by 1 N of friction; now a 3 N pull) and tighter T-bar clearances that made the push stop on the wall more often. The
results above are from the final files.

## Runtime and seeds

The scripted demo needs 1,000-1,160 control steps (50-58 s at 20 Hz), including one back-off and second push. The run
limit is 2,400 steps (120 s at 20 Hz). A trial takes about 5 minutes of wall time on one data-centre GPU. Seeds change
only the lock's start position and turn.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, the T-bar's detents, the key's joint, the cable's pull, `score()` / `success()`, `demo_controller` |
| `lockmodel.py` | Geometry, the key and cable rules and the score bookkeeping (NumPy only) |
| `partkit.py` | Multi-box parts, joints between parts, and the hooks into yam_arena |
| `lockdemo.py` | The closed-loop scripted demo |
| `test_monitor_cable_lock.py` | Tests: start poses, slot and T-bar geometry, seat test, detents and cable, monitor moved, every scoring rule |
| `task.json` | Submission details |

`partkit.py` hooks yam_arena without modifying it: `yam_arena.scene.task_object` and `object_states`, the
embodiment's `_ensure_sim`, `reset` and `_task_step`, and `yam_arena.scripted.YamScriptedPolicy.reset` / `act`.

## Tests

```bash
python -m pytest -q test_monitor_cable_lock.py   # NumPy only; no GPU needed
```

## Real-world build

| Item | Product | Size / mass | Approx. price |
|---|---|---|---|
| Monitor | Dell E2422H 23.8 in monitor on its stand (security slot on the back, lower left as seen from behind) | 552.6 x 171 x 420.4 mm, 4.0 kg | $60-90 used, about $130 new |
| Lock | Kensington MicroSaver 2.0 keyed laptop lock, K65020WW (T-bar, 10 mm head, tubular key, two keys, 1.8 m carbon-steel cable) | 0.17 kg with cable | $35-45 |
| Tape | masking tape marks for the monitor's base corners and the lock's start | | |
| Luggage scale | any hanging scale to 20 N or more, for the tug test | | $10 |

Total about $110-190. Set the monitor on the robot's table with its screen facing away from the robot, its stand base
at the positions above (tape marks at the four corners), panel upright (tilt 0). Loop the lock's cable round the
table's left front leg (or any fixed post on the left) and lay it over the table's left edge. Before each trial,
unlock the lock with the second key if needed, put the key in the head in the unlocked position, and lay the lock on
its tape marks, T-bar toward the monitor, bow flat.

**Forces.** The T-bar slides into the slot freely (a few newtons at most; the demo presses with about 3-5 N). The key
turns with about 0.05-0.1 N m (measure once: hook a luggage scale on the bow's edge, 10 mm from the axis, and turn;
0.1 N m reads 10 N). The gripper pinches the 3 mm bow across its faces, so the turn is carried by the bow's shape,
not by friction. All of this is far inside the YAM's limits (about 40 N per finger, 1.5 kg payload). The head and key
weigh about 60 g. The monitor is only touched by the lock.

**Safety.** Nothing is wet, greased or sharp; the T-bar and key are blunt steel. The monitor need not be powered.
Leave at least 15 cm of table behind the screen so a shoved monitor stays on the table; the operator keeps a hand
near the e-stop and stops the trial if the monitor tips. The steel cable lies on the floor and table edge away from
the arms' bases.

## Simplifications

- The lock and slot are boxes: the barrel is four rotated boxes (about 18 mm across), the T-bar a stem and a
  crossbar whose last 0.6 mm is slightly smaller (its rounded tip), the side pins two boxes; the slot a 7 x 3 mm
  opening in a 1.5 mm wall with a pocket behind. Clearances 0.4-0.6 mm per side (real parts are closer). Exact dimensions of the lock's
  T-bar are not published; they are set to fit the 7 x 3 mm standard slot.
- The key is its own body, held in the lock by a joint along the lock's axis: it turns the T-bar with it and cannot be
  pulled out while the T-bar is in the unlocked position. In the locked position it sits on a stiff spring; pulled
  straight back with 3 N or more (1.5 mm of give), it comes out and is free. The key's shank is drawn only (in the real
  lock it touches nothing but the cylinder); its bow has a collider. On the real MicroSaver 2.0 the key comes out only
  when locked (check on the unit at setup).
- The T-bar's detents are a weak spring (0.06 N m/rad) pulling it to whichever end it is nearer.
- The cable is drawn as a short stub. Once the head is lifted, its weight and drag are a constant 0.6 N force toward
  the desk's left edge, applied at the head's cable collar. Lying on the desk, the head feels none.
- The monitor is one rigid body (panel and stand); its tilt hinge is not modelled (on the real stand it holds well
  above the forces involved).

## Assets and licences

No third-party assets in the task: every object is built from boxes in `task.py`. The robot is the standard yam_arena
YAM (I2RT YAM arm URDF and meshes, MIT licence, `yam_arena/assets/yam/LICENSE.i2rt`); the room and cameras are the
standard station scene.
