# Plug an HDMI cable into a monitor's downward-facing rear port, by feel

Instruction: *"Plug the HDMI cable into the HDMI port on the back of the monitor."*

Category: **Societal relatability**. Anyone who has set up a desk monitor or a TV has done this: the ports sit on the
back, facing down, and you push the plug up into a port you cannot see.

## Why this task

The robot does the job the way people do it, on a real office monitor (Dell P2422H, 23.8 in) on its own stand, with
an ordinary HDMI cable. The difficulty is physical:

- **The port is out of sight.** On this monitor (as on most office monitors) the connectors are in a bay under the
  back of the screen and face straight down. The top camera sees only the screen and its back edge. The wrist camera,
  with the hand under the screen, faces the screen's lower bezel. The robot has to bring the plug to a port it cannot
  see and find it by small moves and by feel.
- **An awkward, constrained approach.** The plug starts lying flat on the desk. The robot must pick it up, turn it to
  point straight up, and bring it in under the panel's lower edge (13 cm above the desk) with its tip kept below that
  edge, to a point 32 mm behind the screen's face. Then it pushes it up. The fingers must hold the plug near the back
  of its overmold: held further forward, they meet the bottom of the screen before the plug is home.
- **Keyed and tight.** The HDMI shell is a trapezoid (14.0 x 4.55 mm). It goes in only the right way round, straight,
  and within about 0.4 mm of the port axis. Off by more, it stops on the port face or the bay ceiling.
- **A firm, straight push.** Once the shell is 4 mm in, the receptacle's contact springs hold it, and it slides further
  only under an axial push of more than 15 N, up to the stop. A plug left part way in is not connected.

### Distinct from

- Board tasks: plugging cable ends into sockets in plain view (two workstation ports, a cable tester's jacks, a USB-A
  port), reseating a half-inserted patch cord, and unplugging a latched Ethernet cable. In all of them the robot sees
  the socket and pushes horizontally. Here the port faces down under the screen, out of sight, and the plug goes in
  upward from under an overhang.
- External benchmarks: no HDMI or monitor-port task in BEHAVIOR-1K, LIBERO, RoboCasa or DataCenterBench. The nearest
  are generic plug and peg insertions (for example RLBench "insert onto square peg", FurnitureBench and the NIST
  board connector insertions). Those approach a socket in view, from above or from the front.

## Design

All parts are full size. Positions are in metres from the midpoint between the two arm bases, on the table top
(x forward, y left, z up).

- **Monitor** (Dell P2422H, 23.8 in, 1920 x 1080, on its height-adjustable stand), facing the robot, centred on y 0.
  Screen face at x 0.400. Panel 538 mm wide and 318.5 mm tall; 52 mm deep including the rear housing. The stand is set
  so that the panel's lower edge is 130 mm above the desk (range 46-178 mm), with no tilt or swivel. Stand base 245 x
  173 mm, front edge at x 0.395. The base is held to the desk by two C-clamps, so the monitor cannot be pushed away.
- **Connector bay.** Across the bottom of the rear housing is an open channel 24 mm deep (x 0.422 to 0.446) and 6 mm
  high. Its ceiling holds the ports, all facing down (Dell user guide, "Bottom view"). In the sim, from the robot's
  right: USB upstream, VGA, DisplayPort, HDMI, power, and further left two USB-A. The HDMI port's centre is at
  (x 0.432, y -0.027), its mouth 136 mm above the desk. On the real monitor, measure the HDMI port's offset from the
  centre line and set the monitor's y so the port is at y -0.027. Its trapezoid's wide side faces
  the screen. Only the HDMI port is used and only it is modelled as an opening; the others are drawn.
- **Cable.** An Amazon Basics high-speed HDMI cable, 0.9 m (3 ft). Its plug: overmold 36 x 21 x 11 mm, steel shell
  11.5 mm long, 30 g with the first few cm of cable. Its far end is plugged into a small HDMI signal generator (9 x 6 x
  2 cm, taped to the desk at (0.15, -0.50)), which drives the monitor.
- **Start.** The plug lies flat on the desk, wide side of the shell up, pointing at the monitor (+x), near
  (0.30, -0.13). The seed (`PLUG_SEED`, default 0) shifts it by up to 2 cm in x and y and turns it by -20 to +20
  degrees (seed 0: (0.305, -0.139), tip turned 18 degrees to the robot's right; seed 4: (0.318, -0.130), 19 degrees to
  the left; seed 10: (0.318, -0.142), 13 degrees to the left). The cable runs from the plug's back to the generator. The
  monitor is on and shows "No signal". The arms start in the ready pose.

## Scoring

| Score | Condition | Real-trial check |
|---|---|---|
| 1.0 | at the end the plug is seated in the HDMI port (overmold within 1 mm of the port face) and no gripper tool point is within 7 cm of it; held 1 s | the monitor shows the generator's picture; a phone photo from below shows no gap at the shell; grippers clear (video) |
| 0.5 | at the end the shell is 3 mm or more into the HDMI port, but it is not seated, or a gripper is still at it | photo from below: the shell is in the port; or the video shows the gripper still on the plug at the end |
| 0.25 | at some point the shell went 3 mm or more into the HDMI port, but at the end it is out | video |
| 0 | none of these | |

"In the HDMI port": the shell's tip is within 2 mm of the port's axis and above its mouth. A plug can only get there
the right way round. In the real trial, any other port does not count: only a seated HDMI plug gives a picture.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
PLUG_SEED=0 inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Plug the HDMI cable into the HDMI port on the back of the monitor." \
  --scorer task_score --max-steps 1600 --seed 0 --store-frames --log-dir ~/yam-eval/hdmi-blind-plug-dream
```

`-E control_hz=20` is required: the score bookkeeping runs once per control step. `PLUG_SEED` sets the plug's start
position and turn (`hdmimodel.start_pose`). `PLUG_DEBUG=1` prints the plug's state five times a second.

## Scripted demo

Same command with `--policy yam_scripted`. `yam_scripted` runs `hdmidemo.Controller` closed loop (`partkit` hooks it
in when the task module defines `demo_controller`). It reads the plug's pose from the simulator state the task keeps,
as scripted demos may; it does not move objects. The sequence:

1. The right hand takes the plug from above, across its 21 mm width, 5 mm from the back of the overmold, and lifts it.
2. It pitches the wrist up so that the plug points up and the fingers point at the monitor, and lowers the plug so
   that its tip is below the panel's edge.
3. It slides the plug in under the screen to 4 mm below the port, lines the tip up with the port from the measured
   pose (a rate-limited integral term corrects for the arm's sag), and pushes it straight up. The arm is compliant, so
   the command goes up to 8 cm past the plug to press it past the springs' 15 N.
4. Seated, it stops pressing (the springs hold the plug), opens, drops 4 cm clear of the plug, pulls back out from
   under the screen and goes home. The left arm does not move.

Loophole checks, same command with `DEMO_CHEAT=naive` (no lining up: drives the plug to where the port should be
from the planned grasp, open loop), `DEMO_CHEAT=partial` (pushes the shell only 4.5 mm in, then lets go) and
`DEMO_CHEAT=hold` (seats the plug but keeps holding it).

## Checked in simulation

| Run | Score | What happened |
|---|---|---|
| Scripted demo, seeds 0, 4, 10 | 1.0, 1.0, 1.0 | plug taken and pitched up; slid under the screen; lined up to within 0.3 mm and 0.3 degrees, 5 mm below the port; pushed up (the springs took hold at 4 mm, then gave way to the pressing hand); released with the overmold 0.3-0.8 mm from the port face; success held 1 s at about 506 steps |
| `DEMO_CHEAT=naive` (no lining up, open loop from the planned grasp) | 0.0 | the plug arrived 3 mm off the port axis (the grasp is not where it was planned); pushed up, its tip struck beside the opening, it tipped 16 degrees and jammed; let go, it fell to the desk |
| `DEMO_CHEAT=partial` (shell pushed 4.5 mm in, let go) | 0.5 | the springs hold it there; it is in, not seated |
| `DEMO_CHEAT=hold` (seated, never let go) | 0.5 | seated but still held at the end |
| Dream-1, seed 0, 1,600 steps | 0.0 | reached for the plug, picked it up off the desk about a minute in, turned it in the air and put it down near where it lay; it never brought it toward the monitor |

While building the demo, two earlier versions lost the plug when the hand opened (scored 0.25: in, then out). Those
were fixed in the simulation's joint stop and in the demo (it now stops pressing before it opens); the results above
are from the final files.

## Runtime and seeds

The scripted demo needs about 506 control steps (25 s at 20 Hz). The run limit is 1,600 steps (80 s at 20 Hz). A trial takes about
3 minutes of wall time on one data-centre GPU. Seeds change only the plug's start position and turn.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, the contact-spring joint and the cable's pull, `score()` / `success()`, `demo_controller` |
| `hdmimodel.py` | Geometry, the spring and cable rules and the score bookkeeping (NumPy only) |
| `partkit.py` | Multi-box parts, switchable joints between parts, and the hooks into yam_arena |
| `hdmidemo.py` | The closed-loop scripted demo |
| `test_hdmi_blind_plug.py` | Tests: start poses, port geometry and key, spring rule, cable rule, every scoring rule |
| `task.json` | Submission details |

`partkit.py` hooks yam_arena without modifying it: `yam_arena.scene.task_object` and `object_states`, the
embodiment's `_ensure_sim`, `reset` and `_task_step`, and `yam_arena.scripted.YamScriptedPolicy.reset` / `act`.

## Tests

```bash
python -m pytest -q test_hdmi_blind_plug.py   # NumPy only; no GPU needed
```

## Real-world build

| Item | Product | Size / mass | Approx. price |
|---|---|---|---|
| Monitor | Dell P2422H 23.8 in monitor with its height-adjustable stand (HDMI, DisplayPort, VGA, USB ports in a bay under the back, facing down) | 538 x 180 x 364-496 mm, 5.6 kg | $90-130 used, about $180 new |
| HDMI cable | Amazon Basics high-speed HDMI cable, 3 ft (0.9 m) | plug 36 x 21 x 11 mm | $7 |
| Signal source | USB-powered HDMI test-pattern generator (any 1080p model, about 9 x 6 x 2 cm), or a laptop | | $20-30 |
| Clamps | two 2 in (50 mm) C-clamps with pads, over the front corners of the stand base | | $10 |
| Tape | masking tape marks for the plug's start | | |

Total about $130-230 (less if the lab has a monitor). Set the stand so the panel's lower edge is 130 mm above the
desk (ruler), the panel upright (no tilt) and square to the robot (no swivel). Before each trial, unplug the cable,
check the generator is on, and lay the plug on its tape marks.

**Forces.** HDMI connectors take at most 44.1 N to insert and 9.8-39.2 N to withdraw (HDMI specification). Measure
the real pair once: hold the monitor and push the plug in with a luggage scale; the sim uses 15 N. The YAM pushes
along the plug, holding its overmold between its two fingers (up to about 40 N each, so about 40 N of axial hold at
a friction coefficient of 0.5). The arm's push upward is well inside its limits.

**Safety.** Nothing is wet, greased or sharp. The monitor runs on its own power cable, which is plugged into the bay
before the trial; its connector is fully shrouded. The clamped stand cannot tip. If the plug is dropped it falls
13 cm onto the desk.

## Simplifications

- The plug's shell is two stacked boxes (wide side 14.0 mm, the narrow side's corners cut 1.3 mm back), and the
  receptacle opening matches it, with 0.4 mm clearance per side (the real one is about 0.1 mm). An upside-down plug
  hits the receptacle's filled corners.
- The receptacle's contact springs are a joint, not contacts. Once the shell is 4 mm in, on the axis, the plug slides
  only along the port's axis, and a damping drive capped at 15 N resists that slide: the plug moves in only under an
  upward push of more than 15 N, and a seated plug stays in. The joint's travel ends with the overmold 0.3 mm short of
  the port face (the seated stop). Pulled back out past 4 mm, the joint lets go.
- The cable is drawn only as a short stub. Once the plug is lifted, its pull is a constant 0.5 N force toward the
  generator, applied at the plug's centre of mass. Lying on the desk, the plug feels none.
- The monitor is one fixed body (the real one is clamped); its tilt hinge is not modelled. The other ports in the bay
  are drawn, not open.
- The real-trial check of a picture on the screen is not simulated; the score uses the plug's depth in the port.

## Assets and licences

No third-party assets in the task: every object is built from boxes in `task.py`. The robot is the standard yam_arena
YAM (I2RT YAM arm URDF and meshes, MIT licence, `yam_arena/assets/yam/LICENSE.i2rt`); the room and cameras are the
standard station scene.
