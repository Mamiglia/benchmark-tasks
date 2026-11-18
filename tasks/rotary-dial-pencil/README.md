# Dial 4-1-1 on a rotary phone with a pencil

A rotary-dial telephone stands on the desk in front of the arms, its handset off the cradle. A pencil lies on a
notepad beside it. The robot picks up the pencil and dials 4, 1, 1 (directory assistance in North America): for each digit it puts the pencil's eraser end
in the digit's finger hole, drags the finger wheel clockwise round to the finger stop against the dial's return
spring, takes the pencil out and lets the dial run back on its own before the next digit.

**Instruction:** `Dial 4, 1, 1 on the rotary phone with the pencil. For each digit, put the pencil's eraser end in the digit's hole, pull the dial round to the finger stop, then take the pencil out and let the dial run back on its own before the next digit.`

## Why this task

- **Recognisable at a glance.** Dialing a rotary phone is one of the best-known everyday actions of the last
  century, shown in countless films; dialing with a pencil was a common office habit (it spares the fingertip and
  nail). A ten-second clip of a robot dragging a pencil round a dial needs no explanation.
- **The skill is a curved, force-guided drag with a held tool, ended at the right moment.** The robot holds a
  7 mm pencil, hooks its end into a 17 mm hole and drags it along a circle of 37 mm radius, on a face sloping
  35 degrees, against a return spring (about 0.7 to 1.2 N at the hole), for 145 degrees (the 4: from 11 o'clock
  over the top and down to half past four) or 55 degrees (each 1), without letting the end ride out of the hole. Letting go early dials
  a lower digit; staying in the hole, or touching the wheel on its way back, holds the governor back and spoils the
  digit. The pencil turns the robot's grip into a lever: the drag pushes the pencil's end sideways in all
  directions in turn, so the grip must keep the pencil from twisting between the fingers all the way round.
- **The difficulty is physical.** Reading the digits on the dial is easy; the hard part is the tool-tip path and
  force: staying engaged along a curve, pressing to the stop, and a clean release.

Category: **Societal relatability.**

### Distinct from

- **Board tasks.** No task on the board dials a phone or drags a part round an arc with a held tool. The nearest
  found: "Open a hanging locker padlock by its dial" (a knob turned in the fingers, both ways, to numbers; no
  tool, no spring return, no stop), "Trace an oblique path with two XY plotter knobs" (knobs turned, no tool) and
  pencil tasks such as "Scribe a shelf's end to an out-of-flat wall" (a pencil drawn along a surface, no
  mechanism driven by the tool).
- **External benchmarks.** RoboCasa's knob tasks (turn a stove burner on or off) and LIBERO's "turn on the stove"
  turn a knob in the fingers to a set position. BEHAVIOR-1K has no telephone-dialing activity. None drives a
  spring-return mechanism through a held tool.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: layout, instruction, scorer, the demo hook |
| `dialmodel.py` | Pure Python: the phone's and dial's geometry, the spring and governor, the pull and pulse counting, the score |
| `dialkit.py` | The Isaac Sim side: the phone articulation (body and finger wheel), the handset, the drawn pencil, the per-physics-step spring, governor, pull tracking and fouls |
| `dialdemo.py` | The closed-loop scripted demo (and its contrast runs) |
| `test_rotary_dial_pencil.py` | Unit tests (no Isaac Sim needed) |

## Setup

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left, z up). One fixed
layout; no randomisation.

| Object | Real item | Where |
|---|---|---|
| Telephone | GPO 746 Rotary Dial Telephone (ProTelX; ivory; about 21.5 x 16 x 14.5 cm, about 1 kg, about $80 to $100): a reproduction of the 1960s British desk phone with a working spring-return finger wheel and governor. Finger wheel about 9.6 cm across with ten 17 mm finger holes (1 at about 2 o'clock round to 0 at about 6.30), chrome finger stop at about 4.30, dial face sloping about 35 degrees toward the user | Body centred at (40, -3), front with the dial facing the arms, standing loose on the desk (rubber feet). Dial centre at about (36, -3), 6.8 cm above the table |
| Handset | The phone's own handset, off the cradle (the hook switch up), its coiled cord to the left | Lying on the desk, centred at (40, 17), along x |
| Notepad | A6 notepad, 10.5 x 14.8 cm, 8 mm thick | Centred at (25, -24.5), long side along y |
| Pencil | Unsharpened HB pencil with eraser (for example Dixon Ticonderoga 13882), 190 mm, hexagonal 7 mm across flats, 6 g | On the notepad at (25, -23.5), along x, eraser end toward the arms |

**Dial geometry (as modelled; measure on the delivered phone).** Hole circle radius 37 mm; holes 30 degrees
apart; with a 7 mm pencil in hole n pushing the hole's clockwise edge, the wheel meets the stop after
55 + 30 (n - 1) degrees (1: 55, 4: 145, 7: 235, 0: 325). The spring holds the wheel at rest with 0.025 N m and
adds 0.008 N m per radian (0.7 N at the hole at rest, 1.2 N at the stop for a 4, 1.6 N for a 0). The governor
runs the wheel back at 300 degrees a second (10 pulses a second, as a real dial). To check the force on the real
phone, hook a 0-5 N spring scale's hook in a hole and pull it round tangentially; the governor rate from the
run-back time of a 0 on the video.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Dial 4, 1, 1 on the rotary phone with the pencil. For each digit, put the pencil's eraser end in the digit's hole, pull the dial round to the finger stop, then take the pencil out and let the dial run back on its own before the next digit." \
  --scorer task_score --max-steps 1200 --seed 0 --store-frames --log-dir ~/yam-eval/rotary-dial-dream
```

`--policy yam_scripted` plays the scripted demo. `DIAL_DEBUG=1` prints the wheel angle, the pulls and digits so
far, the gripper's clearance from the dial and any foul every 0.1 s. The layout is fixed, so the seed changes
nothing in the scene; the demo was run on seeds 0, 4 and 10. A 1,200-step trial takes about 6 to 10 minutes of wall-clock time on one
GPU (RTX 6000 Ada class).

## Scoring

- **Foul (latched for the whole trial): 0.** The phone slid more than 3 cm, tipped more than 10 degrees or lifted
  1 cm; a gripper finger or the gripper itself on the dial (within 15 mm of the dial face inside 5.2 cm of its
  centre: the dial is worked with the pencil only).
- **Otherwise: the number of leading digits dialed correctly / 3.** 1/3 for a 4, 2/3 for 4 then 1, 1.0 for 4, 1, 1.
  A digit is what a pull sends: a pull starts when the wheel leaves rest by 10 degrees and ends when it is back
  within 2 degrees; it sends one pulse for each hole it brought to within 15 degrees (about 1 cm at the hole
  circle) of the stop, so pulling hole 4 to the stop sends 4, and letting go 2 cm short sends 2 or 3. A pull that
  sends no pulse dials nothing. A pull that is held back on its run-back (it takes longer than 1.5 x the free
  run-back + 0.25 s) or is pushed on again before reaching rest dials a wrong digit. A digit after the third makes
  the number wrong (at most 2/3). Success (the trial ends) when the score is 1.0 and the wheel has been at rest
  for 1 s.

### Real trial

Score from the trial video (the top camera, plus a phone camera on a stand 30 cm in front of the dial), frame by
frame:

| Simulation check | Real check |
|---|---|
| Pull reached within 15 degrees of the stop | The pencil's end touched the finger stop, or stopped within 1 cm of it (a 1 cm mark on a strip of tape on the bezel next to the stop) |
| Run-back slower than 1.5 x free + 0.25 s | Count the frames from the pencil leaving the hole to the wheel at rest; compare with the same digit pulled by hand by the operator before the session (free run-back, about 0.5 s for a 4, 0.2 s for a 1) |
| Pushed on again before rest | The pencil or gripper touches the wheel while it runs back |
| Phone slid 3 cm / tipped 10 degrees / lifted 1 cm | Tape marks at the phone's front corners; any visible lift of the feet |
| Gripper on the dial | A finger or the gripper touches the dial (wheel, stop or number plate) |

Optional: plug the phone into a pulse-dial line or a rotary-to-DTMF adapter with a display to read the digits
directly; the video check is the reference.

### Real-trial practicality

- **Cost:** phone about $80 to $100, pencil and notepad under $5. Nothing is used up; reset: wheel at rest, pencil back
  on the notepad, phone on its tape marks.
- **Forces:** about 0.7 to 1.6 N at the pencil's end, a 6 g pencil: far inside the YAM's limits.
- **Safety:** the pencil is unsharpened (no point); its eraser end goes in the holes. Nothing sharp, liquid or
  greasy. The phone needs no power for the trial (the dial is mechanical).

## Scripted demo

`dialdemo.py`, played by `yam_scripted` (the right arm; the left arm stays at its ready pose):

1. **Pick up the pencil** from the top, 35 mm from its middle toward the eraser, the fingers closing across it
   (about 17.5 N each), and lift it 10 cm.
2. **Turn it eraser-down:** the wrist pitches the gripper from pointing down to pointing forward, so the pencil
   hangs vertically from the fingers with its eraser end about 60 mm below them.
3. **Each digit:** the eraser goes 30 mm straight above the digit's hole, then straight down into it, 3.5 mm above
   the dial face. It is dragged clockwise round the hole circle at up to 90 degrees a second, the commanded point
   at most 6 degrees ahead of the hole, until it presses on the finger stop (2 degrees past contact). After 0.25 s
   at the stop the eraser lifts straight up out of the hole (30 mm) and the wheel runs back on its own; the next
   digit starts when the wheel is at rest.
4. After the third digit the pencil is lifted clear and held.

Every control step (20 Hz) it reads the wheel angle, the pencil's pose and the gripper's pose (simulator state, as
scripted demos may; a person sees the same) and places the gripper so that the eraser goes where it is wanted,
with the gripper-to-eraser offset measured each step, so a pencil that turns a little in the fingers is followed.
The demo dials 4-1-1 in about 25 s (about 515 steps); the run limit is 1,200 steps.

### Contrast runs

`DEMO_CHEAT=short`: the naive dial, letting go of each digit 25 degrees (1.6 cm) before the stop: the 4 sends
three pulses (a 3), so the number is wrong from the first digit: **0.0** (seed 0; it dialed 3, then two pulls that sent no pulse).
`DEMO_CHEAT=hold`: keeps the eraser in the hole and rides the wheel back at 60 degrees a second instead of
letting it run back (the governor's rate is 300): the run-back is held, the digit is spoiled: **0.0** (seed 0; three spoiled digits).

## Scores

| Run | Score |
|---|---|
| Scripted demo, seeds 0, 4, 10 | 1.0, 1.0, 1.0 (4-1-1 dialed in about 25 s, about 515 steps) |
| Contrast `short` (lets go 25 degrees before the stop) | 0.0 |
| Contrast `hold` (rides the wheel back) | 0.0 |
| Dream-1, seed 0, 1,200 steps | 0.0 (it picked the pencil up and moved it toward the phone; no digit dialed) |

## Simulation fidelity

- **Phone:** one PhysX articulation standing loose (1.0 kg, rubber feet, friction 0.8): the body, its sloping face,
  number plate and finger stop are one rigid body of box and cylinder colliders; the finger wheel is a second body
  on a revolute joint about the face normal, rest stop at 0, travel 335 degrees. The wheel's finger holes are
  collided as ten pockets between radial webs, a hub and a rim (each pocket at least 15.8 mm wide at the hole
  circle, against the real 17 mm round hole); it is drawn as a disc with round holes.
- **Spring and governor:** applied as a joint torque each physics step (240 Hz): 0.025 N m + 0.008 N m/rad, and on
  the run-back only a brake that holds 300 deg/s (the real governor is a centrifugal brake behind a one-way
  clutch). Values chosen to match a real dial's feel (about 1 N at the hole, 10 pulses a second), not measured on
  the GPO 746; README gives how to measure them.
- **Pulses** are counted from the wheel's furthest angle, as the real dial's cam does; a pull counts a pulse per
  hole brought within 15 degrees of the stop.
- **Pencil:** a 190 x 7 x 7 mm box (yam_arena's own free box, 6 g) drawn as a hexagonal pencil with ferrule and
  eraser. Its friction with the pads is 2.0 (max combine): the simulated gripper's finger drive gives only about
  17.5 N per finger on a 7 mm part (it saturates at fully closed), the real YAM about 40 N; 2.0 x 17.5 N matches
  the real friction capacity of rubber pads on a painted pencil (about 0.9 x 40 N). With 1.0 the pencil twists
  between the pads under about 1 N at its end, which the real grip does not.
- **Handset** static (its coiled cord not simulated); the notepad is a fixed box.

## Assets and licences

- Robot and station: the YAM arm and station assets that ship with Isaac Lab-Arena's `yam_arena` embodiment
  (YAM arm model by I2RT Robotics, under its licence in `yam_arena/assets/yam/LICENSE.i2rt`; the station from the
  Robocurve simulation walkthrough). Not redistributed here.
- Everything else (phone, dial, handset, notepad, pencil) is built from USD primitives and a generated mesh in
  `dialkit.py` and `task.py`, written for this task. No third-party meshes, scans or textures.
- The real-trial items are bought products (above); none of their designs is copied into this folder.
