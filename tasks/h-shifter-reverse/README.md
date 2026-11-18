# Change gear 1-2-3 through a hidden H-gate, then find reverse

A gear shifter with a six-speed H-pattern and push-down reverse (Logitech G Driving Force Shifter) is bolted to a
plywood board on the table in front of the right arm. The lever is in neutral.

Instruction: `Change gear from neutral into first, then second, then third, then reverse. Leave the lever in reverse.`

## Why this task

Changing gear in a manual car is one of the most familiar hand skills there is: most adults have done it, or watched
it from the passenger seat. What makes it a skill, and not a reach-and-place, is that the gate is felt, not seen:

- **The gate is hidden.** The H-shaped slots are under the leather boot. The lever only goes fore-aft where it is
  lined up with a slot, and only sideways in the neutral channel between the rows. Pushing on a diagonal (straight
  from 2 to 3) jams the lever against the gate.
- **Springs pull the lever back to the 3-4 plane.** First needs a push across against the centring spring and
  then forward; second is a straight pull back through neutral, keeping the sideways push on; third means easing
  that push at neutral so the spring brings the lever to the middle plane, and then going forward.
- **Reverse is behind a lockout.** It shares the 6 slot. Only with the knob pushed down about 5 mm as the lever goes
  across and back does the box select R; without it the lever goes into 6. That is two force directions at once,
  down and sideways-back, held while moving.
- **Constrained order.** 1, 2, 3, R; a wrong gear on the way ends the credit.

The same gate, centring springs and push-down reverse are found in many cars' gearboxes (VW Golf and Polo push down
for reverse, for example); the robot works the lever the way a driver does, by feel, with one hand round the knob.

### Nearest existing tasks and why the ability differs

- Board (accepted): extending a suitcase handle past its first stop with the button held (TK-28LP2A) and pulling a
  drawer off its slides with two release levers pressed (TK-FN6RF6). Both are one straight pull with a release held.
  Here the path has four segments through a 2-D gate whose walls are not visible, against a centring spring, with
  the lockout pushed in a different direction from the motion.
- Board: no gear lever, joystick or H-gate task (searched "shifter", "gear lever", "gearstick", "H-pattern",
  "reverse", "gate").
- External: LIBERO, RoboCasa, BEHAVIOR-1K and SimplerEnv lever tasks ("push the lever down", "turn on the stove")
  move one joint to its stop; none follows a hidden 2-D gate or uses a push-down lockout.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the shifter, the gate and springs at run time, `score()` / `success()`, `demo_controller()` |
| `shiftmodel.py` | Pure Python: the gate geometry and limits, the springs, the gear readout (`Gearbox`) and the score (`Sequence`) |
| `bikekit.py` | Multi-shape parts, a fixed-base articulation with chained joints, per-step joint efforts and the hooks into yam_arena (our kit from an earlier task, unchanged) |
| `shiftdemo.py` | The closed-loop scripted demo that `yam_scripted` plays |
| `test_h_shifter_reverse.py` | Unit tests of the gate, the readout and every scoring outcome (no Isaac Sim needed) |
| `task.json` | Submission details |

## Setup

Centimetres from the midpoint between the arm bases, x forward, y left, heights above the table top.

- **Shifter:** Logitech G Driving Force Shifter, part 941-000119 (black plastic, 14.6 cm deep x 17.6 cm wide x
  20.7 cm tall, 0.72 kg). Six speeds in an H-pattern plus reverse: 1-2 in the left plane, 3-4 middle, 5-6 right,
  odd gears forward. Reverse: push the knob down and go to the 6 position. Black leather knob, about 4.5 cm across
  and 5 cm tall, with the gear pattern printed on its top; knob centre about 19 cm above the table. Lever pivot
  under the boot at (27, -21), so the knob is in front of the right arm. Fore-aft throw about 3 cm each way at the
  knob, planes about 2.2 cm apart.
- **Mounting:** the shifter is bolted through its two underside M6 holes (Logitech's rig mounting points) with two
  M6 x 20 mm screws to a 30 x 25 cm board of 12 mm plywood, which is taped to the table with double-sided carpet
  tape. Draw the shifter's outline on the board in pencil.
- **Reading the gear:** the shifter's DE-9 cable goes to a USB adapter (an Arduino Leonardo running the open
  Driving-Force-Shifter-USB-Adapter firmware, GPL-3.0, or any commercial G29-shifter-to-USB adapter) and on to a
  laptop on a side table, out of the arms' reach, showing the game-controller test page: each gear lights as a
  button.
- **Robot:** both arms in the ready pose, grippers open. The left arm is not needed.

## Mechanisms (what the simulation does each 240 Hz physics step)

The lever is a fixed-base articulation of three joints: `select` (the lever tipping sideways about a fore-aft
axis through the pivot), `shift` (tipping fore-aft, chained on `select`) and `press` (the knob sliding 6 mm down
along the lever). Lever 12.5 cm pivot to knob centre.

- **Hidden gate as joint limits.** Each physics step `gate_limits` sets the two hinges' limits from where the knob
  is: in a slot (more than 4 mm from neutral fore-aft) it can only move along that slot (8 mm wide); in the neutral
  channel it moves sideways over the whole gate, but fore-aft only where it is lined up with a slot within 4 mm.
  PhysX enforces limits as hard stops, so a push on the wrong line stops dead, as a gate does.
- **Springs (efforts).** Sideways centring toward the 3-4 plane: 2 N + 100 N/m from the 1-2 side (4.2 N at the
  plane), 3.5 N + 150 N/m from the 5-6 side (6.8 N). Fore-aft detent: up to 6 N back toward neutral in the first
  half of a slot, then 6 N into the gear. Knob push-down: 6 N preload + 0.8 N/mm (10 N at 5 mm).
- **Gear readout (`Gearbox`).** A gear is engaged when the knob is 90 % of the way along its slot (27 mm from
  neutral). As the lever passes half-way into the 6/R slot, the box decides: knob pushed down 5 mm or more, R;
  otherwise 6. That holds until the lever leaves the slot; pushing down once in the slot changes nothing.

Spring forces are estimates for this class of shifter. Before trials, measure the real unit with a luggage scale
hooked on the knob (sideways at neutral toward 1-2 and toward 5-6; fore-aft pulling into a gear; straight down on the
knob), and set `SIDE_F0`, `DETENT_F`, `PRESS_F0` in `shiftmodel.py` if they differ by more than 2 N. All are well
inside a YAM gripper's reach (about 40 N per finger, a few kilograms of arm push).

## Scoring

Read at the end of the trial from the shifter's own output (the laptop's controller page, recorded on screen or
filmed with the trial) and the trial video:

| Condition | Score |
|---|---|
| First engaged and held 1 s or more | 0.25 |
| ... then second, held 1 s | +0.25 |
| ... then third, held 1 s | +0.25 |
| ... then reverse, and reverse still engaged at the end | +0.25 |
| Any other gear engaged 0.25 s or more on the way (4, 5, 6 instead of R, back to an earlier gear, reverse before third) | no further credit; what was earned stays |
| The shifter base moved off its pencil outline by 1 cm or more, or the knob came off | 0 |

Re-engaging the gear just counted (popping out of 1 and back into 1) costs nothing. 1.0 is success.
In simulation the base is fixed, so the last row cannot fire there; on the real board it catches a robot that
forces the lever hard enough to rip the shifter loose.

## Unintended strategies and what stops them

- **Reverse without the push-down.** The box reads 6, a wrong gear: the score stops at 0.75 (`DEMO_CHEAT=nopress`).
- **Into 6, then push the knob down.** Still 6: 0.75 (`DEMO_CHEAT=late`).
- **Straight lines from gear to gear, ignoring the gate.** The gate turns the push aside: aiming for 1 on a
  diagonal from neutral, the lever ends in 3 and the run scores 0 (`DEMO_CHEAT=diagonal`).
- **Going straight to reverse, or sweeping through every gear.** Reverse first, or 4/5/6 on the way, is a wrong
  gear: no further credit.
- **Leaving the lever in neutral after reverse.** Reverse must still be engaged at the end: the last 0.25 is lost.

## Run a trial

From a yam_arena install (Isaac Lab-Arena, `inspect-robots`):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E device=cuda:0 \
      -E task=tasks/h-shifter-reverse/task.py -E object_jitter=0 -E control_hz=20 \
      --scorer task_score --max-steps 1800 --seed 0 \
      --instruction "Change gear from neutral into first, then second, then third, then reverse. Leave the lever in reverse."

Use `--policy dream` for Dream-1. `DEMO_CHEAT=nopress`, `late` or `diagonal` plays a contrast run;
`SHIFT_DEBUG=1` prints the lever state. The layout is fixed; the seed only changes the policy's randomness.

## Scripted demo

`shiftdemo.py`, closed loop at 20 Hz, right arm only. It grips the knob from behind and above, leaning 30 degrees
back from the lever as a driver's hand does, the fingers closing sideways across it; before reverse, at neutral, it
turns upright so the push-down goes straight down the lever. From then on the hand's target is where the knob should be for a wanted gate position, with the
gripper tilted with the lever. It reads the lever's joint state every step (as a driver feels it) and adds an
integral correction for the arm's give against the springs. It follows the gate: it moves sideways only once the
lever is back within 2 mm of the neutral channel, fore-aft only once it is lined up with the wanted slot. Each gear
is held until the shifter has read it for 1.2 s. For reverse it pushes the knob down to its 6 mm stop at neutral, checks the
knob is down past 5 mm, then goes across and back slowly, keeping the push on. The hand stays on the knob, still pushing down, to the end of the run, so the shifter
reads R whether or not its lockout keeps the knob down by itself.

## Verified in simulation

Run with the command above (`object_jitter=0`, 20 Hz, 320 x 176 cameras):

| Run | Seed | Score | Notes |
|---|---|---|---|
| Scripted demo | 0 | 1.0 | 1, 2, 3 held from 8.3, 10.4, 14.2 s; R from 20.8 s; ends at 22 s (440 steps) |
| Scripted demo | 4 | 1.0 | |
| Scripted demo | 10 | 1.0 | |
| `DEMO_CHEAT=nopress` (reverse without the push-down) | 0 | 0.75 | the box reads 6: wrong gear |
| `DEMO_CHEAT=late` (into the 6/R slot, then push down) | 0 | 0.75 | still 6 |
| `DEMO_CHEAT=diagonal` (straight lines gear to gear) | 0 | 0.0 | heading for 1 on a diagonal, the centring spring and the gate send the lever into 3 |
| Dream-1 | 0 | 0.0 | full 1,800 steps; reaches toward the knob and moves round it, never engages a gear |

The demo's video is a rerun of seed 0 at 640 x 352 cameras (1.0).

## Runtime

About 3-4 minutes per trial in simulation on one GPU (Isaac Sim start-up included). A real trial takes about
2 minutes including the reset (pull the lever back to neutral).

## Tests

    pytest tasks/h-shifter-reverse/test_h_shifter_reverse.py

## Real-world build

| Item | What | Approx. price |
|---|---|---|
| Logitech G Driving Force Shifter, 941-000119 | the shifter; its own leather knob and boot | $60 |
| USB adapter | Arduino Leonardo (or Uno) wired to the shifter's DE-9 plug, running the open Driving-Force-Shifter-USB-Adapter firmware (GPL-3.0), or a commercial G29-shifter USB adapter | $25 |
| 12 mm plywood, 30 x 25 cm; two M6 x 20 mm screws with washers | mounting board | $10 |
| Double-sided carpet tape | board to table | $8 |
| Laptop with a game-controller test page (Windows "Set up USB game controllers"), luggage scale, pencil | reading the gear, checking the springs, marking the outline | (lab) |

Total about $105. No custom parts.

**Real-trial safety.** Nothing sharp, hot, wet or powered beyond USB 5 V. The shifter's springs are a few newtons;
the lever cannot pinch a gripper (the boot covers the gate). The laptop and cables stay on a side table out of the
arms' reach; the cable leaves the shifter at the back, away from the arm.

## Third-party assets

None in the simulation: the shifter, board and knob are boxes and cylinders built in `task.py`. The YAM arms,
grippers and station are the program's yam_arena embodiment (its own licence terms, unchanged by this task). For
the real trial only: the Driving-Force-Shifter-USB-Adapter firmware (GitHub, GPL-3.0), used as is.

## What is approximate

- The base is fixed. The real shifter is bolted to a board taped to the table; a robot can tear it loose only by
  pulling far harder than any gear needs (the pencil-outline rule).
- The gate is switched joint limits, not contact with a gate plate; the springs are joint efforts.
- R versus 6 is decided by a rule at slot entry (the knob's push-down at that moment). On the real unit the
  push-down moves the lever below a ledge in the gate and a switch reports R; check on the bought unit that the
  shifter keeps reading R with the knob let go once in (as the simulation does). If the real unit needs the knob
  held down to read R, a run must end with the hand still pressing it (the demo does).
- The knob is a plain cylinder; the real knob is rounder. Spring values are estimates (see Mechanisms).
