# Secure a clevis pin by bending its cotter pin's legs

A clevis joint is clamped on a steel stand in front of the right arm: a fork clevis holds a yellow rod end on an
upright 8 mm clevis pin. A 2 mm steel cotter pin has been pushed fully home through the cross-hole near the pin's
top, eye toward the arms, but its two legs are still straight and lie together. A straight cotter pin shakes out of
its hole under vibration; the clevis pin then walks out and the linkage comes apart. The joint is only secure once
both legs are bent, one each way.

Instruction: `Finish securing the clevis pin with its cotter pin, then move both arms clear.`

The instruction states the goal only. It does not say that the legs must be bent, which way, or how far.

## The real parts

| Part | Real item | Modelled as |
|---|---|---|
| Cotter pin | ISO 1234 2 x 25, low-carbon steel, zinc plated. Wire 1.8 mm (two 0.9 mm half-rounds), eye 3.6 mm wide, length 25 mm under the eye. The long leg is 4 mm longer (the "extended prong" b). 1.0 g | Eye and shank fixed in the cross-hole. Legs: 0.9 x 1.8 mm boxes, 17 mm (short) and 21 mm (long) past the pin's far surface, 0.17 g and 0.21 g |
| Clevis pin | ISO 2341-B 8 x 22, steel, 2 mm cross-hole 3 mm below its end; head 14 mm | 8 mm cylinder, head 14 x 3 mm (fixed) |
| Washer | ISO 7089 8, 16 mm OD, 1.6 mm, steel | Cylinder (fixed) |
| Fork clevis | Steel fork clevis for an M8 rod (DIN 71752 type), 8 mm slot, prongs 4 mm thick and 16 mm wide | Boxes (fixed) |
| Rod end | Yellow painted steel flat bar 12 x 7.6 mm between the prongs, 12 cm long | Box (fixed) |
| Stand | Steel blocks clamped to the table under the fork and under the rod's far end | Boxes (fixed) |

Positions in cm from the midpoint between the arm bases (x forward, y left): the clevis pin's axis at x 30, y -8;
the cotter's axis 10 cm above the table, along +x; the legs point away from the arms. The long leg is on the left
(+y) side. Only the stand touches the table.

## Bending force: why a YAM can do it

A cotter leg bends where it leaves the hole. Fully yielded, a half-round of radius r = 0.9 mm takes the plastic
moment M_p = σ_y Z_p, with Z_p = 0.354 r^3 = 0.258 mm^3 (bending about the flat face's direction, the way the legs
spread).

| Quantity | Value |
|---|---|
| Yield stress σ_y, annealed low-carbon cotter wire | 300 MPa (assumed; 250-400 MPa covers soft to half-hard wire) |
| Plastic moment M_p at the root | 0.077 N m (0.065-0.103 over that range) |
| Hardening | yield moment +25% over the first 90 degrees of bend |
| Elastic stiffness 3EI/L (E 200 GPa, I 0.072 mm^4, L 17 mm) | 2.5 N m/rad, so 1.7-2.2 degrees of springback |
| Side force to bend the long leg, pushed 19.5 mm from its root | 4.0 N, rising to 5.0 N at 90 degrees |
| Side force to bend the short leg, pushed 12.5 mm from its root | 6.2 N, rising to 7.7 N at 90 degrees |
| Same, with 500 MPa wire (worst case) | under 13 N |
| YAM rated payload / gripper force | 15 N / 50 N |

So the forces are real but small: a fingertip push within the YAM's rated payload. The difficulty is where and how
the push is made, not its size:

- **The legs start together.** Only the long leg's 4 mm extension, past the short leg's end, can be pushed from the
  inside. The fingertip must land in a window about 3 mm long, beside a 0.9 mm wire.
- **One each way.** The short leg can only be pushed once the long leg has moved out of the way, from the side the
  long leg has left.
- **Swing round the root.** A push that does not follow the leg round its root slides off its end. The arm's joints are
  compliant (PD position control): the finger lags its command by millimetres under a few newtons, so a policy has to
  push past where it wants the leg and allow for springback.
- **No second chance.** A leg that is bent and then straightened again is a re-bent cotter, which fatigue-cracks at
  the bend and must be scrapped. The trial scores 0.

## Scoring

Read at the end of the trial, with one latch over the whole trial. A leg's bend is the angle between it and the
cotter's axis (a protractor on a top-down photo).

- **0** if either leg was ever re-bent: bent back 15 degrees or more after it had been set 30 degrees or more.
- **1.0 (success)**: both legs bent 60 degrees or more, every finger 2 cm or more from the cotter, legs still (under
  5 deg/s), held for 1 s.
- **0.75**: the same at the end of the run but not yet held 1 s.
- **0.5**: both legs bent 60 degrees or more, but a finger is within 2 cm or a leg is moving.
- **0.25**: one leg bent 60 degrees or more.
- **0** otherwise.

The legs can only open away from each other (they lie flat together and the cotter is in the hole), so "one each
way" needs no separate check. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

## Files

- `task.py`: layout, scoring and the scripted demo.
- `cotterkit.py`: the clevis-and-cotter articulation, the legs' elastic-plastic bending and the hooks into yam_arena.
- `test_cotter_pin_clevis.py`: unit tests (bending numbers, the leg model, scoring, demo geometry). No Isaac Sim.

## Run a trial

From `runs/` with the yam_arena environment (`source IsaacLab-Arena/yam.env`):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E device=cuda:0 \
      -E task=$PWD/../tasks/cotter-pin-clevis/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 --store-frames --scorer task_score \
      --instruction "Finish securing the clevis pin with its cotter pin, then move both arms clear." \
      --max-steps 900 --seed 0 --log-dir cotter

Swap `--policy` for the model under test. The layout is the same for every seed (`object_jitter=0`). Runtime:
about 6 minutes per 900-step trial on a shared RTX A6000, start-up included. `COTTER_DEBUG=1` logs the legs' bend, set and peak moment and the right
arm's tool point every 5 control steps.

Tests: `pytest tasks/cotter-pin-clevis/test_cotter_pin_clevis.py` (yam_arena environment).

## Scripted demo

`demo()` in task.py, played open loop by `yam_scripted`: the layout is fixed, so it is a fixed plan and reads
nothing from the simulator. Right arm only:

1. Fingers 4 mm apart, down astride the long leg's 4 mm extension (the pads start 0.6 mm past the short leg's end,
   tips 4.5 mm below the legs), then swing round the leg's root toward +y, the trailing pad pushing. The arm lags
   its command by over a centimetre under the ~4 N this takes, so the commanded 70 degrees bends the leg about 18
   degrees before the pad slides off its end. That is enough to part the legs.
2. Down astride the long leg along its length, grip it like pliers and swing it round its root to 100 degrees
   (commanded), the gripper turning with it. Gripped, the bending moment is taken mostly as a twist at the wrist,
   which lags far less than a push. The leg ends about 73 degrees out.
3. The same on the short leg, now free on both sides, toward -y. It ends about 67 degrees out.
4. Open and move clear.

What we learned building it, and what a policy faces: a closed fingertip pushed sideways against a 1.8 mm wire
rides up over it (the fingertips' side edges slope); a push far from the root slides off the leg's end as it turns;
a push near the root needs more force, and the arm's lag grows with it. Gripping the leg works.

Results (Isaac Sim, RTX A6000, `--max-steps 900`):

| Run | Seed | Long leg | Short leg | Score |
|---|---|---|---|---|
| Demo | 0, 4, 10 | 73.0 deg | 67.4 deg | 1.0 each (success held 1 s at step 715) |
| `DEMO_LONG_ONLY=1` | 0 | 73.0 deg | 0 | 0.25 |
| `DEMO_REBEND=1` (straighten the long leg from 73 to 45 deg, bend it out again to 88, then the short leg; `--max-steps 1250` so it can finish) | 0 | 87.9 deg | 67.5 deg | 0 (re-bent) |
| Closed fingertip pushed sideways (earlier versions of the demo) | 0 | under 1 deg, or 18 deg then slid off | 0 | 0 |

The layout does not change with the seed, so the three demo seeds match to 0.1 degree. During the pliers grip the
long leg is first pulled back from 17.6 to 8.7 degrees (the gripper closes slightly off-centre); that is under the
30 degrees after which bending back counts as a re-bend.

Dream-1 (seed 0, 900 steps): 0.0. Both arms stay near their bases, folded low, and never reach the clevis; the
legs stay straight.

## Real-world trial

Clamp the stand to the table; the clevis, pin and washer are bolted together and do not move. Use new ISO 1234
2 x 25 zinc-plated steel cotter pins from one box, one per trial; check one per box with a bend test: clamp a leg at its root and hang
a mass 12.5 mm out. With M_p about 0.08 N m, 0.5 kg (0.06 N m) should leave no permanent bend and 1 kg (0.12 N m)
should bend it steadily. Push the cotter fully home, legs together and the long leg on the left, before each trial. Score from a
top-down photo with a protractor overlay. No person is near the robot during a trial. Observers wear safety glasses
(a leg could snap off a brittle cotter); the forces involved are under 15 N.

Reset: pull the cotter out with pliers and discard it (a bent cotter is not reused), push a new one home.

## Why this task, and how it differs

Category: Impact on jobs. Securing clevis pins with cotter pins is routine for farm equipment, heavy equipment and
industrial machinery mechanics (O*NET 49-3041, 49-3042, 49-9041) and for aircraft mechanics, whose practice (FAA AC
43.13-1B, chapter 7) is to bend the prongs one each way and never to straighten and re-use a cotter.

- **The core skill is plastically deforming a fastener after it is in place**, with small, precisely placed pushes.
  Our pending shackle tasks (screw a shackle pin fully home through a towing lug) are insertion and screwing; nothing
  there deforms. "Line up a castle nut, then pin it" (rejected) inserted a cotter pin but left its legs straight;
  "Install wheel on axle and lock with lynch pin" (rejected) is a sprung clip. "Pick the pin that fits the hole and
  seat it" (accepted) is peg insertion.
- **External benchmarks.** LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench and RoboDojo have pick-and-place,
  articulated objects (doors, drawers, knobs), insertion and cable routing; none asks a robot to bend a metal part
  to a permanent set. The nearest are deformable-object tasks (cloth, rope, dough), which are elastic or plastic
  continua handled by grasping; here a stiff part must be bent past its yield moment by a fingertip push.

## Simulation fidelity

Approximate. Isaac Sim (PhysX), 240 Hz physics, 20 Hz control. The clevis, pin, washer, rod end, stand and the
cotter's eye and shank are one fixed body; the cotter cannot be pulled out of its hole (a real one could, with
pliers). Each leg is a rigid 0.9 x 1.8 mm box on a revolute joint at its root (where it leaves the pin), limited to
0-100 degrees; real legs bend over a short length, not at a point. The joint's implicit drive has the leg's elastic
stiffness, 2.5 N m/rad, and its target is the leg's permanent set, updated every physics step: elastic-perfectly
plastic with linear hardening, yield moment 0.077 N m rising 25% over 90 degrees. Bending back past yield
straightens the leg (and latches a re-bend). There is no fatigue cracking, no twisting of a leg about its own axis
and no plastic flow in any other direction. The cross-hole is not modelled (the shank is fixed). Steel friction 0.6.
Each leg has 20 g in the solver (real: 0.17 and 0.21 g): a 0.2 g link gives PhysX too weak a contact against the
PD-driven finger, which then passes through it. The legs turn only about the vertical, so their weight does nothing,
and at a few mm/s their inertia adds well under 0.01 N.

No third-party assets: every part is a box or cylinder defined in task.py.

## Credit

The idea for this task was first proposed in robocurve/benchmark-tasks#994 (TK-T6U364). The code, models and text here are an independent implementation.
