# Unlock a reverse-turning lock without forcing the key

A light grey box stands loose on the table, between the arms and a little toward the right one. It has an upright
handle fin on its left side. A cam lock sits in its front deck with an orange key standing upright in it, and a blue
sliding lid covers the rest of the box. The lock opens **clockwise** seen from the key side, the opposite of what
people expect. Counter-clockwise, the key sits against a solid stop. The key is held on by a shear pin, so forcing it
the wrong way breaks the pin, and the box can then never be unlocked. The box is loose and light: turning the key or
sliding the lid with one hand spins or drags the box, so the other arm has to steady it.

Instruction: `Unlock the box with the orange key, then slide its blue lid open.`

The instruction does not say which way the key turns. Cam locks are fitted either way, so the usual direction
may be wrong. The task asks whether a policy that meets the hard stop backs off and tries the other way, or keeps
forcing. It also asks whether the policy steadies the object it is working on. Dream-1 works from joint positions and cameras. It can
see the stall: the bow does not move against the dial ticks, and measured joint6 lags its command.

## Why this lock: grounding in a human convention

- **A real reversed lock.** Quarter-turn cam locks (mailboxes, filing cabinets, tool chests, lockers, cash boxes)
  are sold for either rotation, and installers flip the cam or stop to suit the door. Most people expect "turn
  counter-clockwise to open", from screw caps, taps and most locks. A clockwise-to-unlock cam lock is common in
  real life and goes against that habit. This is the case we build.
- **No arrow, no hint.** The dial is 24 unlabeled ticks every 15 degrees: it shows how far the key has turned,
  not which way unlocks. Real cam locks usually carry no direction marking, and the instruction names no direction.
  A person who turns the usual way and feels the key stop tries the other way. Forcing it snaps the key.
- **A loose, light box.** Cash boxes, document boxes and tool cases sit loose on a desk. People hold them with one
  hand while they turn the key with the other.
- **Category: Societal relatability.** Most people unlock a mailbox, filing cabinet, locker or cash box with a key,
  and a lock that turns the unexpected way is a familiar annoyance. The risk is to the key and the box, not to a
  person, so the task is not filed under safety.
- **Distinct from "Lock a bayonet cap into its socket" (pending).** That task is an assembly task: push and twist a
  quarter turn, with the direction given in the instruction. Here the instruction names no direction, and the risk
  sits in the habitual direction. Over-torque counts as damage and scores 0.

## Torque balance: why one hand is not enough

The box weighs 0.70 kg in all and stands on four hard feet at the corners of an 18 x 14 cm rectangle, with friction
μ ≈ 0.3 on the table. The feet are 11.4 cm from the centre.

| | Value |
|---|---|
| Box friction torque about its centre, μ·m·g·r = 0.3 · 0.70 · 9.81 · 0.114 | **0.235 N m** |
| Key: breakaway peak / running / near the end stop | **0.40 / 0.30 / 0.40 N m** |
| Box sliding friction, μ·m·g | **2.06 N** |
| Lid rail friction | **3 N** |
| Shear pin | **0.6 N m** |

- The key needs more torque than the box's friction holds at every angle past the first few degrees: 1.3 times while
  running and 1.7 times at the peak. Turned one-handed, the box spins with the key, and the key turns relative to
  the box only as far as the box is steadied.
- Pulling the lid one-handed drags the box, because the rail friction is 1.5 times the box's sliding friction.
- An unheld box spins at 0.235 N m, long before the pin's 0.6 N m. So forcing the wrong way with the box unheld
  spins the box, which the box rule below scores. Forcing it with the box held breaks the pin and scores 0.
- One loophole is legitimate. Pressing down on the key about 4.8 N harder than its weight raises the friction torque
  to 0.40 N m, so one hand pushing down while turning can also hold the box still. People do this, and the scoring
  only asks that the box does not end displaced. The demo uses the second arm.

The test `test_one_handed_turning_or_pulling_moves_the_loose_box` checks these inequalities.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the lock box, `score()` / `success()`, and `demo()` (played by `yam_scripted`) |
| `lockkit.py` | The articulated lock box (floating base, key joint, lid joint), the lock mechanics (`LockParams`, `Lock`), and the hooks into yam_arena |
| `test_reverse_turn_lock.py` | Tests: torque balance, resistance curve, key and lid friction, catch, stop and shear-pin latch, box latches, every scoring outcome, and the demo's reach and joint limits for both arms |
| `task.json` | Submission details |

`lockkit.py` hooks yam_arena without modifying it. It patches `yam_arena.scene.task_object` and `object_states`
when that module is imported, and the embodiment's `_ensure_sim` and `reset`. It also wraps the environment's
`scene.write_data_to_sim`, which the env calls once per physics substep, so the lock's efforts are updated at
240 Hz. They are joint efforts, which act between the key or lid and the box, so the reactions reach the loose box.
The task runs with the standard `yam_arena` embodiment, `yam_scripted` policy and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unlock the box with the orange key, then slide its blue lid open." --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/reverse-lock-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unlock the box with the orange key, then slide its blue lid open." --scorer task_score \
  --max-steps 600 --store-frames --log-dir ~/yam-eval/reverse-lock-demo
```

The demo reaches success at about step 445 (22 s); the plan itself is 24 s long:

1. **Left arm** grips the handle fin at x 30, y 3, fingers across its 1 cm thickness, and holds the box still until
   the right arm has let go of the knob.
2. **Right arm** waits over the key until the left has gripped. It then grips the bow at x 24, y -10 with the
   fingers closing across the bow's broad faces (grip yaw 90 degrees, since the bow starts along y).
3. It tries the conventional way: 4 degrees counter-clockwise. In simulation this puts 0.23 N m on the stop, under
   the 0.6 N m pin. It holds there for 0.5 s.
4. It turns 92 degrees clockwise with wrist roll (joint6). The cam leaves the notch at 75 degrees, and the end stop
   is at 95 degrees.
5. It lets go of the key, grips the knob and slides the lid 9.5 cm toward -y. Then both arms let go.

`yam_scripted` plays fixed waypoints, so this demo is open loop: it always reverses after the probe. It cannot
branch on a sensed stall. A closed-loop version would need its own runner outside `yam_scripted`, and none is
included. The demo is written for the submitted lock. The mirrored variant (`reversed=False`) would need the key's
rest orientation changed to keep joint6 within its limits.

**Keep-forcing contrast (expected score 0):** the same plan, but with a 60-degree counter-clockwise turn instead of
the 4-degree probe:

```bash
DEMO_FORCE_WRONG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unlock the box with the orange key, then slide its blue lid open." --scorer task_score \
  --max-steps 600 --store-frames --log-dir ~/yam-eval/reverse-lock-forced
```

The left arm holds the box, so the stop torque passes 0.6 N m (0.67 in simulation) and the pin latch trips. The
key then swings free, and the lid stays locked. A third contrast, `DEMO_ONE_HAND=1`, is described under
"Checked in simulation".

Joint ranges from a forward and inverse kinematics check on the arm's URDF:
- Right arm: joint1 is about 40 degrees at the key. Joint6 goes from -50 (grip) to -53 (probe) to +42 (unlocked),
  and to -109 for the 60-degree forcing, against a ±120-degree limit.
- Left arm: joint1 is about -43 degrees at the fin.

The test `test_the_demo_is_reachable_within_the_joint_limits_and_inside_the_run_limit` checks both plans with
yam_arena's own IK.

## Scoring

Evaluated every control step. The score at the end of the episode counts (`SCORE_OVER = "end"`).

| Condition | Score |
|---|---|
| Shear pin broken (stop torque over 0.6 N m for 12.5 ms), a locked lid pulled more than 1 cm open (pried), or the box at any point tilted more than 20 degrees or dropped 5 cm below its start (off the table) | 0 |
| Lid slid open 8 cm or more after unlocking (success, must hold 1 s) | 1.0 |
| Lid open 2 cm or more after unlocking | 0.75 |
| Key ends 75 degrees or more clockwise of its start (unlocked) | 0.5 |
| Key ends 30 degrees or more clockwise (past the breakaway detent) | 0.25 |
| Anything else | 0 |
| **Cap:** the box ends with its centre more than 3 cm from its start, or turned more than 15 degrees | at most 0.5 |

**Why tipping and falling score 0 but displacement only caps.** A box that tips or falls has been lost: in real use
its contents spill or it drops off the desk, a hazard on a par with breaking the key. A box that skidded or spun
while being worked on means the policy did not steady it. In real use that is a lapse of control, but the lock and
box are unharmed and the job may still get done. So it limits the score instead of zeroing it. 0.5 is the
"unlocked" level: a policy that opens the lid without steadying the box gets no more credit than one that unlocks
it cleanly. The 3 cm / 15 degree margin covers the jostling a firm two-handed grip still causes, so the score does
not depend on small, irrelevant differences. The cap uses the end position, so a box nudged and put back is not
penalised.

In a real trial:
- **Key angle:** read from a top photo of the dial (ticks every 15 degrees; 30 and 75 are ticks).
- **Lid travel:** measure with a ruler from a pencil mark at its start.
- **Box displacement:** read from the same photo against the taped outline. The centre is where the outline's
  diagonals cross; turn is the front edge against the tape.
- **Tipping and falling:** read from the video.
- **Broken pin:** visible after the trial. The key turns freely with a light touch, and the pin halves are offset.
  The video also shows the bow swinging more than 15 degrees counter-clockwise of its start tick, which it cannot
  do while the pin is intact (it gives at most about 3 degrees before shearing).
- **Prying:** a rigid catch cannot be pried more than 1 cm without breaking it, which is also visible.

Ways to cheat get nothing:
- The key and lid are captive.
- A locked lid gives at most a few mm, and lid credit needs the unlock.
- Breaking the key always scores 0, even if the lid ends open.
- Knocking the box over scores 0, and pushing it around caps the score.

## Resistance curve and parameters

All in `lockkit.LockParams`. `task.PARAMS = LockParams()` is the submitted lock. Angles are in degrees in the
unlock sense; torques are in N m.

| Parameter | Submitted value | Meaning |
|---|---|---|
| `curve` | `(0, 0.20), (6, 0.40), (15, 0.30), (70, 0.30), (90, 0.40)` | Friction limit vs key angle, piecewise linear: a breakaway peak of 0.40 at 6 degrees, 0.30 running, then rising to 0.40 as the cam drives the bolt back |
| `reversed` | `True` | Unlocks clockwise seen from above; `False` gives the conventional lock (a control variant) |
| `stop_deg`, `stop_stiffness` | 0, 12 N m/rad | Wrong-way stop at the start position, and its give (about 3 degrees at the pin's strength) |
| `break_nm`, `break_hold_steps` | 0.6, 3 | Shear-pin strength, and how many 240 Hz steps it must be exceeded in a row |
| `unlock_deg`, `open_end_deg` | 75, 95 | The cam leaves the lid's notch / solid end stop (joint limit) |
| `broken_travel_deg` | 45 | How far the decoupled key swings the wrong way after shearing (joint limit) |
| `stick_stiffness` | 20 N m/rad | Key friction pre-slip stiffness |
| `lid_friction_n`, `lid_stick_stiffness` | 3 N, 2000 N/m | Lid rail friction and its pre-slip stiffness |
| `catch_stiffness`, `catch_engage_m`, `pry_m` | 3000 N/m, 5 mm, 1 cm | Cam holding a locked lid; the lid re-latches when shut to within 5 mm with the key under 75 degrees; prying threshold |
| `key_damping`, `key_armature`, `lid_damping`, `lid_armature` | 0.1, 0.002, 10, 0.05 | Numerical stability of the 240 Hz explicit effort update |

Box parameters are in `task.py`: `LockBox(mass=0.55, feet_friction=0.3, ...)` and the scoring constants `TIP_DEG`,
`FALL_M`, `SHIFT_M`, `BOX_TURN_DEG` and `UNSTEADY_CAP`.

The unlock-side curve peaks at 2/3 of the pin strength. Any variant must keep both orderings: box friction torque
below the running torque (so the second arm is needed) and the curve peak below `break_nm` (so a correct turn never
breaks the key). To make a less forgiving key, lower `break_nm` or raise `stop_stiffness`. Variants must keep one
fixed layout per submission.

## Runtime and seeds

On an RTX workstation, a full demo run (Isaac Sim start, reset, about 445 control steps and the episode end) took
about 2 minutes of wall time. A 1,200-step policy run takes longer, depending on the policy's latency. The layout
is fixed, so always run with `-E object_jitter=0`. A seeded reset with jitter would shift the loose box and break
the open-loop demo. There is no other randomness and no seeds to choose.

## Tests

```bash
pytest test_reverse_turn_lock.py      # in the yam_arena environment; no GPU needed
```

## Real-world build

All printed parts are PLA. Torques and forces here are small, the box weighs 0.7 kg, and nothing is sharp or
spring-loaded beyond a ball detent, so trials are safe.

- **Box**: 20 x 16 x 6 cm, 6 mm walls, with four 1.2 x 1.2 x 0.3 cm feet at the corners of an 18 x 14 cm
  rectangle. Rails on the side walls for the lid. Bring the box with deck, feet and handle to 550 g with a little
  ballast under the floor if the print is lighter. Check the feet friction on the actual table: a 0.70 kg box (with
  key and lid) should start to slide at about 2 N (μ 0.25 to 0.35). Polish or tape the feet to adjust.
- **Handle**: a 6 x 1 x 10.7 cm fin on a 6 x 5 x 1 cm bracket glued to the left wall's foot.
- **Deck**: 8 x 16 x 0.6 cm, glued on.
- **Lock**: a printed plug (12 mm diameter) turning in the deck. Under the deck, a 2 cm cam arm on the plug reaches
  into a notch in the lid's front edge when the key is at 0. Counter-clockwise, a solid stop block holds the plug
  at 0. Clockwise, the cam stops at 95 degrees.
- **Resistance**: a spring ball plunger (M6) presses on the plug with a dimple at 6 degrees, giving the breakaway
  peak. An O-ring on the plug gives the running friction. Tune both to the curve with a luggage scale on a string
  wound on a 2.5 cm-radius spool on the key: 0.40 N m is 16 N, and 0.30 N m is 12 N. Lid rails: about 3 N to slide
  (a felt strip in the rail).
- **Key and shear pin**: the orange bow (5 x 1 x 4 cm) on a 7 mm shank that sits in the plug. A 9 mm piece of
  1.75 mm PLA filament goes through matching cross-holes in the shank and plug and carries all the torque. In double
  shear across a 7 mm shank it fails at roughly 0.6 N m. Calibrate each filament batch on a spare plug: the pin must
  hold 0.45 kg hung 10 cm from the axis (0.44 N m, above the 0.40 N m curve peak) and break before 0.8 kg
  (0.78 N m). Adjust the shank diameter if it does not.
- **Dial**: a printed paper ring of 24 ticks every 15 degrees, 2.9 to 3.5 cm from the axis, with no words or
  arrows.
- **Lid**: 12 x 16 x 0.6 cm plate with a 3 x 3 x 2.5 cm knob at its centre, sliding 10 cm toward -y. Mark its start
  edge in pencil for measuring travel. Tape the box's start outline on the table.

## Checked in simulation

Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1) on an RTX workstation, with the commands above and `--max-steps 600`.
`LOCK_DEBUG=1` prints the lock state every 10 control steps (key angle, lid travel, box pose, stop-torque peak and
latches); it is off by default.

| Run | `task_score` | What the lock log and video show |
|---|---|---|
| Scripted demo | **1.0** | The probe peaks at 0.23 N m on the stop (pin 0.6). The key turns to about 88 degrees and unlocks at 75. The lid slides 8.6 cm. The box, held by the left arm, ends 0.7 cm off and turned 1.5 degrees. Success at about step 445 |
| `DEMO_FORCE_WRONG=1` | **0** | Stop torque reaches 0.67 N m, the pin latch trips, and the key swings free to 45 degrees counter-clockwise. The later clockwise turn no longer releases the lid. The right arm's pull on the still-locked lid drags the held box 4 cm |
| `DEMO_ONE_HAND=1` (left arm idle) | **0** | The box turns with the key, 87 degrees about the table, while the key turns 2 degrees relative to the box. The lid is never reached |

`DEMO_ONE_HAND=1` is a third contrast: the same right-arm plan, with no one holding the box.

```bash
DEMO_ONE_HAND=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unlock the box with the orange key, then slide its blue lid open." --scorer task_score \
  --max-steps 600 --store-frames --log-dir ~/yam-eval/reverse-lock-one-hand
```

The tests (`pytest test_reverse_turn_lock.py`) pass in the same environment.

## Still approximate or unverified

- The 4-degree probe is real but hard to see on video: the bow simply does not move for 1.5 s. The lock log shows
  the stop torque.
- Contact friction on four small feet, and how the grippers load them, is simulated, not measured. The one-handed
  run shows the box spinning as the torque balance predicts. The real box's feet friction must be checked as
  described in the build notes.
- The real shear-pin strength scatters with filament and print; calibrate per batch.
- One Dream-1 run (1,200 steps, `-E object_jitter=0`) scored 0: only the right arm moved, reaching over the box toward the key in the last 20 s without turning it, and the run hit the step limit.

## Simulation fidelity (full)

Approximate. The box, key, lid and handle are rigid boxes. The box is a loose (floating-base) articulation with a revolute key joint and a prismatic lid joint. Its feet carry their own friction, 0.3 with the table's 'min' combine, and the rest is grippy. The lock is not modelled as geometry. Every 240 Hz physics step, the key joint gets a torque and the lid joint a force. Both act between the part and the box, so the box takes every reaction. The key torque is friction whose limit follows the resistance curve (elasto-plastic, about 1 deg of pre-slip), plus the counter-clockwise stop as a stiff spring (12 N m/rad). The lid force is 3 N of rail friction, plus the cam as a 3000 N/m spring until the key passes 75 degrees. So a hard pull opens a locked lid a few mm, and past 1 cm counts as prying (score 0). The shear pin is a latch: damage when the stop torque exceeds 0.6 N m for 3 physics steps (12.5 ms) in a row. After that the key swings freely up to 45 degrees counter-clockwise and never releases the lid. The clockwise end stop at 95 degrees is a joint limit. The key and lid joints have added armature and light damping for numerical stability. Box tilt and drop are checked once per control step. Real shear-pin strength and foot friction scatter with print, filament and table surface (calibrate per batch, see README). No breakage of other parts is modelled.
