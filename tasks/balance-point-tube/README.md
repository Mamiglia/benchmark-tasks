# Hang a weighted tube level on a single conveyor hook

A sealed aluminium tube, 1 m long and 50 mm across, lies on the table in front of the arms. It has a red plug in its
left end and a blue plug in its right end. A steel slug is glued inside at a place that changes with the trial's seed,
so the tube's balance point is 6 to 14 cm from its middle, toward either end. Beyond the tube stands one hanging point
of an overhead conveyor: a J-hook hangs from an arm on a post, its cradle open toward the robot, 10 cm above the table
and only 4 cm wide. The tube hangs level from the hook only if its balance point is within about 2 cm of the cradle's
middle.

**Instruction:** `Hang the tube level on the hook.`

The instruction states the goal only. Nothing on the outside of the tube shows where the slug is.

## Why this task

- **The real job.** In powder-coating, paint and e-coat lines, and in dip and wash lines, parts are hung from hooks on
  an overhead conveyor. Long parts (tube, bar, extrusions, frames) are often hung from a single hook at their balance
  point. They must hang level: a tilted part coats unevenly (runs, sags, thin spots at the high end), drips from one
  end, swings, and can strike the next part, the oven entrance or the booth. The same holds for a single-point hoist
  or crane lift of a long part. Hangers do this many times a shift. A part with mass inside it (a welded-in fitting,
  a cast boss, a sealed weight) does not balance at its middle, and the hanger finds the balance point by test lifts:
  pick it up, see which end drops, move the hand toward that end, and repeat. This is the skill tested here: find a
  hidden centre of mass from how the part hangs in the grip, regrasp, and hang it so it balances, with no hand on it.
- **The difficulty is physical.** The camera image does not show the balance point. The robot must make the part show
  it: lift, read the tilt, set it down, move the grip. Then it must hang the part on a hook it cannot simply drop it
  onto: the cradle opens sideways, under the conveyor arm, so the part is carried in from the front and lowered in.
  A grip at the middle, or any fixed point, hangs the tube 6 cm or more off balance, and it tips off the hook.
- **A real-size part.** The tube is full size (1 m, 1.0 kg); the hook is a standard J-hook with a rubber-lined seat.

### Distinct from

- **"Build an ordered balance chain with three eccentric bars" (#1706, not accepted, final).** Reviewers found
  balancing weighted bars in an ordered tower on a narrow post "an artificial balancing puzzle" and not how workers
  rack or store uneven parts. This task is not a support or storage puzzle: it reproduces a standard work step,
  hanging a part on a conveyor hook (or a single-point lift), where hanging at the balance point is the correct and
  usual practice, done by the hanger with test lifts. There is one part and one hook, as on the line; the part
  changes from trial to trial (seeded slug), as parts do; and the score asks for the part to hang level and released,
  which is what the line needs.
- **"Level a passive gimbal by moving ballast" (#1128, pending).** There the robot changes the object's mass
  distribution (it moves a ballast weight) until a gimbal is level. Here the mass distribution is fixed and hidden; the
  robot must find it by test lifts and choose where to hang the part.
- **"Balance and lock a counterweighted service arm" (#1647, pending).** A pivoted arm with a sliding counterweight
  and a locking pin: again the robot adjusts the mass, on a fixed pivot. Here there is no mechanism; the robot chooses
  the support point on a loose part and hangs it.

Category: **Impact on jobs** (a work procedure on coating and finishing lines and in material handling; no risk to
people is scored).

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, `score()` / `success()`, and `controller()` (the demo `yam_scripted` plays) |
| `tubekit.py` | The tube and hook-station bodies, the seed's slug and mass properties, the contact readout and latches, and the hooks into yam_arena |
| `tubedemo.py` | The closed-loop scripted demo: camera-based tube finding and tilt reading, the grip search, the placement |
| `test_balance_point_tube.py` | Tests: every scoring outcome, the mass model and seeds, the demo's vision on synthetic images, and the demo's search, reach and joint limits for three seeds |
| `task.json` | Submission details |

`tubekit.py` hooks yam_arena without modifying it. It patches `yam_arena.scene.task_object` and `object_states`, the
embodiment's `_ensure_sim` and `reset`, and `yam_arena.scripted.YamScriptedPolicy` (when a task file defines
`controller()`, `yam_scripted` asks it for each action and passes it the observation). It also wraps the environment's
`scene.write_data_to_sim`, which the env calls once per physics substep, so the latches are checked at 240 Hz. The
task runs with the standard `yam_arena` embodiment, `yam_scripted` policy and `task_score` scorer.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left, z up). Arms start in
the ready pose.

| Object | Description |
|---|---|
| Tube | 6061 aluminium tube, 50 mm outside diameter, 1.5 mm wall, 1.00 m long (617 g), lying on the table along y, middle at x 18, y -8. Sealed with two 3 cm press-fit PE end plugs (25 g each): red at the left (+y) end, blue at the right (-y) end. Inside, a steel slug (40 mm round bar, 35 mm long, 345 g) glued at a seeded position (below). Total 1.01 kg |
| Hook station | Fixed (clamped) steel frame standing in for one hanging point of an overhead conveyor: a 12 x 12 x 1.2 cm base plate and a 3 x 3 cm post at x 48, y -5; from the post's top an arm (2 x 2 cm) reaches toward the arms at 22 to 24 cm height; from its end a J-hook (1 x 1 cm shank) hangs down to a cradle centred at x 33, y -5. The cradle is 4 cm wide (along y) and opens toward the arms (-x): a 90-degree V of two rubber-lined faces, 3 cm across each, whose upper edges are 10.3 cm above the table (the front face's edge is the hook's lip); the shank rises 2 cm behind a seated tube. A 50 mm tube hanging in it has its axis 11.0 cm above the table and touches each face 9.2 cm above the table. Nothing else is within 15 cm of the cradle |

Nothing else is on the table. Mark the tube's start outline with tape.

## Seeds

The slug position is drawn from the trial seed: the balance point is 6 to 14 cm from the middle (uniform), toward
either end. `inspect-robots run --seed N` makes a trial reproducible (the trial seed is derived from N, the scene and
the epoch); without `--seed`, inspect-robots draws a fresh seed, so the offset differs every trial. The kit prints the
offset at every reset (`[tubekit] seed ...`). Always pass `-E object_jitter=0`: the layout is fixed; only the slug moves.

| `--seed` | Trial seed | Slug centre from the middle | Balance point from the middle |
|---|---|---|---|
| 0 (the submitted trial) | 1415158438 | +34.1 cm (red end) | +11.6 cm |
| 4 | 3490944092 | +29.6 cm | +10.1 cm |
| 10 | 3996681712 | -19.5 cm (blue end) | -6.7 cm |
| 1 | 2566101560 | -37.6 cm | -12.8 cm |

(+ is toward the red, left end.)

For a real trial, draw the offset from the table above (or uniformly from 6 to 14 cm, either sign) and glue the slug
at the matching position before the trial. The slug sits at offset x 2.93 from the middle (the tube's total mass over
the slug's mass).

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hang the tube level on the hook." --scorer task_score \
  --max-steps 2400 --seed 0 --store-frames --log-dir ~/yam-eval/balance-tube-dream
```

`task_score` gives 1.0 on success, otherwise `score()` at the end of the episode. `TUBE_DEBUG=1` prints the tube's
state every second (pose, tilt, lowest point, hook and robot contact forces, pad gaps, latches).

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hang the tube level on the hook." --scorer task_score \
  --max-steps 2400 --seed 0 --store-frames --log-dir ~/yam-eval/balance-tube-demo
```

The demo is closed loop. It uses only what the robot observes: the top camera image and the arms' measured joint
positions. It reads no simulator state (no tube pose, no slug, no centre of mass). Joint efforts are not in the
yam_arena observation, so it does not use them.

1. Both arms turn 70 degrees outward at the base, out of the top camera's view of the tube's ends.
2. **Locate.** It finds the red and blue plugs by colour in the top image. With the camera's calibration (pose and
   intrinsics), it intersects each plug's ray with the horizontal plane at the tube's axis height: that gives the
   tube's position. It also records how far the plane-x reading of each plug sits above its true height (a small
   bias from seeing the plug's top), to subtract later.
3. **Test lift.** It grips the tube from above at the middle of the search range (the arm on that side) and reads the
   tilt once with the tube still on the table (the zero). It lifts (5 cm commanded; the loaded arm sags, so the tube
   rises about 3 cm, below the 5 cm lift-clear height) and holds still. The pinch grip is a pivot, and the arm is compliant: the heavy end drops (and usually touches the
   table). It reads the tilt as the slope from the grip point (forward kinematics of the gripping arm's measured joints)
   to the plug at the far end (camera ray on the vertical plane through the tube's axis), minus the zero. It sets the
   tube down, lets go and parks the arm.
4. **Bisect.** It keeps the half of the range toward the low end and repeats from step 2, from +-40 cm, until the
   range is 1.5 cm wide (6 test lifts).
5. **Roll back, if needed.** The tube creeps a few centimetres away from the arms over the test lifts. If its middle
   is more than 20 cm out, both hands (fingers closed) come down behind it and roll it back to 17 cm, so the grippers
   stay clear of the hook when they pick it up; then it is located again.
6. **Hang.** Both arms grip the tube, 11 cm either side of the found balance point, so they share its weight equally
   and it stays level in the grips, and the hook fits between the hands. (A single grip off the balance point lets
   the tube pivot between the pads; that is what the test lifts read.) They lift it 5 cm, bring it in front of the
   hook, raise it above the lip and slide it in over the cradle under the conveyor arm. The loaded arms lag their
   commands by one to three centimetres, so the demo measures the grips' midpoint by forward kinematics of the measured
   joints and aims off by the error. It lowers the tube until it is 2 mm above its seat, opens both hands (the V seat
   centres the tube), rises 5 cm and backs away.

Contrasts (same command with an environment variable): `DEMO_CHEAT=middle` skips the search and hangs the tube by its
middle (expected 0: it tips off the hook and touches the table); `DEMO_CHEAT=hold` hangs it correctly but keeps the
gripper closed on it (expected 0.5).

### Checked in simulation

Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1), the commands above with `--max-steps 2400`, `TUBE_DEBUG=1`. All camera
frames were checked (normal renders).

| Run | Balance point (true) | Found by the search | `task_score` | What happened |
|---|---|---|---|---|
| Demo, `--seed 0` | +11.6 cm | +11.9 cm | **1.0** | Hung level (tilt under 0.1 degrees), released, success after about 1,820 steps (91 s) |
| Demo, `--seed 4` | +10.1 cm | +8.1 cm | **1.0** | Hung at 0.4 degrees, 1.9 cm off the cradle's middle, inside its 2 cm |
| Demo, `--seed 10` | -6.7 cm | -6.9 cm | **1.0** | Hung level (0.1 degrees) |
| `DEMO_CHEAT=middle`, seed 0 | +11.6 cm | (no search) | **0** | Hung by its middle, it tipped off the hook and touched the table |
| `DEMO_CHEAT=hold`, seed 0 | +11.6 cm | +11.9 cm | **0.5** | On the hook and level, but held |

The slug's centre of mass is written and read back from PhysX at every reset (`[tubekit] seed ...` line). Lifted off
balance, the gripped tube tilts 1.5 to 10 degrees toward its heavy end and its low end rests on the table; test lifts
stay below the 5 cm lift-clear height. Near the balance point the tilt reading is noisy (about 1 degree), so the found
point can be up to about 2 cm off (seed 4); the 4 cm cradle still takes it.

One Dream-1 run (`--policy dream`, same command, seed 0, 2,400 steps) scored 0: it let the tube drop 2.3 cm at 31 s (a latched failure), then lifted the red end with the right arm only and never brought it to the hook.

## Scoring

Evaluated every control step; the latches every physics step (240 Hz). The score at the end of the episode counts
(`SCORE_OVER = "end"`).

| Condition | Score |
|---|---|
| After the tube was lifted clear (its lowest point 5 cm or more above the table, or the hook alone carrying it), it touched the table or the floor again, at any time | 0 |
| The tube was dropped: it fell 2 cm with nothing touching it, at any time | 0 |
| Hanging on the hook (below), level within 3 degrees, still (under 1 cm/s and 0.05 rad/s), and released (both grippers at least 80% open, every finger pad tip and the tool point 5 cm or more from the tube's surface, no robot contact): success once this has held **2 s** | 1.0 |
| The same, but not yet held 2 s when the episode ends | 0.75 |
| On the hook and level, but held, touched, not released or still moving | 0.5 |
| Lifted clear (latched) and none of the above | 0.25 |
| Anything else | 0 |

**On the hook:** the hook carries the tube (contact force over 0.05 N), its lowest point is off the table, the hook
lies between its plugs, and its axis passes over the cradle's centre line within 1.5 cm sideways and within 1 cm of
the seated height (11.0 cm above the table).

In a real trial, all of this is read from the video and a ruler:
- **Lifted clear / touched the table again:** a 5 cm gap under the tube's lowest point (a 5 cm block next to the
  tube in the video is a gauge), or the tube hanging on the hook; then any contact with the table or floor.
- **Dropped:** the tube falls visibly (2 cm or more) after both grippers let go of it.
- **Level:** a digital inclinometer or phone level on the tube after the trial: under 3 degrees.
- **Released and 5 cm clear:** the video; the grippers' open state and a 5 cm gap to the tube.
- **2 s still:** the video.

Ways around the method get nothing:
- **Pushing, rolling or dragging the tube onto the hook.** The cradle is 10 cm up: a level tube hanging in it has its
  lowest point 8.5 cm above the table, so it cannot be pushed or rolled there along the table. A tube levered up with
  one end still on the table is not level. As soon as the hook carries the tube clear of the table, 'lifted' latches.
- **Trial and error on the hook** (hang it, let go, watch it tip, catch or regrasp). If it tips off, it touches the
  table: 0. Probing on the hook while still holding it is allowed; hangers do that too.
- **Holding it there.** Success needs both grippers open, every pad 5 cm away and no contact from any robot link
  (an arm leaning on the tube counts), for 2 s. Held: 0.5 at most.
- **Hanging it by its middle** (a fixed grasp): the balance point is 6 to 14 cm off, the cradle tolerates about 2 cm,
  so it tips off: 0.
- **Dropping it onto the hook** from above: dropped, 0 (and the conveyor arm is in the way).
- **A fixed schedule cannot be waited out:** every condition is a latch over the whole trial or a state that must hold
  for 2 s at the end.

## Simulation fidelity

Approximate. The tube is one rigid body with a cylinder collider (50 mm, 1 m) and friction 0.8. The slug has no
geometry: it sets the body's centre of mass and inertia, written with PhysX's tensor API (`set_coms`,
`set_inertias`) at every reset from the trial seed. The plugs are coloured rings (visual only). The hook
station is a kinematic (fixed) body of boxes; the hook's V seat is two 45-degree rubber plates (boxes turned about y). A real conveyor hook
swings a little on its chain; this one is rigid. Contacts are PhysX's. Contact forces between the tube and the hook
station and every robot link come from a PhysX contact view; the table contact is geometric
(the tube's lowest point within 2 mm of the table top). The YAM's joint PD and gravity compensation are yam_arena's;
the arm's compliance is what makes the gripped tube tilt (a real pinch grip also lets the tube pivot between the pads).
A round tube on the table can roll; in simulation it creeps up to a few centimetres per grasp, and the demo
re-locates it before every grasp.

## Runtime

On the RTX workstation used (one GPU), a scripted demo trial (Isaac Sim start, reset, about 1,800 control steps,
episode end) takes about 8.5 minutes of wall time, most of it the 240 Hz physics with three rendered cameras. A
2,400-step policy run takes longer, depending on the policy's latency.

## Tests

```bash
pytest test_balance_point_tube.py      # in the yam_arena environment; no GPU needed (about 30 s)
```

## Real-world build and safety

The tube, plugs and slug are standard stock: 50 x 1.5 mm 6061 tube cut to 1.00 m, deburred; two 50 mm PE tube
plugs, one red and one blue (or painted); 40 mm steel round bar cut to 35 mm, wrapped in tape to fit the bore snugly
and fixed with a dab of epoxy or a set screw at the seeded position. The hook station: a 30 mm square steel post on a
base plate clamped to the table, a 20 mm square arm, and a 10 mm steel J-hook with a 4 cm wide rubber V seat (a
standard powder-coating hook with a rubber sleeve fits). The tube weighs 1.0 kg, within the YAM's payload (about
1.5 kg in i2rt's specification; the yam_arena code gives no payload). A 1 kg tube falling 10 cm onto a table is not a
hazard to people; keep people out of the arms' reach as usual.

## Assets and licences

No third-party assets. The tube, plugs and hook station are primitives defined in `task.py` / `tubekit.py`. The
robot, room and cameras are the standard yam_arena embodiment.
