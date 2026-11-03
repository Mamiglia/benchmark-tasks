# Open a bottle either way and pour it into a cup

A closed square jug-style bottle stands on the table with an empty cup between it and the arms. Its cap can be opened in two
ways: it unscrews (it comes off after 1.25 turns), or the flip-top lid on the cap snaps open when its tab is pushed
up. The robot must open the bottle and pour its contents into the cup.

**Instruction:** `Open the bottle and pour its contents into the cup.`

## Why this task

Opening a drink and pouring it is a daily task that many older and disabled people need help with, so it is
filed under *Societal relevance*. It measures whether a model finds and uses an available opening rather than
replaying one script. Both routes are physically possible and both lead to a full score:

- **Flip-top** (quick): push the tab up past the detent. The tab sits 13 cm up on a light bottle, so steadying the
  bottle with the other hand works best.
- **Unscrew** (slow): hold the bottle with one hand and turn the cap 450 degrees with the other. The wrist's roll
  joint covers 240 degrees, so this needs at least two re-grasps. The thread's friction (about 0.2 N m) reacts on
  the bottle, so an unheld bottle turns with its cap.

Either way, the bottle must then be lifted and tipped over the cup. The score depends only on whether the bottle
was opened and how much reached the cup. Speed doesn't count within the run limit. Spills aren't penalised
separately, but spilled contents don't count.

How it differs from tasks on the board: *Open a medicine bottle* uses one push-and-turn child-resistant cap and
ends in sorting pills. *Open a jar, load it, close it* unscrews a lid along one prescribed route and puts a block
inside. *Pour beads through a funnel* pours from a container that is already open, with the funnel held. *Water
plants with a spray bottle* presses a trigger. *Lock a bayonet cap* and *Tighten a nut onto a bolt* are assembly
turns. This task combines a closure with two working openings (the robot picks its own strategy), the two-handed
stabilisation and re-grasping that the unscrew route needs, and a pour scored by how much reaches the cup.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, the thread and flip-top mechanisms, `success()`, `score()`, and `demo()` for both routes |
| `jointkit.py` | Multi-box parts, a spring-driven hinge, a run-time switchable joint between parts, kit-spawned spheres, and step and reset hooks, for yam_arena task files (hooks yam_arena without modifying it) |
| `test_open_bottle_and_pour.py` | Scoring tests on handcrafted states for both routes and the cheats, the thread's friction rule, and the demos' reachability (no Isaac Sim needed) |

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder. Always pass
`-E object_jitter=0`: the beads start inside the closed bottle, and jitter would scatter them.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the bottle and pour its contents into the cup." --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/bottle-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demos

The unscrew route (the full-score demo to submit):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the bottle and pour its contents into the cup." --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/bottle-demo
```

1. The left arm grips the bottle's front and back faces 7 cm up, reaching from the left with its wrist pitched
   20 degrees down. The cap sits 8 cm above this grip, clear of the left gripper.
2. The right arm, pointing straight down, pinches the cap's two tab-free faces and turns it 180 degrees
   counterclockwise. It lets go, turns back outside the cap, and grips again, four times in all (720 degrees
   commanded; each stroke slips a few degrees, and the thread lets go at 450, during the third stroke). Then it
   lifts the cap off, drops it on the table at (18, -14) cm and clears.
3. The left arm lifts the bottle to 20 cm and tips it 115 degrees about the world y axis, so the neck swings back
   over the cup's centre and the beads slide out along the wall the neck is flush with. The trial ends in success
   once 32 beads have settled in the cup (at about step 895, 45 s, in the run below).

The flip-top route: the same command with `DEMO_ROUTE=flip` in front of `inspect-robots`, and
`--log-dir ~/yam-eval/bottle-demo-flip`. The left arm grips the bottle as above. The right arm, with its fingers
closed and pointing left, moves under the lid's tab, lifts 2 cm, then lifts and pushes inward along the lid's arc.
The lid passes the detent and springs open, and the left arm pours as above.

`yam_scripted` holds the last pose for 3 s, then stops unless the trial has already ended in success. Make videos
and the setup screenshot as in the walkthrough (`inspect-robots video <log>.json`, then the first top-camera
frame).

## Scoring

Evaluated every control step. The score at the end of the episode counts (`SCORE_OVER = "end"`).

- **0** if the cup was ever moved more than 3 cm or tilted more than 20 degrees (this stays 0 even if the cup is
  put back), or if the bottle was never opened.
- **Opened** (sticky) once the lid has been at least 60 degrees open, or the cap has been 3 cm or more from its
  seat on the neck.
- **0.25** opened; **0.5** at least 4 beads in the cup; **0.75** at least 20; **1.0** at least 32 (80 %).
- **Success** (ends the trial with 1.0): at least 32 beads at rest in the cup, and the cup at rest, held for 1 s.

A bead counts when it is inside the cup's walls, below the rim, and not inside the bottle. If the bottle is
dropped into the cup, the beads still in it don't count. Real-world trials use 200 mL of water and a kitchen
scale: 160 g, 100 g and 20 g in the cup give the same scores (see `task.json`).

Unintended strategies and what stops them:

- Tipping or knocking the cup: the cup guard scores 0.
- Dropping the bottle into the cup: beads inside the bottle don't count.
- Prying the cap off without unscrewing it: in simulation the thread joint holds the cap until it has turned 450
  degrees. In the real world a threaded cap doesn't pull off, and breaking anything scores 0.
- Tearing the lid off its hinge: in simulation the hinge can't break. In the real world any broken or deformed
  part scores 0.
- Squeezing the bottle to squirt water out: the real bottle is specified rigid, and in simulation it is rigid.
- Picking beads out one by one: 32 beads through a 20 mm opening is not practical within the run limit, and
  real water can't be picked up.
- Knocking the open bottle over: the contents spill onto the table, where they don't count.

## Mechanisms

- **Thread.** A real revolute joint (PhysX, outside any articulation) holds the cap on the neck about the neck's
  axis. A gripper turns the cap through contact, and the joint's drive is the thread's friction, which reacts on
  the bottle. The friction is breakaway friction. The drive is a stiff spring (3 N m/rad) toward a set angle, and
  after every control step the task code moves that set angle along whenever the cap has slipped more than
  4 degrees past it. So the cap holds against small torques (pushing the flip-top's tab doesn't turn it) and slips
  at about 0.2 N m. The code counts the cap's net turn. After 450 degrees (counterclockwise from above) it switches
  the joint off (`physics:jointEnabled`), and the cap is a free body resting on the neck's rim. Its 4 cm skirt
  clears the square neck at any angle, so it lifts straight off. The cap does not rise as it turns.
- **Flip-top.** A revolute hinge along the cap's back edge (0 to 120 degrees), with a spring drive of
  0.3 N m/rad. Its target is closed (0) below 15 degrees and open (115 degrees) past 15 degrees. This makes a
  detent of 0.08 N m, about 1.3 N at the tab, and the lid snaps open and stays open, as a real flip cap does.
- **Contents.** The bottle has real walls, a flat shoulder and a 2.0 cm neck bore, flush with the wall that faces
  the cup (as on a jug, so the beads slide straight out when the bottle is tipped). The cap's top plate has a real
  2.0 cm opening under the lid. The beads leave only through an opening that exists. They are 0.8 g each,
  heavier than real 6 mm beads, because lighter ones in the 0.17 kg bottle were unstable in PhysX (they were
  shot out through the walls).

## Runtime and seeds

Expect about 1 to 3 minutes per trial on an L40S for up to 3,600 steps (the 43 loose bodies add some per-step
cost), plus about 1 minute for Isaac Sim to start (6 minutes on a fresh instance). The layout is fixed
(`-E object_jitter=0`), so there are no seeds to choose. Run limit: 3,600 steps (3 minutes at 20 Hz). The
unscrew demo takes about 45 s and the flip-top demo about 20 s.

## Tests

```bash
pytest test_open_bottle_and_pour.py
```

Needs `yam_arena` installed (no GPU). The tests cover both routes' partial credit and success, the sticky
opening, the cup guard, the bottle-in-cup case, spilled beads, the thread's breakaway rule, the twist measurement,
and both demos' reachability through `yam_arena.demo.ArmPlan`.

## Verified in simulation

Run in Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1) with the commands above, `--max-steps 3600`:

| Run | Result |
|---|---|
| Unscrew demo | `task_score` 1.0, success at about step 895 (thread released after 450 degrees, cap set aside on the table) |
| Flip-top demo (`DEMO_ROUTE=flip`) | `task_score` 1.0, success at about step 375 (lid open, beads in the cup) |
| `pytest test_open_bottle_and_pour.py` | 16 passed (with yam_arena installed) |

One Dream-1 run (3,600 steps, `-E object_jitter=0`) scored 0: it hit the step limit. Beads are small at 320 x 176 (about two pixels in the top camera); they are easy to see in the
cup at 640 x 352.

## Real-world build

- **Bottle.** A rigid square jug-style bottle of about 350 mL (6 x 6 cm, 11 cm to the shoulder), with its neck at
  one edge, as on an F-style jug, set flush with the side facing the arms. Its neck takes a flip-top dispensing cap
  that releases in about 1 to 1.5 turns and has a thumb tab sticking out at least 1 cm (cut a tab from 1 mm sheet
  and glue it on if the cap's own lip is smaller). A bottle with a centred neck and a sloped shoulder also pours
  water; the simulation's neck sits at the edge because its beads can't climb a flat shoulder. Fill it with 200 mL
  of tap water.
- **Cup.** A clear square plastic container, 8 x 8 x 7 cm, weighted to 0.15 kg.
- **Tools.** A kitchen scale with 1 g resolution, tape for the cup's outline, and a tray or towel. Use 40 beads of
  6 mm instead of water if the rig must stay dry.
- **Safety.** Nothing is sharp, hot or heavy (0.23 kg is the heaviest item). Keep water away from the arms'
  electronics and wipe up spills between trials.

## Simulation fidelity (full)

Approximate. Rigid bodies built from boxes: the bottle, neck, cap and cup are square in section. The water is 40 loose 6 mm spheres of 0.8 g (heavier than real 6 mm beads, for solver stability); the bottle (0.17 kg) plus beads weighs about what the filled bottle does. Real water flows and glugs, beads roll and can briefly jam (the opening is 3.3 bead widths). The screw thread is a real revolute joint between bottle and cap, so a gripper must turn the cap through contact, and the thread's friction reacts on the bottle (an unheld bottle turns with its cap). Its friction is modelled as breakaway friction: a stiff spring (3 N m/rad) toward a set angle that the task code moves along whenever the cap has slipped more than 4 degrees past it, about 0.2 N m. The code counts the cap's net turn and switches the joint off after 1.25 turns (450 degrees), leaving the cap as a free body on the neck's rim; the cap does not rise as it turns. The flip-top lid is a real hinge with a spring drive whose target the task code switches from closed to open once the lid passes 15 degrees (a detent of about 0.08 N m). No contact-force cap is enforced.
