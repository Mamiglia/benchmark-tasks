# Play one turn of Jenga: find a loose block, push it through, draw it out and stack it crosswise on top

A freshly built Jenga tower (54 blocks, 18 levels) stands on the table. The robot plays one turn by the game's
rules: with one hand at a time, it takes a block from below the top level and stacks it on top, crosswise to the
top level, and the tower must still stand five seconds later.

**Instruction:** `Play one turn of Jenga: using one hand at a time, take a block out from below the top level and stack it on top, crosswise to the top level, without knocking the tower over.`

**Category:** Societal relatability. Jenga is one of the best-known tabletop games (Hasbro). Most people have played
a turn or watched one, and anyone seeing a ten-second clip knows what the robot is doing and why it is hard.

## Why this task

- **The skill is force-limited extraction from under load.** Real Jenga blocks are not all the same thickness. In
  every level the thickest blocks carry the levels above and are pinched; the thinner ones are loose. Nothing visible
  tells them apart. A player feels for a loose block with a light push, gives up on any block that resists, pushes a
  loose one through from one side, then draws it out from the other side, straight along its length. Pushing a
  pinched block, or pushing or pulling a few degrees off its axis, drags the blocks above and shifts or topples the
  tower.
- **Keeping everything else undisturbed.** Every other block must stay within 5 mm of where it was. The 12.8 mm tall
  closed fingertips must enter a 15 mm tall level, with 1.1 mm to spare above and below.
- **One hand at a time, as the rules say.** The other hand must not steady the tower. The push and the pull are
  done by different hands, one after the other, and the scorer checks that both never touch the tower at once.
- **Real rules, real game.** The steps and the five-second wait come from Hasbro's printed rules (below), with the
  real Hasbro Jenga Classic set.

### The rules followed

Hasbro Jenga Classic (A2120), instruction sheet (2019, instructions.hasbro.com):

1. "Set up the tower. Place three blocks per layer, alternating direction by layer. Use the stacking sleeve if necessary."
2. "Using one hand, remove a block from anywhere below the highest completed layer. You can touch the blocks to find loose ones."
3. "Place the block on top of the tower. Always complete a layer before starting a new one. ... Your turn ends five
   seconds after you stack your block or as soon as the next player has touched a block."

The tower starts complete (18 levels), so the block starts level 19, crosswise to level 18 ("alternating direction").
Older Hasbro sheets state the one-hand rule as "only one hand at a time may touch the tower"; the scorer enforces
that reading: both grippers may never touch tower blocks at the same moment. Touching blocks to find a loose one is
allowed, as the rules say; moving another block out of place is penalised (cap at 0.1).

### Distinct from

| Task | Its skill | Here |
|---|---|---|
| TK-E6ECW7 Take a recessed book off a packed shelf by tipping it out by its head (accepted) | Take one item from among neighbours by tipping it | No tipping and no free top: the block is clamped by the weight of up to 17 levels above. It must be found by feel (pinched or loose), pushed through and drawn out straight along its axis |
| TK-X7LZDM Pull a cable from under stacked cases (pending) | Pull a flexible cord from under a load without dragging the load | A rigid block inside a tower, pushed then pulled by two hands in turn, with a decision (pinched or loose) from the feel of the push; the result is stacked |
| TK-VM6TE4 Shelve a paperback into a gap in a packed shelf (pending) | Insert into a gap | Extraction under load, not insertion |
| PR #387 stack-on-a-shared-tower | Stacking | Stacking is only the last step here, and an easy one |

External benchmarks: LIBERO, RoboCasa, BEHAVIOR-1K and SimplerEnv have block stacking and pick-and-place but no
Jenga extraction. The nearest known work is Jenga as a research problem (Fazeli et al., "See, feel, act: Hierarchical
learning for complex manipulation skills with multisensory fusion", Science Robotics 2019), which used a
single-arm push only, with a force-torque sensor and a custom probe. This task is a full turn by the game's rules with
a standard bimanual parallel-gripper robot: probe, push through, draw out with the other hand, stack crosswise.

## Design

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left).

| Object | Size | Mass | Notes |
|---|---|---|---|
| 54 Jenga blocks | 7.5 x 2.5 x 1.5 cm nominal; thickness 1.5 cm +- 0.03 cm (seeded per block) | 15 g each | Hardwood (light wood colour). 18 levels of 3, 0.3 mm play between neighbours |
| Tower | 7.6 x 7.6 cm, about 27 cm tall | about 0.81 kg | Centred at (30, 0). Level 1 runs along x; even levels (2, 4, ..., 18) along y |

**Loose and pinched blocks.** Each block's thickness is 15 mm plus a uniform draw in +-0.3 mm from the seed. A
level rests on the thickest blocks of the level below, so in each level the thinner blocks carry little or no load.
Whether the middle block is loose (thinner than both edges), or one or both edge blocks are, changes from seed to seed.
Every level has at least one loose block.

## Scoring

Over the whole trial (video at 20 frames a second), with a ruler and before/after photos.

**The taken block** is the block of levels 1-17 that moved most. **Out of the tower:** no part of it is inside the
tower's 7.6 x 7.6 cm footprint below the top level's upper face.

| Score | Condition | Real-trial check |
|---|---|---|
| 0 | The tower fell: at any time, a block other than the taken one moved 2.5 cm or more or tilted over 15 deg | Video |
| 0 | Both grippers touched tower blocks at the same moment (the taken block counts as tower until it is out) | Video |
| 1.0 | The taken block lies on the top level (underside within 3 mm of it, within 10 deg of flat), its length within 20 deg of crosswise (along x), its centre within 3 cm of the tower's axis in x and y, no gripper touching it, at rest; held 5 s (the rules' five seconds) | Photo from above and from the side, ruler |
| 0.6 | The taken block is out of the tower, but not as for 1.0 (held, dropped, laid along the top level, off centre) | Video, photo |
| 0.3 | The taken block slid half its length (3.75 cm) or more along its length at some point | Video, ruler |
| 0.1 | It slid 5 mm or more | Video, ruler |
| 0 | Otherwise | |

**Cap 0.1:** another block ever moved 1 cm or more (video), or at the end any other block is 5 mm or more from where
it started (ruler against the tower's faces and top; before/after photos). The cap and the zero conditions are
latched: pushing a block back does not undo them.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder. The layout is fixed: always pass
`-E object_jitter=0`.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Play one turn of Jenga: using one hand at a time, take a block out from below the top level and stack it on top, crosswise to the top level, without knocking the tower over." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/jenga-dream
```

Prefix `JENGA_SEED=4` (or any integer) for other block thicknesses (the default is seed 0). `JENGA_DEBUG=1` prints the
scorer's state, the robot's block contacts and the demo's probe results.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Play one turn of Jenga: using one hand at a time, take a block out from below the top level and stack it on top, crosswise to the top level, without knocking the tower over." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/jenga-demo
```

`yam_scripted` runs `jenga_demo.Controller` closed loop (`jengakit` hooks it in when the task module defines
`demo_controller`). It reads only the joint positions and its own commands; it knows the nominal layout, never which
blocks are loose. See the docstring of `jenga_demo.py` for the steps. `DEMO_MODE=noprobe`, `brace` and `parallel`
play the loopholes listed under "Checked in simulation". On seeds 0, 4 and 10 the first block the demo tries (the middle
block of level 10) was loose, so the recorded demos never back off from a pinched block; see "Checked in simulation"
for what happened when the demo started at a pinched one.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: tower layout, seeds, `score()` / `success()`, `demo_controller` |
| `jengakit.py` | The wooden block (friction, contact offset, solver iterations), per-arm contact sensing, and the hooks into yam_arena |
| `jenga_demo.py` | The closed-loop scripted demo |
| `test_jenga_push_and_stack.py` | Tests: layout, seeds, every scoring rule on handcrafted states |
| `task.json` | Submission details |

`jengakit.py` hooks yam_arena without modifying it: `yam_arena.scene.task_object` and `object_states`, the
embodiment's `reset`, and `yam_arena.scripted.YamScriptedPolicy.reset` / `act`.

## Assets and licences

No third-party scene assets: every block is a box primitive built in `jengakit.py` from the sizes in `task.py`. The
robot, room and cameras are the standard yam_arena embodiment; its YAM arm and gripper model (URDF and meshes) is by
I2RT Robotics under the MIT License (`yam_arena/assets/yam/LICENSE.i2rt`). The task code is the authors' own.

## Tests

```bash
python -m pytest -q      # in the yam_arena environment; no GPU needed
```

## Checked in simulation

Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1) on shared RTX workstations, with the commands above (`JENGA_DEBUG=1`,
`--max-steps 3000`). Frames of every run listed were checked (not blank).

| Run | `task_score` | What the log shows |
|---|---|---|
| No robot motion, 60 s (tower standing test) | 0 (nothing taken) | Largest block movement 0.88 mm (a top-level block), largest tilt 1.4 deg; no latch |
| Scripted demo, seed 0 | **1.0** | Tower top touched at 273.3 mm (true 273.0). Level 10 middle block loose: pushed 15.3 mm at about 1.3 N, drawn out by the right hand, stacked crosswise; other blocks under 0.8 mm; success at about 49 s (about 1,000 steps) |
| Scripted demo, seed 4 | **1.0** | Top at 273.7 mm. Level 10 middle pushed 15.1 mm at about 0.9 N; stacked; other blocks under 0.6 mm |
| Scripted demo, seed 10 | **1.0** | Top at 273.2 mm. Level 10 middle pushed 15.1 mm at about 0.4 N; stacked; other blocks under 0.9 mm |
| Demo probing level 6 first, seed 0 (that middle block is the thickest of its level: pinched) | **0** | The probe did not stop: the 12 levels above slid with the block, and the tower fell (fell latch). Kept as evidence that pushing a pinched block topples the tower in the simulation; it also shows the demo's probe threshold is coarse (see Simplifications) |
| `DEMO_MODE=noprobe`, seed 0 (naive: push the middle block of level 6, a pinched one, 15 mm with no force limit, then carry on) | **0** | The levels above slid with the block and the tower fell (fell latch at 19 s); the rest of the turn earns nothing |
| `DEMO_MODE=brace`, seed 10 (loophole: the right hand steadies the tower's far face at level 13 while the left pushes) | **0** | Both hands on tower blocks at once (two-hands latch at 19 s), although the block was then drawn out and stacked |
| Dream-1 (`dreamlabs/dream-1-beta`), seed 0 | **0** | 3,000 steps. Both arms moved about near their bases and the table; neither gripper came near the tower; no block moved |

Scorer rules not reached by these runs (both hands on the tower, a block put back after moving 1 cm, a top-level block
taken, a block laid along the top level, held, off centre or still moving) are covered by the unit tests.

## Runtime and seeds

A scripted trial is about 1,000 control steps (50 s simulated: touch-off 11 s, probe and push 9 s, draw out and
stack 25 s, then the 5 s hold); wall time 6 to 30 minutes on a shared RTX workstation, mostly Isaac Sim start-up
and GPU sharing. A 3,000-step policy run takes longer, depending on the policy's latency. Seeds (`JENGA_SEED`): 0, 4
and 10 checked; the default is 0. The layout is fixed with `-E object_jitter=0`.

## Real-world trial

**Hardware**

| Item | Detail | Approx. price |
|---|---|---|
| Hasbro Jenga Classic, A2120 | 54 hardwood blocks 7.5 x 2.5 x 1.5 cm, loading sleeve | $15 |
| Masking tape | Mark the tower's footprint and the robot-facing direction on the table | $3 |
| Ruler | Check other blocks' displacement (5 mm, 1 cm) and the placed block's centre (3 cm) | $3 |

Total under $25. Weigh a few blocks once (the simulation uses 15 g; the set's blocks are hardwood of about 0.5 g/cm3,
28 cm3 each).

**Forces.** A loose block slides with well under 0.5 N (its own weight, 0.15 N, times wood-on-wood friction of about
0.4). A pinched block of level 10 carries up to about 3.5 N from the 8 levels above; pushing it would take about 3 N,
which is what the probe must not do. The demo gives up at about 1.5 N, and grips at most a few newtons; the YAM's
fingers give about 40 N, its arm carries 1.5 kg. Measure: a luggage scale on a thread taped to a block's end, pulled
slowly.

**Setup per trial.** Shuffle the 54 blocks, build the tower in the loading sleeve on the taped footprint with level 1
along x (pointing away from the robot), slide the sleeve up and off. Check that the tower stands straight (no block
sticking out over 1 mm). Which blocks are loose changes every rebuild, as in a game; that is intended.

**Safety.** Blocks are light, smooth wood with no sharp edges; a falling tower is harmless to people and to the rig.
Keep the area around the table clear of anything a block could knock off.

## Simplifications

- Wood is a rigid box: no surface roughness, compliance or edge chamfers. The thickness spread (+-0.3 mm) stands in
  for the real variation in a set; it makes loose and pinched blocks, which is what the task needs.
- PhysX settings for a standing 54-box tower: TGS, 32 position and 4 velocity iterations per block, 2 mm contact offset,
  0 rest offset, depenetration speed capped at 0.05 m/s (at 0.5 or 1 m/s the loaded tower crept several mm a minute).
  With no robot the tower moves under 1 mm in 60 s (checked: top block 0.88 mm, no block tilting more than 1.4 deg),
  within the 5 mm limit.
- The robot's hand force is read from joint lag, as on the real arm; the demo's probe threshold (command 5 mm ahead
  of the fingertip) is coarse. See "Checked in simulation" for a pinched block it did not catch.
