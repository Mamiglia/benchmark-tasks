# Put glasses away in a sliding-lid box

A pine box with a sliding lid is fixed on the table. Its lid is shut, and a set of keys lies on it. The lid is a plain
board 18 cm wide, with no knob, held between two guide strips; the guides run on past the box's right-hand end over a
fixed block, so the lid can only slide along them. A pair of open reading glasses stands on the table to the left, a
ceramic dish stands in front of the box, and a mug of coffee stands just beyond it.

**Instruction:** `Put the glasses away in the box and the keys in the dish, and close the box.`

The instruction states the goal only. How the glasses go in is set by the box: it is 7 cm deep inside, so the open
glasses fit under the lid only standing (on their lower rims and ear bends) or upside down (on the brow bar and arms).
Lying on their lenses or on a side, their arms or front stand 13 cm tall and the lid cannot shut. Both fitting stances
keep the lenses off the wood. The keys must go in the dish, not in with the glasses; the mug must not be upset (a
standing side-effect rule, not stated).

Category: **Societal relatability** (an everyday tidying job at home; what is protected is the glasses, not a person).

## Why this task

- **The everyday job.** Putting glasses away in a lidded box or tray on a hall table or bedside is a daily household
  task, and a frequent request to home-assistance robots from people with limited hand use. Glasses are put away open
  or folded; here they are open, as they are usually left on a table.
- **The difficulty is physical.** The lid is wider (18 cm) than the gripper opens (9.4 cm) and has no knob or edge to
  grip: its sides are inside the guides and its ends are flush. It can only be moved by pushing on its end face,
  which stands 1 cm above the rim, with the fingers 3 mm above the box wall, and pushed straight along the guides
  (2 mm clearance each side) or it yaws and binds. The lid opens toward one arm and shuts toward the other, so the
  arms hand the job over, each pushing from the far side. The keys on the lid have to come off first, or they ride
  along. The glasses are a 28 g open frame, 13 cm by 13 cm and 4 mm thick in most places: they are picked by the
  bridge, carried over the guides, and lowered into a box with 1.3 cm to spare on each side, so they land standing
  (or upside down) and do not come to rest on their lenses or a side. The lid slides with pine-on-pine friction.
- **Real objects at full size.** Every object is a real, everyday item with real dimensions and masses (below).

### Distinct from

- **"Fold the glasses' temples and close the glasses in their hard case" (not accepted, final), and DexJoCo's Fold
  Glasses.** Their core skill is folding the two hinged temples so the glasses fit a case. Here the glasses stay open
  and nothing is folded; the core skills are sliding a lid that cannot be grasped, by pushing its end along its guides
  in both directions with a hand-over between the arms, and setting a thin open frame down in a tight box in a stance that fits under the lid.
- **"Slide a toy box lid fully shut against its stop, never faster than 5 cm/s" (not accepted, final).** That task
  only closes a lid at a speed cap. Here the lid is one step: it must be cleared, opened, the box filled, and shut
  again, with the closing position (no end stop) set by the push itself.
- **"Find the buttons among four identical sewing boxes" (not accepted, final).** Its sliding lids have knobs and are
  pulled out; its difficulty was remembering a search order. Here there is no search; the lid has no knob.
- **LIBERO ("open the top drawer and put the bowl inside") and RoboCasa drawers and sliding doors.** Those parts have
  handles that are grasped and pulled. Here the lid has no handle, is wider than the gripper and is moved
  non-prehensilely, the box must first be cleared, and the object placed must keep its orientation.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, `score()` / `success()` and the scripted demo `demo()` |
| `glasskit.py` | Compound objects built from boxes, each with its own friction (glasses, keys, lid, box) and a per-trial reset hook, added to yam_arena without modifying it |
| `test_glasses_sliding_box.py` | Tests of every scoring outcome and latch, and that the layout fits; no Isaac Sim needed |
| `task.json` | Submission details |

`glasskit.py` builds the glasses, keys, lid and the box with its guides and block (one fixed body), and patches `yam_arena.scene.task_object` and `object_states` (a compound object's position is its own
frame's origin) and wraps the embodiment's `reset` so the task's whole-trial latches are cleared at each reset. The
task runs with the standard `yam_arena` embodiment, `yam_scripted` policy and `task_score` scorer.

**Third-party assets: none.** Every object, including the glasses and the keys, is built in `task.py` from boxes and
one cylinder. No mesh, texture or model library item is used.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table top (x away from the arms, y to their left,
z up). Everything is square to the table edges. Arms start in the ready pose.

| Object | Description |
|---|---|
| Box | Pine box, fixed (screwed to a baseboard taped to the table): 18 (x) x 20 (y) cm outside, 7.5 cm tall, 1 cm walls, 0.5 cm floor; centre x 30, y 4; inside x 22 to 38, y -5 to 13 |
| Lid | Pine board 18 x 20 x 1 cm, 160 g, loose, lying shut on the box's rim (centre x 30, y 4). No knob or finger hole |
| Guides | Two pine strips 1 x 9.3 cm in section, 42 cm long (y -27 to 15), fixed along both long sides of the box with 2 mm clearance to the lid (inner faces at x 20.8 and 39.2). They stand 8 mm above the lid's top |
| Lid block | Pine block 18 x 20 x 7.5 cm, fixed, directly to the right of the box (centre x 30, y -16), its top level with the box's rim, so the lid stays supported when slid open to the right |
| Keys | A brass house key (blade 30 x 18 x 3 mm) on a brown leather fob (40 x 25 x 12 mm), 32 g together, lying on the lid, fob centred at x 30, y 7, the key toward the far side |
| Dish | White glazed ceramic catch-all tray, fixed: 12 x 12 cm outside, 1.8 cm tall, 8 mm walls, 4 mm base; centre x 13, y 0 |
| Glasses | Reading glasses, open, 28 g: black acetate front 134 mm wide and 43 mm tall (brow bar and bridge 6 mm tall, lower rims 4 mm, end pieces 6 mm wide), two CR-39 lenses 46 x 30 x 3 mm, two arms 115 mm long with ear bends 38 mm down at their ends. Standing on the table on the front's lower rims and the two ear bends, front frame across x with its bottom middle at x 16, y 30, facing left (+y), arms pointing right (-y), toward the box |
| Mug | Red ceramic mug, 8.2 cm across, 9.5 cm tall, 380 g with 100 ml of coffee (room temperature in a real trial), standing at x 47, y 4, just beyond the box |

Nothing else is on the table. Tape the glasses', keys' and mug's start marks.

## Scoring

`score()` runs after every control step. **Violations are latched for the whole trial** (cleared only by a reset),
and any of them makes the score 0:

- the glasses come to rest (under 2 cm/s and 0.5 rad/s) 60 to 120 degrees from standing, that is on their lenses or
  on a side, anywhere (carrying them through that range in the air does not count), or fall off the table;
- any part of the keys goes inside the box (below its rim, within its inner walls), or the keys fall off the table;
- the lid leaves its guides: it tilts more than 10 degrees, its underside rises 1.5 cm above the rim or block, or it
  drops 1 cm below them;
- the mug tips more than 15 degrees or moves 2 cm.

Otherwise the score is the sum of:

- **0.2** keys in the dish: the fob's centre within the dish's inner walls, no higher than its rim;
- **0.4** glasses in the box: every corner of every part of the glasses within the box's inner walls, the front's
  lowest point within 1 cm of the floor, the glasses within 20 degrees of standing or of upside down, and still
  (under 2 cm/s);
- **0.4** more, only with the glasses in the box: the lid shut, its centre within 1.5 cm of the box's centre, tilted
  under 5 degrees, resting on the rim.

Possible scores: 0, 0.2, 0.4, 0.6, 0.8, 1.0. Success (1.0) must hold 1 s and ends the episode; otherwise `task_score`
gives the score at the end. Doing nothing scores 0: the lid is shut at the start, but it counts only over the glasses.
A violation cannot be undone: standing knocked-over glasses back up, or taking the keys back out of the box, still
scores 0. Leaving the keys on the lid and sliding it open and shut with them riding on it scores at most 0.8.

**Real trial.** Measured at the end with a tape measure and a top-down photo, and over the trial from the video: the
glasses' tilt (lying on their lenses or side is unmistakable), whether the keys ever dropped into the box, whether the
lid ever came off its guides, and the mug's position against its taped outline (2 cm) and whether it tipped. The lid
is shut when its edges are within 1.5 cm of the box's edges.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the glasses away in the box and the keys in the dish, and close the box." \
  --scorer task_score --max-steps 1200 --seed 0 --store-frames --log-dir ~/yam-eval/glasses-box-dream
```

Run limit: 1,200 steps (60 s at 20 Hz). The layout is fixed: always pass `-E object_jitter=0`; the seed changes
nothing in the scene. `GSB_DEBUG=1` prints the lid, glasses, keys and mug poses every second, and every violation is
printed when it latches (`[glasses-sliding-box] violation: ...`).

## Scripted demo

Replace `--policy dream` with `--policy yam_scripted` in the command above. The demo is an open-loop plan of
top-down gripper waypoints, written from the fixed layout in `task.py`; it reads nothing from the simulator.

1. The left arm picks the keys off the lid by the fob and puts them in the dish.
2. The left arm, fingers closed, comes down just past the lid's left end, 3 mm above the rim, and pushes the lid
   20 cm to the right along its guides, onto the block.
3. The left arm picks the glasses by the bridge, carries them over the near guide, lowers them into the box until
   the rims are 4 mm above the floor, and lets go; they stand. It moves clear.
4. The right arm, fingers closed, comes down behind the lid's right end and pushes it 20 cm back, shut over the box,
   and moves clear.

### Results

| Run | Score | What happened |
|---|---|---|
| Scripted demo, `--seed 0`, 4, 10 | 1.0, 1.0, 1.0 | Success held at about 38 s (770 steps). Lid ends 1 to 10 mm from the box's centre; glasses standing on the floor, tilted 1 degree |
| Keys left on the lid (the lid slid open and shut with them riding on it; glasses put in) | 0.8 | Keys not in the dish |
| Keys dropped into the open box beside the glasses, then lid shut | 0.0 | `keys_in_box` latched; the rest of the run would have scored 1.0 |
| Dream-1, `--seed 0` | 0.0 | Both arms folded back beside the table and stayed there for the whole minute; nothing moved |

The two cheat runs are the demo with steps changed (`cheat_*.py` task files that import `task.py` and replace
`demo()`); they are not part of the submission.

## Real-world trial

- **Hardware.** Two YAM arms with their parallel grippers (94 mm opening) on the station table. The pushes need
  under 1 N (160 g lid, pine on pine); the heaviest lift is the 32 g keys.
- **Objects.** A shop-made pine sliding-lid box, guides and lid block as above (one evening's work with a saw, glue
  and screws), a pair of non-prescription reading glasses, a house key on a leather fob, a ceramic dish and a mug.
- **Safety.** No person is in the workspace during a trial; the operator stands outside the arms' reach with the
  emergency stop. The mug holds room-temperature water, not hot coffee. The box, guides and block are screwed to a
  baseboard taped to the table, and the dish is taped down. Use inexpensive reading glasses: a trial can drop them.

## Fidelity

Approximate, at full size. Every object is rigid, built from boxes (the mug a cylinder). The glasses are one rigid
body: the hinges do not fold and the arms do not flex; their mass (28 g) is spread over the parts by volume.
Lens contact is not sensed; only the glasses' orientation is scored. The keys are one rigid body (no ring), the mug
has no handle and its coffee is part of its mass (spills are not simulated; tipping or moving it counts). All
contacts with the box, guides, block and lid use pine on pine: 0.4 static, 0.3 dynamic (wood on wood, clean and dry,
0.25 to 0.5 static: Engineering ToolBox, "Friction and Friction Coefficients for some Common Materials"); the kit
combines a contact's two materials by their minimum. Other contacts use yam_arena's default (1.2 static, 1.0 dynamic).
Pushing the lid takes about 0.6 N. The lid is a free box between fixed guides with 2 mm clearance a side.
