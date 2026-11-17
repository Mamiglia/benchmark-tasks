# File punched sheets in a lever arch file and lock its rings

Instruction: **"File the punched sheets in the lever arch file and lock its rings shut."**

A blue A4 lever arch file lies open on the table, spine on the far side, rings shut, lever folded flat. In front of
it, a stack of 20 two-hole punched A4 sheets lies on two reams of copier paper. To file them, the robot throws the
lever up past its over-centre point, which swings both rings' moving halves back and leaves the two fixed posts
standing free. It then carries the floppy stack over the file and lowers it so that **both** 6 mm holes go over
**both** 4.6 mm posts at once, and lets the sheets slide down the posts. Last, it pushes the lever back down past its
centre so the spring snaps both rings shut through the holes.

This is ordinary office and school filing. Anyone who has worked in an office, or been to school in Europe, has
done it.

## What makes it hard

- **Two-point blind alignment through a floppy part.** The two holes are 80 mm apart on a stack that bends. Each
  hole has 0.7 mm of clearance on its post. Both posts must enter both holes together. The hand cannot see the
  holes from above once the stack is over the posts, and the sheets hang down from the holed strip.
- **Bimanual carry of a thin, flexible stack.** The stack is 2 mm thick. It can only be pinched at its two
  overhanging ends, so both hands must carry it as one rigid unit.
- **An over-centre lever in both directions.** The lever resists up to about 10 N until it passes its centre, then
  snaps over. It has to be thrown fully open, and later driven back past its centre with the sheets in place,
  without the hand landing on the sheets.
- **Constrained order.** Open, thread, then close. A closed ring cannot be threaded, and forcing the moving halves
  open by hand scores 0.

## Run it

On a machine with the `yam_arena` embodiment for Inspect Robots installed:

```
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/lever-arch-filing/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --store-frames --scorer task_score \
  --instruction "File the punched sheets in the lever arch file and lock its rings shut." \
  --max-steps 2400 --seed 0 --log-dir logs
```

For Dream-1, use `--policy dream` instead of `yam_scripted`. The score is `results.metrics.task_score` in the
log's JSON. `inspect-robots video <log.json>` writes the camera videos.

- **Seeds:** the layout is fixed. The seed only changes the simulator's noise. The demo was run with seeds 0, 4
  and 10.
- **Runtime per trial:** the scripted demo is 670 control steps (33.5 s of simulated time). On our shared GPU that took about 25 minutes of wall time; a Dream-1 trial of 2,400 steps takes proportionally longer.
- **Run limit:** 2,400 control steps (120 s at 20 Hz). The scripted demo needs about 670 steps.
- **Loophole runs:** set `DEMO_CHEAT=naive` (stack lowered onto the shut rings, lever never touched), `open` (stack
  filed, rings left open) or `force` (the hand pushes on a ring's moving half instead of the lever).

## Results

| Run | Score |
|---|---|
| Scripted demo, seeds 0, 4, 10 | 1.0, 1.0, 1.0 (about 670 steps each) |
| `DEMO_CHEAT=naive`: stack lowered onto the shut rings | 0.0 (the sheets rest on top of the rings) |
| `DEMO_CHEAT=open`: stack filed, lever left open | 0.5 |
| `DEMO_CHEAT=force`: ring half pushed by hand | 0.0 (the half did not open; the sheets rest on the rings) |
| Dream-1, seed 0, 1,200 steps | 0.0 (the arms rise and hover; nothing is touched) |
- **Unit tests** (no simulator): `pytest tasks/lever-arch-filing/test_lever_arch_filing.py`.
- `BINDER_DEBUG=1` prints the lever, arch, holes and score every half second.

## Scripted demo

`binderdemo.py` is closed loop. Like other scripted demos, it reads the simulator's state (lever angle, stack
pose) to plan its moves. The left hand pinches the lever's finger plate across its width and throws it past its
centre; the spring opens it the rest of the way. Both hands pinch the stack's holed strip at its two overhanging ends,
with the fingers tilted 12° down so the wrists stay off their limit. The right hand pinches lightly and only supports
the stack, so the two arms do not fight over it. The left hand servoes both holes onto both posts (it shifts and
turns the stack) and lowers them 8 mm onto the posts. Both hands let go, and the sheets slide down the posts. The left
hand then pinches the lever again, swings it back to 15° (past its centre) and lets go; the spring snaps both rings
shut.

## Files

- `task.py`: the scene, the hooks that run the mechanism, the trial record and the score.
- `bindermodel.py`: geometry, the lever's over-centre law, the cam between lever and rings, and the score rules.
  Plain NumPy.
- `binderkit.py`: multi-box articulated parts, joint efforts each physics step, and hooks into `yam_arena`. The
  `yam_arena` package is not modified.
- `binderdemo.py`: the closed-loop scripted demo and the loophole runs.
- `test_lever_arch_filing.py`: unit tests.

## Scene (real trial)

Positions are in cm from the midpoint between the two arm bases, on the table top. x is forward, y is to the
robot's left.

| Item | Real product | Size, mass | Approx. price |
|---|---|---|---|
| Lever arch file | Leitz 180° lever arch file, polypropylene, A4, 80 mm spine, blue (10101035) | 318 x 285 mm boards, 80 mm spine, 0.49 kg | $7 |
| Sheets | 20 sheets of A4 80 g/m² copier paper, two-hole punched to ISO 838 (holes 6 mm, 80 mm apart, 12 mm from the edge) with any 2-hole desk punch, for example Leitz 5008 | 297 x 210 x 2 mm, 100 g | $1 (punch $15) |
| Reams | two wrapped 500-sheet reams of A4 80 g/m² copier paper (for example Navigator Universal), stacked | 297 x 210 x 100 mm together, 5 kg | $12 |

Total about $35.

- **File:** open flat on the table. The back board (the one with the mechanism) is at x 20.5 to 49, y -15.9 to
  15.9. The spine is at x 49 to 57, and the front cover lies flat at x 57 to 85.5. The rings' fixed posts are at x
  44, y +4 and -4. The lever is at the file's left end (y +16). It starts folded flat and the rings start shut.
  Tape the back board's two near corners to the table.
- **Reams:** stacked, laid crosswise in front of the file. They cover x -10.7 to 19, y -10.5 to 10.5, and are taped
  to the table.
- **Sheets:** a squared stack of 20 sheets on the reams. The holed long edge is at x 15, parallel to the file's
  spine, facing it. The stack is centred on y 0, so each short end overhangs the reams by 4.4 cm.
- **Arms:** start in the ready pose.

Measured figures of the file's mechanism, used in the simulation (check on your file):
- ring wire 4.6 mm across; each fixed post stands 65 mm above the board;
- lever: about 10 N at the finger plate at its stiffest point, 20° up; centre at 40°; open stop at 80° (the lever stands up). To
  measure, hook a luggage scale on the finger plate and pull it square to the lever. YAM fingers give about 40 N.

## Scoring

The score is read at the end of the trial. Violations are recorded over the whole trial and stay recorded.

| Condition (sim) | Real-trial check |
|---|---|
| **0 for the whole trial** if at any time: the stack is creased (bent over 120° at a fold); the stack falls off the table; or the rings' moving halves are pushed open by hand further than the lever has opened them (more than 6° beyond the cam, with a gripper within 4 cm of a ring). | Video: a sheet folded or creased, sheets dropped off the table, a gripper pulling or pushing a ring half instead of working the lever. Photo after the trial: a sheet torn at a hole also scores 0. |
| **1.0**: both posts pass through both holes (each hole's centre within 3 mm of its post's axis and 5 mm or more below the post's tip), the lever is folded down within 5° of flat and both rings are shut (moving halves within 2°), and both grippers are 3 cm or more from the stack's holed strip and from the lever. This must hold for 1 s. | Photo: all 20 sheets are on both rings, the lever lies flat and both rings are closed. Video: both grippers are off the sheets and the lever. |
| **0.5**: both posts through both holes, but the rings are not shut, or a gripper is still on the sheets or the lever. | Photo: the sheets are on both posts. |
| **0.2**: one post through its hole. | Photo. |
| **0.1**: the lever was thrown past its centre (40°) at some point. | Video: the rings opened. |
| **0** otherwise. | |

In a real trial, score 1.0 only if all 20 sheets are on both rings. If some sheets are off a ring, score the trial
as 0.2.

## Fidelity

- **Sheets.** The stack of 20 sheets is three rigid slabs, each 2 mm thick and 297 mm long: the holed strip (40 mm
  wide, with the two holes), then two parts of 85 mm. They are hinged along lines parallel to the holed edge, with
  bending springs of 0.12 and 0.04 N m/rad. Held by the strip, the rest droops like a real stack. Sheets cannot slide
  over each other, fan out or tear. A torn hole can only be checked in a real trial.
- **Holes and posts.** The holes are 6 mm square (ISO 838 holes are 6 mm round). The posts are 4.6 mm round with a
  3 mm rounded tip, so there is 0.7 mm of clearance on each side.
- **Mechanism.** The lever and the rings' moving halves are two hinged parts of the file. A joint torque set every
  physics step gives the lever its over-centre spring: it pulls the lever shut below 40° and pushes it open above,
  with up to 10 N at the finger plate. A stiff joint drive stands in for the cam: the moving halves follow the lever
  at 0.6 times its angle, and a hand pushing on them feels the lever's resistance.
- **File.** The boards, spine and mechanism plate are rigid and fixed to the table, as if taped.

## Real-trial safety

The objects are paper and a polypropylene file. There are no liquids, sharp edges or heavy parts. The ring tips
are rounded. Nothing can damage the rig.

## Nearest tasks

- **On the board:** TK-T5MAQE "Refit a car headrest" (accepted): two rigid posts into two sleeves. Here the two holes
  are in a thin, floppy stack that the robot can hold only by its ends, the posts must first be freed by working a
  lever, and the rings are closed afterwards. TK-FFD5AU "Close a swing-top bottle" (accepted): an over-centre lever
  pushed through its centre. Here the lever is only one step; the core skill is threading a floppy two-hole stack
  over two posts. TK-CCL8DV clipboard (pending) and TK-45MYA3 hanging file (pending) are other office filing tasks:
  sliding a card under a sprung clip, and hooking a folder's rods on two rails. Neither threads holes over posts.
  TK-MGT984 (stapler) and TK-UCDSYT (binder clip) do not use a ring mechanism.
- **External benchmarks:** we found no ring-binder or lever-arch filing task in LIBERO, RoboCasa, BEHAVIOR-1K,
  SimplerEnv, DataCenterBench or RoboDojo. The nearest are peg-in-hole insertions (for example in the Functional
  Manipulation Benchmark and NIST assembly boards): one rigid peg into one rigid hole, with nothing flexible and no
  mechanism to open first.

## Third-party assets

This task adds no third-party assets. Every shape (file, mechanism, sheets, reams) is built from boxes and
cylinders in `task.py`. All the code in this folder is the task authors' own.

The trial runs on the program's `yam_arena` embodiment, which this folder does not include. It provides:
- the YAM arm model (URDF, meshes and USD in `yam_arena/assets/yam/`), from i2rt
  (github.com/i2rt-robotics/i2rt), under the MIT licence (`LICENSE.i2rt` there);
- the station table, lights and cameras, from the `yam_arena` package. Its README says it has no licence yet.
