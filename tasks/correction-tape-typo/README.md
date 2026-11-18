# Cover a typo with correction tape

A printed one-page letter is taped flat to the table in front of the right arm. Its fifth line has a typo, "teh",
between "file" and "report". A correction-tape dispenser lies flat beside the sheet, its nose pointing away from
the arms.

**Instruction:** `Cover the typo 'teh' in the letter with the correction tape, without covering the words next to it.`

## Why this task

- **What it is.** Whiting out one word with a correction-tape roller. Correction tape is in nearly every school
  bag and office drawer, and most people have used it. Anyone watching a short clip knows what the robot is doing.
- **The difficulty is in the stroke.** The tape comes off the nose only while the nose is pressed flat on the
  paper and pulled sideways the way it lays tape. Too light and the film does not grip the paper (gaps). Tilted
  too steep or too flat, or rolled to one side, and one edge of the nose lifts. Pushed the wrong way and the tape
  bunches up. The tape is 4.2 mm wide and the word is 3.1 mm tall, so the nose must run along the word within about
  0.5 mm of its centre line. It must also start and stop in the 1.1-1.2 mm gaps between the words. The robot has to
  hold a light, steady contact force at a fixed tool angle while moving along a line, with millimetre start and stop
  points. This is force and position control of a held tool on a surface. Seeing the word is easy: it is printed
  in a fixed place.
- **The score is the outcome.** The score is what a person checks on the paper: how much of the typo is hidden,
  whether the neighbouring words are untouched, and whether it took one stroke.

### Distinct from

- **"Seal a filled carton with a tape gun" (accepted).** A tape gun is run along a 160 mm seam with a 10 mm
  tolerance, then torn off; the gun's roller presses the tape. Here the stroke is 7 mm long and must start and stop
  within about 1 mm, the tape must stay within 0.5 mm of a line, and it only comes off within a window of nose angle
  and pressure.
- **"Write the use-by date on a container's label with a marker" (pending).** The skill there is forming
  characters. A marker leaves ink at any angle and light pressure. Here the stroke is a single straight line, and
  the tool only works inside a narrow angle and force window.
- **"Snap a chalk line between two marks" (accepted).** That task is a taut string plucked once. Here a held
  tool is pressed and dragged.
- **Known suites.** Wiping and drawing tasks in BEHAVIOR-1K, RoboCasa and LIBERO score area coverage with a cloth
  or sponge, where angle and force hardly matter. None of them covers a single word with an exact start and stop.

Category: **Societal relatability.** Covering a typo with correction tape is an everyday task that most people
have done themselves.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, `score()` / `success()`, and `controller()` (the demo `yam_scripted` plays) |
| `tapemodel.py` | The tape rule (when tape comes off the nose, the 0.1 mm coverage grid) and the score; pure Python |
| `tapekit.py` | The sheet and the dispenser, the 240 Hz update of the tape rule, the drawn tape, the hooks into yam_arena |
| `tapedemo.py` | The scripted demo |
| `make_sheet.py` | Makes `sheet.png` (the page to print), `sheet_tex.png` (its texture in the simulation) and `sheet_layout.json` (every word's box) |
| `sheet.png` | The letter, A4 at 300 dpi: print at 100% |
| `test_correction_tape_typo.py` | Unit tests of the rule and the score (no Isaac Sim needed) |

`tapekit.py` hooks yam_arena without modifying it. It patches `yam_arena.scene.task_object` and `object_states`,
the embodiment's `_ensure_sim` and `reset`, and `yam_arena.scripted.YamScriptedPolicy`. When a task file defines
`controller()`, `yam_scripted` asks it for each action. The kit wraps `InteractiveScene.write_data_to_sim` to step
the tape rule before every physics step. The task runs with the standard `yam_arena` embodiment, `yam_scripted`
policy and `task_score` scorer.

## Setup

Positions are in cm from the midpoint between the two arm bases on the table top (x forward, y left). The arms
start in the ready pose. The layout is the same in every trial.

- **Letter:** `sheet.png` printed at 100% (no scaling) on one A4 sheet of 80 g/m2 white copy paper (210 x 297 mm),
  portrait. Eight lines of Liberation Sans 12 pt (metrically the same as Arial), first baseline 140 mm from the top
  edge, 25 mm left margin, 18 pt line pitch. Line 5 reads "Please file teh report by Friday so the board can see
  it." The ink box of "teh" is 5.9 mm long and 3.1 mm tall. The gaps to "file" and to "report" are 1.2 mm and
  1.1 mm.
- The sheet lies with its text readable from the arms' side. Its top edge is at x 46.4, its left edge at y -3.1,
  so it spans x 16.7 to 46.4 and y -3.1 to -24.1. The centre of "teh" is at x 30.0, y -8.0. It is taped down at
  its four corners with 19 mm clear office tape (for example Scotch Magic Tape 810), about 3 cm per corner.
- **Correction tape:** BIC Wite-Out EZ Correct correction tape, single dispenser (BIC WOTAPP11; about $3), 4.2 mm
  (1/6 in) x 12 m (472 in). It is a sideways-pull dispenser: hold it with the logo up and the nose flat on the
  paper at an angle, press down and move it sideways along the line, left to right. Its plastic case is about
  85 x 33 x 13 mm and about 16 g (measure yours; the simulation uses 85 x 33 x 14 mm, 16 g). It lies flat, logo up,
  long side along x, nose toward +x (away from the arms). The nose tip is at x 30, y -27, and the case runs back
  to x 21.5. Use a new or nearly new dispenser, so that the first stroke lays tape.
- Nothing else is on the table. The other arm is free; it need not do anything.

Real-trial kit cost: about $3 for the dispenser, $0.05 per printed sheet and tape. Nothing in the kit is sharp,
wet or messy: the correction film goes on dry.

## Run a trial

```
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/correction-tape-typo/task.py -E object_jitter=0 -E control_hz=20 \
  --scorer task_score \
  --instruction "Cover the typo 'teh' in the letter with the correction tape, without covering the words next to it." \
  --max-steps 1200 --seed 0 --store-frames --log-dir ~/yam-eval/correction-tape-dream
```

Run limit: 1,200 steps (60 s at 20 Hz). The scripted demo succeeds in 383 steps (19.2 s); a trial takes about 5 min
of wall time on one GPU, mostly start-up.

## Scoring

The score is computed from the tape on the sheet at the end. The stroke count and bunching are latched over the
whole trial. Success (which ends the episode early) is a score of 1.0 with the dispenser lifted 3 cm or more off
the paper, held for 1 s.

| Score | When |
|---|---|
| 0 | the tape hides more than 1.5 mm of any other word's length (for example "file" or "report"), or tape was laid off the letter |
| 0.6 x C | otherwise. C is the share of the ink box of "teh" under tape, counted in full at 95% or more (0.6 x min(1, C / 0.95)) |
| + 0.2 | C is 90% or more, and no other word is touched at all (no more than 0.2 mm of its length under tape) |
| + 0.2 | C is 90% or more, and the tape went on in one stroke (one touch-down and lift that laid tape) without bunching |

So 1.0 is "teh" fully hidden in one clean stroke. 0.8 is hidden but a nick into a neighbour, or in two strokes.
0.6 is hidden but both. Partial cover scores in proportion. Running into the next word scores 0.

### Real trial

After the trial, photograph the line from straight above with a steel ruler (0.5 mm marks) laid along it. A phone
macro shot is enough.

- **Share of "teh" hidden (C):** a single stroke lays a straight white strip. Measure where it starts and ends
  along the line, and where its upper and lower edges are, against the word's box: from the left edge of the "t"
  to the right edge of the "h", and from the top of the "h" to the baseline. C is the overlap area over the box
  area. With several strips, add up the overlap (the sheet layout gives the box to the 0.1 mm).
- **Other words:** how much of a neighbouring word's length is under the strip, measured with the ruler: over
  1.5 mm scores 0; any visible nick into a letter (over 0.2 mm) loses the clean credit.
- **One stroke, bunching:** from the trial video. One stroke means the nose was put down once and lifted once
  while laying tape. Bunched means the tape crinkled or folded at the nose, which happens when the dispenser is
  pushed the wrong way.
- **Off the letter:** any tape on the table or past the sheet's edge.

## The simulation

- **Sheet:** a kinematic box 0.2 mm thick (taped down), with `sheet_tex.png` (the printed letter) on it.
- **Dispenser:** one rigid body. The case is a box 73 x 33 x 14 mm, with a nose tongue 12 x 8 x 4 mm and the
  pressing lip, a pad 3 x 8 x 1.5 mm with its face at 45 degrees to the case. Total mass 16 g. Friction:
  case to gripper pads 1.5, lip to paper 0.15. The lip slides on the tape's backing, and the only drag is the spool
  mechanism.
- **Tape rule** (`tapemodel.py`), every physics step (240 Hz). It reads the lip's position on the sheet and its
  PhysX contact force, and the dispenser's orientation. Tape comes off the lip only when all of these hold:
  - the force is 0.8 N or more;
  - the lip is within 15 degrees of flat about the tape's width (the case at 30-60 degrees to the paper);
  - the lip is within 8 degrees of flat sideways;
  - the lip moves in the pull direction (within 45 degrees of it), at any speed: the film feeds out as far as the
    lip moves.

  The force is the largest PhysX reading over the last 50 ms (PhysX misses a light sliding contact on some
  substeps).

  The tape covers the 4.2 mm strip the lip sweeps, on a 0.1 mm grid. Pressed and moved against the pull direction
  for more than 1 mm, the tape bunches (latched). A new stroke starts when tape is laid again after the lip has
  been off the paper for 0.1 s. Laid tape is drawn as white strips.
- These thresholds are estimates. To check them on the real product, lay strokes on a sheet resting on a kitchen
  scale (0.8 N reads as 80 g), and at set angles against a protractor, and see where the film stops transferring.
  The BIC instructions say to press down firmly. Our estimate is 1-3 N, far below the arm's limits.

## Scripted demo

```
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/correction-tape-typo/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --scorer task_score --instruction "..." --max-steps 1200 --seed 0 --store-frames --log-dir ~/yam-eval/correction-tape-demo
```

`tapedemo.py` (right arm; the left arm turns away):

1. Grip the dispenser across its 33 mm width, from above and behind (pitched 45 degrees), and lift it. It reads
   the dispenser's pose once, right after lifting, to know how it sits in the fingers (as a person looks at the tool
   in their hand). It reads nothing else from the simulator: not the contact force and not the tape.
2. Tilt the dispenser to 45 degrees, nose down and forward, with the case's width along the line. Hold the nose
   3 mm above the paper, just before the "t", centred on the word's height. From here on it corrects the tool
   point in x and y from the measured joints: forward kinematics, the measured wrist tilt, and the in-hand pose
   give where the nose is. Under the pressing force the wrist tilts a little, and 48 mm above the nose that moves
   the nose by about a millimetre.
3. Lower the nose to 4 mm below the paper. The arm's joints are springs, so the nose rests on the paper with about
   2 N. Pull it along the line at 3 mm/s to the end of the "h". The pressed arm lags behind the command, so it
   holds there for 1.5 s and lets the feedback bring the nose on to the end (forward only). Then it lifts straight
   off from where the nose is, and turns away.

Scores (scripted demo, `-E object_jitter=0`): seeds 0, 4 and 10 all 1.0 (98% of "teh" covered, one stroke, no
other word touched). The layout is fixed, so the seeds give the same run.

Dream-1 (seed 0, 1,200 steps): 0.0. It reached toward the dispenser but laid no tape.

Contrasts (environment variable `DEMO_CHEAT`):

| `DEMO_CHEAT` | What it does | Score |
|---|---|---|
| `sweep` | one stroke along the whole line, start to end | 0.0 (the whole line is covered, including "file" and "report") |
| `light` | the same stroke with the nose only touching the paper | 0.0 (no tape comes off) |
| `push` | the same hold, pushed the wrong way (right to left) | 0.006 (the tape bunches at once; a 0.1 mm speck) |
| `two` | the word in two strokes, lifting in the middle | 0.48 (2 strokes, 76% covered: no stroke credit, and the restart misses a strip) |

## Tests

```
python -m pytest tasks/correction-tape-typo/test_correction_tape_typo.py
```

## Third-party assets

- **Robot:** the YAM arms and the station scene come from `yam_arena` and its assets. They are used as the program
  provides them, under their licence terms; nothing of them is redistributed here.
- **Font:** the letter is set in Liberation Sans Regular (SIL Open Font License 1.1). Only the rendered page
  (`sheet.png`, `sheet_tex.png`) is included, not the font file.
- Everything else (the dispenser, the sheet, the tape rule) is modelled in code here from boxes, with no meshes.

## Real-trial safety

Nothing sharp, hot or wet. The dispenser is a 16 g plastic case, and the contact forces are a few newtons. The
correction film is dry and non-toxic. If a stroke leaves film on a gripper pad, it peels off by hand.
