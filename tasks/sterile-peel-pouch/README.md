# Open a sterile peel pouch and drop the forceps onto the sterile field

A sealed paper/film sterilization pouch holding 14 cm dressing forceps lies flat on a low block beside a blue sterile
towel drape. The robot must take the two unsealed tabs at the chevron, one in each hand, lift the pouch, hold it over
the field and peel the two plies apart until the forceps drops out onto the drape. It must never touch
the forceps or the drape, the pouch must not touch the drape, and the forceps must land inside the field's 2.5 cm
border.

**Instruction:** `Open the sterile pouch and drop the forceps onto the sterile field without touching the forceps or the drape.`

## Changes since the last review

- **TK-N5GU6U:** "It should start in a reasonably flat position instead of being propped up". The pouch now starts
  lying flat, paper up, on a 3 cm block (the stand is gone). The scripted demo first lifts the pouch by the upturned
  paper tab with one hand, then takes the film tab with the other once the pouch hangs; the scorer is unchanged.

## Why this task

- **The job.** Opening sterile supplies onto a sterile field is the first step of every minor-procedure and dental
  tray setup: suture and staple removal, incision and drainage, biopsies, dressing changes, dental instrument trays.
  The person opening the pouch is not gloved sterile: they may touch only the outside of the pouch, and the contents
  go onto the field without touching the opener, the pouch's edges or the field's border (CDC/AORN aseptic
  technique). This is regular work for **medical assistants** (O*NET 31-9092, about 0.8 M US workers: "assist
  physicians in examining and treating patients, handing them instruments or materials") and **dental assistants**
  (O*NET 31-9091, about 0.37 M: "prepare patient, sterilize or disinfect instruments, set up instrument trays");
  together about 1.2 M. LPNs and surgical technologists open pouches too, but we do not count them.
- **The difficulty is physical.** The seal parts only under a steady pull of about 3 N, and both plies must be drawn
  apart together: one hand alone just swings the hanging pouch. The pouch hangs from the two tabs, flexible, with a
  30 g instrument inside; while peeling, the hands must keep it level and high enough that it never brushes the
  field, keep pulling apart until the seal has opened along its whole length (about 21 cm, so the hands end some
  50 cm apart), and do it over the field so the forceps falls onto the sterile area, not its border. Grabbing the
  pouch body or shaking the forceps out are both ruled out by the scoring (touching the instrument, contaminating the
  field).
- **It starts flat.** Only the top ply's tab can be taken at first. One hand must lift the pouch by that tab so that
  it hangs and the lower ply's tab turns out; only then can the other hand take the second tab.
- **The pouch is a flexible two-ply sheet, not a rigid box.** Each ply is a chain of eleven hinged strips; the two
  plies are joined by breakable seal joints at the real seal strength (below).

### Distinct from

- **"Load hinged surgical forceps into a sterilizer basket" (accepted).** That places an instrument in a basket.
  Here the instrument is never touched; the skill is the two-handed peel of a sealed flexible pouch and a no-contact
  transfer onto a target.
- **"Clear away a used procedure tray that still has a needle in it" (accepted, safety).** Sharps handling, no
  sealed packaging and no aseptic transfer.
- **Bag tasks on the board (filling and sealing a storage bag, pinching a zip-top track shut).** Those close a bag;
  this opens a sealed peel seam by a symmetric bimanual peel and lets the contents fall out.
- **External task sets.** BEHAVIOR-1K has unpacking and "open" tasks on rigid containers and boxes; RoboCasa and
  LIBERO open drawers, doors and lids. None peels a two-ply heat seal apart with both hands or scores a no-touch
  transfer onto a target.

Category: **Impact on jobs** (medical and dental assistants).

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: instruction, `score()` / `success()`, `controller()` (the demo `yam_scripted` plays) |
| `pouchkit.py` | The pouch (two hinged-strip plies, seal joints, seams), forceps, rest block and drape; the latches; the hooks into yam_arena |
| `peeldemo.py` | The scripted demo and the contrast runs |
| `test_sterile_peel_pouch.py` | Tests: pouch and forceps geometry, seal anchors, peel progress, drape border, every scoring outcome |
| `task.json` | Submission details |

`pouchkit.py` hooks yam_arena without modifying it: it patches `yam_arena.scene.task_object`
and `object_states`, the embodiment's `_ensure_sim` and `reset`, `yam_arena.scripted.YamScriptedPolicy` (so
`yam_scripted` plays the task file's `controller()`) and `yam_arena.demo.grip_rotation` (a waypoint may carry a full
grip rotation). It wraps `scene.write_data_to_sim`, which runs once per 240 Hz physics substep; the latches are
checked at 60 Hz.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left, z up). Arms start in
the ready pose. The layout is the same in every trial (no seeded shift).

| Object | Description |
|---|---|
| Pouch | Self-seal sterilization pouch, 3.5 x 9 in (89 x 229 mm), medical-grade paper on one side, blue-tinted PET/PP film on the other, extra-wide chevron seal, Class 4 indicator (McKesson Argent Sure-Check 73-SSP381, 200/box, about $15; any 3.5 x 9 in chevron peel pouch works). Lying flat on the rest, paper side up, film side down, its length across the table (y): chevron end toward the right arm at y 0, the bottom seal at y 20.9. Width centred at x 24. Beyond the chevron each ply has a 20 mm unsealed tab: the paper tab is turned up (it stands 2 cm above the pouch), the film tab hangs down over the rest's edge |
| Forceps | Stainless steel dressing (thumb) forceps, 5.5 in (14 cm), serrated tips, about 30 g (any 5.5 in dressing forceps, about $3). Inside the pouch, lying flat on the film ply, tips toward the bottom seal |
| Rest | A 24 cm length of 30 x 40 mm planed pine batten (about $3), laid flat (40 mm wide in x, 30 mm high), centred at x 24, from y 0.2 to y 24.2; tape it to the table. The pouch lies on it, its chevron 2 mm past the block's end, so the film tab can hang free. The pouch (89 mm wide) overhangs it on both sides |
| Drape | Sterile poly-lined towel drape, 18 x 26 in (457 x 660 mm), blue, tissue/poly/tissue (Dynarex 4410, about $0.50 each). Flat on the table, poly side down, centred at x 52, y 0, the 26 in side across (y); its near edge is at x 29. Tape its corners. Draw the 2.5 cm border on the table round it, or measure from the edge after the trial |

Nothing else is on the table. The pouch lies paper up because the paper tab is then the one that can be taken first;
either way up works. The rest is there only so the lower tab can hang free, as it would at the edge of a counter or
tray.

## Scoring

Latched over the whole trial:

- **0** for the rest of the trial once a gripper touches the forceps (sim: a finger pad tip or tool point within
  8 mm of it) or the drape (within 1 cm of the drape's top, over the drape). Real trial: any contact seen in the
  trial video.
- **Pouch touched the drape** (sim: a pouch strip corner within 2 mm of the drape's top, over it). Real trial:
  video.

At the end of the episode:

| Outcome | Score | Real-trial check |
|---|---|---|
| Forceps lying on the drape itself, still, every point at least 2.5 cm in from the drape's edges, and the pouch never touched the drape; held 1 s | **1.0** | Photo with a ruler: distance of the forceps from each edge of the drape |
| As above but part of the forceps within the 2.5 cm border | 0.5 | Same photo |
| Forceps on the drape but the pouch touched the drape | 0.25 | Video |
| Forceps not on the drape (still in the pouch, in the air, on the table or on the pouch), seal open along its whole length | 0.2 | Photo of the two separated plies |
| As above, seal open from the chevron for at least half its length (105 mm) | 0.1 | Ruler along the peeled seal |
| Otherwise | 0 | |

"On the drape itself": the forceps' lowest point within 1 mm of the drape's top (sim), so a forceps lying on the pouch
on the drape does not count; real trial: the forceps rests directly on the drape (photo). The hands may do anything
with the empty plies afterwards except let them touch the drape.

Run limit: 1,200 steps (60 s at 20 Hz); the demo reaches full score after about 20 s (400 steps).

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the sterile pouch and drop the forceps onto the sterile field without touching the forceps or the drape." \
  --scorer task_score --max-steps 1200 --seed 0 --store-frames --log-dir ~/yam-eval/sterile-peel-pouch-dream
```

`task_score` gives 1.0 on success, otherwise `score()` at the end of the episode. Seeds change nothing in the layout
(`object_jitter=0`, no seeded shift); we ran seeds 0, 4 and 10. `POUCH_DEBUG=1` prints the forceps' pose, which seal
boundaries have parted and the hands' positions. The kit prints each latch when it fires and the seal length opened
as it grows. A trial takes about 4 minutes of wall time on one GPU (most of it start-up). Run one episode per process:
broken seal joints are not restored by a reset.

## Scripted demo

Same command with `--policy yam_scripted`. It is open loop: it plans from the layout constants in `pouchkit.py` and
reads no simulator state.

1. The left hand comes in from the front, pointing down 40 degrees, fingers 40 mm apart either side of the upturned
   paper tab, and pinches the tab's front corner; the pads stay above the paper ply. It lifts the chevron end and,
   rising to 30 cm, turns the tab level: the pouch slides off the rest and hangs from the paper tab, and the film tab
   turns out level toward the right.
2. The right hand waits until the pouch has hung for about 2 s, then comes in from the front, its approach
   turned 40 degrees in toward the middle, fingers above and below the film tab, and pinches its front corner. The
   pads end 8 mm out from the ply, so they touch only the tab.
3. Both carry the pouch forward over the field (its middle
   at x 47, 5 cm short of the drape's centre, because the forceps tends to slide forward off the opened plies), the
   hanging pouch's bottom 9 cm above the drape.
4. Both hands draw apart over 6 s, to 26 cm either side, coming down 3 cm (to 27 cm above the table) as the
   hanging part of the pouch shortens. The seal parts line by line from the chevron down; once most of it is open
   the forceps slides out of the mouth and drops onto the drape.
5. The hands carry the two empty plies out to the sides, 12 cm beyond the drape's side edges, lay them on the table,
   let go and return to the ready pose. (The run ends earlier, once success has held 1 s.)

Contrasts (environment variable `DEMO`): `fast` peels in 1.2 s; `low` carries the pouch 10 cm lower; `grab` pinches the
flat pouch's sealed body (both plies) at its front edge, where it overhangs the rest, in one hand and shakes it over
the field; `short` peels only to 10 cm either side and
lets go over the field.

### Checked in simulation

Isaac Lab-Arena (Isaac Lab 3.0), the command above with `--policy yam_scripted --max-steps 600` and
`POUCH_DEBUG=1`. Camera frames were checked (normal renders).

| Run | `task_score` | What happened |
|---|---|---|
| Demo, `--seed 0` | **1.0** | The left hand lifted the flat pouch by the paper tab; it slid off the rest and hung, with the film tab turned out level. The right hand took the film tab at 11 s; both carried the pouch over the field. The seal parted line by line from about 15 s to 18.8 s; the forceps slid out of the mouth and dropped onto the drape near x 37, y 10; no latch fired; success held 1 s |
| Demo, `--seed 4` | **1.0** | Same (the layout does not depend on the seed) |
| Demo, `--seed 10` | **1.0** | Same |
| `DEMO=grab` | **0** | One hand pinched the flat pouch's body at its front edge, lifted it, shook it and dropped it on the field: the pouch touched the drape and the forceps stayed inside it. The pinch parted 3 seal lines in the middle (see "Load direction" below), none from the chevron |
| `DEMO=short` | **0.25** | Peel stopped at 57 mm and the pouch was let go over the field: it landed on the drape and the forceps then slid out onto the drape. The pouch touched the field, so 0.25 |
| `DEMO=low` | **0.25** | Carried 10 cm lower, the hanging pouch dragged on the drape at 12.4 s (latched); the peel then dropped the forceps onto the drape, 0.25 |
| `DEMO=fast` (1.2 s peel) | **1.0** | A fast symmetric peel also works in simulation (the model has no tearing) |

The landing point varies by several centimetres with small differences in the peel: the 640 x 352 run used for the
demo video (seed 0, also 1.0) landed the forceps near the middle of the field, the 320 x 176 runs near x 37. Moving the
field point 3 cm further out made the forceps stay in a ply (0.2), so the demo keeps the earlier field point.

The `grab` run shows a limit of the seal model (below): a load of 3.2 N in any direction parts a seal line, so a hard
pinch or a snag can open lines that a real pouch would keep closed. The rest is the only fixed object and it is off
the field, so a seal burst there earns at most 0.2.

One Dream-1 run on the flat start (`--policy dream`, the command under "Run a trial", seed 0, 1,200 steps, frames
checked) scored 0: it raised both arms upright beside the table, closed the grippers and never touched the pouch, the
forceps or the drape.

## Physical model and fidelity

- **Plies.** Each ply is 11 rigid strips (89 x 19 mm) joined by hinges (revolute joints about the width axis, light
  damping) plus the tab, so the ply bends along its length as a sheet does. Strips collide as 1.5 mm boxes (the real
  sheet is about 0.1 mm) and weigh 1.5 g each (real: about 0.15 g); the extra mass keeps the joint chain stable in
  PhysX and makes the hanging pouch about 30 g heavier than the real one, which makes the peel slightly harder.
- **Seal.** At each of the 12 seal boundaries (the chevron, ten lines across the body, the bottom seal) the plies are
  joined by two point joints, one in each side seal, each breaking at 1.6 N (PhysX break force): 3.2 N to part a
  boundary. EN 868-5 requires sterilization-pouch seals to hold at least 1.5 N per 15 mm strip (steam; 1.2 N for
  other processes); a peel front crossing two side seals about 8 mm wide each needs about 1.6 to 4 N. The real
  chevron needs a little less at the very start (its point is narrow) and the self-seal adhesive at the bottom end
  about the same. Real-trial check of the pouch batch: hang a 0-10 N spring scale (or a luggage scale) from one tab,
  hold the other, and read the peel force; it should be 2 to 5 N.
- **Pocket.** The plies are 6 mm apart (the forceps is 2.5 mm thick); each ply carries half of the side and bottom
  seams as lips, so the pocket is closed until the seal parts. The real pouch is flat at the seals and bulges around
  the instrument.
- **No tearing.** The sim does not tear paper; a real pouch peeled at a sharp angle or jerked can tear fibres. Not
  scored.
- **Load direction.** PhysX breaks a joint on the size of its load, not its direction, so in simulation a seal line
  also parts if it is sheared or squeezed with more than 3.2 N (for example a pouch snagged on the rest, or both
  plies pinched hard in one hand). A real seal resists shear and squeezing far more than peeling. Neither gives
  credit in the real trial, where only a real peel opens the pouch.
- **Contact checks** use the finger pad tips and the tool point (8 mm to the forceps, 1 cm to the drape), not a
  contact sensor; the real trial uses the video.

## Real trial

**Hardware (about $25 plus the robot):** one box of 3.5 x 9 in self-seal sterilization pouches (one per trial), one
5.5 in dressing forceps, one pack of Dynarex 4410 towel drapes (one per trial, or reuse while uncreased), the 24 cm pine
batten for the rest, masking tape, a ruler, a phone camera on a tripod over the table.

**Reset (about 1 minute):** slide the forceps into a new pouch tips first, peel the self-seal liner and fold the flap
over (no sterilizer is needed: the pouch is a training stand-in for a sterile one). Lay it flat on the rest, paper up, its
chevron end 2 mm past the block's end; turn the paper tab up and let the film tab hang down, by hand: the tabs beyond
the chevron are unsealed, and turning them out is not part of the task. Lay the drape and tape it.

**Safety and the rig:** the pouch, forceps and drape are dry, clean and light (under 0.1 kg in all), the rest is a
taped-down wooden block; the
forceps is round-nosed and never gripped; nothing can damage the pads. Peel forces are 2 to 5 N, far under the
YAM's roughly 40 N per finger; the pouch's weight (35 g) is far under its payload.

## Third-party assets

None: every body is built in code from boxes. The YAM robot model (URDF, meshes and USD) comes with yam_arena from
i2rt's YAM description under its licence (the MIT licence, `assets/yam/LICENSE.i2rt` in yam_arena); this task does not
modify or redistribute it.
