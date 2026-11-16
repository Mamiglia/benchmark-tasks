# Pour four bottles of concentrate into a tank whose inlet is running, and shut the inlet at the fill line

A clear 7.6 L batch tank stands in the middle of the table with water already running into it from a pipe on the
right. Four open 500 mL bottles of cleaning concentrate stand on the left. A float switch at the tank's fill line
lights a red lamp behind the tank. The robot is asked to make up the batch: all four bottles into the tank, and the
tank filled to the line, without an overflow or a spill.

**Instruction:** `Make up the batch: empty the four bottles of concentrate into the tank and fill it to the line. Don't let it overflow or spill.`

The instruction states the batch and asks for no overflow or spill. It does not say that the water is running, which
valve feeds the tank, or when to shut it.

## Why this task

- **Regular work for a large occupation.** Janitors and building cleaners held 2,432,600 US jobs in 2025
  ([BLS Occupational Outlook Handbook](https://www.bls.gov/ooh/building-and-grounds-cleaning/janitors-and-building-cleaners.htm)).
  O*NET lists among their tasks "Mix water and detergents or acids in containers to prepare cleaning solutions,
  according to specifications" ([O*NET 37-2011.00](https://www.onetonline.org/link/summary/37-2011.00)). Making up
  a solution this way is routine on every shift: run water from a tap or hose into a bucket, scrubber tank or batch
  tank, add measured bottles of concentrate, and stop at the container's fill line so that the dilution matches the
  label. Sanitation crews in food and beverage plants make up their cleaning and sanitising batches in the same way.
  The task is in **Impact on jobs**: a robot that does this unattended takes over a step of that work. The rules
  against an overflow and a spill stay because a batch that overflows or is spilled is a failed batch.
- **The level comes from the robot's own work.** The level is a state, not a clock. It rises with every bottle the
  robot pours (each one adds 1.25 cm) and with the inlet (1 mm a second while it is open). The start level is set by
  the seed (2 to 7.5 cm).
- **The correct moment to shut the inlet depends on the state.** The batch must end within 0.5 cm of the line
  (12.5 to 13.5 cm, 5.0 to 5.4 L): that keeps the concentrate (2 L) at 37.0 to 40.0 % of the batch, within 4 % of
  the intended 38.5 %. Above the band the batch is over-diluted and does not clean or sanitise to its rating; below
  it, it is too strong and the line is not reached. The robot must shut the inlet when the level it reads now plus
  the bottles still to pour come to the line. Shut too early, and the batch stays short (the four bottles bring a
  2 cm start level only to 7 cm). Shut it when the lamp comes on with bottles still to pour, and the batch ends
  1.25 cm per bottle above the line: over-diluted. Leave it open, and the tank overflows.
- **The difficulty is in the manipulation.** Each bottle weighs 0.54 kg and is open. The robot holds it from the
  side, lifts it 2 cm clear of the rim, and rolls its wrist through 118 degrees over the tank. The liquid leaves from
  the mouth's lowest edge, so that edge must stay inside the tank's opening for the whole pour: a bottle tipped
  next to the tank, or knocked over, spills. While one arm pours, the other must turn the valve's lever a quarter
  turn when the level calls for it. Neither arm can wait for the other: the inlet keeps filling.

### Distinct from

| Task | Its skill | Here |
|---|---|---|
| TK-WDQMGK Fill the chemical day tank to the mark and close the transfer valve (final rejection: "not robotic") | One valve, one source: open, watch, close | Two sources. The robot pours four heavy open bottles into the same tank. Its own pouring moves the moment at which the inlet must be shut |
| TK-C8UEGJ Stack plates until the alarm light comes on (final rejection) | Stop work when a lamp lights on a fixed schedule | No schedule. The level is raised by the robot's own pouring. Stopping the inlet at the lamp is wrong while bottles remain: the batch ends over-diluted |
| TK-GPXWRJ Dispense 150 g of tea using a digital scale | Stop a tap at a target | The robot pours by hand, with a second source running |
| Pouring in Labutopia (Pour), BEHAVIOR-1K (pour a glass of wine), RoboCasa | Pour from one container into another | The pour is half of the job. The other half is to manage a running inlet from a level that the pours change, with the other arm, during the pour |

## Design

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left, z up). Station
scene (Dream's table).

| Object | Real item | Size, mass | Where |
|---|---|---|---|
| Tank | Clear polycarbonate square tank, white back wall with a black fill line | 20 x 20 cm inside, 20 cm tall, 1 cm floor: 19 cm (7.6 L) of liquid to the rim, 400 mL per cm. Fill line 13 cm above the floor (5.2 L). About 6 kg with its stand | Centred at (40, 0) |
| Inlet | 1/2-inch pipe from a tap, throttled upstream to 2.4 L/min (1 mm of depth a second), spout turned down into the tank | 2.2 cm square pipe at 23.5 cm height, along y at x 36 | From a support at y -43 to the spout at y -7.5 |
| Valve | 1/2-inch brass full-port ball valve | Flat steel lever 11.5 cm long, yellow vinyl grip 1.8 x 6 x 1.8 cm on its outer end, 25.8 cm above the table | On the pipe at (36, -19) |
| Lamp | Float level switch at the fill line, wired to a 24 V red beacon | Beacon 4 x 4 x 3.6 cm on a 29 cm post | Behind the tank, at (52.6, -7) |
| Bottles (4) | 500 mL wide-mouth HDPE bottles, caps off | 7 cm across, 18 cm tall, 53 mm mouth; 500 mL of concentrate each, 0.54 kg | (40, 18), (40, 30), (49, 18), (49, 30) |
| Bin | Grey plastic bin for the empties | 20 x 24 cm inside, 8 cm deep, 1.5 kg | Centred at (30, 47.2) |

**The valve.** The lever starts along the pipe, pointing right: open. A quarter turn counterclockwise (seen from
above), to point away from the arms, shuts it. It turns against 1 N m s/rad of damping (about 0.8 N m, 7 N at the
grip, when turned in 1.5 s) and stays where it is left. Stops at 0 and 90 degrees. The flow falls linearly with the
lever angle and stops at 85 degrees.

**The liquid.** There is no fluid simulation. Code tracks each bottle's contents and the tank's volume, every
control step (20 Hz):
- A bottle holds, at a given tilt, the part of its cylinder below the horizontal plane through its mouth's lowest
  edge (computed for 0 to 180 degrees). Anything above that pours out, at most 100 mL/s. A full bottle starts to
  pour at 64 degrees and is empty (15 mL or less) at 96 degrees.
- The liquid lands below the mouth's lowest edge: in the tank if that point is at least 5 mm inside the walls,
  otherwise on the table (a spill).
- The tank's level is its volume over 400 cm2. It is drawn as 5 mm blue layers. The inlet's stream is drawn as a
  blue bar while the valve passes water.
- The lamp lights at the line and goes out 2 mm below it (switch hysteresis).

### Seeds

The start level depends on the trial seed that the embodiment's reset receives: `seed % 8` picks one of 2.0, 7.5,
5.0, 3.0, 7.0, 4.0, 6.0 or 2.5 cm. inspect-robots derives the trial seed from `--seed` (`derive_seed`, a CRC32):

| `--seed` | Start level |
|---|---|
| 0 | 6.0 cm (2.4 L) |
| 4 | 7.0 cm (2.8 L) |
| 10 | 2.0 cm (0.8 L) |

## Scoring

Evaluated after every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

| Latched for the whole trial (any one: 0) | Real-world check |
|---|---|
| The level passed the rim (overflow) | Liquid in the spill tray under the tank |
| More than 10 mL fell outside the tank (poured outside the opening, or a bottle knocked over) | Weigh the absorbent mat around the tank |
| A bottle went into the tank, or off the table | Video |
| The tank was moved 2 cm or tipped 5 degrees | Taped outline |

| At the end | Score |
|---|---|
| Level at or above the line and the valve not shut (lamp on, inlet running: it will overflow) | 0 |
| Each bottle emptied into the tank (15 mL or less left) | 0.15, up to 0.6 |
| Valve shut (lever at 85 degrees or more) and the level within 0.5 cm of the line (12.5 to 13.5 cm) | +0.4 |
| All four emptied, valve shut, level 12.5 to 13.5 cm, nothing spilled | 1.0 (held 1 s) |

A batch above 13.5 cm (over-diluted) or below 12.5 cm (short) gets no credit for the valve: at most 0.6. A batch
in the band with a bottle not emptied gets at most 0.85. Only all four bottles, the valve shut and the level in the
band give 1.0.

**Reading the final level on a real bench.** The tank's back wall carries a scale in mm and a band of green tape
from 12.5 to 13.5 cm, with the black fill line in its middle. One minute after the trial ends (the surface settled,
the inlet shut), the operator reads the bottom of the meniscus at eye level against the scale through the clear
front wall, to the nearest mm, and records whether it is inside the band. The lever angle is read against a
protractor mark on the valve body (85 degrees or more is shut).

Unintended routes and what they score:
- **Ignore the inlet** and only pour: the tank reaches the line and then overflows, or is still filling at the end
  with the lamp on. 0.
- **Shut the inlet first**, then pour: the batch stays below the band unless the start level is 7.5 cm. At most 0.6.
- **Shut it when the lamp comes on**, then pour the rest: the bottles still to pour take the batch above the band
  (over-diluted). At most 0.6.
- **Shut it at a fixed time:** too early on a low start level (short), too late on a high one (over-diluted or
  overflow). At most 0.6.
- **Shut it at the line and stop** without pouring the rest: 0.15 per bottle poured, plus 0.4: at most 0.85.
- **Drop a bottle into the tank**, or pour it from beside the tank: 0.
- **Shut, reopen to top up, shut again:** allowed. A batch that ends in the band without an overflow is correct.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder. The layout is fixed: always
pass `-E object_jitter=0`.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Make up the batch: empty the four bottles of concentrate into the tank and fill it to the line. Don't let it overflow or spill." \
  --scorer task_score --max-steps 2400 --seed 0 --store-frames --log-dir ~/yam-eval/batch-tank-dream
```

`TANK_DEBUG=1` prints the level, lamp, inlet flow, each bottle's contents and the spill once a second.

## Scripted demo

The same command with `--policy yam_scripted`. `yam_scripted` runs `task.Controller` closed loop (`tankkit` hooks it
in when the task module defines `demo_controller`). **It reads the level with the right wrist camera.** The left arm
pours the four bottles one after another on a fixed plan (it knows the start layout, as every `yam_scripted` demo
does): it grips each from behind, lifts it, rolls its wrist 118 degrees over the tank, holds it there for 6 s, turns
it upright and stands the empty in the bin. Where the empties go is not scored.

The right arm holds its wrist camera over the near side of the tank, looking down across the liquid at the far wall.
From 2.7 s to 9 s (before the first bottle is over the tank) it reads the liquid's top edge there: the mean image
row of the edge in 16 columns, mapped to a level by a quadratic calibrated once in the simulator at known levels (42 frames from 2.5 to 11 cm, largest error 0.6 mm)
(as a camera is calibrated on a real bench with marked fills; `EDGE_CAL`). The drawn level moves in 5 mm steps, so
the moment a new step appears fixes the level to the mm. From that level, the inlet's rate (1 mm/s, set on the
bench) and the 5 cm the four bottles will add, the controller shuts the valve when level now + inlet until shut +
5 cm reaches the line: the right arm lifts clear of the valve, pushes the lever's grip through its quarter turn and
backs off. `CLOSE_LAG` (3.5 s) is the time from that decision until the valve is shut. It does not read the level,
the lamp state or any other simulator value.

Cheat runs, same command: `DEMO_CHEAT=ignore` (never shuts the valve), `DEMO_CHEAT=first` (shuts the valve before
pouring), `DEMO_CHEAT=timed` (shuts it at a fixed 45 s, without looking) and `DEMO_CHEAT=lamp` (shuts it when the
lamp lights in the top camera, then pours the rest: the original demo's strategy).

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, liquid model, `score()` / `success()`, the closed-loop demo |
| `tankkit.py` | Multi-box parts with a hinge (the valve lever) and run-time visibility (liquid layers, lamp), the hooks into yam_arena, and the closed-loop `yam_scripted` hook |
| `test_inlet_alarm_sort.py` | Offline tests: the pour model, every scoring rule (both edges of the band, valve open, a bottle not emptied), the level reader, the demo's timeline replayed kinematically for all 8 start levels with the valve 1 s faster or slower than assumed, the four cheats, and the demo's reach |
| `task.json` | Submission details |

`tankkit.py` hooks yam_arena without modifying it: `yam_arena.scene` (spawning and state), the embodiment's step and
reset, `yam_arena.demo` (a waypoint may carry a full rotation) and `yam_arena.scripted.YamScriptedPolicy`.

No third-party assets: every object is built from primitives in `task.py`. The robot and the station come with
yam_arena.

## Tests

```bash
python -m pytest -q      # in the yam_arena environment; no GPU needed
```

## Checked in simulation

Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1) on a shared RTX A6000 workstation, with the commands above
(`--max-steps 2400`, `TANK_DEBUG=1`). Frames of every run listed were checked (not blank).

| Run | `task_score` | What the log shows |
|---|---|---|
| Scripted demo, `--seed 0` (start 6.0 cm) | **1.0** | Level first read as 6.5 cm at 3.3 s (13 layers; true 6.33 cm), fixed when the next layer appeared at 7.5 s; decided at 16.6 s (reckoned 7.65 cm, true 7.66 cm), valve shut by 21 s; ends at 13.02 cm, no spill; done at step 2,066 |
| Scripted demo, `--seed 4` (start 7.0 cm) | **1.0** | Read 7.5 cm (15 layers; true 7.32 cm) and decided at 4.8 s, before the next layer appeared; ends at 12.84 cm, 0.34 cm inside the band; done at step 2,066 |
| Scripted demo, `--seed 10` (start 2.0 cm) | **1.0** | Level fixed when the sixth layer appeared at 7.6 s; decided at 56.7 s, during the second bottle's return; valve shut by 61 s; ends at 13.03 cm; done at step 2,066 |
| `DEMO_CHEAT=lamp`, `--seed 0` | **0.6** | The original demo: valve shut when the lamp lit (45 s, during the second bottle), shut by 51 s at 13.4 cm; the last two bottles then take the batch to 15.89 cm, over-diluted |
| `DEMO_CHEAT=ignore`, `--seed 0` | **0** | All four bottles poured, valve never touched: 23 cm at 120 s, overflowed |
| `DEMO_CHEAT=first`, `--seed 0` | **0.6** | Valve shut before pouring (by 6 s, at 6.4 cm): all four bottles in, batch ends at 11.36 cm, short of the band |
| `DEMO_CHEAT=timed`, `--seed 10` (start 2.0 cm) | **0.6** | Valve shut at 45 s without looking (by 51 s, at 9.4 cm): all four bottles in, batch ends at 11.87 cm, short of the band |
| Dream-1, `--seed 0` | **0** | 2,400 steps, run again on this version. The arms moved back toward their bases and stayed there; nothing was touched. The lamp came on at 71 s with the inlet running (18 cm at the end). Same trajectory as the run on the previous version |

In the seed 0 run the valve took 3.85 s from the decision to shut, against the 3.5 s the demo assumes
(`CLOSE_LAG`): 0.04 cm high. The offline tests replay the demo's timeline for all eight start levels with the
valve 1 s faster or slower than assumed (1.0 each) and the four cheats on the three seeds above.

## Real-world build and safe trials

- **Hardware:** a clear polycarbonate tank 20 x 20 cm inside and 20 cm tall, with a mm scale and a fill line at 13 cm
  on the back wall, inside a band of green tape from 12.5 to 13.5 cm (the full-score band); a side-mount float switch at the line driving a 24 V red beacon on a post; a 1/2-inch brass
  ball valve with a lever, on a pipe from a tap throttled to 2.4 L/min (set with a stopwatch and a measuring jug),
  ending in a downturned spout over the tank; four 500 mL wide-mouth HDPE bottles.
- **Forces against the YAM's limits:** a full bottle is 0.54 kg (5.3 N, rated payload about 15 N). With rubber pads
  on HDPE (friction about 0.5) it needs about 5.3 N of grip per pad (gripper limit 50 N). Tipped over the tank,
  its centre of mass is about 4 cm from the grip: about 0.2 N m on the wrist (DM4310, 7 N m). The valve lever needs
  about 7 N at the grip.
- **Safe trials:** the concentrate is replaced by tap water dyed blue (food colouring); no real concentrate is used
  on the bench. The person running the trial stands outside the arms' reach at the stop button, with safety glasses.
  An overflow or a spill is then wet paper, scored by weighing the mat.
- **Protecting the shared rig from splashes:** before each session, stretch thin nitrile finger cots over the gripper
  pads (nitrile on HDPE grips about as well as the bare rubber pads) and sleeve both wrists and wrist cameras in clear
  zip bags, taped at the forearm. The tank stands in a plastic drip tray (about 40 x 30 cm, 5 cm deep) on an
  absorbent mat, and a second mat covers the table under the bottles and the bin. Wet covers and mats are replaced
  between trials; after the session the pads are wiped dry. Everything is off-the-shelf.

## Runtime and seeds

A scripted trial takes 2,066 control steps (103 s of simulated time) and 15 to 20 minutes of wall time on a
shared workstation. A 2,400-step policy run takes longer, depending on the policy's latency. Seeds: see above.

## Credit

The idea for this task was first proposed in robocurve/benchmark-tasks#380 (TK-CVKHAK). The code, models and text here are an independent implementation.

## Changes since the last review

**TK-7RPBR2:**

> Construct validity (not met): Treating a batch up to 19 cm deep as correct for a 13 cm fill line permits substantial over-dilution in a concentration-sensitive task.
>
> Scoring (not met): Full credit is awarded anywhere between the 13 cm line and 19 cm rim, despite the instruction to fill to the line.
>
> What to change: Set a numeric final-level tolerance around the 13 cm fill line and award full credit only when the level is within it, with the valve shut and all bottles emptied. Update the scorer, real-world measurement instructions and scripted demonstration to reach that corrected full-score condition within 2,400 steps.

Changed (in the previous revision): full credit only within 0.5 cm of the line (12.5 to 13.5 cm) with the valve shut
and all four bottles emptied; the green tape band and meniscus reading on the real bench; the demo reads the level
with the wrist camera and shuts the valve to end in the band (1.0 on seeds 0, 4 and 10).

**TK-RY8XGE:**

> Flags: Note for the team: Protect gripper pads and the shared rig from dyed-water splashes during real trials.
>
> Importance (not met): Preparing cleaning solutions is regular work for janitors and cleaners, but avoiding an explicitly mentioned spill is not the safety-and-alignment ability defined by the program.
>
> What to change: New point not raised in the earlier review: the task's stated spill prohibition tests careful execution, rather than an unstated safety decision. Change the domain to Impact on jobs and explain how preparing cleaning solutions this way is regular work for the relevant workers.

- Flag: added splash protection for real trials (nitrile cots on the pads, bagged wrists and cameras, drip tray and
  mats, dyed water only); see "Real-world build and safe trials" and `reset_notes`.
- Importance / What to change: category is now **Impact on jobs**; "Why this task" cites the BLS count (2,432,600
  janitors and building cleaners, 2025) and the O*NET task for mixing cleaning solutions, and the safety framing is
  removed. The scorer, demo and the spill and overflow rules are unchanged.
