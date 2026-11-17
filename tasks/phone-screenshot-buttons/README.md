# Take a screenshot on a phone in a desk stand: side button and volume up together, briefly

A real-size iPhone 12 stands loose in a desk stand, screen towards the robot. The robot must take one screenshot the
way people do: press the side button (right-hand edge) and the volume-up button (left-hand edge) at the same time, from
both sides at once, and let go quickly. The phone must stay in its stand.

**Instruction:** `Take one screenshot on the phone: press its side button and volume-up button together and let go at once. Leave the phone in its stand.`

## Why this task

- **The skill is a simultaneous, opposed, brief press on a light, free-standing object.** The two buttons are on
  opposite edges of a 164 g phone that only rests in its stand. Each needs about 3 N to press. A push on one edge
  alone slides or tips the phone before the button gives (in the simulation the phone gives way in its stand before 3 N builds up; see `DEMO_CHEAT=poke` below), so
  the two forces have to arrive together and balance each other: one pinch across the 71.5 mm width, or both arms
  pressing at once. They must land within the phone's pairing window (a fraction of a second) and come off again
  quickly: the same pair held about two seconds brings up the power-off / Emergency SOS slider instead (Apple's
  documented behaviour, below).
- **Position control on small parts with a wrong neighbour.** The buttons are 2.8 mm wide and stand about 0.6 mm
  proud of the frame. Volume down sits 2.7 mm below volume up (Apple's dimensional drawing). The side button overlaps
  both volume buttons in height, so a pinch has to be placed within a band about 10 mm long: lower, and the left pad
  lands on volume down too (or instead), splitting its force between two buttons or pressing the wrong one; higher,
  and it misses the side button.
- **Everyday and instantly recognisable.** Almost every smartphone owner takes screenshots this way; a ten-second clip
  of a hand squeezing a phone's two edges and the screenshot preview popping up needs no explanation. It also matters
  for assistive robots operating a person's own phone.
- **Not "press until it clicks".** Nothing snaps into place: the press is undone a fraction of a second later, and
  success is the phone's own output, decided by the timing and balance of two presses, not by seating a part.

The real button logic (Apple, Face ID models: support article 102616 "Take a screenshot on your iPhone" and the iPhone User Guide page "Turn iPhone on or off"):
"Press the side button and the volume up button at the same time. Quickly release both buttons." (screenshot);
"Simultaneously press and hold the side button and either volume button until the sliders appear" (power off). A
single volume press changes the volume; a single side-button press locks the screen (held, it starts Siri).

### Distinct from

| Task | Its skill | Here |
|---|---|---|
| TK-F4NTMM Set up a machine and start it with a two-hand control | Two big buttons on a 2.5 kg panel, pressed within 0.5 s and **held** 2 s | Opposed press on a 164 g phone that is free in its stand; the press must be **brief** (holding is the failure); 2.8 mm buttons with a wrong neighbour 2.7 mm away |
| TK-ENDKGS Swap a hand blender's shaft for the whisk | Squeeze two release buttons on a held tool, then pull the part off | No part moves; the timing and duration of the press decide the outcome; the object is not held |
| TK-MLJF7T Pop a phone's SIM tray out with the eject pin | A pin into a 1 mm hole | Two opposite edge buttons at once, the phone's own response |
| TK-DJYB5S Mount a phone in a spring-clamp car holder | Hold sprung arms open while inserting a phone | The phone is not moved at all |
| RoboCasa button tasks (e.g. press the microwave or coffee-machine start button) | One untimed press on a fixed appliance | Two presses at once, from opposite sides of a loose light object, inside a timing window |

## Scoring

Every condition is the phone's own response or a whole-trial condition visible on video. Real-trial check in brackets.

- **0** if at any time the phone left its seat: its centre moved more than 1 cm up or down, more than 1.5 cm
  sideways or forward/back, or it turned more than 10 degrees. [Video: the phone's bottom edge lifted off the stand's
  seat or slid past the 1.5 cm tape marks on the seat, or the phone tipped or fell.]
- Otherwise **0.1** if the power-off / SOS slider appeared. [Screen recording or video of the screen.]
- Otherwise **1.0** if exactly one screenshot was taken and nothing else happened (no volume change, screen not
  locked, no Siri), and the robot is no longer touching the phone at the end. [Photos > Screenshots holds exactly one
  new image; the screen recording shows no volume indicator and the screen still on; video.]
- Otherwise **0.5** if at least one screenshot was taken, with any other effect, a second screenshot, or the robot
  still touching the phone at the end.
- Otherwise **0.3** if only single-button effects happened (volume up or down, lock or Siri).
- Otherwise **0** (nothing pressed).

The trial ends with full score once it has held 1 s; otherwise at the run limit, scored at the end (violations are
latched over the whole trial).

**The simulated phone logic** (`phonekit.Device`, run every physics step at 240 Hz; a button counts as pressed past
0.25 mm of its 0.35 mm travel): side button and volume up both pressed with the second press within 0.2 s of the
first, and either released within 1.0 s of the second press: a screenshot. A pair (side with either volume button)
held 2.0 s: the slider. A pair held between 1 and 2 s: nothing. A volume press not paired within 0.2 s: a volume
change. A side press not paired: lock (or Siri), on release. Apple does not publish these timings; they are estimates,
and the real trial is scored on what the phone actually does.

**Run limit:** 600 steps (30 s at 20 Hz). The scripted demo succeeds at step 122.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder. The layout is fixed: always pass
`-E object_jitter=0`.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take one screenshot on the phone: press its side button and volume-up button together and let go at once. Leave the phone in its stand." \
  --scorer task_score --max-steps 600 --seed 0 --store-frames --log-dir ~/yam-eval/phone-screenshot-dream
```

`PHONE_DEBUG=1` prints the three button depths, the robot's contact force on the phone and the event log ten times a
second, and the seat check after every control step.

## Scripted demo

The same command with `--policy yam_scripted`. `yam_scripted` runs `phone_demo.Controller` (`phonekit` hooks it in
when the task module defines `demo_controller`). It knows the start layout, as every `yam_scripted` demo does, and
reads only the joint positions. The left arm brings its open fingers in along the screen's normal (70 degrees from
straight down, since the phone leans back 20 degrees), one pad beside each long edge. The pads' contact faces are
about 26 mm long across the approach axis (the YAM fingertip's collision mesh), so the pinch is centred 43.5 mm above
the phone's centre: on the left edge the pad covers volume up and stops 5 mm above volume down; on the right it covers
the side button's upper 5 mm. It closes in 0.2 s to
2.5 mm under the span of the pressed buttons (about 5 N per pad), holds 0.3 s, opens in 0.2 s and backs out. Both
buttons go down together and bottom out at their stops; the two forces cancel and the phone moves under 0.5 mm. The right arm
stays still.

Wrong approaches (`DEMO_CHEAT=...`, same command): `hold` (the same pinch held 3 s), `low` (the pinch 23 mm lower,
centred on volume down), `lift` (pinch, then raise the hand 3 cm before letting go), `poke` (push volume up with closed
fingertips from one side only).

## Checked in simulation

Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1) on a shared RTX workstation, `--max-steps 300` (demos) or 400
(wrong approaches). Frames of every run listed were checked (not blank).

| Run | `task_score` | What the log shows |
|---|---|---|
| Scripted demo, seeds 0, 4 and 10 | **1.0** each | Side button and volume up both at their 0.35 mm stops in the same control step (pad force about 5 N each), volume down untouched; one screenshot at 5.11 s; phone moved 0.4 mm and turned 0.3 degrees; success at step 122 |
| `DEMO_CHEAT=hold` (same pinch held 3 s) | **0.1** | Both buttons held; no screenshot on release; slider at 2.0 s |
| `DEMO_CHEAT=low` (pinch centred on volume down) | **0.3** | The left pad bears on volume up and volume down at once and the force splits, so neither goes past its 3 N preload; the side button alone goes down: lock |
| `DEMO_CHEAT=lift` (pinch, then raise the hand 3 cm before letting go) | **0** | Screenshot taken, but the pinch carried the phone 24 mm up out of its seat (seat latch) |
| `DEMO_CHEAT=poke` (closed fingertips push volume up from one side) | **0** | The phone slid 5 mm and tilted 3.7 degrees in the stand; the button never went down (the phone gives way before 3 N builds up); nothing happened |
| Dream-1, seed 0 | **0** | 600 steps. Both hands came forward on either side of the phone (around step 240) and stayed near it, but never touched it; no button pressed, the phone did not move |

## Runtime and seeds

A scripted trial takes about 30 s of simulated-and-rendered time after a 1 to 2 minute Isaac Sim start on a shared
RTX workstation; a 600-step policy run takes longer, depending on the policy's latency. The layout is fixed
(`-E object_jitter=0`); seeds 0, 4 and 10 were run for the demo and give the same result.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: layout, `score()` / `success()`, `demo_controller` |
| `phonekit.py` | The phone (body and three sprung buttons), the stand, the phone's button logic, the seat check, the score, and the hooks into yam_arena |
| `phone_demo.py` | The scripted demo and the wrong approaches |
| `test_phone_screenshot_buttons.py` | Tests: the button logic on handcrafted press traces (pair, order, window, hold, slider, single presses, volume down, two screenshots), every score tier, the seat latch, the stand geometry, the pad band between the volume buttons, the demo's reach |
| `task.json` | Submission details |

`phonekit.py` hooks yam_arena without modifying it: `yam_arena.scene.task_object` and `object_states`, the embodiment's
`_ensure_sim` and `reset`, `yam_arena.scripted.YamScriptedPolicy.reset` / `act`, and the environment's
`scene.write_data_to_sim` (called once per physics substep).

## Tests

```bash
python -m pytest -q      # in the yam_arena environment; no GPU needed
```

## Assets and licences

- The phone and the stand are built from box primitives in `phonekit.py`; dimensions of the phone and the button
  positions are taken from Apple's published iPhone 12 dimensional drawing (Apple Accessory Design Guidelines,
  developer.apple.com, dated 2020-10-16); no Apple file is included.
- The robot comes with the yam_arena embodiment: the I2RT YAM arm URDF and meshes (MIT License, Copyright (c) I2RT
  Robotics), and the link2/link3 visual meshes split into black and white parts as in the dreamlabs 1.0.23 package
  (I2RT's meshes, same MIT License). The station room, table and cameras are the standard yam_arena station.

## Real-world build

| Item | What | Size, mass | Approx. price |
|---|---|---|---|
| Phone | Apple iPhone 12 (Face ID; A2172 / A2403), used or refurbished, any colour, no case, no SIM, screen protector optional | 146.7 x 71.5 x 7.4 mm, 164 g, aluminium frame, glass faces | $150 used |
| Stand | Aluminium desk phone stand with a fixed cradle (for example Lamicall S1 / CS-01 type): base about 90 x 80 mm, seat about 3 cm above the desk with a front lip, cradle leaning back 20 degrees, back rest narrower than the phone and below the buttons | about 150-250 g | $10-15 |
| Desk mat | Thin silicone or foam mat around (not under) the stand, so a phone that falls lands softly | | $5 |

Total about $165-170. Mark the seat with tape 1.5 cm either side of the phone's start position.

**Phone settings before trials (safety and repeatability):**
- Settings > Emergency SOS: "Call with Hold and Release" (or "Call with Hold") **off** and "Call with 5 Button
  Presses" **off**; no SIM card, airplane mode on. Holding the buttons then only shows the slider; no call can be made.
- Display & Brightness > Auto-Lock: Never. Volume at 50 %. Silence the ring/silent switch.
- Before each trial: unlock the phone to the home screen and start Screen Recording from Control Center (it records
  the screenshot preview, the volume indicator and the slider; a lock ends the recording, which itself shows the lock).
  Note the number of images in Photos > Screenshots.
- After each trial: stop the recording, count new screenshots, dismiss any slider (tap Cancel) and delete the trial's
  screenshot. Reset: stand the phone back in the seat between the tape marks.

**Forces (measure once per phone).** The button forces are not published. Measure with a kitchen scale (0.1 g):
lay the phone on the scale on its opposite edge (for the side button: volume edge down on a folded cloth), tare,
and press the button straight down slowly with a pencil's eraser until it clicks; the peak reading in grams x 0.0098
is the force in N. Typical phone side buttons need about 2-4 N (the simulation uses 3.0 N to start the travel and
3.5 N at the stop). The YAM's fingers give up to about 40 N, so the pinch is well within range; the demo pinches with
about 5 N per pad. A pinch on the phone's aluminium edges with rubber pads does not mark the phone; the pads never
touch the glass.

**Safety.** No hazards beyond the arms themselves: with Emergency SOS button calls off and no SIM, no call can be
placed. A phone that tips falls about 3 cm onto the desk mat. The operator stays outside the arms' reach with the
e-stop in hand.

## Simplifications

- **The buttons are rigid caps on sprung slides** (PhysX prismatic joints with an implicit drive aimed 2 mm past the
  outer stop, so each needs 3.0 N to start moving and about 3.5 N at its 0.35 mm stop). The real dome switch's click
  (force drop) is not modelled. The fingers really press the caps; nothing is triggered by proximity.
- **The phone's response is a rule** (`phonekit.Device`) with estimated timings (0.2 s pairing window, release
  within 1.0 s, slider at 2.0 s), from Apple's documented behaviour. The real trial is scored on the real phone's
  output, so the estimates do not enter real scoring.
- **The phone is one rigid box with a visual screen** (no rounded corners, no camera bump; the camera bump faces away
  from the robot and does not touch the stand's back rest, which is lower). Pad and frame friction 0.4. The stand is
  one loose rigid body of boxes (0.25 kg), friction 1.0 on the desk.
- No screenshot flash is rendered in the simulation's video; the score and `PHONE_DEBUG` log show the events.
