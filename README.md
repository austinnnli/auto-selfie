# Autonomous Framing Rover

A rear-facing iPhone streams frames and motion data to a laptop, the laptop
runs detection and control, and an ESP32-S3-CAM applies the resulting motor and
tilt commands — so the rover positions itself to frame the subject against a
chosen background object, then coaches the subject's pose before capturing.

Three targets in one repository, built to the PRD in `docs/PRD.md`.

| | Deliverable | Language | Target |
|---|---|---|---|
| D1 | `firmware/` | C (ESP-IDF) | ESP32-S3-CAM |
| D2 | `client/index.html` | HTML + JS, one file | Safari on iOS, served over HTTPS by D3 |
| D3 | `app/` | Python 3.11+ | laptop |

---

## Quick start

```bash
# 1. The risk to retire first (C-6): does HTTPS + wss actually work?
python tools/gen_certs.py
python -m app.server --selftest          # no phone, no rover, no model

# 2. Everything that can be tested at a desk
python tools/run_tests.py                # 654 tests, no install needed
make -C firmware/test                    # 38 C tests, host build

# 3. Perception (the only part that needs packages)
pip install -r requirements.txt
python tools/export_model.py             # 2-3x on a CPU-only laptop

# 4. Firmware
python tools/gen_config_header.py
cd firmware && idf.py set-target esp32s3
idf.py -DROVER_WIFI_SSID=my-net -DROVER_WIFI_PASS=secret build flash monitor
# note the IP the rover prints on join - config.json's net.firmware_host
# must point at it, or pass it as: python -m app.run --host <that ip>

# 5. Drive it
python -m app.teleop --dry-run           # prints what it would send
python -m app.teleop                     # wheels off the ground first
python -m app.run --dry-run              # full pipeline, motors forced to zero
python -m app.run                        # the real thing
```

The phone connects to the URL `app.server` prints. Safari warns about the
self-signed certificate once; accept it, or the camera will never start.

---

## The five rules that bind every module

1. **The decision layer is pure.** `decide(obs, state, config)` imports nothing
   but `math`. Enforced by `tests/test_purity.py`, which reads the source: the
   import allowlist, no I/O calls, no `global`, no mutable module state, and no
   mutation of the caller's arguments.
2. **All image coordinates are normalised 0–1.** Changing resolution or camera
   must not invalidate a single gain. `shoulder_px` keeps the PRD's name but is
   a fraction of frame width like everything else.
3. **The rover never tracks its own position.** The subject is the origin,
   every measurement is re-taken each frame, nothing accumulates error, and
   there are no wheel encoders.
4. **Safety lives in firmware.** Every stop path in `safety.c` works with the
   laptop switched off. The 100 Hz safety task runs on core 1, away from Wi-Fi.
5. **Everything tunable lives in `config.json`**, hot-reloaded by `app/config.py`
   and compiled into the firmware defaults by `tools/gen_config_header.py`.

Rule 1 is the one that pays for itself: it is what lets `tests/golden/` mean
anything, what makes `replay.py` possible, and what will make the eventual
JavaScript port a two-day job.

---

## Layout

```
firmware/                 D1
  main/
    main.c                task setup, core pinning (F-1)
    net.c                 Wi-Fi, UDP rx/tx, sequence handling (W-5)
    drive.c               L298N: duty -> PWM + direction (F-2)
    tilt.c                28BYJ-48 phase sequencing, backlash (F-3)
    safety.c              watchdog, range stop, enable gate (F-4)
    protocol.h            SHARED packet definitions
    app_config.h          pin map, with compile-time reserved-GPIO checks
    rover_hal.h           the line between logic and silicon
    hal_esp32.c           rover_hal.h against ESP-IDF
  test/                   host build: the firmware logic, tested at a desk
client/
  index.html              D2, one file (C-1..C-7)
app/                      D3
  server.py               HTTPS + WebSocket for the phone
  websocket.py            RFC 6455, standard library only
  streams.py              newest-frame-only, timestamp alignment (P-1, P-2)
  perceive.py             YOLO-pose, tracking, flow -> Observation (P-3..P-7)
  decide.py     PURE      Observation + State + Config -> Command (D3c)
  pose.py       PURE      pose goals and errors (D3d)
  coach.py      PURE      error -> phrase, rate limited (D3e)
  goal.py       PURE      tier-1 spoken-goal grammar (D3b)
  session.py    PURE      the CO-3 state machine
  goal_tier2.py           the cloud fallback parser (G-5.2)
  comms.py                UDP to firmware, telemetry in
  record.py / replay.py   logging and desk replay (LG-1, LG-2)
  teleop.py               keyboard driving and the safety drills (M8)
  run.py                  the control loop; owns no policy
  config.py               config.json, hot-reloaded (CF-1)
  protocol.py             SHARED, mirrors protocol.h
config.json               all tunables
tests/golden/             the LG-3 corpus, 61 cases
tools/                    certs, config header, test vectors, golden, runner
```

### Where this departs from the PRD

Each of these was a judgement call; the reasoning is in the file that made it.

- **`app/goal.py`, `app/goal_tier2.py`, `app/run.py`, `app/config.py` and
  `app/websocket.py` are new files.** D3b describes goal capture but gives it
  no home; CF-1 requires hot reload; something has to be `__main__`. The goal
  grammar is pure and heavily tested, so it earns its own module.
- **`rate_limit` runs before `apply_min_duty`, not after.** D-2's pseudocode has
  the other order, which spends the first ticks of every move emitting duties
  inside the bridge's dead band that the firmware's floor then lifts anyway —
  so the log would say 160 while the wheels saw 320, and every gain tuned
  against that log would be tuned against fiction. Current surge is limited
  where it belongs, in `drive.c`. Deceleration to zero is not rate limited at
  all: stopping is always safe and always wanted immediately.
- **`min_duty` is applied in both places, and that is deliberate.** The laptop
  maps its normalised command onto the live band so the control law is not
  silently discarded; the firmware enforces the same value as a floor so it
  still holds with a stale or hand-written sender. The firmware's version is
  idempotent on anything the laptop produces, and a C test pins that.
- **`trim` is applied only in firmware.** It is a mechanical property of the
  chassis, not a control decision, so the pure layer never sees it.
- **A range trip removes the forward component, not each side's duty.**
  Clamping per side would turn a commanded spin into a one-sided arc. F-4 asks
  for "reverse and turns still allowed", so `safety_limit_forward()` splits the
  command into common and differential parts and zeroes only the common one.
- **`decide()` has the PRD's exact signature; `step()` is the real entry
  point.** The loop genuinely needs memory — ramp limiting, the sweep,
  hysteresis, the feasibility window — so `step()` returns `(command, state)`
  and never mutates its arguments. `decide()` is `step()[0]`.
- **The corpus is 61 cases, not 50**, because covering every phase, both
  self-motion gate branches and each pose goal took that many.
- **`server.py` uses no third-party packages.** The phone client's whole
  premise is "no app, no install"; a laptop-side server that needs a working
  package index before the rover can move is the same problem one step back.

---

## The parts worth reading first

**`decide.py`** is the system. Four independent axes (D-1), one control law
(D-2), a gyro inner loop because vision at 12 Hz cannot close a turn (D-3), a
distance coordinate that cannot drift (D-4), a sweep that fits a curve instead
of chasing a noisy error (D-5), a self-motion gate that resolves "did I move or
did they?" (D-6), a feasibility test that gives up out loud instead of
searching forever (D-7), and a tilt loop closed against the phone's gravity
vector because the stepper has no home switch (D-9).

**`tilt.c`** is where the 28BYJ-48's three real problems get solved: balance is
mechanical and the firmware will not detect slip, backlash is handled by
approaching every target from the same direction, and there is no homing
routine because the phone is the reference.

**`coach.py`** decides whether the pose feature is liked or switched off. The
one rule that matters most is CO-2.4: the phone faces the subject, so their
left is image-right. Getting it backwards destroys trust on the first run, and
it has three tests.

---

## Testing

```bash
python tools/run_tests.py          # everything, no install
pytest -q                          # the same tests, if pytest is available
make -C firmware/test              # the C side
python tools/gen_golden.py --check # is the corpus current?
python -m app.server --selftest    # C-6, end to end
```

`tools/run_tests.py` exists because the control laptop is often a fresh machine
on a field network, and "I cannot run the tests until pip works" is a bad place
to be when the rover is behaving oddly. The tests themselves are ordinary
pytest and run unchanged under it.

### Closed-loop tests

`tests/simulator.py` is a kinematic rover: differential drive with a real
L298N dead band, a pinhole projection of the subject and object into a
normalised frame, shoulder width by similar triangles, a 28BYJ-48 that slews at
a finite rate with backlash, and the two self-motion cues. It exists because
the only interesting question about a control law — does it *converge* — cannot
be asked of a single tick.

`tests/test_integration.py` runs the milestone acceptance tests against it:
M9 (subject held on the target line), M10 (eyeline held through pitch changes),
M12 (same composition from several start distances), M13 (one trigger produces
a capture, unattended) and M14 (a pose run that frames, then coaches, and never
both at once). Three of them exist because they caught real bugs:

- the tilt command wound up far ahead of what the stepper could slew, so the
  camera sailed past the subject and kept going;
- the yaw inner loop tried to damp the last few degrees per second, which on a
  bridge with a dead band means rocking left-right on the spot forever with the
  framing perfectly correct;
- the sweep always backed away first, so a target that needed approaching ate
  the whole time budget going the wrong way and then extrapolated a
  confidently wrong answer.

There is also a test that asserts the rover **fails** when `rate_z_sign` is
inverted. If that one ever passes, the gyro inner loop has stopped being
connected to anything.

### Two kinds of test

Two test files do different jobs, and the distinction matters:

| | says what | fails when |
|---|---|---|
| `test_decide_semantics.py` | the behaviour is **correct** | a sign flips, a gain is misapplied, a rule is dropped |
| `test_golden.py` | the behaviour is **unchanged** | anything moves, including things no one thought to assert |

Regenerating the corpus always makes the golden tests pass again, so it is
worthless on its own. **Never re-bless a golden mismatch until the semantic
tests still pass and the failing case's `why` line has been read and agreed
with.** See `tests/golden/README.md`.

`protocol.h` and `protocol.py` are checked against the same generated vectors
from both languages. If `pytest tests/test_protocol.py` passes and
`make -C firmware/test` fails, the two files have drifted — which the PRD calls
the most likely source of silent misbehaviour, caught at a desk.

---

## Bring-up order

Each step verified before the next (F-6), with the milestone that covers it:

1. Blink — `hal_led_set` in `safety_task`
2. Wi-Fi join — `net_init`, watch the log for an IP
3. UDP echo — `python -m app.teleop --dry-run`
4. One motor on the bench — `teleop`, `w`/`s`, **wheels off the ground** (M1)
5. Both sides — `a`/`d`, check `trim` (M1)
6. Watchdog drill — `python -m app.teleop --drill watchdog` (M2)
7. Stepper sweep — `teleop`, `[` and `]` (M3)
8. Backlash measurement — `--drill backlash`, store `backlash_steps` (M3)
9. Range sensor — drop the VL53L1X driver into `hal_range_read_mm`,
   then `--drill range` (M2)
10. Full packet path — `python -m app.run --dry-run` (M4 onward)

**Working rules.** Wheels off the ground for the first test of any new logic.
Dry-run before energising. One variable per tuning run, logged. Never move
fast. Calibrate before tuning — most "bad gains" are a wrong `hfov_deg` or an
unmeasured `min_duty`.

---

## Calibration (CF-2)

| Value | How |
|---|---|
| `hfov_deg` | tape measure across the view at a known distance |
| `vfov_deg` | the same, vertically |
| `min_duty_left/right` | `teleop`, `-` to zero then `+` until each side just turns |
| `trim` | drive straight on a flat floor, measure the curve |
| `steps_per_degree` | command 1000 steps, measure the phone's pitch change |
| `backlash_steps` | `--drill backlash`; reverse and count steps until motion |
| `size_ratio` | read off a reference photo the user likes |
| `vfov_deg` | as `hfov_deg`, vertically |
| `rate_z_sign` | turn the rover right by hand; if `rate_z` goes negative, flip it |
| `subject_shoulder_m` | tape measure across the subject's shoulders |

`rate_z_sign` is not in the PRD's table but belongs there: the phone's z axis
depends on how the mount is built, and a sign error turns the yaw inner loop
from a damper into an oscillator.

Four other keys were added to `config.json` for the same reason — they are
tunable, so rule 5 says they cannot be literals:

| Key | What it does |
|---|---|
| `control.tilt.max_lead_deg` | how far the tilt command may lead the mount before the loop stops integrating; roughly one slew-time's worth of angle |
| `control.yaw.rate_deadband_dps` | below this residual turn rate, stop rather than brake — the cure for the dead-band limit cycle |
| `sweep.max_corrections` | how many Newton steps the re-check may take before giving up and re-sweeping |
| `pose.subject_shoulder_m` | the one metric calibration, used only for spoken distances ("step a metre to your left") |

---

## Safety drills

Repeat whenever the firmware changes.

| Drill | Pass | Command |
|---|---|---|
| Disable laptop Wi-Fi mid-drive | motors stop < 250 ms | `--drill watchdog` |
| Hand over the distance sensor at full forward | forward refused, reverse allowed | `--drill range` |
| Kill the Python process | motors stop | `Ctrl-C` during `teleop` |
| Subject steps out of frame | halt < 500 ms, then search, then speak | `python -m app.run` |
| Battery down to `vbat_min_mv` | motion refused, status bit set | watch `teleop`'s `t` |

---

## Known limits

- **Safari has never supported ImageCapture**, so the v1 capture is a canvas
  grab at video resolution — 1080p to 4K, without Apple's computational
  photography. Accepted for v1.
- **The VL53L1X driver is a stub.** `hal_range_read_mm` returns "no reading",
  which `safety.c` treats as *unknown* and never as *clear*: it holds the
  previous trip state rather than re-enabling forward motion. Drop the vendor
  driver in at bring-up step 9.
- **One subject, stationary.** Absolute navigation, moving-subject tracking,
  multiple subjects and rough terrain are all out of scope for v1.
- **Tier 2 needs `ANTHROPIC_API_KEY`.** Without it the parser is tier 1 only,
  which handles every phrase in G-3 offline; a run never blocks on the network
  either way.
- **`replay.py` reconstructs state by re-running from the log's start.** The
  log carries observations, commands and transitions, not the decision layer's
  internals, so a partial log replays from the point it begins.
