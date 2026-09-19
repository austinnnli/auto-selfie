"""decide.py - PURE.  Observation + State + Config -> Command.

Imports ``math`` and its two pure siblings, ``pose`` and ``coach``.  No imaging
library, no I/O, no globals, no hidden state.  This is what makes a later port
to a phone-only build a two-day job instead of a two-week one, and it is what
lets ``tests/golden/`` mean anything: 50 recorded observations replayed through
this file must produce the same 50 commands, forever.

Contracts, straight from the PRD:

    Observation = {
      "t": float,                  # laptop clock at frame receipt
      "t_frame": float,            # phone clock when captured
      "subject": {"cx","cy","w","h","conf"} | None,
      "keypoints": {name: (x, y, conf)} | None,   # normalised 0-1
      "shoulder_px": float | None, # the distance coordinate, normalised
      "object": {"cx","cy","w","h","conf"} | None,
      "bg_flow": (dx, dy) | None,  # self-motion cue
      "gyro": {"rate_z", "pitch", "roll"},
      "lost_for": float            # seconds since subject last seen
    }

    Command = {"left": int, "right": int, "tilt_deg": float,
               "enable": bool, "capture": bool, "say": str | None}

``shoulder_px`` keeps the PRD's name but is normalised like every other image
measurement (design rule 2): it is the shoulder-to-shoulder distance as a
fraction of frame width.

STATE.  ``decide()`` has the PRD's exact signature and returns only a Command.
Because the loop genuinely needs memory - ramp limiting, the sweep, hysteresis,
the feasibility window - the real entry point is ``step()``, which returns
``(command, new_state)`` and never mutates its arguments.  ``decide()`` is
``step()[0]``.  The golden corpus exercises ``step()``, so state evolution is
pinned too.
"""

import math

from app import coach as coach_lib
from app import pose as pose_lib

# Session phases (mirrored by session.py, which owns the transitions).
PHASE_IDLE = "idle"
PHASE_LISTEN = "listen"
PHASE_PARSE = "parse"
PHASE_CONFIRM = "confirm"
PHASE_ARM = "arm"
PHASE_FRAME = "frame"
PHASE_COACH = "coach"
PHASE_SETTLE = "settle"
PHASE_CAPTURE = "capture"
PHASE_HALTED = "halted"
PHASE_SEARCH = "search"

# CO-3: motors are inhibited in these phases.  The GoalSpec is locked before
# the first command that sets enable (G-1).
INHIBITED_PHASES = (PHASE_IDLE, PHASE_LISTEN, PHASE_PARSE, PHASE_CONFIRM)

# Phases where the rover holds position on purpose.  PG-5: rover positioning
# and subject coaching must never run at the same time - two coupled loops
# chasing each other oscillate.
HOLD_PHASES = (PHASE_COACH, PHASE_SETTLE, PHASE_CAPTURE, PHASE_HALTED)


# ---------------------------------------------------------------------------
# D-2  the control law, identical in shape on every axis
# ---------------------------------------------------------------------------

def clamp(value, lo, hi):
    return lo if value < lo else hi if value > hi else value


def deadband(error, band):
    """Below the deadband the axis is considered satisfied.  Without this the
    rover hunts forever around its target, because the minimum-duty
    compensation below turns any non-zero output into a real lurch."""
    return 0.0 if abs(error) < band else error


def proportional(error, kp, band, max_cmd):
    err = deadband(error, band)
    if err == 0.0:
        return 0.0
    return clamp(kp * err, -max_cmd, max_cmd)


def apply_min_duty(cmd, min_duty, max_duty):
    """Map a normalised command onto the bridge's live band.

    D-2: minimum-duty compensation is a requirement, not a refinement.  Below
    roughly 25-40% duty an L298N bridge turns nothing, so a small proportional
    output would otherwise be silently discarded and the loop would appear dead
    near its target.

    ``cmd`` is -1..1; the result is an integer duty.  Zero stays zero - a
    satisfied axis must not make the rover creep.  The firmware applies the
    same floor again (drive_apply_min_duty), which is idempotent on anything
    this function produces.
    """
    if cmd == 0.0:
        return 0
    sign = 1 if cmd > 0 else -1
    mag = min(1.0, abs(cmd))
    duty = min_duty + mag * (max_duty - min_duty)
    return int(round(sign * duty))


def rate_limit(cmd, prev, max_delta):
    """Limit how fast the commanded output may change.

    Two deliberate departures from the PRD's D-2 pseudocode, both to keep the
    logged command equal to what the bridge actually received:

    1. This runs on the NORMALISED command, before ``apply_min_duty``, not
       after.  Rate-limiting the duty instead would spend the first ticks of
       every move emitting values inside the bridge's dead band, which the
       firmware's own floor then lifts anyway - so the log would say 160 while
       the wheels saw 320, and every gain tuned against that log would be
       tuned against fiction.  Current surge is limited where it belongs, by
       the ramp in drive.c.

    2. Deceleration to zero is NOT limited.  The limiter exists to stop a
       sudden demand from slamming the bridge; stopping is always safe and
       always wanted immediately.  Ramping down through the dead band would
       also leave the motors energised and stalled on the way.
    """
    if max_delta <= 0 or cmd == 0.0:
        return cmd
    delta = cmd - prev
    if delta > max_delta:
        return prev + max_delta
    if delta < -max_delta:
        return prev - max_delta
    return cmd


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------------------
# state
# ---------------------------------------------------------------------------

def initial_state(now=0.0, goal=None, phase=PHASE_IDLE):
    """A fresh session state.  Plain dict so tests can build one by hand."""
    return {
        "phase": phase,
        "t_phase": now,
        "goal": goal,
        "prev_left": 0,
        "prev_right": 0,
        "prev_left_u": 0.0,
        "prev_right_u": 0.0,
        "tilt_cmd_deg": 0.0,
        "tilt_offset_deg": None,   # commanded angle minus measured pitch
        "last_shoulder": None,
        "sweep": None,
        "spacing_hist": [],
        "infeasible": False,
        "reframes": 0,
        "replan": False,
        "capture_frames": 0,
        "coach": coach_lib.initial_state(now),
        "last_search_dir": 1.0,
        "last_bearing": 0.0,
        "seq": 0,
    }


def _composition(state, config):
    """Rule-of-thirds defaults unless the spoken sentence overrode one (G-3)."""
    defaults = dict(config.get("composition_defaults", {}))
    goal = state.get("goal") or {}
    defaults.update(goal.get("composition") or {})
    return defaults


def _blank_command():
    return {"left": 0, "right": 0, "tilt_deg": 0.0,
            "enable": False, "capture": False, "say": None}


# ---------------------------------------------------------------------------
# measurements taken from one Observation
# ---------------------------------------------------------------------------

def eyeline_y(obs, config):
    """The y the tilt axis controls: the subject's eyeline.

    Eyes first, nose second, and only then a fraction of the box height -
    which is a guess, so it is marked as such by returning ``None`` confidence
    is unavailable.  P-7: never substitute a guess silently.
    """
    cfg = config.get("pose", {})
    min_conf = cfg.get("min_keypoint_conf", 0.4)
    kps = obs.get("keypoints") or {}

    eyes = pose_lib.midpoint(pose_lib.keypoint(kps, pose_lib.LEFT_EYE, min_conf),
                             pose_lib.keypoint(kps, pose_lib.RIGHT_EYE, min_conf))
    if eyes is not None:
        return eyes[1]
    nose = pose_lib.keypoint(kps, pose_lib.NOSE, min_conf)
    if nose is not None:
        return nose[1]
    subject = obs.get("subject")
    if subject is not None:
        # Eyes sit roughly 7% of body height below the top of the box.
        return subject["cy"] - subject["h"] * 0.43
    return None


def size_ratio(obs):
    """The composition ratio the distance axis is ultimately solving for:
    how large the subject reads against the background object.

    Built from ``shoulder_px`` rather than the subject's box height, because
    box height changes when the subject raises their arms - which the control
    loop would read as 40 cm of travel (P-4).
    """
    obj = obs.get("object")
    shoulder = obs.get("shoulder_px")
    if obj is None or not shoulder or obj.get("h", 0) <= 0:
        return None
    return shoulder / obj["h"]


def self_moved(obs, config):
    """D-6 self-motion gate.

    The central ambiguity: background shifted and subject size changed means
    the rover moved; background static and subject size changed means the
    subject moved.  ``bg_flow`` is the only cue that separates them.

    Returns True if the background moved, None if there is no flow measurement.
    """
    flow = obs.get("bg_flow")
    if flow is None:
        return None
    return math.hypot(flow[0], flow[1]) >= config.get("control", {}).get("flow_eps", 0.004)


# ---------------------------------------------------------------------------
# D-3  yaw, with the gyro closing the inner loop
# ---------------------------------------------------------------------------

def yaw_command(obs, state, config, limit_scale):
    """Differential duty for the yaw axis.

    D-3: vision at 10-15 Hz is too slow to close a turn without overshoot.  The
    outer loop (vision) sets a target yaw RATE; the inner loop (gyro, 50 Hz)
    chases that rate.  The rover therefore keeps turning smoothly between
    frames instead of stepping and ringing.

    Error is the midpoint of subject and object against the target midpoint
    (D-1), so a turn moves both toward their thirds at once.
    """
    subject = obs.get("subject")
    if subject is None:
        return 0.0, 0.0, None

    comp = _composition(state, config)
    obj = obs.get("object")
    bias = state.get("pose_bias", {}).get("subject_x_bias", 0.0)

    if obj is not None:
        measured = 0.5 * (subject["cx"] + obj["cx"])
        target = 0.5 * ((comp.get("subject_x", 0.333) + bias) + comp.get("object_x", 0.667))
    else:
        measured = subject["cx"]
        target = comp.get("subject_x", 0.333) + bias

    err = measured - target

    ycfg = config.get("control", {}).get("yaw", {})
    hfov = config.get("camera", {}).get("hfov_deg", 66.0)
    max_rate = ycfg.get("max_rate_dps", 40.0)

    # Outer loop: bearing error -> target yaw rate.
    bearing_err = deadband(err, ycfg.get("deadband", 0.02)) * hfov
    rate_target = clamp(ycfg.get("kp", 1.8) * bearing_err, -max_rate, max_rate)

    # Inner loop: chase that rate against the measured one.  rate_z_sign is a
    # calibration value (M6), not a guess - the phone's z axis orientation
    # depends on how the mount is built.
    gyro = obs.get("gyro") or {}
    rate_meas = ycfg.get("rate_z_sign", 1.0) * gyro.get("rate_z", 0.0)

    # When the outer loop is satisfied, STOP rather than trying to damp the
    # last few degrees per second out of the chassis.
    #
    # This is not an optimisation, it is a limit-cycle cure.  Any non-zero
    # command has to clear the bridge's dead band, so "brake gently" is not
    # something this hardware can do - the smallest available correction is a
    # real lurch, which overshoots, which asks for a correction the other way.
    # The rover ends up rocking left-right on the spot forever, with the
    # framing perfectly correct the whole time.  A geared skid-steer stops on
    # its own in well under a tick; friction is the better damper.
    #
    # The inner loop still does its real job (D-3): while the outer loop is
    # asking for a turn, rate_target is non-zero and the gyro closes it
    # without the overshoot vision alone would produce.
    if rate_target == 0.0 and abs(rate_meas) < ycfg.get("rate_deadband_dps", 8.0):
        return 0.0, err, rate_target

    u = ycfg.get("kp_rate", 0.02) * (rate_target - rate_meas)
    max_cmd = ycfg.get("max_cmd", 0.5) * limit_scale
    return clamp(u, -max_cmd, max_cmd), err, rate_target


# ---------------------------------------------------------------------------
# D-5  the sweep, and D-4 the distance axis
# ---------------------------------------------------------------------------

def _fit_line(samples):
    """Least-squares fit of ratio against shoulder_px.

    D-5: fitting a curve through many noisy samples beats chasing a noisy
    instantaneous error.  A straight line is the right shape here - both
    quantities scale with 1/distance, so their relationship is close to linear
    over the few metres the rover can travel - and it is monotonic by
    construction, which the solve step needs.

    Returns ``(slope, intercept)`` or None if the samples are degenerate.
    """
    n = len(samples)
    if n < 2:
        return None
    sx = sum(s for s, _ in samples)
    sy = sum(r for _, r in samples)
    sxx = sum(s * s for s, _ in samples)
    sxy = sum(s * r for s, r in samples)
    denom = n * sxx - sx * sx
    if abs(denom) < 1e-12:
        return None
    slope = (n * sxy - sx * sy) / denom
    intercept = (sy - slope * sx) / n
    return (slope, intercept)


def solve_sweep(samples, target_ratio, min_slope=1e-3):
    """Solve the fitted curve for the shoulder_px that gives the target ratio.

    Returns ``(shoulder_px, None)`` on success, or ``(None, reason)`` when the
    composition is out of reach - which happens when the object scales with the
    subject as the rover moves (an object close behind them), so no distance
    produces the requested ratio.
    """
    fit = _fit_line(samples)
    if fit is None:
        return (None, "not enough usable samples")
    slope, intercept = fit
    if abs(slope) < min_slope:
        return (None, "object scales with the subject; distance cannot set this ratio")

    shoulder = (target_ratio - intercept) / slope
    lo = min(s for s, _ in samples)
    hi = max(s for s, _ in samples)
    span = max(hi - lo, 1e-6)
    # Allow extrapolation, but not a fantasy.  Two spans either side is about
    # as far as a straight line through this data can be trusted; past that
    # the honest answer is that the sweep did not cover the range.
    if shoulder < lo - 2.0 * span or shoulder > hi + 2.0 * span:
        return (None, "target ratio is outside the drivable range")
    return (max(1e-4, shoulder), None)


def _sweep_step(obs, state, config, limit_scale):
    """Run the D-5 sweep.  Returns ``(common_duty, sweep_state, say)``.

    drive slowly through the usable range logging (shoulder_px, ratio),
    fit a monotonic curve, solve for the target ratio, drive back to that
    shoulder_px, re-check once.
    """
    scfg = config.get("sweep", {})
    sw = dict(state.get("sweep") or {})
    now = obs.get("t", 0.0)
    shoulder = obs.get("shoulder_px")
    ratio = size_ratio(obs)
    comp = _composition(state, config)
    target_ratio = comp.get("size_ratio", 2.4)

    if not sw:
        sw = {"phase": "collect", "dir": -1.0, "samples": [], "t0": now,
              "last_sample_t": -1e9, "target_shoulder": None, "rechecked": False,
              "start_shoulder": shoulder}

    phase = sw.get("phase")

    if phase == "collect":
        if shoulder is not None and ratio is not None:
            if now - sw["last_sample_t"] >= scfg.get("sample_interval_s", 0.25):
                sw["samples"] = sw["samples"] + [(shoulder, ratio)]
                sw["last_sample_t"] = now

        samples = sw["samples"]
        ratios = [r for _, r in samples]

        # Once two samples are in, the local slope says which way the ratio
        # moves with distance, so the sweep can head TOWARD the target instead
        # of guessing.  Without this the sweep always backs away, and a target
        # that needs approaching eats the whole time budget going the wrong
        # way and then has to extrapolate - which is exactly how a sweep
        # started from a long way out returns a confidently wrong answer.
        if len(samples) >= 2 and not sw.get("aimed"):
            fit = _fit_line(samples)
            if fit and abs(fit[0]) > 1e-6:
                needed = (target_ratio - ratios[-1]) / fit[0]
                sw["dir"] = 1.0 if needed > 0 else -1.0
                sw["aimed"] = True

        # Bracketing the target beats extrapolating past it, so stop as soon
        # as the samples straddle it.
        bracketed = (len(samples) >= 2
                     and min(ratios) <= target_ratio <= max(ratios))

        span_ok = False
        if len(samples) >= scfg.get("min_samples", 8):
            lo = min(s for s, _ in samples)
            hi = max(s for s, _ in samples)
            # Explicit None check: start_shoulder is a measurement, and a
            # measurement of zero is not the same as no measurement.
            start = sw.get("start_shoulder")
            start = lo if start is None else start
            span_ok = (hi - lo) >= scfg.get("shoulder_span", 0.35) * max(1e-6, start)

        timed_out = (now - sw["t0"]) > scfg.get("max_duration_s", 12.0)

        if (bracketed and len(samples) >= scfg.get("min_samples", 8)) \
                or span_ok or timed_out:
            sw["phase"] = "solve"
            return (0.0, sw, None)

        # Drive slowly.  The first leg backs away - the safe direction, since
        # the far end is unknown - until the two samples above say otherwise.
        duty = scfg.get("duty", 300) / 1000.0 * sw["dir"]
        return (duty * limit_scale, sw, None)

    if phase == "solve":
        solved, reason = solve_sweep(sw["samples"], target_ratio)
        if solved is None:
            sw["phase"] = "failed"
            sw["reason"] = reason
            return (0.0, sw, None)
        sw["target_shoulder"] = solved
        fit = _fit_line(sw["samples"])
        sw["slope"] = fit[0] if fit else None
        sw["corrections"] = 0
        sw["phase"] = "return"
        return (0.0, sw, None)

    if phase == "return":
        if shoulder is None:
            return (0.0, sw, None)
        target = sw["target_shoulder"]
        err = (shoulder - target) / max(1e-6, target)
        dcfg = config.get("control", {}).get("dist", {})
        if abs(err) < dcfg.get("deadband", 0.03):
            sw["phase"] = "recheck"
            sw["recheck_t"] = now
            return (0.0, sw, None)
        u = proportional(-err, dcfg.get("kp", 1.2), 0.0,
                         dcfg.get("max_cmd", 0.4) * limit_scale)
        return (u, sw, None)

    if phase == "recheck":
        # D-8: a full stop before any measurement used for a final decision.
        if now - sw.get("recheck_t", now) < config.get("control", {}).get("settle_still_s", 1.0):
            return (0.0, sw, None)
        if ratio is None:
            return (0.0, sw, None)
        if abs(ratio - target_ratio) <= scfg.get("recheck_tolerance", 0.04):
            sw["phase"] = "done"
            return (0.0, sw, None)

        # The re-check missed.  The usual reason is not a bad fit but a moved
        # baseline: the sweep measures (shoulder_px, ratio) along one radial
        # line, and the yaw axis has been shifting the rover sideways the
        # whole time, so the object's apparent size on the return path is not
        # quite what it was on the way out.
        #
        # Correcting beats re-sweeping.  The fitted slope is still the local
        # relationship between the two quantities, so one Newton step from the
        # measurement just taken lands very close, and it costs a second
        # rather than another twelve.
        corrections = sw.get("corrections", 0)
        slope = sw.get("slope")
        if slope and abs(slope) > 1e-6 and corrections < scfg.get("max_corrections", 3):
            sw["target_shoulder"] = max(1e-4, shoulder + (target_ratio - ratio) / slope)
            sw["corrections"] = corrections + 1
            sw["phase"] = "return"
            return (0.0, sw, None)

        # Out of corrections, or no usable slope: one full re-sweep, then
        # accept whatever we have rather than circling forever.
        if sw.get("rechecked"):
            sw["phase"] = "done"
            return (0.0, sw, None)
        sw = {"phase": "collect", "dir": -sw["dir"], "samples": [], "t0": now,
              "last_sample_t": -1e9, "target_shoulder": sw["target_shoulder"],
              "rechecked": True, "start_shoulder": shoulder}
        return (0.0, sw, None)

    return (0.0, sw, None)


def distance_command(obs, state, config, limit_scale):
    """Common duty for the distance axis.

    D-4: the distance coordinate is ``shoulder_px``, never metres.  It is
    re-measured absolutely each frame and cannot drift.  The target comes from
    the sweep solution; without one, from the composition's subject height.
    """
    shoulder = obs.get("shoulder_px")
    if shoulder is None:
        return 0.0, None, None

    sw = state.get("sweep") or {}
    target = sw.get("target_shoulder")
    if target is None:
        comp = _composition(state, config)
        # Fall back to the subject-height default: shoulder width is about 45%
        # of the visible body height for a standing adult.
        target = comp.get("subject_height", 0.5) * 0.45

    bias = state.get("pose_bias", {}).get("dist_bias", 0.0)
    target = max(1e-4, target * (1.0 + bias))

    err = (shoulder - target) / target
    dcfg = config.get("control", {}).get("dist", {})
    u = proportional(-err, dcfg.get("kp", 1.2), dcfg.get("deadband", 0.03),
                     dcfg.get("max_cmd", 0.4) * limit_scale)
    return u, err, target


# ---------------------------------------------------------------------------
# D-9  tilt, closed-loop in software
# ---------------------------------------------------------------------------

def tilt_command(obs, state, config):
    """Return the absolute tilt angle to command, and the vision error.

    D-9: the firmware's step count is an open-loop guess.  ``decide`` compares
    the phone's measured pitch against the target and issues a corrected angle,
    which absorbs backlash, missed steps and any slip in the gear train.

    The correction is INCREMENTAL - the firmware's angle frame is offset from
    gravity by an unknown constant, so the loop adds a delta to the last
    commanded angle rather than commanding an absolute gravity-referenced
    number the firmware could not honour.
    """
    prev = state.get("tilt_cmd_deg", 0.0)
    gyro = obs.get("gyro") or {}
    measured_y = eyeline_y(obs, config)
    if measured_y is None:
        return prev, None

    comp = _composition(state, config)
    target_y = comp.get("eyeline_y", 0.333)
    err = measured_y - target_y

    tcfg = config.get("control", {}).get("tilt", {})
    vfov = config.get("camera", {}).get("vfov_deg", 50.5)

    # Deadband is stated in degrees, so convert the normalised error first.
    err_deg = err * vfov
    if abs(err_deg) < tcfg.get("deadband", 0.5):
        return prev, err

    # Subject sits low in frame (measured_y > target_y): the camera must tilt
    # DOWN, which is a decreasing angle, to bring them up the frame.
    delta = -tcfg.get("kp", 0.8) * err_deg
    delta = clamp(delta, -tcfg.get("max_step_deg", 6.0), tcfg.get("max_step_deg", 6.0))

    hw = config.get("hardware", {})
    angle = clamp(prev + delta, hw.get("tilt_min_deg", -35.0), hw.get("tilt_max_deg", 35.0))

    # ANTI-WINDUP.  The loop runs at 25 Hz and may add up to max_step_deg on
    # every tick, but a 28BYJ-48 slews at only max_step_rate / steps_per_degree
    # - about 44 deg/s with the default numbers.  Ask for more than the
    # mechanism can deliver and the commanded angle runs away from the mount,
    # the image error stays large because the mount has not arrived yet, and
    # the loop keeps integrating.  The result is a camera that sails past the
    # subject and keeps going.
    #
    # The cure uses the feedback that already exists: the phone's gravity
    # vector says where the mount actually is (D-9), so the command is never
    # allowed to lead it by more than one slew-time's worth of angle.
    pitch = gyro.get("pitch")
    if pitch is not None:
        lead = tcfg.get("max_lead_deg", 15.0)
        angle = clamp(angle, pitch - lead, pitch + lead)
    return angle, err


# ---------------------------------------------------------------------------
# D-7  feasibility
# ---------------------------------------------------------------------------

def spacing_error(obs, state, config):
    """How far the subject-object spacing is from what the composition wants.

    Turning cannot change this: a yaw moves both together.  Only driving can,
    through parallax, and only within a limited range.
    """
    subject = obs.get("subject")
    obj = obs.get("object")
    if subject is None or obj is None:
        return None
    comp = _composition(state, config)
    target = abs(comp.get("subject_x", 0.333) - comp.get("object_x", 0.667))
    measured = abs(subject["cx"] - obj["cx"])
    return target - measured


def _metres_phrase(metres):
    if metres is None:
        return "a step"
    if metres < 0.4:
        return "half a metre"
    if metres < 1.5:
        return "a metre"
    if metres < 2.5:
        return "two metres"
    return "a few metres"


def feasibility(obs, state, config):
    """D-7.  When the requested composition is unreachable, propose the fix
    rather than continuing to search.

    The test is behavioural, not geometric: if the spacing error has not
    improved over ``window_s`` of driving, driving is not going to fix it.
    That catches the real cause - a distant object, whose bearing barely moves
    however far the rover travels - without needing to know the object's range.

    Returns ``(say_or_None, new_history, infeasible)``.
    """
    fcfg = config.get("feasibility", {})
    err = spacing_error(obs, state, config)
    now = obs.get("t", 0.0)
    hist = list(state.get("spacing_hist", []))

    if err is None:
        return (None, hist, False)

    hist.append((now, err))
    window = fcfg.get("window_s", 4.0)
    hist = [(t, e) for (t, e) in hist if now - t <= window]

    if abs(err) <= fcfg.get("spacing_tolerance", 0.05):
        return (None, hist, False)
    if len(hist) < 2 or (now - hist[0][0]) < window:
        return (None, hist, False)

    progress = abs(hist[0][1]) - abs(err)
    if progress >= fcfg.get("min_progress", 0.015):
        return (None, hist, False)

    # Driving is not closing it.  Work out which way the subject should step
    # and roughly how far, then say so - in their frame, not the image's.
    subject = obs.get("subject")
    obj = obs.get("object")
    hfov = config.get("camera", {}).get("hfov_deg", 66.0)
    distance_m = pose_lib.shoulder_distance_m(
        obs.get("shoulder_px"), hfov,
        config.get("pose", {}).get("subject_shoulder_m", 0.40))

    metres = None
    if distance_m is not None:
        metres = abs(err) * math.radians(hfov) * distance_m

    # err > 0 means the gap is too small, so the subject moves AWAY from the
    # object.  err < 0 means the gap is too wide, so they move toward it.
    away = 1.0 if subject["cx"] >= obj["cx"] else -1.0
    image_dir_sign = away if err > 0 else -away
    spoken = coach_lib.subject_direction("image_right" if image_dir_sign > 0 else "image_left")

    return (f"Could you step {_metres_phrase(metres)} to {spoken}?", hist, True)


# ---------------------------------------------------------------------------
# D-1 scale axis: crop at capture
# ---------------------------------------------------------------------------

def capture_crop(obs, state, config):
    """The fourth axis.  Subject height against the target fraction is fixed by
    cropping at capture rather than by driving, because the last 10% of framing
    is not worth another manoeuvre.

    Returns a normalised ``(x0, y0, x1, y1)`` rect, or None when no crop is
    needed or the subject is not measurable.
    """
    subject = obs.get("subject")
    if subject is None or subject.get("h", 0) <= 0:
        return None
    comp = _composition(state, config)
    want = comp.get("subject_height", 0.5)
    have = subject["h"]
    if have <= 0 or want <= 0:
        return None

    scale = have / want          # >1 means the subject is larger than wanted
    if scale >= 1.0:
        return None              # cannot crop outward; leave the frame alone
    if scale > 0.98:
        return None              # not worth it

    cx, cy = subject["cx"], subject["cy"]
    half_w, half_h = scale / 2.0, scale / 2.0
    x0 = clamp(cx - half_w, 0.0, 1.0 - scale)
    y0 = clamp(cy - half_h, 0.0, 1.0 - scale)
    return (x0, y0, x0 + scale, y0 + scale)


# ---------------------------------------------------------------------------
# CO-4  anchor loss
# ---------------------------------------------------------------------------

def search_command(obs, state, config, limit_scale):
    """A slow gyro-guided spin toward the last known bearing, capped at +-60.

    CO-4: the subject is the only positional reference, so losing them means
    losing all sense of place.  The rover must never DRIVE while it cannot see
    the subject - this turns in place and nothing else.
    """
    scfg = config.get("safety", {})
    duty = scfg.get("search_duty", 280) / 1000.0
    direction = 1.0 if state.get("last_bearing", 0.0) >= 0 else -1.0

    # Cap the swept arc by integrating gyro rate since the search began.
    swept = abs(state.get("search_swept_deg", 0.0))
    if swept >= scfg.get("search_sweep_deg", 60.0):
        direction = -direction
    return direction * duty * limit_scale


# ---------------------------------------------------------------------------
# the entry points
# ---------------------------------------------------------------------------

def _limit_scale(obs, state, config):
    """D-8 speed limits.  0.2 m/s equivalent while framing, half that when the
    range sensor sees anything under a metre."""
    ccfg = config.get("control", {})
    scale = 1.0
    range_mm = (obs.get("telemetry") or {}).get("range_mm")
    if range_mm is not None and range_mm < ccfg.get("slow_range_mm", 1000):
        scale *= ccfg.get("slow_factor", 0.5)
    return scale


def step(obs, state, config):
    """The real entry point: ``(command, new_state)``, nothing mutated.

    ``decide()`` below is this function's first return value, with the PRD's
    signature.
    """
    st = dict(state)
    st["coach"] = dict(state.get("coach") or coach_lib.initial_state(obs.get("t", 0.0)))
    cmd = _blank_command()
    now = obs.get("t", 0.0)
    phase = st.get("phase", PHASE_IDLE)
    hw = config.get("hardware", {})
    ccfg = config.get("control", {})

    # Tilt is tracked in every phase so the commanded angle never jumps when
    # the session resumes, but it is only UPDATED where the rover is working.
    st["tilt_cmd_deg"] = st.get("tilt_cmd_deg", 0.0)
    gyro = obs.get("gyro") or {}
    if "pitch" in gyro:
        st["tilt_offset_deg"] = st["tilt_cmd_deg"] - gyro["pitch"]

    # ---- phases where the motors are inhibited (CO-3, G-1) ----
    goal = st.get("goal")
    locked = bool(goal and goal.get("locked"))
    if phase in INHIBITED_PHASES or not locked:
        cmd["tilt_deg"] = st["tilt_cmd_deg"]
        cmd["enable"] = False
        st["prev_left"] = 0
        st["prev_right"] = 0
        st["prev_left_u"] = 0.0
        st["prev_right_u"] = 0.0
        return cmd, st

    # ---- anchor loss (CO-4) ----
    subject = obs.get("subject")
    if subject is None or phase in (PHASE_HALTED, PHASE_SEARCH):
        cmd["tilt_deg"] = st["tilt_cmd_deg"]
        if phase == PHASE_SEARCH:
            u = search_command(obs, st, config, _limit_scale(obs, st, config))
            # Each side has its own dead band; using one side's floor for both
            # would make the search arc instead of spinning on the spot.
            cmd["left"] = apply_min_duty(u, hw.get("min_duty_left", 320),
                                         hw.get("max_duty", 650))
            cmd["right"] = apply_min_duty(-u, hw.get("min_duty_right", 330),
                                          hw.get("max_duty", 650))
            cmd["enable"] = True
            rate = abs(gyro.get("rate_z", 0.0))
            st["search_swept_deg"] = st.get("search_swept_deg", 0.0) + rate / max(1.0, ccfg.get("loop_hz", 25.0))
        else:
            cmd["enable"] = False
        st["prev_left"] = cmd["left"]
        st["prev_right"] = cmd["right"]
        st["prev_left_u"] = 0.0
        st["prev_right_u"] = 0.0
        return cmd, st

    st["search_swept_deg"] = 0.0
    if obs.get("subject"):
        st["last_bearing"] = subject["cx"] - 0.5

    limit_scale = _limit_scale(obs, st, config)

    # ---- pose measurement and the rover-first bias (PG-4) ----
    perr = pose_lib.evaluate(goal, obs, config.get("pose", {}))
    if perr is not None:
        perr = dict(perr)
        perr["limb"] = pose_lib.choose_limb(
            obs.get("keypoints") or {}, obs.get("object"), goal.get("limb"),
            config.get("pose", {}).get("min_keypoint_conf", 0.4))
    st["pose_error"] = perr
    st["pose_bias"] = pose_lib.rover_bias(perr, config.get("pose", {})) \
        if phase == PHASE_FRAME else {"subject_x_bias": 0.0, "dist_bias": 0.0}

    # ---- speech ----
    say = None

    # ---- D-6 self-motion gate, before acting on any change in shoulder_px ----
    shoulder = obs.get("shoulder_px")
    gate_hold = False
    if shoulder is not None and st.get("last_shoulder") is not None:
        jumped = abs(shoulder - st["last_shoulder"]) > ccfg.get("shoulder_jump", 0.02)
        if jumped:
            moved = self_moved(obs, config)
            if moved is False:
                # Background static, subject size changed: the SUBJECT moved.
                # Do not react - hold and re-plan.
                gate_hold = True
                st["replan"] = True
                st["sweep"] = None
    if shoulder is not None:
        st["last_shoulder"] = shoulder

    # ---- the four axes ----
    u_yaw = 0.0
    u_dist = 0.0

    if phase == PHASE_FRAME and not gate_hold:
        u_yaw, _yaw_err, _rate_target = yaw_command(obs, st, config, limit_scale)

        scfg = config.get("sweep", {})
        sweep = st.get("sweep") or {}
        sweep_running = scfg.get("enabled", True) and obs.get("object") is not None \
            and sweep.get("phase") not in ("done", "failed")

        if sweep_running:
            u_dist, sw, _ = _sweep_step(obs, st, config, limit_scale)
            st["sweep"] = sw
            if sw.get("phase") == "failed":
                say = "I can't get that framing from here - the background is too far away."
                st["infeasible"] = True
        else:
            u_dist, _dist_err, _target = distance_command(obs, st, config, limit_scale)

        # D-7 feasibility runs on the same cadence as the framing loop.
        fsay, hist, infeasible = feasibility(obs, st, config)
        st["spacing_hist"] = hist
        if fsay and say is None:
            say = fsay
            st["infeasible"] = infeasible
    elif phase in HOLD_PHASES:
        # PG-5: position is locked; coaching never runs alongside driving.
        u_yaw = 0.0
        u_dist = 0.0

    # ---- tilt (D-9) ----
    if phase in (PHASE_FRAME, PHASE_ARM, PHASE_SETTLE):
        st["tilt_cmd_deg"], _tilt_err = tilt_command(obs, st, config)
    cmd["tilt_deg"] = st["tilt_cmd_deg"]

    # ---- mix, rate limit, apply min duty (D-2) ----
    left_u = clamp(u_dist + u_yaw, -1.0, 1.0)
    right_u = clamp(u_dist - u_yaw, -1.0, 1.0)

    # max_duty_per_tick is quoted per 100 Hz firmware tick; the control loop
    # runs slower, so one control tick is worth several firmware ticks.
    max_delta = (hw.get("max_duty_per_tick", 40) / 1000.0) * \
        max(1.0, 100.0 / max(1.0, ccfg.get("loop_hz", 25.0)))

    left_u = rate_limit(left_u, st.get("prev_left_u", 0.0), max_delta)
    right_u = rate_limit(right_u, st.get("prev_right_u", 0.0), max_delta)

    left = apply_min_duty(left_u, hw.get("min_duty_left", 320), hw.get("max_duty", 650))
    right = apply_min_duty(right_u, hw.get("min_duty_right", 330), hw.get("max_duty", 650))

    cmd["left"] = left
    cmd["right"] = right
    cmd["enable"] = True
    st["prev_left_u"] = left_u
    st["prev_right_u"] = right_u
    st["prev_left"] = left
    st["prev_right"] = right

    # ---- coaching (D3e), only in the coach phase (PG-5) ----
    coach_cfg = config.get("coach", {})
    obs_age = max(0.0, now - obs.get("t_frame_local", obs.get("t", now)))

    if phase == PHASE_COACH:
        phrase, coach_state = coach_lib.coach(perr, st["coach"], coach_cfg, now, obs_age)
        st["coach"] = coach_state
        if phrase:
            say = phrase
    elif say is not None:
        spoken, coach_state = coach_lib.announce(st["coach"], say, now, coach_cfg)
        st["coach"] = coach_state
        say = spoken

    # ---- capture ----
    if phase == PHASE_CAPTURE:
        burst = coach_cfg.get("burst_frames", 7)
        if st.get("capture_frames", 0) < burst:
            cmd["capture"] = True
            st["capture_frames"] = st.get("capture_frames", 0) + 1
    else:
        st["capture_frames"] = 0

    cmd["say"] = say
    return cmd, st


def decide(obs, state, config):
    """The PRD's signature: Observation + State + Config -> Command.

    Thin wrapper over ``step()``; use ``step()`` when you need the evolved
    state, which the session loop does.
    """
    return step(obs, state, config)[0]
