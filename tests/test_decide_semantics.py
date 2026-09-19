"""What decide.py is supposed to MEAN.

The golden corpus in ``tests/golden/`` pins the decision layer against
regression: it records what the code does and shouts when that changes.  These
tests are the other half, and the more important one - they say what the code
should do, with values worked out by hand, so that a golden mismatch can be
adjudicated instead of just re-blessed.

Every sign convention in this file was got wrong at least once while writing
it, which is exactly why each one is a named test.
"""

import pytest

from app import decide
from tests.conftest import framing_goal, make_keypoints, make_obs, pose_goal


def framing_state(now=0.0, **overrides):
    st = decide.initial_state(now, framing_goal(), decide.PHASE_FRAME)
    st.update(overrides)
    return st


# ---------------------------------------------------------------------------
# D-2  the control law
# ---------------------------------------------------------------------------

def test_deadband_is_a_deadband():
    assert decide.deadband(0.01, 0.02) == 0.0
    assert decide.deadband(-0.01, 0.02) == 0.0
    assert decide.deadband(0.03, 0.02) == 0.03
    assert decide.deadband(-0.03, 0.02) == -0.03


def test_zero_command_stays_zero():
    """The single most embarrassing possible bug: a satisfied control loop
    that still creeps, because minimum-duty compensation lifted a zero."""
    assert decide.apply_min_duty(0.0, 320, 650) == 0


@pytest.mark.parametrize("u", [0.001, 0.01, 0.1, 0.5, 1.0, -0.001, -0.5, -1.0])
def test_any_non_zero_command_clears_the_bridge_deadband(u):
    """D-2: below roughly 25-40% duty an L298N bridge turns nothing, so a small
    proportional output would be silently discarded and the loop would appear
    dead near its target."""
    duty = decide.apply_min_duty(u, 320, 650)
    assert abs(duty) >= 320
    assert abs(duty) <= 650
    assert (duty > 0) == (u > 0)


def test_min_duty_spans_the_live_band_monotonically():
    prev = 0
    for i in range(1, 101):
        duty = decide.apply_min_duty(i / 100.0, 320, 650)
        assert duty >= prev
        prev = duty
    assert decide.apply_min_duty(1.0, 320, 650) == 650


def test_rate_limit_caps_acceleration_but_not_stopping():
    """Ramping down through the dead band would leave the motors energised and
    stalled, and stopping is always safe."""
    assert decide.rate_limit(1.0, 0.0, 0.16) == pytest.approx(0.16)
    assert decide.rate_limit(-1.0, 0.0, 0.16) == pytest.approx(-0.16)
    assert decide.rate_limit(0.0, 1.0, 0.16) == 0.0        # immediate stop
    assert decide.rate_limit(0.5, 0.45, 0.16) == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# D-1 / D-3  yaw
# ---------------------------------------------------------------------------

def test_yaw_turns_right_when_the_framing_sits_right_of_target(cfg):
    """Subject at 0.60 and object at 0.80 put the midpoint at 0.70; the target
    midpoint is (0.333 + 0.667) / 2 = 0.50.  The content must move LEFT in the
    frame, which means rotating the rover RIGHT, which means the left side
    drives forward harder than the right."""
    obs = make_obs(subject=(0.60, 0.55, 0.18, 0.62), obj=(0.80, 0.40, 0.14, 0.32))
    u, err, rate_target = decide.yaw_command(obs, framing_state(), cfg, 1.0)
    assert err == pytest.approx(0.20)
    assert rate_target > 0
    assert u > 0, "positive yaw command must mean 'turn right'"


def test_yaw_turns_left_when_the_framing_sits_left_of_target(cfg):
    obs = make_obs(subject=(0.20, 0.55, 0.18, 0.62), obj=(0.40, 0.40, 0.14, 0.32))
    u, err, _rate = decide.yaw_command(obs, framing_state(), cfg, 1.0)
    assert err == pytest.approx(-0.20)
    assert u < 0


def test_yaw_is_silent_inside_its_deadband(cfg):
    """Subject and object already on their thirds: nothing to do."""
    obs = make_obs(subject=(0.333, 0.55, 0.18, 0.62), obj=(0.667, 0.40, 0.14, 0.32))
    u, err, rate_target = decide.yaw_command(obs, framing_state(), cfg, 1.0)
    assert err == pytest.approx(0.0, abs=1e-9)
    assert rate_target == 0.0
    assert u == 0.0


def test_yaw_inner_loop_brakes_an_overshooting_turn(cfg):
    """D-3: vision at 10-15 Hz is too slow to close a turn without overshoot.
    With the framing already correct but the rover still rotating, the inner
    loop must command the opposite way to stop it - that is the whole reason
    the gyro is in the loop."""
    obs = make_obs(subject=(0.333, 0.55, 0.18, 0.62), obj=(0.667, 0.40, 0.14, 0.32),
                   rate_z=25.0)
    u, _err, rate_target = decide.yaw_command(obs, framing_state(), cfg, 1.0)
    assert rate_target == 0.0
    assert u < 0, "the rover is still rotating; the loop must brake it"


def test_yaw_falls_back_to_the_subject_alone_when_no_object(cfg):
    obs = make_obs(subject=(0.60, 0.55, 0.18, 0.62), obj=None)
    u, err, _rate = decide.yaw_command(obs, framing_state(), cfg, 1.0)
    assert err == pytest.approx(0.60 - 0.333)
    assert u > 0


# ---------------------------------------------------------------------------
# D-4 / D-5  distance
# ---------------------------------------------------------------------------

def test_too_close_drives_backwards(cfg):
    """shoulder_px larger than target means the subject fills more of the
    frame than wanted, which means the rover is too close."""
    st = framing_state()
    st["sweep"] = {"phase": "done", "target_shoulder": 0.100}
    obs = make_obs(shoulder_px=0.140)
    u, err, target = decide.distance_command(obs, st, cfg, 1.0)
    assert target == pytest.approx(0.100)
    assert err == pytest.approx(0.40)
    assert u < 0, "too close must drive backwards"


def test_too_far_drives_forwards(cfg):
    st = framing_state()
    st["sweep"] = {"phase": "done", "target_shoulder": 0.140}
    obs = make_obs(shoulder_px=0.100)
    u, err, _target = decide.distance_command(obs, st, cfg, 1.0)
    assert err < 0
    assert u > 0


def test_distance_never_uses_box_height(cfg):
    """P-4: box height changes when the subject raises their arms, which the
    control loop would read as 40 cm of travel.  Raising the arms must move
    the distance command by exactly nothing."""
    st = framing_state()
    st["sweep"] = {"phase": "done", "target_shoulder": 0.120}

    arms_down = make_obs(subject=(0.333, 0.55, 0.18, 0.62), shoulder_px=0.120)
    arms_up = make_obs(subject=(0.333, 0.48, 0.34, 0.78), shoulder_px=0.120,
                       keypoints=make_keypoints(
                           left_wrist=(0.62, 0.20, 0.85), right_wrist=(0.38, 0.20, 0.85)))

    u_down, _e1, _t1 = decide.distance_command(arms_down, st, cfg, 1.0)
    u_up, _e2, _t2 = decide.distance_command(arms_up, st, cfg, 1.0)
    assert u_down == u_up == 0.0


def test_sweep_solves_for_the_target_ratio():
    """D-5: fit a monotonic curve through the (shoulder_px, ratio) samples and
    solve for the target ratio.  With ratio = 20 * shoulder exactly, a target
    of 2.4 must come back as shoulder 0.12."""
    samples = [(s / 1000.0, 20.0 * s / 1000.0) for s in range(80, 161, 10)]
    shoulder, reason = decide.solve_sweep(samples, 2.4)
    assert reason is None
    assert shoulder == pytest.approx(0.12, rel=1e-6)


def test_sweep_survives_noisy_samples():
    """Fitting a curve through many noisy samples beats chasing a noisy
    instantaneous error (D-5)."""
    noise = [+0.03, -0.04, +0.02, -0.01, +0.05, -0.03, +0.01, -0.02, +0.04]
    samples = [(0.08 + i * 0.01, 20.0 * (0.08 + i * 0.01) + noise[i])
               for i in range(9)]
    shoulder, reason = decide.solve_sweep(samples, 2.4)
    assert reason is None
    assert shoulder == pytest.approx(0.12, abs=0.01)


def test_sweep_reports_an_unreachable_ratio():
    """When the object scales with the subject - it is close behind them - no
    distance produces the requested ratio, and the loop must say so rather
    than drive back and forth forever."""
    samples = [(0.08 + i * 0.01, 2.40) for i in range(9)]
    shoulder, reason = decide.solve_sweep(samples, 3.5)
    assert shoulder is None
    assert "scales" in reason


# ---------------------------------------------------------------------------
# D-9  tilt
# ---------------------------------------------------------------------------

def test_tilt_goes_up_when_the_eyeline_is_too_high_in_frame(cfg):
    """Eyes at y = 0.29 against a target of 0.333: the subject sits too HIGH in
    the frame, so the content must move DOWN, so the camera tilts UP."""
    obs = make_obs()
    angle, err = decide.tilt_command(obs, framing_state(), cfg)
    assert err == pytest.approx(0.290 - 0.333)
    assert angle > 0


def test_tilt_goes_down_when_the_eyeline_is_too_low_in_frame(cfg):
    kps = make_keypoints(left_eye=(0.515, 0.50, 0.93), right_eye=(0.485, 0.50, 0.93))
    obs = make_obs(keypoints=kps)
    angle, err = decide.tilt_command(obs, framing_state(), cfg)
    assert err > 0
    assert angle < 0


def test_tilt_correction_is_incremental_so_it_absorbs_backlash(cfg):
    """D-9: the firmware's step count is an open-loop guess and its angle frame
    is offset from gravity by an unknown constant.  The loop therefore adds a
    delta to the LAST COMMANDED angle - which is what absorbs backlash, missed
    steps and slip - rather than commanding an absolute gravity-referenced
    number the firmware could not honour."""
    st = framing_state()
    st["tilt_cmd_deg"] = 12.0
    obs = make_obs()
    angle, _err = decide.tilt_command(obs, st, cfg)
    assert angle > 12.0, "the correction must build on the previous command"
    assert angle < 12.0 + cfg["control"]["tilt"]["max_step_deg"] + 1e-9


def test_tilt_respects_the_mechanical_limits(cfg):
    st = framing_state()
    st["tilt_cmd_deg"] = cfg["hardware"]["tilt_max_deg"] - 0.5
    obs = make_obs()
    angle, _err = decide.tilt_command(obs, st, cfg)
    assert angle <= cfg["hardware"]["tilt_max_deg"]


def test_tilt_holds_when_the_eyeline_cannot_be_measured(cfg):
    """P-7: perception emits None rather than a guess, and the loop must hold
    rather than invent a correction."""
    obs = make_obs(keypoints={})
    obs["subject"] = None
    st = framing_state()
    st["tilt_cmd_deg"] = 7.5
    angle, err = decide.tilt_command(obs, st, cfg)
    assert angle == 7.5
    assert err is None


# ---------------------------------------------------------------------------
# D-6  the self-motion gate
# ---------------------------------------------------------------------------

def test_static_background_plus_size_change_means_the_subject_moved(cfg):
    """The central ambiguity.  Background static and subject size changed means
    the SUBJECT moved - do not react; hold and re-plan."""
    st = framing_state()
    st["sweep"] = {"phase": "done", "target_shoulder": 0.100}
    st["last_shoulder"] = 0.100

    obs = make_obs(shoulder_px=0.140, bg_flow=(0.0, 0.0))
    cmd, st2 = decide.step(obs, st, cfg)
    assert cmd["left"] == 0 and cmd["right"] == 0
    assert st2["replan"] is True
    assert st2["sweep"] is None, "the sweep solution is invalidated by the move"


def test_moving_background_plus_size_change_means_the_rover_moved(cfg):
    """Background shifted and subject size changed means the ROVER moved, so
    the measurement is real and the loop should act on it."""
    st = framing_state()
    st["sweep"] = {"phase": "done", "target_shoulder": 0.100}
    st["last_shoulder"] = 0.100

    obs = make_obs(shoulder_px=0.140, bg_flow=(0.05, 0.01))
    cmd, st2 = decide.step(obs, st, cfg)
    assert st2.get("replan") is not True
    assert cmd["left"] != 0 or cmd["right"] != 0


def test_missing_flow_measurement_does_not_gate(cfg):
    """No flow measurement is not evidence the background was still."""
    st = framing_state()
    st["sweep"] = {"phase": "done", "target_shoulder": 0.100}
    st["last_shoulder"] = 0.100
    obs = make_obs(shoulder_px=0.140, bg_flow=None)
    _cmd, st2 = decide.step(obs, st, cfg)
    assert st2.get("replan") is not True


# ---------------------------------------------------------------------------
# G-1 / CO-3  the enable gate
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("phase", ["idle", "listen", "parse", "confirm"])
def test_motors_are_inhibited_before_the_goal_is_locked(cfg, phase):
    """CO-3 and G-1: no command with enable set is issued until the GoalSpec is
    locked, and these four phases are where it is not."""
    st = decide.initial_state(0.0, framing_goal(), phase)
    cmd, _st = decide.step(make_obs(subject=(0.6, 0.55, 0.18, 0.62)), st, cfg)
    assert cmd["enable"] is False
    assert cmd["left"] == 0 and cmd["right"] == 0


def test_an_unlocked_goal_inhibits_the_motors_in_any_phase(cfg):
    unlocked = framing_goal(locked=False)
    st = decide.initial_state(0.0, unlocked, decide.PHASE_FRAME)
    cmd, _st = decide.step(make_obs(subject=(0.6, 0.55, 0.18, 0.62)), st, cfg)
    assert cmd["enable"] is False


def test_no_goal_at_all_inhibits_the_motors(cfg):
    st = decide.initial_state(0.0, None, decide.PHASE_FRAME)
    cmd, _st = decide.step(make_obs(), st, cfg)
    assert cmd["enable"] is False


# ---------------------------------------------------------------------------
# CO-4  anchor loss
# ---------------------------------------------------------------------------

def test_losing_the_subject_stops_the_motors(cfg):
    st = framing_state()
    cmd, _st = decide.step(make_obs(subject=None, lost_for=0.6), st, cfg)
    assert cmd["left"] == 0 and cmd["right"] == 0
    assert cmd["enable"] is False


def test_search_spins_in_place_and_never_drives(cfg):
    """CO-4: the rover must never drive while it cannot see the subject.  A
    spin has equal and opposite duty; anything else is translation."""
    st = decide.initial_state(0.0, framing_goal(), decide.PHASE_SEARCH)
    st["last_bearing"] = 0.3
    cmd, _st = decide.step(make_obs(subject=None, lost_for=3.0), st, cfg)
    assert cmd["enable"] is True
    assert cmd["left"] > 0 > cmd["right"], "search must be a spin, not a drive"
    assert abs(abs(cmd["left"]) - abs(cmd["right"])) <= \
        abs(cfg["hardware"]["min_duty_left"] - cfg["hardware"]["min_duty_right"]), \
        "the two sides should differ only by their own dead bands"


def test_search_reverses_after_sweeping_its_limit(cfg):
    """CO-4: capped at +-60 degrees."""
    st = decide.initial_state(0.0, framing_goal(), decide.PHASE_SEARCH)
    st["last_bearing"] = 0.3
    st["search_swept_deg"] = 65.0
    cmd, _st = decide.step(make_obs(subject=None, lost_for=3.0), st, cfg)
    assert cmd["left"] < 0, "past the sweep limit the search must turn back"


# ---------------------------------------------------------------------------
# PG-5  sequencing
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("phase", ["coach", "settle", "capture"])
def test_the_rover_holds_still_while_coaching_and_capturing(cfg, phase):
    """PG-5: rover positioning and subject coaching must never run at the same
    time - two coupled loops chasing each other oscillate."""
    st = decide.initial_state(0.0, pose_goal(), phase)
    obs = make_obs(subject=(0.60, 0.55, 0.18, 0.62), shoulder_px=0.150)
    cmd, _st = decide.step(obs, st, cfg)
    assert cmd["left"] == 0 and cmd["right"] == 0


def test_capture_fires_a_burst_then_stops(cfg):
    """CO-2.6: capture a burst of 5-10 frames in tolerance."""
    st = decide.initial_state(0.0, framing_goal(), decide.PHASE_CAPTURE)
    fired = 0
    for i in range(20):
        cmd, st = decide.step(make_obs(t=float(i)), st, cfg)
        fired += bool(cmd["capture"])
    assert fired == cfg["coach"]["burst_frames"]


# ---------------------------------------------------------------------------
# D-7  feasibility
# ---------------------------------------------------------------------------

def test_no_progress_on_spacing_asks_the_subject_to_move(cfg):
    """D-7: turning cannot change the spacing between subject and object; only
    driving can, within a limited range.  When it is not working, propose the
    fix rather than continuing to search."""
    st = framing_state()
    st["sweep"] = {"phase": "done", "target_shoulder": 0.120}

    say = None
    # Subject and object almost on top of each other, and never separating.
    for i in range(40):
        obs = make_obs(t=i * 0.2, subject=(0.48, 0.55, 0.18, 0.62),
                       obj=(0.52, 0.40, 0.14, 0.32))
        cmd, st = decide.step(obs, st, cfg)
        if cmd["say"]:
            say = cmd["say"]
            break

    assert say is not None, "the loop never gave up on an unreachable composition"
    assert "step" in say.lower()
    assert "your" in say.lower(), "the suggestion must be in the subject's frame"


def test_progress_on_spacing_stays_quiet(cfg):
    """If driving IS closing the gap, say nothing and keep driving."""
    st = framing_state()
    st["sweep"] = {"phase": "done", "target_shoulder": 0.120}
    for i in range(40):
        gap = 0.04 + i * 0.008
        obs = make_obs(t=i * 0.2, subject=(0.5 - gap / 2, 0.55, 0.18, 0.62),
                       obj=(0.5 + gap / 2, 0.40, 0.14, 0.32))
        cmd, st = decide.step(obs, st, cfg)
        assert not (cmd["say"] and "step" in cmd["say"].lower()), \
            f"gave up at frame {i} while the gap was still opening"


# ---------------------------------------------------------------------------
# D-8  speed limits
# ---------------------------------------------------------------------------

def test_close_range_halves_the_speed(cfg):
    """D-8: 0.1 m/s when range_mm is under 1000."""
    st = framing_state()
    st["sweep"] = {"phase": "done", "target_shoulder": 0.050}
    st["prev_left_u"] = 1.0
    st["prev_right_u"] = 1.0

    far = make_obs(shoulder_px=0.120, telemetry={"range_mm": 2500})
    near = make_obs(shoulder_px=0.120, telemetry={"range_mm": 600})

    u_far, _e, _t = decide.distance_command(far, st, cfg, decide._limit_scale(far, st, cfg))
    u_near, _e2, _t2 = decide.distance_command(near, st, cfg, decide._limit_scale(near, st, cfg))
    assert abs(u_near) == pytest.approx(abs(u_far) * cfg["control"]["slow_factor"])


# ---------------------------------------------------------------------------
# D-1  the scale axis
# ---------------------------------------------------------------------------

def test_crop_shrinks_the_frame_toward_the_target_height(cfg):
    """The fourth axis: subject height against the target fraction, fixed by
    cropping at capture rather than by another manoeuvre."""
    obs = make_obs(subject=(0.4, 0.5, 0.12, 0.30))
    rect = decide.capture_crop(obs, framing_state(), cfg)
    assert rect is not None
    x0, y0, x1, y1 = rect
    assert (x1 - x0) == pytest.approx(0.30 / 0.5)
    assert 0.0 <= x0 < x1 <= 1.0
    assert 0.0 <= y0 < y1 <= 1.0


def test_no_crop_when_the_subject_is_already_large_enough(cfg):
    obs = make_obs(subject=(0.4, 0.5, 0.2, 0.70))
    assert decide.capture_crop(obs, framing_state(), cfg) is None


# ---------------------------------------------------------------------------
# the Command contract
# ---------------------------------------------------------------------------

def test_command_shape_is_exactly_the_prd_contract(cfg):
    st = framing_state()
    cmd, _st = decide.step(make_obs(), st, cfg)
    assert set(cmd) == {"left", "right", "tilt_deg", "enable", "capture", "say"}
    assert isinstance(cmd["left"], int) and isinstance(cmd["right"], int)
    assert isinstance(cmd["tilt_deg"], float)
    assert isinstance(cmd["enable"], bool) and isinstance(cmd["capture"], bool)
    assert cmd["say"] is None or isinstance(cmd["say"], str)


@pytest.mark.parametrize("phase", ["idle", "arm", "frame", "coach", "settle",
                                   "capture", "halted", "search"])
def test_duty_is_always_inside_the_wire_range(cfg, phase):
    st = decide.initial_state(0.0, pose_goal(), phase)
    for i in range(30):
        obs = make_obs(t=i * 0.04, subject=(0.9, 0.9, 0.4, 0.9), shoulder_px=0.30,
                       obj=(0.05, 0.05, 0.02, 0.02))
        cmd, st = decide.step(obs, st, cfg)
        assert -1000 <= cmd["left"] <= 1000
        assert -1000 <= cmd["right"] <= 1000
        assert cfg["hardware"]["tilt_min_deg"] <= cmd["tilt_deg"] <= cfg["hardware"]["tilt_max_deg"]
