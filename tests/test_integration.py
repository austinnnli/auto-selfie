"""Closed-loop tests against the kinematic rover in tests/simulator.py.

These are the milestone acceptance tests that a unit test cannot express,
because what they check is whether the loops CONVERGE:

    M9   subject held on the target line
    M10  eyeline held on the upper third through pitch changes
    M12  same composition from several start distances
    M13  one trigger produces a framed photo, unattended
    M14  a pointing shot converges and captures

The simulator models the one hardware fact that shapes the whole control law:
an L298N bridge below min_duty turns nothing, so "move a little" is not
something this rover can do.
"""

import json
import math

import pytest

from app import decide, goal as goal_lib, session
from tests.simulator import RoverSim

DT = 1.0 / 25.0            # config.control.loop_hz


def make_goal(cfg, obs, kind="framing", pose=None, size_ratio=0.42, limb=None):
    """A locked GoalSpec, with the sides assigned from the object's real
    position the way run.py does it before confirmation (G-4, G-6)."""
    text = "make me point at the tree" if kind == "pose" else "frame me next to the tree"
    spec, _conf = goal_lib.parse(text, cfg)
    spec = goal_lib.assign_sides(spec, obs["object"]["cx"])
    spec["kind"] = kind
    spec["pose_goal"] = pose
    spec["limb"] = limb
    # CF-2: size_ratio is read off a reference photo the user likes.  The
    # default 2.4 suits a small object; a 2.2 m tree at 7 m cannot be 2.4x
    # smaller than a person's shoulders, so this scene states its own.
    spec["composition"]["size_ratio"] = size_ratio
    spec["locked"] = True
    return spec


def run_framing(cfg, sim, ticks=2000, phase=decide.PHASE_FRAME, **goal_kwargs):
    obs = sim.observation(0.0)
    spec = make_goal(cfg, obs, **goal_kwargs)
    state = decide.initial_state(0.0, spec, phase)

    lost = 0
    for i in range(ticks):
        obs = sim.observation(i * DT)
        if obs["subject"] is None:
            lost += 1
        cmd, state = decide.step(obs, state, cfg)
        sim.step(cmd, DT)
    return sim.observation(ticks * DT), state, spec, lost


# ---------------------------------------------------------------------------
# M9 / M12  framing
# ---------------------------------------------------------------------------

def test_the_framing_loop_converges_and_stops(cfg):
    """M9 and M12 together: from a standing start the rover sweeps, solves for
    the distance that gives the requested size ratio, drives back to it, and
    stops with every controlled axis inside its deadband."""
    sim = RoverSim(cfg)
    obs, state, spec, lost = run_framing(cfg, sim)
    comp = spec["composition"]

    assert lost == 0, f"the subject left the frame on {lost} ticks"
    assert (state["sweep"] or {}).get("phase") == "done", "the sweep never finished"

    midpoint = 0.5 * (obs["subject"]["cx"] + obs["object"]["cx"])
    target_midpoint = 0.5 * (comp["subject_x"] + comp["object_x"])
    assert abs(midpoint - target_midpoint) < cfg["control"]["yaw"]["deadband"] * 1.5

    eye_y = obs["keypoints"]["left_eye"][1]
    tilt_err_deg = abs(eye_y - comp["eyeline_y"]) * cfg["camera"]["vfov_deg"]
    assert tilt_err_deg < cfg["control"]["tilt"]["deadband"] * 3.0

    cmd, _st = decide.step(obs, state, cfg)
    assert cmd["left"] == 0 and cmd["right"] == 0, \
        "the rover is still moving with the framing correct - a limit cycle"


def test_the_loop_does_not_oscillate_once_settled(cfg):
    """The failure this test exists for: because every non-zero command must
    clear the bridge's dead band, a controller that tries to damp the last few
    degrees per second rocks left-right on the spot forever, with the framing
    perfectly correct the whole time."""
    sim = RoverSim(cfg)
    _obs, state, _spec, _lost = run_framing(cfg, sim)

    moved = 0
    for i in range(250):
        obs = sim.observation((2000 + i) * DT)
        cmd, state = decide.step(obs, state, cfg)
        sim.step(cmd, DT)
        if cmd["left"] or cmd["right"]:
            moved += 1
    assert moved == 0, f"still commanding motion on {moved}/250 settled ticks"


@pytest.mark.parametrize("start_distance", [1.8, 2.6, 3.4, 4.2])
def test_the_same_composition_from_several_start_distances(cfg, start_distance):
    """M12's acceptance test.  The distance coordinate is re-measured
    absolutely every frame (D-4), so where the rover started must not matter."""
    sim = RoverSim(cfg, x=-start_distance, y=0.0)
    obs, _state, spec, _lost = run_framing(cfg, sim, ticks=2500)

    ratio = obs["shoulder_px"] / obs["object"]["h"]
    assert abs(ratio - spec["composition"]["size_ratio"]) < 0.08, (
        f"from {start_distance} m the size ratio settled at {ratio:.3f}, "
        f"not {spec['composition']['size_ratio']}")


# ---------------------------------------------------------------------------
# M10  tilt
# ---------------------------------------------------------------------------

def test_the_eyeline_is_held_through_a_distance_change(cfg):
    """M10.  Driving toward the subject swings the elevation angle to their
    eyes a long way, and the tilt loop must track it - closed against the
    phone's gravity vector, not against the firmware's step count (D-9)."""
    sim = RoverSim(cfg, x=-4.5)
    obs = sim.observation(0.0)
    spec = make_goal(cfg, obs)
    state = decide.initial_state(0.0, spec, decide.PHASE_FRAME)

    worst = 0.0
    for i in range(2500):
        obs = sim.observation(i * DT)
        cmd, state = decide.step(obs, state, cfg)
        sim.step(cmd, DT)
        # Ignore the first second, while the mount slews from level.
        if i > 25 and obs["keypoints"]:
            err = abs(obs["keypoints"]["left_eye"][1] - spec["composition"]["eyeline_y"])
            worst = max(worst, err)

    assert worst < 0.09, f"the eyeline wandered {worst:.3f} of frame height"


def test_the_tilt_command_never_runs_away_from_the_mount(cfg):
    """The anti-windup in tilt_command().  The loop may add up to max_step_deg
    every tick at 25 Hz; a 28BYJ-48 slews at about 44 deg/s.  Without the
    guard the command sails past the subject and keeps going."""
    sim = RoverSim(cfg, x=-1.6)          # a large initial elevation error
    obs = sim.observation(0.0)
    spec = make_goal(cfg, obs)
    state = decide.initial_state(0.0, spec, decide.PHASE_FRAME)

    worst_lead = 0.0
    for i in range(600):
        obs = sim.observation(i * DT)
        cmd, state = decide.step(obs, state, cfg)
        sim.step(cmd, DT)
        worst_lead = max(worst_lead, abs(cmd["tilt_deg"] - sim.tilt_deg))

    assert worst_lead <= cfg["control"]["tilt"]["max_lead_deg"] + 1e-6, \
        f"the command led the mount by {worst_lead:.1f} degrees"


# ---------------------------------------------------------------------------
# D-3 / CF-2  the gyro sign
# ---------------------------------------------------------------------------

def test_an_inverted_gyro_sign_makes_the_loop_unstable(cfg):
    """Why rate_z_sign is a calibration value and not a constant.

    The phone's z axis depends on how the mount is built.  Get it backwards
    and the yaw inner loop stops damping the turn and starts driving it - so
    this test asserts the failure, to show the calibration has teeth.  If this
    ever passes, either the sign no longer matters (it does) or the inner loop
    has stopped being connected to anything.
    """
    broken = json.loads(json.dumps(cfg))
    broken["control"]["yaw"]["rate_z_sign"] = -1.0

    sim = RoverSim(broken, x=-3.0, y=-0.9)      # start well off the target line
    obs = sim.observation(0.0)
    spec = make_goal(broken, obs)
    state = decide.initial_state(0.0, spec, decide.PHASE_FRAME)

    lost = 0
    for i in range(500):
        obs = sim.observation(i * DT)
        if obs["subject"] is None:
            lost += 1
        cmd, state = decide.step(obs, state, broken)
        sim.step(cmd, DT)

    assert lost > 0, ("with the gyro sign inverted the rover still framed the "
                      "subject, which means the inner loop is not doing anything")


# ---------------------------------------------------------------------------
# M13  one trigger produces a framed photo, unattended
# ---------------------------------------------------------------------------

def test_one_trigger_produces_a_capture_unattended(cfg):
    """M13's acceptance test, end to end through session.py and decide.py:
    a tap, a sentence, a confirmation, then the rover frames itself and fires
    with nobody touching anything."""
    sim = RoverSim(cfg)
    state = session.initial_state(0.0)

    captured = False
    phases = []
    said = []

    for i in range(4000):
        t = i * DT
        obs = sim.observation(t)

        signals = {}
        if i == 1:
            signals["tap"] = True
        elif i == 20:
            signals["transcript"] = "frame me next to the big tree"
        elif i == 25 and obs["object"]:
            signals["goal_spec"] = dict(make_goal(cfg, obs), locked=False)

        state, events = session.step(state, obs, cfg, t, signals)
        if not phases or phases[-1] != state["phase"]:
            phases.append(state["phase"])
        said += [e["text"] for e in events if e["kind"] == "say"]

        cmd, state = decide.step(obs, state, cfg)
        sim.step(cmd, DT)
        if cmd["capture"]:
            captured = True

    assert captured, f"never captured; phases visited: {phases}"
    assert session.CAPTURE in phases
    assert phases[:5] == [session.IDLE, session.LISTEN, session.PARSE,
                          session.CONFIRM, session.ARM], phases
    assert any("Starting" in s for s in said), "the goal was never confirmed aloud"


def test_the_motors_stay_off_until_the_goal_is_locked(cfg):
    """G-1, checked on a live run rather than in isolation: not one command
    with enable set may be issued before the confirmation window closes."""
    sim = RoverSim(cfg)
    state = session.initial_state(0.0)
    enabled_before_lock = 0

    for i in range(200):
        t = i * DT
        obs = sim.observation(t)
        signals = {}
        if i == 1:
            signals["tap"] = True
        elif i == 20:
            signals["transcript"] = "frame me next to the big tree"
        elif i == 25 and obs["object"]:
            signals["goal_spec"] = dict(make_goal(cfg, obs), locked=False)

        state, _events = session.step(state, obs, cfg, t, signals)
        locked = bool((state.get("goal") or {}).get("locked"))
        cmd, state = decide.step(obs, state, cfg)
        if cmd["enable"] and not locked:
            enabled_before_lock += 1
        sim.step(cmd, DT)

    assert enabled_before_lock == 0


# ---------------------------------------------------------------------------
# M14  the pose run
# ---------------------------------------------------------------------------

def test_a_pose_run_frames_then_coaches_and_never_both(cfg):
    """PG-5: rover positioning and subject coaching must never run at the same
    time.  Checked on a live run: every tick in the coach phase must command
    zero duty."""
    sim = RoverSim(cfg)
    obs = sim.observation(0.0)
    spec = make_goal(cfg, obs, kind="pose", pose="point_at", limb="right_arm")
    state = session.initial_state(0.0)
    state["goal"] = spec
    state["phase"] = session.FRAME

    coached_while_moving = 0
    reached_coach = False

    for i in range(3000):
        t = i * DT
        obs = sim.observation(t)
        state, _events = session.step(state, obs, cfg, t, {})
        cmd, state = decide.step(obs, state, cfg)
        if state["phase"] == session.COACH:
            reached_coach = True
            if cmd["left"] or cmd["right"]:
                coached_while_moving += 1
        sim.step(cmd, DT)

    assert reached_coach, "the pose run never reached the coach phase"
    assert coached_while_moving == 0, \
        f"drove on {coached_while_moving} ticks while coaching"


def test_the_coach_speaks_about_the_right_arm(cfg):
    """CO-2.4 on a live run.  The simulated subject's arms hang down, so a
    pointing goal at a tree up and to the left of them needs the arm raised -
    and the cue must name the arm the goal asked for."""
    sim = RoverSim(cfg)
    obs = sim.observation(0.0)
    spec = make_goal(cfg, obs, kind="pose", pose="point_at", limb="right_arm")
    state = decide.initial_state(0.0, spec, decide.PHASE_COACH)

    lines = []
    for i in range(400):
        obs = sim.observation(i * DT)
        cmd, state = decide.step(obs, state, cfg)
        sim.step(cmd, DT)
        if cmd["say"]:
            lines.append(cmd["say"])

    assert lines, "the coach never said anything"
    assert all("left arm" not in line for line in lines), \
        f"coached the wrong arm: {lines}"
