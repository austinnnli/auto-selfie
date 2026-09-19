"""coach.py: rate limiting and hysteresis under a synthetic error sequence,
and the left/right case that gets its own test by name in the PRD.

CO-2 lists eight rules and says "all required".  There is one test per rule.
"""

import pytest

from app import coach


@pytest.fixture
def ccfg(cfg):
    return cfg["coach"]


def perr(error, goal="point_at", unit="deg", tol=3.0, axis="vertical",
         remedy="person", limb="right_arm", signed=None):
    out = {"goal": goal, "error": error, "unit": unit, "tolerance": tol,
           "in_tolerance": error is not None and abs(error) <= tol,
           "measurable": error is not None, "remedy": remedy, "axis": axis,
           "detail": "", "limb": limb}
    if signed is not None:
        out["signed"] = signed
    return out


# ---------------------------------------------------------------------------
# CO-2.4  THE MIRROR
# ---------------------------------------------------------------------------

def test_the_subjects_left_is_image_right():
    """The most common bug in this class of feature, and it destroys trust on
    the first run.  The phone is rear-facing and pointed AT the subject, so
    what the image needs to move right, the subject achieves by going to their
    own LEFT."""
    assert coach.subject_direction("image_right") == "your left"
    assert coach.subject_direction("image_left") == "your right"


def test_vertical_directions_are_not_mirrored():
    """Gravity is shared; only left and right flip."""
    assert coach.subject_direction("image_up") == "up"
    assert coach.subject_direction("image_down") == "down"


def test_lateral_phrases_are_spoken_in_the_subjects_frame():
    """A positive signed error means the subject must move toward +x, which is
    image-right, which they hear as 'your left'."""
    assert "your left" in coach.phrase_for_lateral(+0.20, 0.02)
    assert "your right" in coach.phrase_for_lateral(-0.20, 0.02)


def test_limb_names_are_already_the_subjects_own():
    """``left_arm`` means the subject's left arm, because that is how the
    keypoints are named.  It must not be mirrored a second time."""
    assert coach.mirror_limb("left_arm") == "left"
    assert coach.mirror_limb("right_arm") == "right"


def test_the_full_lateral_path_does_not_double_mirror(ccfg):
    """The regression that matters: an error measured in image space, turned
    into a phrase, must flip exactly once."""
    state = coach.initial_state(0.0)
    error = perr(0.30, goal="hold", unit="frac", tol=0.02, axis="horizontal",
                 signed=0.30)
    phrase, _st = coach.coach(error, state, ccfg, 0.0)
    assert "your left" in phrase
    assert "your right" not in phrase


# ---------------------------------------------------------------------------
# CO-1  the band table
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("error,expected", [
    (30.0, "a lot"),
    (-30.0, "a lot"),
    (12.0, "A bit higher"),
    (-12.0, "A bit lower"),
    (5.0, "tiny bit up"),
    (-5.0, "tiny bit down"),
])
def test_error_maps_to_the_right_band(error, expected):
    phrase = coach.phrase_for_angle(error, 3.0, "right_arm")
    assert expected in phrase


def test_inside_tolerance_produces_no_phrase():
    assert coach.phrase_for_angle(1.0, 3.0, "right_arm") is None


def test_big_errors_name_the_arm():
    assert "right arm" in coach.phrase_for_angle(30.0, 3.0, "right_arm")
    assert "left arm" in coach.phrase_for_angle(30.0, 3.0, "left_arm")


# ---------------------------------------------------------------------------
# CO-2.2  rate limiting
# ---------------------------------------------------------------------------

def test_at_most_one_utterance_per_interval(ccfg):
    """Faster is unusable - the subject is still acting on the previous cue."""
    state = coach.initial_state(0.0)
    spoken = []
    t = 0.0
    # 30 seconds of a large, slowly changing error at 25 Hz.
    for i in range(750):
        t = i * 0.04
        error = perr(30.0 - i * 0.02)
        phrase, state = coach.coach(error, state, dict(ccfg, coach_timeout=999), t)
        if phrase:
            spoken.append(t)

    assert spoken, "the coach never said anything"
    gaps = [b - a for a, b in zip(spoken, spoken[1:])]
    assert all(g >= ccfg["min_speak_interval"] - 1e-6 for g in gaps), \
        f"spoke too fast: smallest gap {min(gaps):.2f}s"


def test_urgent_announcements_may_speak_sooner(ccfg):
    state = coach.initial_state(0.0)
    _text, state = coach.announce(state, "Stopping.", 0.0, ccfg)
    blocked, _ = coach.announce(state, "Careful.", 0.5, ccfg)
    assert blocked is None
    urgent, _ = coach.announce(state, "Careful.", 0.7, ccfg, urgent=True)
    assert urgent == "Careful."


# ---------------------------------------------------------------------------
# CO-2.3  hysteresis
# ---------------------------------------------------------------------------

def test_hysteresis_stops_the_up_down_up_oscillation(ccfg):
    """Once inside tolerance the zone widens.  A subject hovering just outside
    the original band must hear the countdown, not an endless correction.

    The synthetic sequence: settle inside tolerance, then drift to 4 degrees -
    outside the 3 degree tolerance but inside 3 x 1.6 = 4.8."""
    state = coach.initial_state(0.0)
    t = 0.0
    phrase, state = coach.coach(perr(1.0), state, ccfg, t)
    assert state["in_tolerance"]
    assert phrase == "Hold - three"

    said = []
    for i in range(1, 60):
        t = i * 0.25
        phrase, state = coach.coach(perr(4.0), state, ccfg, t)
        if phrase:
            said.append(phrase)

    assert not any("bit" in p or "Raise" in p or "Lower" in p for p in said), \
        f"hysteresis failed; the coach resumed correcting: {said}"


def test_leaving_the_widened_zone_resumes_coaching(ccfg):
    state = coach.initial_state(0.0)
    _phrase, state = coach.coach(perr(1.0), state, ccfg, 0.0)
    assert state["in_tolerance"]

    phrase, state = coach.coach(perr(15.0), state, ccfg, 5.0)
    assert state["in_tolerance"] is False
    assert phrase is not None and "bit" in phrase


# ---------------------------------------------------------------------------
# CO-2.5  the countdown
# ---------------------------------------------------------------------------

def test_countdown_runs_three_two_one_then_signals_the_capture(ccfg):
    state = coach.initial_state(0.0)
    heard = []
    for i in range(20):
        t = i * 0.5
        phrase, state = coach.coach(perr(0.5), state, ccfg, t)
        if phrase:
            heard.append(phrase)
        if coach.countdown_complete(state):
            break

    assert heard == ["Hold - three", "two", "one"]
    assert coach.countdown_complete(state)


def test_countdown_restarts_if_the_subject_moves(ccfg):
    state = coach.initial_state(0.0)
    phrase, state = coach.coach(perr(0.5), state, ccfg, 0.0)
    assert phrase == "Hold - three"

    _phrase, state = coach.coach(perr(30.0), state, ccfg, 4.0)
    assert not state["in_tolerance"]

    phrase, state = coach.coach(perr(0.5), state, ccfg, 8.0)
    assert phrase == "Hold - three", "the countdown must start again, not resume"


# ---------------------------------------------------------------------------
# CO-2.7  give up
# ---------------------------------------------------------------------------

def test_the_coach_gives_up_and_says_so(ccfg):
    """An endless correction loop is worse than an imperfect photo."""
    state = coach.initial_state(0.0)
    phrase = None
    for i in range(2000):
        t = i * 0.04
        out, state = coach.coach(perr(40.0), state, ccfg, t)
        if out and "good enough" in out:
            phrase = out
            break
    assert phrase is not None
    assert state["gave_up"]


def test_giving_up_is_said_once(ccfg):
    state = coach.initial_state(0.0)
    said = 0
    for i in range(2000):
        out, state = coach.coach(perr(40.0), state, ccfg, i * 0.04)
        if out and "good enough" in out:
            said += 1
    assert said == 1


# ---------------------------------------------------------------------------
# CO-2.8  staleness
# ---------------------------------------------------------------------------

def test_a_stale_measurement_produces_no_cue(ccfg):
    """A cue must reflect a measurement no more than 500 ms old, or the subject
    corrects against stale information."""
    state = coach.initial_state(0.0)
    phrase, _st = coach.coach(perr(30.0), state, ccfg, 1.0, obs_age=0.9)
    assert phrase is None

    phrase, _st = coach.coach(perr(30.0), state, ccfg, 1.0, obs_age=0.2)
    assert phrase is not None


# ---------------------------------------------------------------------------
# CO-2.1  one instruction at a time
# ---------------------------------------------------------------------------

def test_the_same_phrase_is_not_repeated_back_to_back(ccfg):
    """Repeating reads as the system not having noticed the person moved."""
    state = coach.initial_state(0.0)
    first, state = coach.coach(perr(30.0), state, ccfg, 0.0)
    second, state = coach.coach(perr(30.0), state, ccfg, 2.0)
    assert first is not None
    assert second is None


def test_an_unmeasurable_error_is_silent(ccfg):
    """P-7 all the way through: no measurement, no cue, no guess."""
    state = coach.initial_state(0.0)
    phrase, _st = coach.coach(perr(None), state, ccfg, 0.0)
    assert phrase is None


def test_foreshortening_is_coached_as_a_turn_not_an_angle(ccfg):
    """PG-3: the angle cannot be trusted, so do not coach the angle."""
    error = perr(None, remedy="turn_side_on")
    error["measurable"] = True
    error["error"] = 0.0
    phrase = coach.phrase_for(error, ccfg)
    assert "side-on" in phrase


def test_foreshortening_is_spoken_even_though_there_is_no_angle(ccfg):
    """The regression this test exists for: pose.py returns turn_side_on with
    error None, and an over-eager "no measurement, no cue" guard swallowed it.
    The subject would then stand there being told nothing while the rover
    waited for an angle it could never measure."""
    error = perr(None, remedy="turn_side_on")
    state = coach.initial_state(0.0)
    phrase, state = coach.coach(error, state, ccfg, 0.0)
    assert phrase is not None and "side-on" in phrase
    assert state["last_say_t"] == 0.0


def test_the_turn_side_on_cue_is_still_rate_limited(ccfg):
    error = perr(None, remedy="turn_side_on")
    state = coach.initial_state(0.0)
    first, state = coach.coach(error, state, ccfg, 0.0)
    second, state = coach.coach(error, state, ccfg, 0.3)
    assert first is not None and second is None


# ---------------------------------------------------------------------------
# CO-2.6  the burst
# ---------------------------------------------------------------------------

def test_best_frame_prefers_sharp_and_well_posed():
    candidates = [
        {"index": 0, "sharpness": 100.0, "pose_error": 8.0},
        {"index": 1, "sharpness": 95.0, "pose_error": 0.5},
        {"index": 2, "sharpness": 20.0, "pose_error": 0.2},
    ]
    assert coach.pick_best_frame(candidates)["index"] == 1


def test_best_frame_of_nothing_is_nothing():
    assert coach.pick_best_frame([]) is None
