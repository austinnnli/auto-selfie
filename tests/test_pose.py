"""pose.py: every goal at known angles, including the mirrored left/right case.

Named in the PRD's unit-test list.  The angles here are worked out by hand from
the geometry, not read back from the implementation.
"""

import math

import pytest

from app import pose
from tests.conftest import make_keypoints


@pytest.fixture
def pcfg(cfg):
    c = dict(cfg["pose"])
    c["subject_shoulder_m"] = 0.40
    return c


def arm(elbow, wrist, side="right"):
    """Keypoints with one arm placed exactly where we want it."""
    return make_keypoints(**{
        f"{side}_elbow": (elbow[0], elbow[1], 0.9),
        f"{side}_wrist": (wrist[0], wrist[1], 0.9),
    })


# ---------------------------------------------------------------------------
# PG-2  the pointing computation
# ---------------------------------------------------------------------------

def test_perfectly_aligned_arm_has_zero_error():
    elbow, wrist, obj = (0.40, 0.50), (0.50, 0.50), (0.90, 0.50)
    assert pose.point_at_error(elbow, wrist, *obj) == pytest.approx(0.0)


def test_positive_error_means_raise_the_hand():
    """The sign convention the PRD states, and the one that is easiest to get
    backwards: image y increases DOWNWARD, so raising the hand makes the arm
    angle smaller.  Arm horizontal, object 45 degrees above -> +45."""
    elbow, wrist = (0.50, 0.50), (0.60, 0.50)
    obj = (0.60, 0.40)          # dx = +0.10, dy = -0.10 -> 45 degrees up
    err = pose.point_at_error(elbow, wrist, *obj)
    assert err == pytest.approx(45.0)


def test_negative_error_means_lower_the_hand():
    elbow, wrist = (0.50, 0.50), (0.60, 0.50)
    obj = (0.60, 0.60)          # 45 degrees below
    assert pose.point_at_error(elbow, wrist, *obj) == pytest.approx(-45.0)


def test_pointing_error_wraps_the_short_way():
    """An arm pointing left at an object on the right is 180 degrees out, and
    must never come back as 350."""
    elbow, wrist = (0.50, 0.50), (0.40, 0.50)
    err = pose.point_at_error(elbow, wrist, 0.90, 0.50)
    assert abs(err) == pytest.approx(180.0)
    for oy in (0.49, 0.51):
        assert -180.0 <= pose.point_at_error(elbow, wrist, 0.90, oy) < 180.0


@pytest.mark.parametrize("degrees", [-150, -90, -30, -3, 0, 3, 30, 90, 150])
def test_pointing_error_recovers_a_known_offset(degrees):
    """Place the arm at a known angle from the object direction and check the
    measurement returns exactly that."""
    elbow = (0.50, 0.50)
    obj_angle = math.radians(20.0)
    obj = (elbow[0] + 0.4 * math.cos(obj_angle), elbow[1] + 0.4 * math.sin(obj_angle))
    arm_angle = obj_angle + math.radians(degrees)
    wrist = (elbow[0] + 0.1 * math.cos(arm_angle), elbow[1] + 0.1 * math.sin(arm_angle))
    assert pose.point_at_error(elbow, wrist, *obj) == pytest.approx(degrees, abs=1e-6)


def test_point_at_goal_end_to_end(pcfg):
    kps = arm((0.45, 0.50), (0.55, 0.50), "right")
    obj = {"cx": 0.85, "cy": 0.30, "w": 0.1, "h": 0.1, "conf": 0.9}
    out = pose.goal_point_at(kps, obj, 0.12, "right_arm", pcfg)
    assert out["measurable"]
    assert out["error"] > 0        # object is above the arm: raise
    assert out["unit"] == "deg"
    assert out["axis"] == "vertical"


def test_point_at_is_in_tolerance_at_two_degrees(pcfg):
    elbow = (0.45, 0.50)
    obj = {"cx": 0.85, "cy": 0.50, "w": 0.1, "h": 0.1, "conf": 0.9}
    angle = math.radians(-2.0)
    wrist = (elbow[0] + 0.1 * math.cos(angle), elbow[1] + 0.1 * math.sin(angle))
    kps = arm(elbow, wrist, "right")
    out = pose.goal_point_at(kps, obj, 0.12, "right_arm", pcfg)
    assert out["in_tolerance"]
    assert out["remedy"] == pose.REMEDY_NONE


def test_point_at_without_an_object_is_unmeasurable(pcfg):
    out = pose.goal_point_at(make_keypoints(), None, 0.12, "right_arm", pcfg)
    assert not out["measurable"]
    assert out["error"] is None


def test_point_at_with_a_hidden_wrist_is_unmeasurable(pcfg):
    """P-7: perception emits a low-confidence keypoint and the geometry layer
    must refuse it rather than measure noise."""
    kps = make_keypoints(right_wrist=(0.55, 0.50, 0.05))
    obj = {"cx": 0.85, "cy": 0.30, "w": 0.1, "h": 0.1, "conf": 0.9}
    out = pose.goal_point_at(kps, obj, 0.12, "right_arm", pcfg)
    assert not out["measurable"]


# ---------------------------------------------------------------------------
# PG-3  foreshortening
# ---------------------------------------------------------------------------

def test_a_foreshortened_arm_asks_the_subject_to_turn(pcfg):
    """PG-3: an arm angled toward or away from the camera projects to a
    misleading angle, so no angle correction can help.  Expected forearm at
    shoulder_px 0.12 is 0.12 * 0.63 = 0.0756; a 0.03 forearm is 40% of that,
    well under the 0.7 threshold."""
    kps = arm((0.45, 0.50), (0.48, 0.50), "right")
    obj = {"cx": 0.85, "cy": 0.30, "w": 0.1, "h": 0.1, "conf": 0.9}
    out = pose.goal_point_at(kps, obj, 0.12, "right_arm", pcfg)
    assert out["remedy"] == pose.REMEDY_TURN_SIDE_ON
    assert out["error"] is None, "a foreshortened angle must not be reported"


def test_a_full_length_arm_is_measured_normally(pcfg):
    kps = arm((0.45, 0.50), (0.53, 0.50), "right")   # 0.08 > 0.7 * 0.0756
    obj = {"cx": 0.85, "cy": 0.30, "w": 0.1, "h": 0.1, "conf": 0.9}
    out = pose.goal_point_at(kps, obj, 0.12, "right_arm", pcfg)
    assert out["remedy"] != pose.REMEDY_TURN_SIDE_ON
    assert out["measurable"]


def test_foreshortening_check_is_skipped_without_a_shoulder_measurement(pcfg):
    short, ratio = pose.foreshortened((0.4, 0.4), (0.45, 0.5), (0.48, 0.5), None, pcfg)
    assert short is False and ratio is None


# ---------------------------------------------------------------------------
# G-4  the mirrored left/right case
# ---------------------------------------------------------------------------

def test_limb_choice_picks_the_arm_nearest_the_object(pcfg):
    """G-4: pointing across the body reads badly and foreshortens, so the arm
    on the same side as the object wins.

    THE MIRROR: the phone faces the subject, so the subject's LEFT wrist is at
    the LARGER image x.  An object on the image-right is therefore reached by
    the subject's LEFT arm, not their right."""
    kps = make_keypoints()
    assert kps["left_wrist"][0] > kps["right_wrist"][0]

    right_side_object = {"cx": 0.92, "cy": 0.4, "w": 0.1, "h": 0.1, "conf": 0.9}
    left_side_object = {"cx": 0.08, "cy": 0.4, "w": 0.1, "h": 0.1, "conf": 0.9}

    assert pose.choose_limb(kps, right_side_object, None, 0.4) == "left_arm"
    assert pose.choose_limb(kps, left_side_object, None, 0.4) == "right_arm"


def test_an_explicit_limb_always_wins(pcfg):
    """"with your left hand" beats the geometry, even when it points across
    the body."""
    kps = make_keypoints()
    obj = {"cx": 0.08, "cy": 0.4, "w": 0.1, "h": 0.1, "conf": 0.9}
    assert pose.choose_limb(kps, obj, "left_arm", 0.4) == "left_arm"


def test_limb_choice_survives_a_subject_facing_away(pcfg):
    """Decided from the keypoints themselves, so a turned-round subject still
    gets the near arm rather than an assumption."""
    kps = make_keypoints(left_wrist=(0.415, 0.575, 0.85),
                         right_wrist=(0.585, 0.575, 0.85))
    obj = {"cx": 0.92, "cy": 0.4, "w": 0.1, "h": 0.1, "conf": 0.9}
    assert pose.choose_limb(kps, obj, None, 0.4) == "right_arm"


# ---------------------------------------------------------------------------
# the rest of the PG-1 table
# ---------------------------------------------------------------------------

def test_hold_measures_wrist_to_object_distance(pcfg):
    kps = make_keypoints(right_wrist=(0.50, 0.50, 0.9))
    obj = {"cx": 0.53, "cy": 0.54, "w": 0.1, "h": 0.1, "conf": 0.9}
    out = pose.goal_hold(kps, obj, "right_arm", pcfg)
    assert out["error"] == pytest.approx(math.hypot(0.03, 0.04))
    assert out["unit"] == "frac"


def test_hold_is_in_tolerance_within_two_percent(pcfg):
    kps = make_keypoints(right_wrist=(0.500, 0.500, 0.9))
    obj = {"cx": 0.510, "cy": 0.500, "w": 0.1, "h": 0.1, "conf": 0.9}
    out = pose.goal_hold(kps, obj, "right_arm", pcfg)
    assert out["in_tolerance"]


def test_look_at_measures_the_gaze_offset(pcfg):
    kps = make_keypoints(nose=(0.50, 0.30, 0.95),
                         left_eye=(0.51, 0.28, 0.9), right_eye=(0.49, 0.28, 0.9))
    straight = {"cx": 0.50, "cy": 0.95, "w": 0.1, "h": 0.1, "conf": 0.9}
    out = pose.goal_look_at(kps, straight, pcfg)
    assert out["measurable"]
    assert abs(out["error"]) < 8.0


def test_look_at_needs_a_visible_face(pcfg):
    kps = make_keypoints(left_eye=(0.51, 0.28, 0.05), right_eye=(0.49, 0.28, 0.05))
    obj = {"cx": 0.9, "cy": 0.3, "w": 0.1, "h": 0.1, "conf": 0.9}
    assert not pose.goal_look_at(kps, obj, pcfg)["measurable"]


def test_beside_wants_the_subject_on_the_correct_side(pcfg):
    subject = {"cx": 0.30, "cy": 0.5, "w": 0.18, "h": 0.6, "conf": 0.9}
    obj = {"cx": 0.55, "cy": 0.5, "w": 0.1, "h": 0.2, "conf": 0.9}
    out = pose.goal_beside(subject, obj, "left", pcfg)
    assert out["measurable"]
    assert out["error"] == pytest.approx(0.0, abs=0.01)


def test_no_overlap_is_zero_when_the_boxes_are_apart(pcfg):
    subject = {"cx": 0.25, "cy": 0.5, "w": 0.18, "h": 0.6, "conf": 0.9}
    obj = {"cx": 0.80, "cy": 0.4, "w": 0.14, "h": 0.3, "conf": 0.9}
    out = pose.goal_no_overlap(subject, obj, pcfg)
    assert out["error"] == pytest.approx(0.0)
    assert out["in_tolerance"]


def test_no_overlap_measures_the_covered_fraction(pcfg):
    """Subject box 0.2 x 0.6 at x 0.30..0.50, object 0.2 x 0.6 at 0.40..0.60:
    they share 0.10 of width and all of the height, so half the subject."""
    subject = {"cx": 0.40, "cy": 0.5, "w": 0.20, "h": 0.60, "conf": 0.9}
    obj = {"cx": 0.50, "cy": 0.5, "w": 0.20, "h": 0.60, "conf": 0.9}
    out = pose.goal_no_overlap(subject, obj, pcfg)
    assert out["error"] == pytest.approx(0.5)
    assert not out["in_tolerance"]


@pytest.mark.parametrize("goal_id", ["arms_out", "arms_down", "hands_hips"])
def test_templates_are_satisfied_by_their_own_template(goal_id, pcfg):
    """Build the arms exactly at the template angles and the error must vanish."""
    tpl = pose.ARM_TEMPLATES[goal_id]
    kps = dict(make_keypoints())
    for side in ("left", "right"):
        shoulder = kps[f"{side}_shoulder"]
        angle = math.radians(tpl[side])
        kps[f"{side}_elbow"] = (shoulder[0] + 0.12 * math.cos(angle),
                                shoulder[1] + 0.12 * math.sin(angle), 0.9)
    out = pose.goal_template(kps, goal_id, pcfg)
    assert out["measurable"]
    assert out["error"] == pytest.approx(0.0, abs=1e-6)
    assert out["in_tolerance"]


def test_template_reports_the_worst_side(pcfg):
    """CO-2.1: one instruction at a time, so the larger error is the one that
    surfaces."""
    tpl = pose.ARM_TEMPLATES["arms_out"]
    kps = dict(make_keypoints())
    for side, offset in (("left", 4.0), ("right", -25.0)):
        shoulder = kps[f"{side}_shoulder"]
        angle = math.radians(tpl[side] + offset)
        kps[f"{side}_elbow"] = (shoulder[0] + 0.12 * math.cos(angle),
                                shoulder[1] + 0.12 * math.sin(angle), 0.9)
    out = pose.goal_template(kps, "arms_out", pcfg)
    assert out["side"] == "right"
    assert abs(out["error"]) == pytest.approx(25.0, abs=0.01)


# ---------------------------------------------------------------------------
# PG-4  the rover-first remedy policy
# ---------------------------------------------------------------------------

def test_small_errors_are_the_rovers_problem(pcfg):
    """A system that asks the person to move when it could have driven 20 cm
    feels bossy; one that quietly repositions feels intelligent."""
    assert pose.remedy_for_angle(5.0, pcfg) == pose.REMEDY_ROVER
    assert pose.remedy_for_angle(-7.9, pcfg) == pose.REMEDY_ROVER


def test_large_errors_are_the_persons_problem(pcfg):
    assert pose.remedy_for_angle(25.0, pcfg) == pose.REMEDY_PERSON
    assert pose.remedy_for_angle(-40.0, pcfg) == pose.REMEDY_PERSON


def test_a_satisfied_goal_needs_no_remedy(pcfg):
    out = pose.result("point_at", 1.0, pose.UNIT_DEG, 3.0, remedy=pose.REMEDY_ROVER)
    assert out["in_tolerance"]
    assert out["remedy"] == pose.REMEDY_NONE


def test_rover_bias_only_moves_for_rover_class_errors(pcfg):
    person = pose.result("point_at", 30.0, pose.UNIT_DEG, 3.0,
                         remedy=pose.REMEDY_PERSON, axis="vertical")
    assert pose.rover_bias(person, pcfg) == {"subject_x_bias": 0.0, "dist_bias": 0.0}

    rover = pose.result("point_at", 6.0, pose.UNIT_DEG, 3.0,
                        remedy=pose.REMEDY_ROVER, axis="vertical")
    bias = pose.rover_bias(rover, pcfg)
    assert bias["dist_bias"] != 0.0


def test_rover_bias_is_capped(pcfg):
    rover = pose.result("hold", 0.9, pose.UNIT_FRAC, 0.02,
                        remedy=pose.REMEDY_ROVER, axis="horizontal")
    rover["signed"] = 0.9
    bias = pose.rover_bias(rover, pcfg)
    assert abs(bias["subject_x_bias"]) <= pcfg["max_subject_x_bias"] + 1e-9


# ---------------------------------------------------------------------------
# D-4  the metric escape hatch
# ---------------------------------------------------------------------------

def test_distance_from_shoulder_width_is_in_the_right_ballpark():
    """D-4: one calibration of the subject's real shoulder width gives distance
    by similar triangles to about +-10%.  A 0.40 m shoulder spanning 12% of a
    66-degree frame puts the subject about 2.6 m away."""
    d = pose.shoulder_distance_m(0.12, 66.0, 0.40)
    assert 2.3 < d < 2.9


def test_distance_halves_when_the_subject_looks_twice_as_wide():
    near = pose.shoulder_distance_m(0.24, 66.0, 0.40)
    far = pose.shoulder_distance_m(0.12, 66.0, 0.40)
    assert far == pytest.approx(2 * near)


def test_distance_is_none_without_a_measurement():
    assert pose.shoulder_distance_m(None, 66.0, 0.40) is None
    assert pose.shoulder_distance_m(0.0, 66.0, 0.40) is None
