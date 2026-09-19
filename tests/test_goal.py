"""goal.py: the tier-1 grammar.

G-5.1 says tier 1 "must handle every phrase in G-3 without network access".
The G-3 table is transcribed here verbatim as parametrised cases, so a change
to the grammar that drops a row fails loudly.
"""

import pytest

from app import goal


# The G-3 table, exactly as the PRD gives it.
G3_TABLE = [
    ("point at", "pose", "point_at"),
    ("pointing at", "pose", "point_at"),
    ("hold", "pose", "hold"),
    ("holding", "pose", "hold"),
    ("touch", "pose", "hold"),
    ("touching", "pose", "hold"),
    ("pinch", "pose", "hold"),
    ("look at", "pose", "look_at"),
    ("looking at", "pose", "look_at"),
    ("facing", "pose", "look_at"),
    ("next to", "framing", None),
    ("beside", "framing", None),
    ("in front of", "framing", None),
    ("behind me", "framing", None),
    ("with", "framing", None),
]


@pytest.mark.parametrize("phrase,kind,pose_goal", G3_TABLE)
def test_every_g3_row_classifies(phrase, kind, pose_goal, cfg):
    spec, confidence = goal.parse(f"take a photo of me {phrase} the red door", cfg)
    assert spec["kind"] == kind, f"{phrase!r} -> {spec['kind']}"
    assert spec["pose_goal"] == pose_goal
    assert confidence >= cfg["goal"]["parse_confidence"], \
        f"{phrase!r} parsed at {confidence}, below the tier-2 threshold"


def test_a_bare_object_with_no_verb_is_framing(cfg):
    """G-3's last row."""
    spec, _conf = goal.parse("the lighthouse", cfg)
    assert spec["kind"] == "framing"
    assert spec["pose_goal"] is None
    assert spec["object_phrase"] == "the lighthouse"


def test_the_two_worked_examples_from_the_prd(cfg):
    """"make me point at the bird" becomes a pose run with point_at, and
    "frame me next to the big tree" becomes a framing run."""
    spec, conf = goal.parse("make me point at the bird", cfg)
    assert (spec["kind"], spec["pose_goal"], spec["object_phrase"]) == \
        ("pose", "point_at", "the bird")
    assert conf > 0.9

    spec, conf = goal.parse("frame me next to the big tree", cfg)
    assert (spec["kind"], spec["pose_goal"], spec["object_phrase"]) == \
        ("framing", None, "the big tree")
    assert conf > 0.85


def test_longer_phrases_are_not_shadowed_by_shorter_ones(cfg):
    """"looking at" must not be eaten by "look", and "holding onto" must not
    leave "onto" in the object phrase."""
    spec, _c = goal.parse("photograph me looking at the sea", cfg)
    assert spec["pose_goal"] == "look_at"
    assert spec["object_phrase"] == "the sea"

    spec, _c = goal.parse("me holding onto the railing", cfg)
    assert spec["pose_goal"] == "hold"
    assert spec["object_phrase"] == "the railing"


# ---------------------------------------------------------------------------
# G-4  the limb
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("phrase,limb", [
    ("with my left hand", "left_arm"),
    ("with my right hand", "right_arm"),
    ("with your left arm", "left_arm"),
    ("using my right arm", "right_arm"),
])
def test_an_explicit_limb_is_extracted(phrase, limb, cfg):
    spec, _conf = goal.parse(f"make me point at the sign {phrase}", cfg)
    assert spec["limb"] == limb


def test_the_limb_phrase_does_not_leak_into_the_object(cfg):
    """"point at the sign with my left hand" must not resolve an object called
    "my left hand" - which is what happens if the limb is not removed first."""
    spec, _conf = goal.parse("point at the sign with my left hand", cfg)
    assert spec["object_phrase"] == "the sign"
    assert "hand" not in spec["object_phrase"]


def test_no_limb_stated_leaves_it_to_the_geometry(cfg):
    """G-4: otherwise choose the arm on the same side as the object - which is
    pose.choose_limb's job, not the parser's."""
    spec, _conf = goal.parse("make me point at the sign", cfg)
    assert spec["limb"] is None


# ---------------------------------------------------------------------------
# G-3  composition defaults and overrides
# ---------------------------------------------------------------------------

def test_defaults_are_the_rule_of_thirds(cfg):
    """G-3: rule of thirds, subject on the third opposite the object, eyeline
    on the upper third, subject at 50% of frame height."""
    spec, _conf = goal.parse("frame me with the tree", cfg)
    comp = spec["composition"]
    assert comp["subject_x"] == 0.333
    assert comp["object_x"] == 0.667
    assert comp["eyeline_y"] == 0.333
    assert comp["subject_height"] == 0.5


@pytest.mark.parametrize("phrase,key,value", [
    ("on the left", "subject_x", 0.333),
    ("on the right", "subject_x", 0.667),
    ("full body", "subject_height", 0.85),
    ("head and shoulders", "subject_height", 0.35),
    ("low angle", "eyeline_y", 0.6),
])
def test_a_stated_override_replaces_one_default(phrase, key, value, cfg):
    """"The user only ever states a default to override it."" """
    spec, _conf = goal.parse(f"frame me with the tree {phrase}", cfg)
    assert spec["composition"][key] == value


def test_an_override_does_not_leak_into_the_object_phrase(cfg):
    spec, _conf = goal.parse("frame me with the tree full body", cfg)
    assert spec["object_phrase"] == "the tree"


# ---------------------------------------------------------------------------
# G-4  sides
# ---------------------------------------------------------------------------

def test_the_subject_takes_the_third_opposite_the_object(cfg):
    """G-4: for a framing run, the object's current position in frame decides
    which third the subject takes."""
    spec, _conf = goal.parse("frame me with the tree", cfg)

    object_on_right = goal.assign_sides(spec, 0.82)
    assert object_on_right["composition"]["object_x"] == 0.667
    assert object_on_right["composition"]["subject_x"] == 0.333

    object_on_left = goal.assign_sides(spec, 0.18)
    assert object_on_left["composition"]["object_x"] == 0.333
    assert object_on_left["composition"]["subject_x"] == 0.667


def test_assigning_sides_without_a_detection_changes_nothing(cfg):
    spec, _conf = goal.parse("frame me with the tree", cfg)
    assert goal.assign_sides(spec, None)["composition"] == spec["composition"]


# ---------------------------------------------------------------------------
# G-5.2  schema validation
# ---------------------------------------------------------------------------

def test_a_well_formed_spec_validates(cfg):
    spec, _conf = goal.parse("make me point at the bird", cfg)
    ok, reason = goal.validate(spec)
    assert ok, reason


@pytest.mark.parametrize("mutation,fragment", [
    ({"kind": "interpretive_dance"}, "kind"),
    ({"kind": "pose", "pose_goal": "jump"}, "does not exist"),
    ({"kind": "pose", "pose_goal": None}, "does not exist"),
    ({"limb": "tail"}, "limb"),
    ({"composition": {"subject_x": "left"}}, "not a number"),
    ({"composition": {"subject_x": 42.0}}, "out of range"),
    ({"composition": "thirds"}, "not an object"),
])
def test_invalid_specs_are_rejected(mutation, fragment, cfg):
    """G-5.2: tier-2 output is validated against the schema and rejected if it
    names a goal that does not exist.  A model that invents a pose goal must
    not be able to put the session into a state with no handler."""
    spec, _conf = goal.parse("make me point at the bird", cfg)
    spec.update(mutation)
    ok, reason = goal.validate(spec)
    assert not ok
    assert fragment in reason


def test_a_framing_run_may_not_carry_a_pose_goal(cfg):
    spec, _conf = goal.parse("frame me with the tree", cfg)
    spec["pose_goal"] = "point_at"
    ok, reason = goal.validate(spec)
    assert not ok and "must not carry" in reason


def test_validate_rejects_non_objects():
    assert goal.validate(None)[0] is False
    assert goal.validate("point at the bird")[0] is False


# ---------------------------------------------------------------------------
# G-7  confirmation
# ---------------------------------------------------------------------------

def test_the_confirmation_paraphrases_rather_than_echoes(cfg):
    """G-7: "Pointing at the bird, you on the left third. Starting."

    Reading the sentence back verbatim would prove only that the microphone
    works, not that the parse is right - which is the entire point of the
    two-second objection window."""
    spec, _conf = goal.parse("erm take a photo of me pointing at the bird", cfg)
    line = goal.confirmation_line(spec)
    assert line == "Pointing at the bird, you on the left third. Starting."
    assert spec["raw"] not in line


def test_the_confirmation_names_an_explicit_limb(cfg):
    spec, _conf = goal.parse("point at the sign with my left hand", cfg)
    assert "left hand" in goal.confirmation_line(spec)


def test_the_confirmation_reflects_the_assigned_side(cfg):
    spec, _conf = goal.parse("frame me with the tree", cfg)
    spec = goal.assign_sides(spec, 0.18)      # object on the left
    assert "right third" in goal.confirmation_line(spec)


# ---------------------------------------------------------------------------
# CO-5  cancel and objection
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("text", ["cancel", "stop", "never mind", "forget it",
                                  "Cancel!", "stop stop stop"])
def test_cancel_is_recognised(text):
    assert goal.is_cancel(text)


@pytest.mark.parametrize("text", ["no", "nope", "wrong", "not that", "no the other one"])
def test_objection_is_recognised(text):
    assert goal.is_objection(text)


def test_an_ordinary_sentence_is_neither():
    assert not goal.is_cancel("point at the bird")
    assert not goal.is_objection("point at the bird")


# ---------------------------------------------------------------------------
# robustness
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("text", ["", "   ", "umm", "the", "a a a a"])
def test_junk_input_never_crashes_and_never_scores_high(text, cfg):
    spec, confidence = goal.parse(text, cfg)
    ok, _reason = goal.validate(spec)
    assert ok, "even a junk parse must produce a schema-valid spec"
    assert confidence < cfg["goal"]["parse_confidence"] or spec["object_phrase"]


def test_punctuation_and_case_do_not_matter(cfg):
    a, _ = goal.parse("Make me point at the bird!", cfg)
    b, _ = goal.parse("make me point at the bird", cfg)
    assert a["kind"] == b["kind"]
    assert a["object_phrase"] == b["object_phrase"]


def test_a_half_heard_sentence_scores_low_enough_for_tier_two(cfg):
    """G-5.2: anything tier 1 cannot classify above parse_confidence goes to
    the cloud fallback.  "point at" with nothing after it is exactly that."""
    _spec, confidence = goal.parse("make me point at", cfg)
    assert confidence < cfg["goal"]["parse_confidence"]
