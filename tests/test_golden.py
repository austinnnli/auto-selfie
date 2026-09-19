"""LG-3: the golden corpus.

tests/golden/ holds recorded observations with the command produced for each.
Any refactor, re-tune or future port to JavaScript must reproduce them.  A
mismatch is then a code bug found at a desk instead of a mystery found in a
field.

READ THIS BEFORE REGENERATING.  A golden corpus records what the code does; it
cannot know what the code should do.  Running ``python tools/gen_golden.py``
after a change always makes this file pass again, which makes it worthless
unless something else says what is correct.  That something else is
``test_decide_semantics.py``.  The rule:

    never re-bless a golden mismatch until the semantic tests still pass and
    the failing case's `why` line has been read and agreed with.

The failure message below prints that `why` for exactly this reason.
"""

import hashlib
import json
import pathlib

import pytest

from app import decide

ROOT = pathlib.Path(__file__).resolve().parents[1]
CORPUS = json.loads((ROOT / "tests" / "golden" / "cases.json").read_text())
CASES = CORPUS["cases"]


def ids(case):
    return case["name"]


# ---------------------------------------------------------------------------
# the corpus itself
# ---------------------------------------------------------------------------

def test_the_corpus_is_at_least_fifty_cases():
    """LG-3 asks for 50."""
    assert len(CASES) >= 50, f"only {len(CASES)} cases"
    assert CORPUS["count"] == len(CASES)


def test_case_names_are_unique():
    names = [c["name"] for c in CASES]
    assert len(names) == len(set(names))


def test_every_case_says_why_it_exists():
    """A case without a rationale cannot be adjudicated when it fails, so it
    would only ever be re-blessed - which is the failure mode this whole file
    is designed to avoid."""
    for case in CASES:
        assert len(case["why"]) > 40, f"{case['name']} has no real rationale"


def test_the_corpus_was_generated_from_the_committed_config(config):
    """A corpus recorded against different gains proves nothing about these
    ones.  If this fails, either config.json moved or the corpus is stale."""
    digest = hashlib.sha256((ROOT / "config.json").read_bytes()).hexdigest()[:16]
    assert CORPUS["config_hash"] == digest, (
        "config.json has changed since the corpus was generated.\n"
        "Re-run the semantic tests first, then: python tools/gen_golden.py")


def test_the_corpus_covers_every_session_phase():
    covered = {c["state_in"]["phase"] for c in CASES}
    expected = {decide.PHASE_IDLE, decide.PHASE_LISTEN, decide.PHASE_PARSE,
                decide.PHASE_CONFIRM, decide.PHASE_ARM, decide.PHASE_FRAME,
                decide.PHASE_COACH, decide.PHASE_SETTLE, decide.PHASE_CAPTURE,
                decide.PHASE_HALTED, decide.PHASE_SEARCH}
    missing = expected - covered
    assert not missing, f"no golden case exercises: {sorted(missing)}"


def test_the_corpus_covers_both_moving_and_holding():
    moving = [c for c in CASES if c["cmd_out"]["left"] or c["cmd_out"]["right"]]
    holding = [c for c in CASES if not (c["cmd_out"]["left"] or c["cmd_out"]["right"])]
    assert len(moving) >= 10, "a corpus of nothing but stationary cases proves little"
    assert len(holding) >= 10


def test_the_corpus_covers_speech():
    speaking = [c for c in CASES if c["cmd_out"]["say"]]
    assert len(speaking) >= 5


# ---------------------------------------------------------------------------
# the corpus as a regression test
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("case", CASES, ids=ids)
def test_command_matches_the_recorded_output(case, config):
    cmd, _state_out = decide.step(case["obs"], case["state_in"], config)

    expected = case["cmd_out"]
    actual = dict(cmd)
    actual["tilt_deg"] = round(float(actual["tilt_deg"]), 6)

    if actual != expected:
        diffs = [f"    {k}: recorded {expected[k]!r}, now {actual[k]!r}"
                 for k in expected if actual.get(k) != expected[k]]
        pytest.fail(
            f"\ngolden case '{case['name']}' produced a different command.\n\n"
            f"  what this case is for:\n    {case['why']}\n\n"
            f"  what changed:\n" + "\n".join(diffs) +
            "\n\n  If the new behaviour is correct, confirm the semantic tests "
            "still pass,\n  then re-record with: python tools/gen_golden.py")


@pytest.mark.parametrize("case", CASES, ids=ids)
def test_state_evolution_matches_the_recorded_output(case, config):
    """The state matters as much as the command: the sweep's progress, the
    tilt setpoint, the rate-limit memory and the coach's hysteresis latch all
    live there, and a regression in any of them would not show up in a single
    tick's Command."""
    _cmd, state_out = decide.step(case["obs"], case["state_in"], config)

    from tools.gen_golden import state_digest
    actual = state_digest(state_out)
    expected = case["state_out"]

    if actual != expected:
        diffs = [f"    {k}: recorded {expected[k]!r}, now {actual.get(k)!r}"
                 for k in expected if actual.get(k) != expected[k]]
        pytest.fail(
            f"\ngolden case '{case['name']}' evolved the state differently.\n\n"
            f"  what this case is for:\n    {case['why']}\n\n"
            f"  what changed:\n" + "\n".join(diffs))


@pytest.mark.parametrize("case", CASES, ids=ids)
def test_replaying_a_case_is_deterministic(case, config):
    """The property that makes replay at 10x meaningful: same inputs, same
    outputs, every time and in any order."""
    first, _s1 = decide.step(case["obs"], case["state_in"], config)
    second, _s2 = decide.step(case["obs"], case["state_in"], config)
    assert first == second


@pytest.mark.parametrize("case", CASES, ids=ids)
def test_no_case_produces_an_out_of_range_command(case):
    cmd = case["cmd_out"]
    assert -1000 <= cmd["left"] <= 1000
    assert -1000 <= cmd["right"] <= 1000
    assert -90.0 <= cmd["tilt_deg"] <= 90.0
    assert isinstance(cmd["enable"], bool)
    assert isinstance(cmd["capture"], bool)


@pytest.mark.parametrize("case", CASES, ids=ids)
def test_no_case_commands_a_duty_inside_the_bridge_deadband(case, config):
    """D-2, checked across the whole corpus: every command is either zero or
    large enough to actually turn a wheel.  A value in between would be a
    command the rover silently ignores."""
    floor = min(config["hardware"]["min_duty_left"],
                config["hardware"]["min_duty_right"])
    for side in ("left", "right"):
        duty = case["cmd_out"][side]
        assert duty == 0 or abs(duty) >= floor, (
            f"{case['name']} commands {side}={duty}, inside the dead band")


@pytest.mark.parametrize("case", [c for c in CASES
                                  if c["state_in"]["phase"] in decide.INHIBITED_PHASES],
                         ids=ids)
def test_inhibited_phases_never_enable(case):
    """G-1 and CO-3, checked across the corpus rather than case by case."""
    assert case["cmd_out"]["enable"] is False
    assert case["cmd_out"]["left"] == 0
    assert case["cmd_out"]["right"] == 0


@pytest.mark.parametrize("case", [c for c in CASES if c["obs"]["subject"] is None],
                         ids=ids)
def test_the_rover_never_drives_without_a_subject(case):
    """CO-4: the subject is the only positional reference.  With none visible
    the only legal motion is a spin - equal and opposite duty."""
    cmd = case["cmd_out"]
    # A spin, not a drive: the two sides oppose, differing only by their own
    # dead bands.  Anything else is translation, which CO-4 forbids here.
    assert cmd["left"] * cmd["right"] <= 0, \
        f"{case['name']} drives while the subject is not visible"
    assert abs(abs(cmd["left"]) - abs(cmd["right"])) <= 20, \
        f"{case['name']} arcs rather than spinning"
