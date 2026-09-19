"""session.py: the CO-3 state machine.

The headline requirement is the last line of CO-3: "every state carries a
timeout and every timeout has a defined exit - there is no state whose only
escape is a human touching the rover."  That is asserted here by construction
rather than trusted to the prose.
"""

import pytest

from app import goal as goal_lib
from app import session
from tests.conftest import framed_obs, framing_goal, make_obs, pose_goal


def run(state, cfg, signals=None, t=0.0, obs=None):
    return session.step(state, obs if obs is not None else make_obs(t=t),
                        cfg, t, signals or {})


def armed(cfg, phase, goal=None, now=0.0):
    st = session.initial_state(now)
    st["goal"] = goal or framing_goal()
    st["phase"] = phase
    st["t_phase"] = now
    return st


# ---------------------------------------------------------------------------
# CO-3  the happy path
# ---------------------------------------------------------------------------

def test_a_whole_run_reaches_capture(cfg):
    """Idle -> Listen -> Parse -> Confirm -> Arm -> Frame -> Settle -> Capture
    -> Idle, for a framing run with no pose goal."""
    st = session.initial_state(0.0)
    spec, _conf = goal_lib.parse("frame me next to the big tree", cfg)
    spec = dict(spec, locked=False)

    seen = [st["phase"]]

    st, _ev = run(st, cfg, {"wake": True}, 0.0)
    seen.append(st["phase"])
    st, _ev = run(st, cfg, {"transcript": "frame me next to the big tree"}, 1.0)
    seen.append(st["phase"])
    st, _ev = run(st, cfg, {"goal_spec": spec}, 1.1)
    seen.append(st["phase"])
    st, _ev = run(st, cfg, {}, 3.5)
    seen.append(st["phase"])

    # Arm -> Frame as soon as the subject is visible.
    st, _ev = run(st, cfg, {}, 3.6)
    seen.append(st["phase"])

    # Framing already correct, sweep already done: Frame -> Settle.
    st["sweep"] = {"phase": "done", "target_shoulder": 0.120}
    st, _ev = run(st, cfg, {}, 4.0, framed_obs(t=4.0))
    seen.append(st["phase"])

    # Settle -> Capture after a second of stillness.
    for i in range(1, 40):
        t = 4.0 + i * 0.1
        st, _ev = run(st, cfg, {}, t, framed_obs(t=t))
        if st["phase"] == session.CAPTURE:
            break
    seen.append(st["phase"])

    assert seen == [session.IDLE, session.LISTEN, session.PARSE, session.CONFIRM,
                    session.ARM, session.FRAME, session.SETTLE, session.CAPTURE], seen


def test_a_pose_run_goes_through_coach(cfg):
    st = armed(cfg, session.FRAME, pose_goal("point_at"))
    st["sweep"] = {"phase": "done", "target_shoulder": 0.120}
    st, _ev = run(st, cfg, {}, 1.0, framed_obs())
    assert st["phase"] == session.COACH


def test_a_framing_run_skips_coach(cfg):
    """CO-3: Coach is entered only when the locked goal has kind == "pose";
    a framing run goes straight from Frame to Settle."""
    st = armed(cfg, session.FRAME, framing_goal())
    st["sweep"] = {"phase": "done", "target_shoulder": 0.120}
    st, _ev = run(st, cfg, {}, 1.0, framed_obs())
    assert st["phase"] == session.SETTLE


# ---------------------------------------------------------------------------
# G-1  the lock
# ---------------------------------------------------------------------------

def test_the_goal_is_locked_exactly_at_confirm_to_arm(cfg):
    """G-1: no command with enable set is issued until the GoalSpec is locked,
    and CO-3 puts the lock at the end of the confirmation window."""
    st = session.initial_state(0.0)
    st["goal"] = framing_goal(locked=False)
    st["phase"] = session.CONFIRM
    st["t_phase"] = 0.0

    st, _ev = run(st, cfg, {}, 1.0)
    assert st["phase"] == session.CONFIRM
    assert not st["goal"]["locked"], "locked before the objection window closed"

    st, _ev = run(st, cfg, {}, 2.1)
    assert st["phase"] == session.ARM
    assert st["goal"]["locked"]


def test_an_objection_during_confirm_returns_to_listen(cfg):
    """G-7: a wrong parse must cost two seconds, not a whole run."""
    st = session.initial_state(0.0)
    st["goal"] = framing_goal(locked=False)
    st["phase"] = session.CONFIRM
    st, events = run(st, cfg, {"objection": True}, 0.5)
    assert st["phase"] == session.LISTEN
    assert st["goal"] is None
    assert any(e["kind"] == "listen" for e in events)


def test_a_rejected_goal_spec_never_locks(cfg):
    """G-5.2: tier-2 output is validated and rejected if it names a goal that
    does not exist.  An invalid spec must not reach Confirm."""
    st = session.initial_state(0.0)
    st["phase"] = session.PARSE
    st["transcript"] = "do something impossible"
    bad = framing_goal(kind="pose", pose_goal="jump")
    st, _ev = run(st, cfg, {"goal_spec": bad}, 0.5)
    assert st["phase"] != session.CONFIRM
    assert st.get("goal") is None


# ---------------------------------------------------------------------------
# G-8  the failure ladder
# ---------------------------------------------------------------------------

def test_not_understood_asks_once(cfg):
    st = session.initial_state(0.0)
    st["phase"] = session.PARSE
    st["transcript"] = "mumble"
    st, events = run(st, cfg, {"parse_failed": True}, 0.5)
    assert st["phase"] == session.LISTEN
    assert any("Sorry" in e.get("text", "") for e in events)


def test_still_not_understood_falls_back_to_plain_framing(cfg):
    st = session.initial_state(0.0)
    st["phase"] = session.PARSE
    st["transcript"] = "mumble"
    st, _ev = run(st, cfg, {"parse_failed": True}, 0.5)

    st["phase"] = session.PARSE
    st["t_phase"] = 1.0
    st["transcript"] = "mumble again"
    st, events = run(st, cfg, {"parse_failed": True,
                               "fallback_goal": framing_goal()}, 1.5)
    assert st["phase"] == session.CONFIRM
    assert any("frame you nicely" in e.get("text", "") for e in events), \
        "the fallback must be announced, never silent"


def test_no_usable_object_stays_in_idle_and_says_why(cfg):
    st = session.initial_state(0.0)
    st["phase"] = session.PARSE
    st["transcript"] = "point at the bird"
    st, _ev = run(st, cfg, {"parse_failed": True}, 0.5)

    st["phase"] = session.PARSE
    st["t_phase"] = 1.0
    st, events = run(st, cfg, {"parse_failed": True, "fallback_goal": None}, 1.5)
    assert st["phase"] == session.IDLE
    assert any("can't see" in e.get("text", "") for e in events)


# ---------------------------------------------------------------------------
# CO-4  anchor loss
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("phase", ["arm", "frame", "coach", "settle", "capture"])
def test_losing_the_subject_halts_from_any_active_phase(cfg, phase):
    st = armed(cfg, phase, pose_goal())
    obs = make_obs(subject=None, lost_for=0.6)
    st, _ev = run(st, cfg, {}, 1.0, obs)
    assert st["phase"] == session.HALTED


def test_halt_search_give_up_ladder(cfg):
    """CO-4: Halted stops the motors within 500 ms, Search is a slow spin
    toward the last known bearing, and after 8 s the rover gives up and says
    so - rather than driving around a field looking for someone."""
    st = armed(cfg, session.FRAME, framing_goal())

    st, _ev = run(st, cfg, {}, 1.0, make_obs(subject=None, lost_for=0.6))
    assert st["phase"] == session.HALTED

    st, _ev = run(st, cfg, {}, 3.0, make_obs(subject=None, lost_for=2.5))
    assert st["phase"] == session.SEARCH

    st, events = run(st, cfg, {}, 10.0, make_obs(subject=None, lost_for=8.5))
    assert st["phase"] == session.IDLE
    assert any("lost you" in e.get("text", "") for e in events)


def test_re_acquiring_returns_to_framing(cfg):
    st = armed(cfg, session.SEARCH, framing_goal())
    st, _ev = run(st, cfg, {}, 1.0, make_obs(subject=(0.5, 0.55, 0.18, 0.62)))
    assert st["phase"] == session.FRAME


# ---------------------------------------------------------------------------
# PG-5  sequencing
# ---------------------------------------------------------------------------

def test_coach_returns_to_frame_at_most_once(cfg):
    """PG-5: at most one return to framing if the subject's movement
    invalidated it.  Without the cap the two loops ping-pong forever."""
    st = armed(cfg, session.COACH, pose_goal())
    st["reframes"] = 0
    bad_framing = make_obs(subject=(0.05, 0.55, 0.18, 0.62),
                           obj=(0.95, 0.40, 0.14, 0.32), shoulder_px=0.30)

    st, _ev = run(st, cfg, {}, 1.0, bad_framing)
    assert st["phase"] == session.FRAME
    assert st["reframes"] == 1

    st["phase"] = session.COACH
    st["t_phase"] = 2.0
    st, _ev = run(st, cfg, {}, 3.0, bad_framing)
    assert st["phase"] != session.FRAME, "reframed twice; PG-5 caps it at one"


# ---------------------------------------------------------------------------
# CO-5  cancel
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("phase", list(session.ALL_PHASES))
def test_cancel_works_from_every_state(cfg, phase):
    """CO-5: a spoken or gestured cancel returns to Idle and zeroes the motors
    from any state, including mid-manoeuvre."""
    st = armed(cfg, phase, framing_goal())
    st, events = run(st, cfg, {"cancel": True}, 1.0)
    assert st["phase"] == session.IDLE
    assert st["goal"] is None
    assert any("Cancelled" in e.get("text", "") for e in events)


# ---------------------------------------------------------------------------
# the CO-3 headline requirement
# ---------------------------------------------------------------------------

def test_every_state_has_a_timeout(cfg):
    for phase in session.ALL_PHASES:
        timeout = session.timeout_for(phase, cfg)
        assert timeout is not None, f"{phase} has no timeout"
        if phase != session.IDLE:
            assert timeout < 1e6, f"{phase}'s timeout is effectively infinite"


def test_no_state_is_a_dead_end(cfg):
    """CO-3's closing promise: there is no state whose only escape is a human
    touching the rover.

    Every phase is put in the worst case it can face - nothing visible, no
    signals, nothing to do - and then run well past its timeout.  Every one
    must end up somewhere else.  Idle is excluded: resting is its job.
    """
    stuck = []
    for phase in session.ALL_PHASES:
        if phase == session.IDLE:
            continue
        st = armed(cfg, phase, framing_goal(), now=0.0)
        timeout = session.timeout_for(phase, cfg)

        escaped = False
        for i in range(1, 400):
            t = i * (timeout / 20.0 if timeout else 0.5)
            obs = make_obs(t=t, subject=None, obj=None, lost_for=t)
            st, _ev = session.step(st, obs, cfg, t, {})
            if st["phase"] != phase:
                escaped = True
                break
        if not escaped:
            stuck.append(phase)

    assert not stuck, (
        f"these phases never exited on their own: {stuck}. CO-3 requires every "
        f"timeout to have a defined exit.")


def test_transitions_are_recorded_for_the_log(cfg):
    """LG-1 logs state transitions; they have to exist to be logged."""
    st = session.initial_state(0.0)
    st, _ev = run(st, cfg, {"wake": True}, 0.0)
    assert st["transitions"][-1]["from"] == session.IDLE
    assert st["transitions"][-1]["to"] == session.LISTEN
    assert st["transitions"][-1]["why"]


def test_transition_history_is_bounded(cfg):
    """A long session must not grow the state dict without limit - it is
    copied on every tick."""
    st = session.initial_state(0.0)
    for i in range(200):
        st, _ev = run(st, cfg, {"cancel": True}, float(i))
        st["phase"] = session.FRAME
    assert len(st["transitions"]) <= 64


def test_motors_are_inhibited_in_the_quiet_phases():
    for phase in (session.IDLE, session.LISTEN, session.PARSE,
                  session.CONFIRM, session.HALTED):
        assert not session.phase_allows_motion(phase)
    for phase in (session.ARM, session.FRAME, session.COACH,
                  session.SETTLE, session.CAPTURE, session.SEARCH):
        assert session.phase_allows_motion(phase)
