"""session.py - PURE.  The CO-3 state machine.

Imports ``math`` and its pure siblings.  Kept pure for the same reason as
``decide.py``: the phase logic is where deadlocks hide, and a deadlock found by
a unit test at a desk costs a minute, while one found in a field costs an
afternoon and a flat battery.

    [*] --> Idle
    Idle     --> Listen : wake gesture or tap
    Listen   --> Parse  : utterance ends
    Parse    --> Listen : not understood
    Parse    --> Confirm: GoalSpec built
    Confirm  --> Arm    : no objection in 2s
    Confirm  --> Listen : objection
    Arm      --> Frame  : subject in place
    Frame    --> Coach  : framing locked, pose goal
    Frame    --> Settle : framing locked, no pose goal
    Coach    --> Frame  : subject moved too far
    Coach    --> Settle : pose in tolerance
    Settle   --> Capture: 1s still
    Capture  --> Idle
    Frame    --> Halted : subject lost
    Coach    --> Halted : subject lost
    Halted   --> Search : lost 2s
    Search   --> Frame  : re-acquired
    Search   --> Idle   : lost 8s, speak

CO-3: every state carries a timeout and every timeout has a defined exit -
there is no state whose only escape is a human touching the rover.  The test
``test_session.py::test_no_state_is_a_dead_end`` enforces that by construction.
"""

import math

from app import coach as coach_lib
from app import decide as decide_lib
from app import goal as goal_lib

IDLE = "idle"
LISTEN = "listen"
PARSE = "parse"
CONFIRM = "confirm"
ARM = "arm"
FRAME = "frame"
COACH = "coach"
SETTLE = "settle"
CAPTURE = "capture"
HALTED = "halted"
SEARCH = "search"

ALL_PHASES = (IDLE, LISTEN, PARSE, CONFIRM, ARM, FRAME, COACH,
              SETTLE, CAPTURE, HALTED, SEARCH)

# Phases in which the rover is allowed to be looking after a subject at all.
ACTIVE_PHASES = (ARM, FRAME, COACH, SETTLE, CAPTURE, HALTED, SEARCH)


def initial_state(now=0.0):
    st = decide_lib.initial_state(now, goal=None, phase=IDLE)
    st["t_phase"] = now
    st["transitions"] = []
    st["announce"] = None
    st["framing_locked_since"] = None
    st["settle_still_since"] = None
    st["listen_attempts"] = 0
    return st


def _enter(state, phase, now, reason, announce=None):
    st = dict(state)
    st["phase"] = phase
    st["t_phase"] = now
    st["announce"] = announce
    st["transitions"] = list(state.get("transitions", []))[-31:] + [
        {"t": now, "from": state.get("phase"), "to": phase, "why": reason}
    ]
    # Anything that is only meaningful inside a phase is cleared on the way in.
    if phase in (IDLE, LISTEN):
        st["sweep"] = None
        st["spacing_hist"] = []
        st["framing_locked_since"] = None
        st["settle_still_since"] = None
        st["capture_frames"] = 0
    if phase == COACH:
        st["coach"] = coach_lib.initial_state(now)
    if phase == SETTLE:
        st["settle_still_since"] = None
    return st


def framing_locked(obs, state, config):
    """Is the framing good enough to stop driving?

    Both controlled axes inside their deadbands, and - because D-8 asks for a
    full stop before any measurement used for a final decision - the sweep
    finished or disabled.
    """
    subject = obs.get("subject")
    if subject is None:
        return False

    sweep = state.get("sweep") or {}
    if config.get("sweep", {}).get("enabled", True) and obs.get("object") is not None:
        if sweep.get("phase") not in ("done", "failed", None):
            return False

    ccfg = config.get("control", {})
    comp = decide_lib._composition(state, config)
    obj = obs.get("object")

    if obj is not None:
        measured = 0.5 * (subject["cx"] + obj["cx"])
        target = 0.5 * (comp.get("subject_x", 0.333) + comp.get("object_x", 0.667))
    else:
        measured = subject["cx"]
        target = comp.get("subject_x", 0.333)
    if abs(measured - target) >= ccfg.get("yaw", {}).get("deadband", 0.02) * 1.5:
        return False

    _u, dist_err, _target = decide_lib.distance_command(obs, state, config, 1.0)
    if dist_err is not None and abs(dist_err) >= ccfg.get("dist", {}).get("deadband", 0.03) * 1.5:
        return False

    eye = decide_lib.eyeline_y(obs, config)
    if eye is not None:
        vfov = config.get("camera", {}).get("vfov_deg", 50.5)
        tilt_err_deg = abs(eye - comp.get("eyeline_y", 0.333)) * vfov
        if tilt_err_deg >= ccfg.get("tilt", {}).get("deadband", 0.5) * 3.0:
            return False
    return True


def subject_still(obs, state, config):
    """Has the subject stopped moving?  Judged on ``shoulder_px`` and box
    centre, both of which are absolute per-frame measurements."""
    subject = obs.get("subject")
    if subject is None:
        return False
    last = state.get("still_ref")
    if last is None:
        return False
    dx = abs(subject["cx"] - last[0])
    dy = abs(subject["cy"] - last[1])
    ds = abs((obs.get("shoulder_px") or 0.0) - last[2])
    return dx < 0.02 and dy < 0.02 and ds < 0.01


def step(state, obs, config, now, signals=None):
    """Advance the session by one control tick.

    ``signals`` carries everything that arrives from outside the vision loop:

        {"wake": bool, "tap": bool, "transcript": str|None,
         "goal_spec": dict|None, "objection": bool, "cancel": bool,
         "parse_failed": bool}

    Returns ``(new_state, events)`` where ``events`` is a list of
    ``{"kind": ..., ...}`` for the caller to act on - ``say``, ``capture``,
    ``listen``, ``parse``.  Nothing is mutated in place.
    """
    sig = signals or {}
    st = dict(state)
    events = []
    phase = st.get("phase", IDLE)
    elapsed = now - st.get("t_phase", now)

    gcfg = config.get("goal", {})
    scfg = config.get("safety", {})
    sscfg = config.get("session", {})
    lost_for = obs.get("lost_for", 0.0) if obs else 0.0

    # CO-5: a spoken or gestured cancel returns to Idle and zeroes the motors
    # from any state, including mid-manoeuvre.  Checked before anything else.
    if sig.get("cancel"):
        st = _enter(st, IDLE, now, "cancelled")
        st["goal"] = None
        return st, [{"kind": "say", "text": "Cancelled."}]

    # ------------------------------------------------------------------
    if phase == IDLE:
        # CO-5: a run starts when the subject wakes the rover - a raised hand
        # held one second, or a tap before walking away.
        if sig.get("wake") or sig.get("tap"):
            st = _enter(st, LISTEN, now, "wake")
            st["listen_attempts"] = 0
            events.append({"kind": "listen"})
            events.append({"kind": "say", "text": "Listening."})
        return st, events

    if phase == LISTEN:
        transcript = sig.get("transcript")
        if transcript:
            st = _enter(st, PARSE, now, "utterance ended")
            st["transcript"] = transcript
            events.append({"kind": "parse", "transcript": transcript})
            return st, events
        if elapsed > gcfg.get("listen_timeout_s", 8.0):
            # G-8's last rung: no usable input, so stay out of everyone's way.
            st = _enter(st, IDLE, now, "listen timeout")
            events.append({"kind": "say", "text": "I didn't catch that. Tap me when you're ready."})
        return st, events

    if phase == PARSE:
        spec = sig.get("goal_spec")
        if spec is not None:
            ok, _reason = goal_lib.validate(spec)
            if ok:
                locked = dict(spec)
                locked["locked"] = False   # locked only after confirmation
                st["goal"] = locked
                st = _enter(st, CONFIRM, now, "goal built")
                events.append({"kind": "say",
                               "text": goal_lib.confirmation_line(locked)})
                return st, events

        # G-8 failure ladder: not understood -> ask once; still not understood
        # -> plain framing on the largest confident detection, announced.
        if sig.get("parse_failed") or spec is not None:
            attempts = st.get("listen_attempts", 0) + 1
            st["listen_attempts"] = attempts
            if attempts == 1:
                st = _enter(st, LISTEN, now, "not understood")
                events.append({"kind": "say", "text": "Sorry - what should I frame you with?"})
                events.append({"kind": "listen"})
            else:
                fallback = sig.get("fallback_goal")
                if fallback is None:
                    st = _enter(st, IDLE, now, "no usable object")
                    events.append({"kind": "say",
                                   "text": "I can't see anything to frame you with. Tap me to try again."})
                else:
                    st["goal"] = fallback
                    st = _enter(st, CONFIRM, now, "fallback to plain framing")
                    events.append({"kind": "say",
                                   "text": "I'll just frame you nicely. " +
                                           goal_lib.confirmation_line(fallback)})
            return st, events

        if elapsed > gcfg.get("tier2_timeout_s", 5.0) + 1.0:
            st = _enter(st, LISTEN, now, "parse timeout")
            events.append({"kind": "listen"})
        return st, events

    if phase == CONFIRM:
        # G-7: wait confirm_window for an objection.  A wrong parse must cost
        # two seconds, not a whole run.
        if sig.get("objection"):
            st = _enter(st, LISTEN, now, "objection")
            st["goal"] = None
            events.append({"kind": "say", "text": "Okay - tell me again."})
            events.append({"kind": "listen"})
            return st, events
        if elapsed >= gcfg.get("confirm_window", 2.0):
            # G-1: no command with enable set is issued until the GoalSpec is
            # locked.  This assignment IS that lock.
            locked = dict(st.get("goal") or {})
            locked["locked"] = True
            locked["locked_t"] = now
            st["goal"] = locked
            st = _enter(st, ARM, now, "confirmed")
        return st, events

    # ---- from here on the motors may run --------------------------------
    if phase in (ARM, FRAME, COACH, SETTLE, CAPTURE):
        # CO-4: anchor loss is a first-class state.  Halted stops the motors
        # within 500 ms; the rover must never drive while it cannot see the
        # subject.
        if lost_for > scfg.get("lost_halt_s", 0.5):
            st = _enter(st, HALTED, now, "subject lost")
            return st, events

    if phase == ARM:
        if obs and obs.get("subject") is not None:
            st = _enter(st, FRAME, now, "subject in place")
            return st, events
        if elapsed > sscfg.get("arm_timeout_s", 30.0):
            st = _enter(st, IDLE, now, "arm timeout")
            events.append({"kind": "say", "text": "I can't see you - tap me when you're set."})
        return st, events

    if phase == FRAME:
        if framing_locked(obs, st, config):
            goal = st.get("goal") or {}
            if goal.get("kind") == "pose":
                st = _enter(st, COACH, now, "framing locked, pose goal")
            else:
                st = _enter(st, SETTLE, now, "framing locked, no pose goal")
            st["framing_locked_since"] = now
            return st, events
        if elapsed > sscfg.get("frame_timeout_s", 45.0):
            # Take what we have rather than circling forever.
            st = _enter(st, SETTLE, now, "frame timeout")
            events.append({"kind": "say", "text": "Close enough - hold still."})
        return st, events

    if phase == COACH:
        perr = st.get("pose_error")

        # PG-5: at most one return to framing, if the subject's movement
        # invalidated it.  Without the cap the two loops ping-pong.
        if not framing_locked(obs, st, config):
            if st.get("reframes", 0) < sscfg.get("max_reframes", 1):
                st["reframes"] = st.get("reframes", 0) + 1
                st = _enter(st, FRAME, now, "subject moved too far")
                return st, events

        if perr is not None and perr.get("in_tolerance") and \
                coach_lib.countdown_complete(st.get("coach") or {}):
            st = _enter(st, SETTLE, now, "pose in tolerance")
            return st, events

        if elapsed > config.get("coach", {}).get("coach_timeout", 20.0) + 2.0:
            st = _enter(st, SETTLE, now, "coach timeout")
        return st, events

    if phase == SETTLE:
        subject = obs.get("subject") if obs else None
        if subject is not None:
            ref = (subject["cx"], subject["cy"], obs.get("shoulder_px") or 0.0)
            if subject_still(obs, st, config):
                if st.get("settle_still_since") is None:
                    st["settle_still_since"] = now
                elif now - st["settle_still_since"] >= config.get("control", {}).get("settle_still_s", 1.0):
                    st = _enter(st, CAPTURE, now, "1s still")
                    st["still_ref"] = ref
                    return st, events
            else:
                st["settle_still_since"] = None
            st["still_ref"] = ref

        if elapsed > sscfg.get("settle_timeout_s", 8.0):
            st = _enter(st, CAPTURE, now, "settle timeout")
        return st, events

    if phase == CAPTURE:
        burst = config.get("coach", {}).get("burst_frames", 7)
        if st.get("capture_frames", 0) >= burst or \
                elapsed > sscfg.get("capture_timeout_s", 4.0):
            st = _enter(st, IDLE, now, "captured")
            st["goal"] = None
            events.append({"kind": "captured"})
            events.append({"kind": "say", "text": "Got it."})
        return st, events

    if phase == HALTED:
        if obs and obs.get("subject") is not None:
            st = _enter(st, FRAME, now, "re-acquired")
            return st, events
        if lost_for > scfg.get("lost_search_s", 2.0):
            st = _enter(st, SEARCH, now, "lost 2s")
        return st, events

    if phase == SEARCH:
        if obs and obs.get("subject") is not None:
            st = _enter(st, FRAME, now, "re-acquired")
            return st, events
        if lost_for > scfg.get("lost_give_up_s", 8.0):
            st = _enter(st, IDLE, now, "lost 8s")
            st["goal"] = None
            events.append({"kind": "say", "text": "I've lost you - tap me to start again."})
        return st, events

    # Unknown phase: the only safe place to be is Idle.
    return _enter(st, IDLE, now, "unknown phase"), events


def phase_allows_motion(phase):
    """CO-3: motors are inhibited in Idle, Listen, Parse and Confirm."""
    return phase not in (IDLE, LISTEN, PARSE, CONFIRM, HALTED)


def timeout_for(phase, config):
    """Every state carries a timeout.  Exposed so a test can assert that none
    is missing rather than trusting the prose above."""
    gcfg = config.get("goal", {})
    scfg = config.get("safety", {})
    sscfg = config.get("session", {})
    return {
        IDLE: math.inf,          # Idle is the resting state; it needs no exit
        LISTEN: gcfg.get("listen_timeout_s", 8.0),
        PARSE: gcfg.get("tier2_timeout_s", 5.0) + 1.0,
        CONFIRM: gcfg.get("confirm_window", 2.0),
        ARM: sscfg.get("arm_timeout_s", 30.0),
        FRAME: sscfg.get("frame_timeout_s", 45.0),
        COACH: config.get("coach", {}).get("coach_timeout", 20.0) + 2.0,
        SETTLE: sscfg.get("settle_timeout_s", 8.0),
        CAPTURE: sscfg.get("capture_timeout_s", 4.0),
        HALTED: scfg.get("lost_search_s", 2.0),
        SEARCH: scfg.get("lost_give_up_s", 8.0),
    }.get(phase)
