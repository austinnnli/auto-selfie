"""coach.py - PURE.  A pose error in, words out, with rate limiting.

Imports ``math`` and nothing else.

Whether the pose feature is liked or switched off is decided entirely here, not
by measurement accuracy (D3e preamble).  A system that measures to a tenth of a
degree and says "up... down... up..." is worse than one that measures coarsely
and speaks like a person.

CO-2, all eight rules, are implemented here and each has a test:

  1. one instruction at a time; fix the largest error, re-evaluate, speak again
  2. at most one utterance per ``min_speak_interval``
  3. hysteresis: once inside tolerance, widen the zone before speaking again
  4. directions from the SUBJECT'S perspective - their left is image-right
  5. countdown for the final hold
  6. capture a burst in tolerance
  7. give up at ``coach_timeout`` and take the best frame
  8. a cue must reflect a measurement no more than 500 ms old
"""

import math

# Phrase bands (CO-1).  Degrees.
BAND_LARGE = 20.0
BAND_MEDIUM = 8.0
BAND_SMALL = 3.0


def initial_state(now=0.0):
    """The coach's slice of the session state.  Plain dict, no globals."""
    return {
        "last_say_t": -1e9,
        "last_say": None,
        "in_tolerance": False,   # hysteresis latch (CO-2.3)
        "hold_since": None,
        "countdown_next": None,
        "started_t": now,
        "spoken_count": 0,
        "gave_up": False,
    }


# ---------------------------------------------------------------------------
# CO-2.4  the mirror
# ---------------------------------------------------------------------------

def subject_direction(image_direction):
    """Translate an image-space direction into the subject's own frame.

    THE most common bug in this class of feature, and it destroys trust on the
    first run.  The phone is rear-facing and pointed AT the subject, so the
    subject's left hand appears on the right-hand side of the image.  Telling
    someone to "move right" when the image needs them to move image-right sends
    them the wrong way every single time.

    Vertical directions are unaffected - gravity is shared.
    """
    if image_direction == "image_left":
        return "your right"
    if image_direction == "image_right":
        return "your left"
    if image_direction == "image_up":
        return "up"
    if image_direction == "image_down":
        return "down"
    return image_direction


def mirror_limb(limb):
    """``left_arm`` is the SUBJECT'S left arm, so it is spoken as "left"."""
    return "left" if limb == "left_arm" else "right"


# ---------------------------------------------------------------------------
# CO-1  error to phrase
# ---------------------------------------------------------------------------

def _vertical_words(error):
    """Positive error means raise the hand (see pose.point_at_error)."""
    return ("up", "higher") if error > 0 else ("down", "lower")


def phrase_for_angle(error, tolerance, limb=None):
    """CO-1 band table, for an angular error in degrees."""
    mag = abs(error)
    up_word, comparative = _vertical_words(error)
    arm = f"your {mirror_limb(limb)} arm" if limb else "your arm"

    if mag > BAND_LARGE:
        return f"{'Raise' if error > 0 else 'Lower'} {arm} a lot"
    if mag > BAND_MEDIUM:
        return f"A bit {comparative}"
    if mag > max(BAND_SMALL, tolerance):
        return f"Almost - tiny bit {up_word}"
    return None   # inside tolerance; the caller moves to the hold countdown


def phrase_for_lateral(signed_error, tolerance):
    """Horizontal errors, spoken in the subject's frame (CO-2.4).

    ``signed_error`` is in image space: positive means the subject needs to
    move toward +x, which is image-right, which is the subject's LEFT.
    """
    mag = abs(signed_error)
    image_dir = "image_right" if signed_error > 0 else "image_left"
    spoken = subject_direction(image_dir)

    if mag > 4 * tolerance:
        return f"Take a big step to {spoken}"
    if mag > 2 * tolerance:
        return f"A step to {spoken}"
    if mag > tolerance:
        return f"Almost - a little to {spoken}"
    return None


def phrase_for(perr, cfg):
    """One phrase for one measured error, or None when inside tolerance."""
    if perr is None:
        return None

    # Checked before measurability: a foreshortened arm has no angle to
    # report, but it does have a remedy (PG-3).
    if perr.get("remedy") == "turn_side_on":
        # PG-3: the angle cannot be trusted, so do not coach an angle.
        return "Turn side-on to me a little"

    if not perr.get("measurable"):
        return None

    error = perr["error"]
    tol = perr.get("tolerance", 0.0)

    if perr["goal"] == "no_overlap":
        if error <= tol:
            return None
        return f"Step to {subject_direction('image_right' if perr.get('signed', 1) > 0 else 'image_left')}"

    if perr["unit"] == "deg":
        if perr["axis"] == "horizontal":
            # look_at: the head turns, it does not translate.
            if abs(error) <= tol:
                return None
            spoken = subject_direction("image_right" if error > 0 else "image_left")
            return ("Turn your head a lot to " + spoken) if abs(error) > BAND_LARGE \
                else ("Look a little to " + spoken)
        return phrase_for_angle(error, tol, perr.get("limb"))

    signed = perr.get("signed", error)
    return phrase_for_lateral(signed, tol)


# ---------------------------------------------------------------------------
# the speaking gate
# ---------------------------------------------------------------------------

def should_speak(state, now, cfg, urgent=False):
    """CO-2.2: at most one utterance per ``min_speak_interval``.  Faster is
    unusable - the subject is still acting on the previous cue."""
    interval = cfg.get("min_speak_interval", 1.8)
    if urgent:
        interval = min(interval, 0.6)
    return (now - state["last_say_t"]) >= interval


def _tolerance_with_hysteresis(perr, state, cfg):
    """CO-2.3.  Once inside tolerance, the zone widens, so a subject hovering
    on the boundary hears nothing instead of "up... down... up..." forever."""
    tol = perr.get("tolerance", 0.0)
    if state.get("in_tolerance"):
        return tol * cfg.get("hysteresis", 1.6)
    return tol


def coach(perr, state, cfg, now, obs_age=0.0):
    """Turn the current pose error into (phrase_or_None, new_state).

    Pure: the caller owns the clock and the state dict; nothing is mutated in
    place and nothing is read from outside the arguments.

    ``obs_age`` is how old the measurement behind ``perr`` is, in seconds.
    """
    st = dict(state)

    # CO-2.8: a cue must reflect a measurement no more than 500 ms old, or the
    # subject corrects against stale information and overshoots.
    if obs_age > cfg.get("max_cue_age_s", 0.5):
        return (None, st)

    # CO-2.7: give up at coach_timeout.  An endless correction loop is worse
    # than an imperfect photo.
    elapsed = now - st.get("started_t", now)
    if elapsed > cfg.get("coach_timeout", 20.0):
        if not st.get("gave_up"):
            st["gave_up"] = True
            st["last_say_t"] = now
            st["last_say"] = "That's good enough - taking it now"
            return (st["last_say"], st)
        return (None, st)

    # PG-3: a foreshortened arm has NO trustworthy angle, so pose.py returns
    # the turn_side_on remedy with error None.  That must still be spoken -
    # checking measurability first would swallow the one cue that can actually
    # fix the situation, and the subject would stand there being told nothing
    # while the rover waited for an angle it can never measure.
    if perr is not None and perr.get("remedy") == "turn_side_on":
        phrase = phrase_for(perr, cfg)
        if phrase and should_speak(st, now, cfg) and phrase != st.get("last_say"):
            st["last_say"] = phrase
            st["last_say_t"] = now
            st["spoken_count"] = st.get("spoken_count", 0) + 1
            return (phrase, st)
        return (None, st)

    if perr is None or not perr.get("measurable"):
        # Nothing to say about a measurement we do not have (P-7).
        return (None, st)

    tol = _tolerance_with_hysteresis(perr, state, cfg)
    inside = abs(perr["error"]) <= tol

    if inside:
        if not st["in_tolerance"]:
            st["in_tolerance"] = True
            st["hold_since"] = now
            st["countdown_next"] = cfg.get("countdown_from", 3)
        return _countdown(st, cfg, now)

    # Left tolerance again - drop the latch and resume coaching.
    st["in_tolerance"] = False
    st["hold_since"] = None
    st["countdown_next"] = None

    phrase = phrase_for(perr, cfg)
    if phrase is None:
        return (None, st)

    # CO-2.1: one instruction at a time.  `perr` is already the single largest
    # error (pose.evaluate picks it), so there is nothing to compound here -
    # but never repeat the identical phrase back to back either, which reads
    # as the system not noticing the person moved.
    if not should_speak(st, now, cfg):
        return (None, st)
    if phrase == st.get("last_say") and (now - st["last_say_t"]) < 2 * cfg.get("min_speak_interval", 1.8):
        return (None, st)

    st["last_say"] = phrase
    st["last_say_t"] = now
    st["spoken_count"] = st.get("spoken_count", 0) + 1
    return (phrase, st)


def _countdown(st, cfg, now):
    """CO-2.5: "Hold - three, two, one." then the capture."""
    n = st.get("countdown_next")
    if n is None:
        return (None, st)

    # One number per second, and the first one comes immediately on entering
    # tolerance so the subject knows the hold has started.
    #
    # `or now` would be wrong here: a hold that began at t = 0.0 is falsy, and
    # the countdown would stall on "three" forever.  Be explicit.
    hold_since = st.get("hold_since")
    since = now - (hold_since if hold_since is not None else now)
    expected_index = cfg.get("countdown_from", 3) - n
    if since + 1e-9 < expected_index * 1.0:
        return (None, st)

    words = {3: "Hold - three", 2: "two", 1: "one"}
    phrase = words.get(n, str(n))
    st["countdown_next"] = n - 1 if n > 1 else None
    st["last_say"] = phrase
    st["last_say_t"] = now
    return (phrase, st)


def countdown_complete(state):
    """True once the last number has been spoken - the session's cue to fire."""
    return state.get("in_tolerance") and state.get("countdown_next") is None


def announce(state, text, now, cfg, urgent=False):
    """Speak a line that is not a pose cue (a confirmation, a feasibility
    suggestion, a give-up notice), through the same rate limiter so nothing
    can bypass CO-2.2."""
    st = dict(state)
    if not should_speak(st, now, cfg, urgent=urgent):
        return (None, st)
    if text == st.get("last_say") and (now - st["last_say_t"]) < 6.0:
        return (None, st)
    st["last_say"] = text
    st["last_say_t"] = now
    return (text, st)


def pick_best_frame(candidates):
    """CO-2.6: capture a burst of 5-10 frames in tolerance, pick the sharpest
    with the best pose score.

    ``candidates`` is a list of ``{"sharpness": float, "pose_error": float,
    "index": int}``.  Sharpness is normalised by the best in the burst so the
    two terms are comparable; pose error is normalised by the worst.
    """
    if not candidates:
        return None
    best_sharp = max(1e-9, max(c.get("sharpness", 0.0) for c in candidates))
    worst_err = max(1e-9, max(abs(c.get("pose_error", 0.0)) for c in candidates))

    def score(c):
        sharp = c.get("sharpness", 0.0) / best_sharp
        err = abs(c.get("pose_error", 0.0)) / worst_err
        return 0.6 * sharp + 0.4 * (1.0 - err)

    return max(candidates, key=score)
