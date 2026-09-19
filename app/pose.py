"""pose.py - PURE.  Keypoints and an object position in, a signed error and a
suggested remedy out.

Imports ``math`` and nothing else.  No imaging library, no I/O, no globals.

The detector already returns 17 keypoints in the pass that finds the person, so
the added compute cost here is effectively zero - this module is geometry, not
perception (D3d preamble).

Everything is in normalised image coordinates, 0..1, with **y increasing
downward**.  That sign convention is load-bearing: see ``point_at_error``.

PG-6: the pose goal is not chosen here.  It arrives in the locked GoalSpec,
parsed from what the user said before any movement.  This module only measures
error against a goal already fixed.
"""

import math

# COCO-17 keypoint names, as emitted by YOLO-pose.
NOSE = "nose"
LEFT_EYE, RIGHT_EYE = "left_eye", "right_eye"
LEFT_EAR, RIGHT_EAR = "left_ear", "right_ear"
LEFT_SHOULDER, RIGHT_SHOULDER = "left_shoulder", "right_shoulder"
LEFT_ELBOW, RIGHT_ELBOW = "left_elbow", "right_elbow"
LEFT_WRIST, RIGHT_WRIST = "left_wrist", "right_wrist"
LEFT_HIP, RIGHT_HIP = "left_hip", "right_hip"
LEFT_KNEE, RIGHT_KNEE = "left_knee", "right_knee"
LEFT_ANKLE, RIGHT_ANKLE = "left_ankle", "right_ankle"

# Remedy classes (PG-4).  "rover" and "person" are the two real branches;
# the rest are specific instructions that only a person can carry out.
REMEDY_ROVER = "rover"
REMEDY_PERSON = "person"
REMEDY_TURN_SIDE_ON = "turn_side_on"
REMEDY_NONE = "none"

# Units an error can carry.
UNIT_DEG = "deg"
UNIT_FRAC = "frac"        # fraction of frame width
UNIT_OVERLAP = "overlap"  # fraction of the subject box covered

# Arm templates for the whole-body goals (PG-1, "joint angles against a
# template").  Angles are the shoulder->elbow direction measured in image
# space, degrees, 0 = pointing right along +x, negative = upward.
ARM_TEMPLATES = {
    "arms_out": {"left": 0.0, "right": 180.0},
    "arms_down": {"left": 90.0, "right": 90.0},
    "hands_hips": {"left": 55.0, "right": 125.0},
}

GOAL_IDS = (
    "point_at", "hold", "look_at", "beside",
    "arms_out", "arms_down", "hands_hips", "no_overlap",
)


# ---------------------------------------------------------------------------
# small geometry helpers
# ---------------------------------------------------------------------------

def wrap_deg(angle):
    """Wrap to [-180, 180).  Every angular error in this module goes through it."""
    return (angle + 180.0) % 360.0 - 180.0


def _dist(ax, ay, bx, by):
    return math.hypot(bx - ax, by - ay)


def keypoint(keypoints, name, min_conf):
    """Return ``(x, y)`` for a keypoint, or ``None`` if it is missing or weak.

    P-7: perception never substitutes a guess for a missing measurement, and
    neither does this module.  A ``None`` here propagates up as an
    ``unmeasurable`` result, and the decision layer chooses what to do.
    """
    if not keypoints:
        return None
    kp = keypoints.get(name)
    if kp is None:
        return None
    x, y, conf = kp[0], kp[1], kp[2]
    if conf < min_conf:
        return None
    return (x, y)


def midpoint(a, b):
    if a is None or b is None:
        return None
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)


def result(goal, error=None, unit=UNIT_DEG, tolerance=0.0, remedy=REMEDY_NONE,
           axis=None, detail=""):
    """The single shape every goal function returns.

    error       signed; ``None`` means "could not measure"
    unit        UNIT_DEG / UNIT_FRAC / UNIT_OVERLAP
    tolerance   from config, carried along so coach.py needs no config lookup
    remedy      REMEDY_* - who fixes this, the rover or the person (PG-4)
    axis        "vertical" | "horizontal" | None - which way the fix goes,
                expressed in IMAGE space.  coach.py mirrors it for speech.
    detail      free text for logs; never spoken
    """
    ok = error is not None and abs(error) <= tolerance
    return {
        "goal": goal,
        "error": error,
        "unit": unit,
        "tolerance": tolerance,
        "in_tolerance": ok,
        "measurable": error is not None,
        "remedy": REMEDY_NONE if ok else remedy,
        "axis": axis,
        "detail": detail,
    }


# ---------------------------------------------------------------------------
# limb selection (G-4)
# ---------------------------------------------------------------------------

def choose_limb(keypoints, obj, explicit, min_conf):
    """Pick the arm to point/hold with.

    G-4: an explicit limb ("with your left hand") wins; otherwise choose the arm
    on the same side as the object, since pointing across the body reads badly
    and foreshortens.

    "Same side" is decided from the keypoints themselves rather than from an
    assumption about which way the subject faces: whichever wrist sits nearer
    the object horizontally is the near arm.  A subject who has turned round
    therefore still gets the right answer.
    """
    if explicit in ("left_arm", "right_arm"):
        return explicit
    if obj is None:
        return "right_arm"

    lw = keypoint(keypoints, LEFT_WRIST, min_conf)
    rw = keypoint(keypoints, RIGHT_WRIST, min_conf)
    ls = keypoint(keypoints, LEFT_SHOULDER, min_conf)
    rs = keypoint(keypoints, RIGHT_SHOULDER, min_conf)

    left_ref = lw or ls
    right_ref = rw or rs
    if left_ref is None and right_ref is None:
        return "right_arm"
    if left_ref is None:
        return "right_arm"
    if right_ref is None:
        return "left_arm"

    if abs(left_ref[0] - obj["cx"]) <= abs(right_ref[0] - obj["cx"]):
        return "left_arm"
    return "right_arm"


def _limb_points(keypoints, limb, min_conf):
    if limb == "left_arm":
        names = (LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST)
    else:
        names = (RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST)
    return tuple(keypoint(keypoints, n, min_conf) for n in names)


# ---------------------------------------------------------------------------
# PG-2  pointing
# ---------------------------------------------------------------------------

def point_at_error(elbow, wrist, obj_cx, obj_cy):
    """Signed pointing error in degrees.  Positive means RAISE the hand.

        theta_arm    = atan2(w_y - e_y, w_x - e_x)
        theta_target = atan2(o_y - e_y, o_x - e_x)
        error        = wrap(theta_arm - theta_target)

    Sign derivation, because getting this backwards is silent and expensive:
    image y increases downward, so raising the hand makes ``w_y - e_y`` more
    negative and ``theta_arm`` smaller.  With the arm horizontal
    (theta_arm = 0) and the object up and to the right (theta_target = -30),
    ``theta_arm - theta_target`` is +30 - positive, and the hand does indeed
    need to come up.  The other order would have given -30 and sent the hand
    the wrong way.
    """
    if elbow is None or wrist is None:
        return None
    theta_arm = math.degrees(math.atan2(wrist[1] - elbow[1], wrist[0] - elbow[0]))
    theta_target = math.degrees(math.atan2(obj_cy - elbow[1], obj_cx - elbow[0]))
    return wrap_deg(theta_arm - theta_target)


def foreshortened(shoulder, elbow, wrist, shoulder_px, cfg):
    """PG-3 foreshortening guard.

    The pointing computation aligns the arm with the object *in the image*,
    which is exactly right for a 2D photograph, but an arm angled toward or
    away from the camera projects to a misleading angle.  If the forearm is
    shorter in pixels than it should be relative to ``shoulder_px``, the angle
    cannot be trusted and no amount of coaching will fix it - the subject has
    to turn side-on.

    Returns ``(is_foreshortened, ratio)``; ratio is measured/expected, or None
    when it cannot be computed.
    """
    if elbow is None or wrist is None or not shoulder_px:
        return (False, None)
    expected = shoulder_px * cfg.get("forearm_to_shoulder", 0.63)
    if expected <= 0:
        return (False, None)
    measured = _dist(elbow[0], elbow[1], wrist[0], wrist[1])
    ratio = measured / expected
    return (ratio < cfg.get("foreshorten_ratio", 0.7), ratio)


def goal_point_at(keypoints, obj, shoulder_px, limb, cfg):
    tol = cfg.get("tol_deg", 3.0)
    min_conf = cfg.get("min_keypoint_conf", 0.4)
    if obj is None:
        return result("point_at", None, UNIT_DEG, tol, detail="no object")

    shoulder, elbow, wrist = _limb_points(keypoints, limb, min_conf)
    if elbow is None or wrist is None:
        return result("point_at", None, UNIT_DEG, tol,
                      detail=f"{limb}: elbow or wrist not visible")

    short, ratio = foreshortened(shoulder, elbow, wrist, shoulder_px, cfg)
    if short:
        return result("point_at", None, UNIT_DEG, tol,
                      remedy=REMEDY_TURN_SIDE_ON, axis=None,
                      detail=f"forearm at {ratio:.2f} of expected length")

    err = point_at_error(elbow, wrist, obj["cx"], obj["cy"])
    return result("point_at", err, UNIT_DEG, tol,
                  remedy=remedy_for_angle(err, cfg), axis="vertical",
                  detail=f"{limb}, forearm ratio {ratio:.2f}" if ratio else limb)


# ---------------------------------------------------------------------------
# the rest of the PG-1 goal table
# ---------------------------------------------------------------------------

def goal_hold(keypoints, obj, limb, cfg):
    """Wrist keypoint distance to object centre, +-2% frame width."""
    tol = cfg.get("hold_tol_frac", 0.02)
    min_conf = cfg.get("min_keypoint_conf", 0.4)
    if obj is None:
        return result("hold", None, UNIT_FRAC, tol, detail="no object")

    name = LEFT_WRIST if limb == "left_arm" else RIGHT_WRIST
    wrist = keypoint(keypoints, name, min_conf)
    if wrist is None:
        return result("hold", None, UNIT_FRAC, tol, detail=f"{name} not visible")

    dx = wrist[0] - obj["cx"]
    dy = wrist[1] - obj["cy"]
    err = math.hypot(dx, dy)
    axis = "horizontal" if abs(dx) >= abs(dy) else "vertical"
    # The error is unsigned distance, but coaching needs a direction, so the
    # dominant component's sign is carried in `detail` and re-derived by the
    # coach from `signed`.
    out = result("hold", err, UNIT_FRAC, tol,
                 remedy=remedy_for_frac(err, cfg), axis=axis,
                 detail=f"dx={dx:+.3f} dy={dy:+.3f}")
    out["signed"] = dx if axis == "horizontal" else dy
    return out


def goal_look_at(keypoints, obj, cfg):
    """Nose and eye keypoints vs head-to-object direction, +-8 degrees."""
    tol = cfg.get("look_tol_deg", 8.0)
    min_conf = cfg.get("min_keypoint_conf", 0.4)
    if obj is None:
        return result("look_at", None, UNIT_DEG, tol, detail="no object")

    nose = keypoint(keypoints, NOSE, min_conf)
    eyes = midpoint(keypoint(keypoints, LEFT_EYE, min_conf),
                    keypoint(keypoints, RIGHT_EYE, min_conf))
    if nose is None or eyes is None:
        return result("look_at", None, UNIT_DEG, tol, detail="face not visible")

    # Gaze direction is approximated by the eyes->nose vector, which swings
    # toward whichever way the head is turned.  It is a coarse cue and the
    # +-8 degree tolerance is set accordingly.
    theta_gaze = math.degrees(math.atan2(nose[1] - eyes[1], nose[0] - eyes[0]))
    theta_target = math.degrees(math.atan2(obj["cy"] - eyes[1], obj["cx"] - eyes[0]))
    err = wrap_deg(theta_gaze - theta_target)
    return result("look_at", err, UNIT_DEG, tol,
                  remedy=remedy_for_angle(err, cfg), axis="horizontal",
                  detail="eyes->nose gaze proxy")


def goal_beside(subject, obj, side, cfg):
    """Body centre vs object centre, correct side, +-3% frame width."""
    tol = cfg.get("beside_tol_frac", 0.03)
    if subject is None or obj is None:
        return result("beside", None, UNIT_FRAC, tol, detail="missing box")

    gap = subject["cx"] - obj["cx"]
    want_left = (side == "left")
    # Signed so that a positive error always means "the subject needs to move
    # in +x"; the coach mirrors that into the subject's own left/right.
    target_gap = -abs(cfg.get("beside_gap", 0.25)) if want_left else abs(cfg.get("beside_gap", 0.25))
    err = target_gap - gap
    out = result("beside", abs(err), UNIT_FRAC, tol,
                 remedy=remedy_for_frac(abs(err), cfg), axis="horizontal",
                 detail=f"gap={gap:+.3f} target={target_gap:+.3f}")
    out["signed"] = err
    return out


def _arm_angle(keypoints, side, min_conf):
    if side == "left":
        s = keypoint(keypoints, LEFT_SHOULDER, min_conf)
        e = keypoint(keypoints, LEFT_ELBOW, min_conf)
    else:
        s = keypoint(keypoints, RIGHT_SHOULDER, min_conf)
        e = keypoint(keypoints, RIGHT_ELBOW, min_conf)
    if s is None or e is None:
        return None
    return math.degrees(math.atan2(e[1] - s[1], e[0] - s[0]))


def goal_template(keypoints, goal_id, cfg):
    """arms_out / arms_down / hands_hips - joint angles against a template."""
    tol = cfg.get("template_tol_deg", 10.0)
    min_conf = cfg.get("min_keypoint_conf", 0.4)
    tpl = ARM_TEMPLATES.get(goal_id)
    if tpl is None:
        return result(goal_id, None, UNIT_DEG, tol, detail="unknown template")

    errors = []
    for side in ("left", "right"):
        angle = _arm_angle(keypoints, side, min_conf)
        if angle is None:
            continue
        errors.append((side, wrap_deg(angle - tpl[side])))
    if not errors:
        return result(goal_id, None, UNIT_DEG, tol, detail="arms not visible")

    # Fix the largest error first (CO-2.1: one instruction at a time).
    side, err = max(errors, key=lambda pair: abs(pair[1]))
    out = result(goal_id, err, UNIT_DEG, tol,
                 remedy=remedy_for_angle(err, cfg), axis="vertical",
                 detail=f"worst side: {side}")
    out["side"] = side
    return out


def goal_no_overlap(subject, obj, cfg):
    """Subject box vs object box intersection.  Tolerance is exactly 0."""
    if subject is None or obj is None:
        return result("no_overlap", None, UNIT_OVERLAP, 0.0, detail="missing box")

    ax0, ax1 = subject["cx"] - subject["w"] / 2, subject["cx"] + subject["w"] / 2
    ay0, ay1 = subject["cy"] - subject["h"] / 2, subject["cy"] + subject["h"] / 2
    bx0, bx1 = obj["cx"] - obj["w"] / 2, obj["cx"] + obj["w"] / 2
    by0, by1 = obj["cy"] - obj["h"] / 2, obj["cy"] + obj["h"] / 2

    ox = max(0.0, min(ax1, bx1) - max(ax0, bx0))
    oy = max(0.0, min(ay1, by1) - max(ay0, by0))
    inter = ox * oy
    area = max(1e-9, subject["w"] * subject["h"])
    err = inter / area

    out = result("no_overlap", err, UNIT_OVERLAP, 0.0,
                 remedy=REMEDY_ROVER if err < cfg.get("overlap_rover_max", 0.25)
                 else REMEDY_PERSON,
                 axis="horizontal",
                 detail=f"intersection {inter:.4f} of {area:.4f}")
    # Push the subject away from the object along the shorter escape route.
    out["signed"] = 1.0 if subject["cx"] >= obj["cx"] else -1.0
    return out


# ---------------------------------------------------------------------------
# PG-4  rover-first remedy policy
# ---------------------------------------------------------------------------

def remedy_for_angle(error, cfg):
    """Who fixes an angular error, the rover or the person?

    PG-4 is a product requirement, not an optimisation: a system that asks the
    person to move when it could have driven 20 cm feels bossy; one that
    quietly repositions feels intelligent.  Small misalignments can be absorbed
    by driving, because moving sideways or changing distance shifts the object
    relative to the subject through parallax.  Large ones cannot.
    """
    if error is None:
        return REMEDY_NONE
    return REMEDY_ROVER if abs(error) <= cfg.get("rover_authority_deg", 8.0) else REMEDY_PERSON


def remedy_for_frac(error, cfg):
    if error is None:
        return REMEDY_NONE
    # The frame-fraction equivalent of the angular authority, at the default
    # 66 degree lens: 8 degrees is about 12% of frame width.
    authority = cfg.get("rover_authority_deg", 8.0) / 66.0
    return REMEDY_ROVER if abs(error) <= authority else REMEDY_PERSON


def rover_bias(perr, cfg):
    """Turn a rover-absorbable pose error into a framing bias.

    Rather than inventing a second controller that fights the framing loop
    (PG-5 forbids exactly that), a rover-class remedy nudges the *targets* the
    framing loop is already chasing.  Driving to the new target shifts the
    object relative to the subject through parallax, which is the only thing
    that can close a pose error without asking the person to move.

    Returns ``{"subject_x_bias": float, "dist_bias": float}``, both small.
    """
    zero = {"subject_x_bias": 0.0, "dist_bias": 0.0}
    if perr is None or perr.get("remedy") != REMEDY_ROVER or not perr.get("measurable"):
        return zero

    err = perr["error"]
    gain = cfg.get("bias_gain", 0.012)
    cap = cfg.get("max_subject_x_bias", 0.12)

    if perr["unit"] == UNIT_DEG:
        bias = max(-cap, min(cap, gain * err))
    else:
        signed = perr.get("signed", err)
        bias = max(-cap, min(cap, signed * 0.5))

    if perr["axis"] == "vertical":
        # A vertical pointing error is absorbed by changing distance: backing
        # off lowers the object in frame relative to the subject's hand.
        return {"subject_x_bias": 0.0, "dist_bias": max(-0.2, min(0.2, bias * 2.0))}
    return {"subject_x_bias": bias, "dist_bias": 0.0}


# ---------------------------------------------------------------------------
# dispatch
# ---------------------------------------------------------------------------

def evaluate(goal_spec, obs, cfg):
    """Measure the locked goal against one Observation.

    ``goal_spec`` is the locked GoalSpec from D3b; ``cfg`` is ``config["pose"]``.
    Returns the result dict described in ``result()``, or None for a framing
    run with no pose goal.
    """
    if not goal_spec or goal_spec.get("kind") != "pose":
        return None

    goal_id = goal_spec.get("pose_goal")
    if goal_id not in GOAL_IDS:
        return None

    keypoints = obs.get("keypoints") or {}
    subject = obs.get("subject")
    obj = obs.get("object")
    shoulder_px = obs.get("shoulder_px")
    min_conf = cfg.get("min_keypoint_conf", 0.4)
    limb = choose_limb(keypoints, obj, goal_spec.get("limb"), min_conf)

    if goal_id == "point_at":
        return goal_point_at(keypoints, obj, shoulder_px, limb, cfg)
    if goal_id == "hold":
        return goal_hold(keypoints, obj, limb, cfg)
    if goal_id == "look_at":
        return goal_look_at(keypoints, obj, cfg)
    if goal_id == "beside":
        comp = goal_spec.get("composition") or {}
        side = "left" if comp.get("subject_x", 0.333) < comp.get("object_x", 0.667) else "right"
        return goal_beside(subject, obj, side, cfg)
    if goal_id == "no_overlap":
        return goal_no_overlap(subject, obj, cfg)
    return goal_template(keypoints, goal_id, cfg)


def shoulder_distance_m(shoulder_px, hfov_deg, shoulder_m):
    """D-4: one calibration of the subject's real shoulder width gives distance
    by similar triangles to about +-10%.

    Only used for the spoken remedies in D-7 ("step one metre to your left") -
    the control loops themselves never leave ``shoulder_px``, which is
    re-measured absolutely every frame and cannot drift.
    """
    if not shoulder_px or shoulder_px <= 0 or hfov_deg <= 0:
        return None
    half = math.tan(math.radians(hfov_deg) / 2.0)
    if half <= 0:
        return None
    return shoulder_m / (2.0 * shoulder_px * half)
