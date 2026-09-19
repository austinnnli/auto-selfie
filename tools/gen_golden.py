#!/usr/bin/env python3
"""Generate tests/golden/cases.json - the 50-case corpus from LG-3.

    python tools/gen_golden.py            # regenerate
    python tools/gen_golden.py --check    # verify without writing

LG-3: tests/golden/ holds 50 recorded observations with the command produced
for each.  Any refactor, re-tune or future port to JavaScript must reproduce
them.  A mismatch is then a code bug found at a desk instead of a mystery
found in a field.

WHAT THESE CASES ARE AND ARE NOT.  A golden corpus records what the code does;
it cannot know what the code SHOULD do.  Regenerating it after a change will
always make it pass, which makes it worthless unless something else says what
is correct.  That something else is tests/test_decide_semantics.py, where the
expected values are worked out by hand.  The rule that keeps both honest:

    a golden mismatch is never re-blessed until the semantic tests still pass
    and the `why` line on the failing case has been read and agreed with.

Every case therefore carries a `why` in plain English.  Cases that need
history - a rate limit part-way through a ramp, a sweep mid-collection, a
feasibility window that has already filled - are warmed up here and the warmed
state is frozen into `state_in`, so each case in the file is exactly one call
to decide.step() with no hidden setup.
"""

from __future__ import annotations

import argparse
import copy
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app import decide  # noqa: E402

CONFIG = json.loads((ROOT / "config.json").read_text())

KEYPOINTS = {
    "nose": (0.500, 0.343, 0.95),
    "left_eye": (0.515, 0.333, 0.93), "right_eye": (0.485, 0.333, 0.93),
    "left_ear": (0.530, 0.338, 0.80), "right_ear": (0.470, 0.338, 0.80),
    "left_shoulder": (0.560, 0.420, 0.92), "right_shoulder": (0.440, 0.420, 0.92),
    "left_elbow": (0.575, 0.520, 0.88), "right_elbow": (0.425, 0.520, 0.88),
    "left_wrist": (0.585, 0.615, 0.85), "right_wrist": (0.415, 0.615, 0.85),
    "left_hip": (0.545, 0.640, 0.90), "right_hip": (0.455, 0.640, 0.90),
    "left_knee": (0.545, 0.800, 0.85), "right_knee": (0.455, 0.800, 0.85),
    "left_ankle": (0.545, 0.940, 0.80), "right_ankle": (0.455, 0.940, 0.80),
}


def kps(**overrides):
    out = dict(KEYPOINTS)
    out.update(overrides)
    return out


def obs(t=1.0, subject=(0.333, 0.55, 0.18, 0.62), obj=(0.667, 0.40, 0.14, 0.32),
        shoulder_px=0.120, keypoints=None, bg_flow=(0.0, 0.0),
        rate_z=0.0, pitch=0.0, roll=0.0, lost_for=0.0, telemetry=None):
    o = {
        "t": t, "t_frame": t - 0.18, "t_frame_local": t,
        "subject": None if subject is None else {
            "cx": subject[0], "cy": subject[1], "w": subject[2],
            "h": subject[3], "conf": 0.9},
        "keypoints": None if subject is None else (kps() if keypoints is None else keypoints),
        "shoulder_px": None if subject is None else shoulder_px,
        "object": None if obj is None else {
            "cx": obj[0], "cy": obj[1], "w": obj[2], "h": obj[3], "conf": 0.85},
        "bg_flow": bg_flow,
        "gyro": {"rate_z": rate_z, "pitch": pitch, "roll": roll},
        "lost_for": lost_for,
    }
    if telemetry is not None:
        o["telemetry"] = telemetry
    return o


def goal(kind="framing", pose=None, limb=None, locked=True, composition=None):
    return {"raw": "generated", "object_phrase": "the tree", "object_ref": None,
            "kind": kind, "pose_goal": pose, "limb": limb,
            "composition": composition or {}, "tier": 1, "locked": locked}


def state(phase=decide.PHASE_FRAME, g=None, sweep="done", target_shoulder=0.120, **over):
    st = decide.initial_state(0.0, g if g is not None else goal(), phase)
    if sweep == "done":
        st["sweep"] = {"phase": "done", "target_shoulder": target_shoulder}
    elif sweep is not None:
        st["sweep"] = sweep
    st.update(over)
    return st


# ---------------------------------------------------------------------------
# the corpus.  (name, why, state, observation, warmup observations)
# ---------------------------------------------------------------------------

def build_cases():
    cases = []

    def add(name, why, st, observation, warmup=()):
        cases.append({"name": name, "why": why, "state": st,
                      "obs": observation, "warmup": list(warmup)})

    # -- framing: yaw ------------------------------------------------------
    add("yaw_settled", "Subject and object already on their thirds and the "
        "eyeline on the upper third: every axis satisfied, so the command is "
        "zero and the rover stays put.",
        state(), obs())
    add("yaw_right_of_target", "Framing midpoint at 0.70 against a target of "
        "0.50: the content must move left, so the rover turns right and the "
        "left side drives harder.",
        state(), obs(subject=(0.60, 0.55, 0.18, 0.62), obj=(0.80, 0.40, 0.14, 0.32)))
    add("yaw_left_of_target", "The mirror image: midpoint at 0.30, so the "
        "rover turns left.",
        state(), obs(subject=(0.20, 0.55, 0.18, 0.62), obj=(0.40, 0.40, 0.14, 0.32)))
    add("yaw_saturated", "A huge bearing error must saturate at max_cmd rather "
        "than producing a command the bridge cannot deliver.",
        state(), obs(subject=(0.95, 0.55, 0.18, 0.62), obj=(0.99, 0.40, 0.14, 0.32)))
    add("yaw_no_object", "With no object resolved the yaw axis falls back to "
        "holding the subject alone on their third.",
        state(), obs(obj=None, subject=(0.60, 0.55, 0.18, 0.62)))
    add("yaw_deadband_inside", "Error just inside the 0.02 deadband: the axis "
        "is satisfied and must not hunt.",
        state(), obs(subject=(0.348, 0.55, 0.18, 0.62), obj=(0.682, 0.40, 0.14, 0.32)))
    add("yaw_deadband_outside", "The same error just outside the deadband, "
        "where the loop must act. The pair brackets the threshold.",
        state(), obs(subject=(0.39, 0.55, 0.18, 0.62), obj=(0.72, 0.40, 0.14, 0.32)))
    add("yaw_gyro_brakes_overshoot", "Framing correct but the rover still "
        "rotating at 25 deg/s: D-3's inner loop must command the opposite way "
        "to stop it. This is the case vision alone cannot handle.",
        state(), obs(rate_z=25.0))
    add("yaw_gyro_already_turning_the_right_way", "Turning toward the target "
        "already, so the inner loop's correction is small.",
        state(), obs(subject=(0.60, 0.55, 0.18, 0.62), obj=(0.80, 0.40, 0.14, 0.32),
                     rate_z=20.0))
    add("yaw_gyro_sign_flipped", "The gyro reading inverted: the inner loop "
        "must fight, not run away. Guards the rate_z_sign calibration.",
        state(), obs(rate_z=-25.0))

    # -- framing: distance -------------------------------------------------
    add("dist_too_close", "shoulder_px 0.150 against a swept target of 0.120: "
        "the subject fills too much of the frame, so the rover backs off.",
        state(), obs(shoulder_px=0.150))
    add("dist_too_far", "The reverse: 0.095 against 0.120, so the rover "
        "closes in.",
        state(), obs(shoulder_px=0.095))
    add("dist_deadband", "Inside the 3% deadband, so the distance axis is "
        "silent even though the number is not exactly the target.",
        state(), obs(shoulder_px=0.1225))
    add("dist_arms_raised", "P-4: the subject raises their arms, so the box "
        "grows a lot and shoulder_px does not move at all. The distance "
        "command must be identical to dist_settled - reading box height here "
        "would look like 40 cm of travel.",
        state(), obs(subject=(0.333, 0.48, 0.34, 0.78),
                     keypoints=kps(left_wrist=(0.62, 0.20, 0.85),
                                   right_wrist=(0.38, 0.20, 0.85))))
    add("dist_no_shoulder_measurement", "Both shoulders occluded, so "
        "shoulder_px is None. P-7: hold rather than guess a distance.",
        state(), obs(shoulder_px=None))

    # -- framing: tilt -----------------------------------------------------
    add("tilt_eyeline_high", "Eyes at 0.28 against a target of 0.333: the "
        "subject sits too high, the content must move down, so the camera "
        "tilts up and the commanded angle increases.",
        state(), obs(keypoints=kps(left_eye=(0.515, 0.28, 0.93),
                                   right_eye=(0.485, 0.28, 0.93))))
    add("tilt_eyeline_low", "Eyes at 0.46 against the same 0.333 target: the "
        "subject sits too low, so the camera tilts down and the commanded "
        "angle decreases. The mirror of the case above.",
        state(), obs(keypoints=kps(left_eye=(0.515, 0.46, 0.93),
                                   right_eye=(0.485, 0.46, 0.93))))
    add("tilt_step_clamped", "A huge tilt error must be taken in one bounded "
        "step, not a single lurch across the whole range.",
        state(), obs(keypoints=kps(left_eye=(0.515, 0.95, 0.93),
                                   right_eye=(0.485, 0.95, 0.93))))
    add("tilt_builds_on_previous_command", "D-9: the correction is added to "
        "the LAST COMMANDED angle, not to a gravity-referenced absolute. That "
        "is what absorbs backlash and missed steps.",
        state(tilt_cmd_deg=12.0), obs(pitch=12.0,
                                      keypoints=kps(left_eye=(0.515, 0.28, 0.93),
                                                    right_eye=(0.485, 0.28, 0.93))))
    add("tilt_at_mechanical_limit", "Already at the top of the tilt range: the "
        "command must clamp rather than wind up.",
        state(tilt_cmd_deg=34.6), obs(pitch=34.6,
                                      keypoints=kps(left_eye=(0.515, 0.28, 0.93),
                                                    right_eye=(0.485, 0.28, 0.93))))
    add("tilt_falls_back_to_nose", "Eyes not visible but the nose is: the "
        "eyeline falls back one rung rather than giving up.",
        state(), obs(keypoints=kps(left_eye=(0.515, 0.30, 0.05),
                                   right_eye=(0.485, 0.30, 0.05),
                                   nose=(0.500, 0.30, 0.95))))
    add("tilt_no_face_at_all", "No eyes, no nose: the eyeline is estimated "
        "from the box, which is the last rung before holding.",
        state(), obs(keypoints={"left_shoulder": (0.560, 0.420, 0.92),
                                "right_shoulder": (0.440, 0.420, 0.92)}))

    # -- D-6 self-motion gate ---------------------------------------------
    add("gate_subject_moved", "D-6: shoulder_px jumped but the background did "
        "not move, so the SUBJECT moved. Hold and re-plan - reacting here "
        "would chase a person who is walking.",
        state(last_shoulder=0.100), obs(shoulder_px=0.140, bg_flow=(0.0, 0.0)))
    add("gate_rover_moved", "The same jump with the background shifting: the "
        "ROVER moved, so the measurement is real and the loop acts on it.",
        state(last_shoulder=0.100), obs(shoulder_px=0.140, bg_flow=(0.05, 0.01)))
    add("gate_no_flow_measurement", "No flow measurement at all is not "
        "evidence the background was still, so the gate must not fire.",
        state(last_shoulder=0.100), obs(shoulder_px=0.140, bg_flow=None))
    add("gate_small_change_ignored", "A change below shoulder_jump is noise, "
        "not motion, and must not trip the gate either way.",
        state(last_shoulder=0.120), obs(shoulder_px=0.128, bg_flow=(0.0, 0.0)))

    # -- D-8 speed limits --------------------------------------------------
    add("speed_limited_near_an_obstacle", "D-8: range under 1000 mm halves "
        "the speed limit. Warmed up past the ramp so the CAP is what differs "
        "from the case below, not the acceleration.",
        state(target_shoulder=0.060),
        obs(t=1.4, shoulder_px=0.120, telemetry={"range_mm": 600}),
        warmup=[obs(t=1.0 + i * 0.04, shoulder_px=0.120,
                    telemetry={"range_mm": 600}) for i in range(10)])
    add("speed_unlimited_in_the_open", "The same error with clear ground, for "
        "comparison with the case above. The duty here must be strictly "
        "larger.",
        state(target_shoulder=0.060),
        obs(t=1.4, shoulder_px=0.120, telemetry={"range_mm": 2500}),
        warmup=[obs(t=1.0 + i * 0.04, shoulder_px=0.120,
                    telemetry={"range_mm": 2500}) for i in range(10)])

    # -- D-5 the sweep -----------------------------------------------------
    add("sweep_first_sample", "The sweep starts by backing away - the safe "
        "direction, since the far end of the leg is unknown.",
        state(sweep=None), obs())
    add("sweep_collecting", "Part-way through collection, still driving and "
        "logging (shoulder_px, ratio) pairs.",
        state(sweep=None),
        obs(t=2.0, shoulder_px=0.105),
        warmup=[obs(t=0.30 * i, shoulder_px=0.120 - 0.002 * i) for i in range(5)])
    add("sweep_solved_and_returning", "Collection finished and solved; the "
        "rover now drives back to the shoulder_px that satisfies the ratio.",
        state(sweep={"phase": "return", "target_shoulder": 0.130,
                     "samples": [[0.10, 2.0], [0.14, 2.8]], "dir": -1.0,
                     "t0": 0.0, "last_sample_t": 0.0, "rechecked": False,
                     "start_shoulder": 0.12}),
        obs(shoulder_px=0.105))
    add("sweep_rechecking_holds_still", "D-8: a full stop before any "
        "measurement used for a final decision, so the recheck happens with "
        "the wheels stopped.",
        state(sweep={"phase": "recheck", "target_shoulder": 0.120,
                     "samples": [[0.10, 2.0]], "dir": -1.0, "t0": 0.0,
                     "last_sample_t": 0.0, "rechecked": False,
                     "recheck_t": 0.9, "start_shoulder": 0.12}),
        obs(t=1.0, shoulder_px=0.120))
    add("sweep_fails_and_speaks_up", "The object scaled with the subject "
        "across the whole sweep, so the fitted curve is flat and no distance "
        "gives the requested ratio. This case catches the TRANSITION into "
        "failure, which is where the spoken line is produced.",
        state(sweep={"phase": "solve", "dir": -1.0,
                     "samples": [[0.08 + 0.01 * i, 2.40] for i in range(9)],
                     "t0": 0.0, "last_sample_t": 0.0, "target_shoulder": None,
                     "rechecked": False, "start_shoulder": 0.12}),
        obs())

    # -- D-7 feasibility ---------------------------------------------------
    add("feasible_still_improving", "The gap between subject and object is "
        "opening as the rover drives, so stay quiet and keep going.",
        state(),
        obs(t=4.0, subject=(0.30, 0.55, 0.18, 0.62), obj=(0.70, 0.40, 0.14, 0.32)),
        warmup=[obs(t=i * 0.4, subject=(0.5 - 0.02 * i, 0.55, 0.18, 0.62),
                    obj=(0.5 + 0.02 * i, 0.40, 0.14, 0.32)) for i in range(10)])
    add("infeasible_asks_the_subject_to_step", "D-7: the spacing has not "
        "improved over the whole window, so turning and driving cannot fix "
        "it. Propose the fix, in the subject's own left/right.",
        state(),
        obs(t=4.0, subject=(0.48, 0.55, 0.18, 0.62), obj=(0.52, 0.40, 0.14, 0.32)),
        warmup=[obs(t=i * 0.4, subject=(0.48, 0.55, 0.18, 0.62),
                    obj=(0.52, 0.40, 0.14, 0.32)) for i in range(10)])

    # -- rate limiting -----------------------------------------------------
    add("ramp_from_standstill", "First tick of a big move: the normalised "
        "command is rate limited, but the duty that comes out still clears "
        "the bridge's dead band, so the logged command is what the wheels got.",
        state(), obs(subject=(0.90, 0.55, 0.18, 0.62), obj=(0.95, 0.40, 0.14, 0.32)))
    add("ramp_mid_move", "Four ticks into the same move, with the ramp "
        "part-way up.",
        state(), obs(t=1.16, subject=(0.90, 0.55, 0.18, 0.62), obj=(0.95, 0.40, 0.14, 0.32)),
        warmup=[obs(t=1.0 + i * 0.04, subject=(0.90, 0.55, 0.18, 0.62),
                    obj=(0.95, 0.40, 0.14, 0.32)) for i in range(4)])
    add("stop_is_immediate", "Deceleration is not rate limited: a satisfied "
        "loop stops on the tick, rather than ramping down through duties the "
        "bridge cannot turn.",
        state(prev_left_u=0.9, prev_right_u=0.9, prev_left=620, prev_right=620),
        obs())

    # -- phases ------------------------------------------------------------
    for phase in ("idle", "listen", "parse", "confirm"):
        add(f"phase_{phase}_inhibits_motors",
            f"CO-3: motors are inhibited in {phase}. The GoalSpec is locked "
            f"before the first command that sets enable (G-1).",
            state(phase=phase), obs(subject=(0.90, 0.55, 0.18, 0.62)))
    add("goal_not_locked_inhibits_motors", "G-1 again, from the other side: "
        "even in the frame phase, an unlocked goal must not energise anything.",
        state(g=goal(locked=False)), obs(subject=(0.90, 0.55, 0.18, 0.62)))
    add("phase_arm_tracks_tilt_only", "Armed but not yet framing: the tilt "
        "axis tracks so the commanded angle does not jump on entry, and the "
        "wheels stay still.",
        state(phase=decide.PHASE_ARM),
        obs(keypoints=kps(left_eye=(0.515, 0.28, 0.93), right_eye=(0.485, 0.28, 0.93))))
    add("phase_settle_holds_position", "PG-5: position is locked before "
        "coaching or capture.",
        state(phase=decide.PHASE_SETTLE), obs(subject=(0.60, 0.55, 0.18, 0.62)))
    add("phase_capture_fires", "First frame of the capture burst: the capture "
        "flag goes up while the wheels stay stopped, so the shutter never "
        "fires during a manoeuvre.",
        state(phase=decide.PHASE_CAPTURE), obs())
    add("phase_capture_burst_exhausted", "After burst_frames, the capture flag "
        "drops so the phone is not asked for an endless stream of stills.",
        state(phase=decide.PHASE_CAPTURE, capture_frames=7), obs())
    add("phase_halted_stops", "CO-4: the subject is the only positional "
        "reference, so losing them stops the motors.",
        state(phase=decide.PHASE_HALTED), obs(subject=None, lost_for=0.8))
    add("phase_search_spins", "CO-4: a slow gyro-guided spin toward the last "
        "known bearing. Equal and opposite duty - the rover must never DRIVE "
        "while it cannot see the subject.",
        state(phase=decide.PHASE_SEARCH, last_bearing=0.3),
        obs(subject=None, lost_for=3.0))
    add("phase_search_reverses_at_the_limit", "Past the +-60 degree cap the "
        "search turns back rather than spinning on forever.",
        state(phase=decide.PHASE_SEARCH, last_bearing=0.3, search_swept_deg=65.0),
        obs(subject=None, lost_for=3.0))

    # -- pose --------------------------------------------------------------
    pointing = kps(right_elbow=(0.45, 0.50, 0.9), right_wrist=(0.53, 0.50, 0.9))
    add("pose_point_large_error", "CO-1: more than 20 degrees out, so the cue "
        "is the blunt one. The limb is pinned so this case measures the ANGLE; "
        "limb selection has its own case below.",
        state(phase=decide.PHASE_COACH, g=goal("pose", "point_at", limb="right_arm")),
        obs(keypoints=kps(right_elbow=(0.45, 0.50, 0.9), right_wrist=(0.53, 0.56, 0.9)),
            obj=(0.90, 0.20, 0.10, 0.10)))
    add("pose_point_medium_error", "CO-1: squarely in the 8-20 degree band, "
        "where the cue is a nudge rather than a correction.",
        state(phase=decide.PHASE_COACH, g=goal("pose", "point_at", limb="right_arm")),
        obs(keypoints=pointing, obj=(0.90, 0.40, 0.10, 0.10)))
    add("pose_point_in_tolerance", "Inside +-3 degrees: no correction, and the "
        "hold countdown begins.",
        state(phase=decide.PHASE_COACH, g=goal("pose", "point_at", limb="right_arm")),
        obs(keypoints=pointing, obj=(0.90, 0.50, 0.10, 0.10)))
    add("pose_point_foreshortened", "PG-3: the forearm is far shorter than "
        "shoulder_px implies, so the angle is untrustworthy and the remedy is "
        "to turn side-on rather than any angle correction.",
        state(phase=decide.PHASE_COACH, g=goal("pose", "point_at", limb="right_arm")),
        obs(keypoints=kps(right_elbow=(0.45, 0.50, 0.9), right_wrist=(0.47, 0.50, 0.9)),
            obj=(0.90, 0.20, 0.10, 0.10)))
    add("pose_limb_chosen_by_object_side", "G-4 and the mirror: the object is "
        "on the image-right, and the subject's LEFT wrist is the one at the "
        "larger image x, so the left arm is chosen.",
        state(phase=decide.PHASE_COACH, g=goal("pose", "point_at")),
        obs(obj=(0.95, 0.40, 0.10, 0.10)))
    add("pose_hold_goal", "PG-1: wrist to object centre, +-2% of frame width.",
        state(phase=decide.PHASE_COACH, g=goal("pose", "hold")),
        obs(keypoints=kps(right_wrist=(0.50, 0.50, 0.9)), obj=(0.62, 0.55, 0.08, 0.08)))
    add("pose_look_at_goal", "PG-1: the eyes-to-nose gaze proxy against the "
        "head-to-object direction, +-8 degrees.",
        state(phase=decide.PHASE_COACH, g=goal("pose", "look_at")),
        obs(obj=(0.90, 0.30, 0.10, 0.10)))
    add("pose_template_arms_out", "PG-1's template row, measured as joint "
        "angles against the template and reporting the worse side.",
        state(phase=decide.PHASE_COACH, g=goal("pose", "arms_out")),
        obs())
    add("pose_no_overlap", "PG-1: subject box against object box, tolerance "
        "exactly zero.",
        state(phase=decide.PHASE_COACH, g=goal("pose", "no_overlap")),
        obs(subject=(0.40, 0.50, 0.20, 0.60), obj=(0.50, 0.50, 0.20, 0.60)))
    add("pose_bias_absorbed_by_the_rover", "PG-4's rover-first policy: a small "
        "pose error in the FRAME phase biases the framing targets instead of "
        "asking the person to move. Driving shifts the object relative to the "
        "subject through parallax.",
        state(phase=decide.PHASE_FRAME, g=goal("pose", "point_at", limb="right_arm")),
        obs(keypoints=kps(right_elbow=(0.45, 0.50, 0.9), right_wrist=(0.53, 0.505, 0.9)),
            obj=(0.90, 0.53, 0.10, 0.10)))
    add("pose_unmeasurable_keypoints", "P-7 end to end: the wrist is "
        "low-confidence, so there is no error, no cue and no guess.",
        state(phase=decide.PHASE_COACH, g=goal("pose", "point_at", limb="right_arm")),
        obs(keypoints=kps(right_wrist=(0.53, 0.50, 0.05),
                          left_wrist=(0.58, 0.50, 0.05)),
            obj=(0.90, 0.20, 0.10, 0.10)))

    return cases


# ---------------------------------------------------------------------------

STATE_KEYS = ("phase", "tilt_cmd_deg", "prev_left", "prev_right",
              "prev_left_u", "prev_right_u", "last_shoulder", "replan",
              "infeasible", "capture_frames", "search_swept_deg")


def state_digest(st):
    """The slice of the state the corpus pins.  Not the whole dict: internal
    bookkeeping that no downstream module reads would make the corpus brittle
    without making it stronger."""
    out = {k: st.get(k) for k in STATE_KEYS}
    sweep = st.get("sweep")
    out["sweep_phase"] = sweep.get("phase") if sweep else None
    out["sweep_target"] = round(sweep["target_shoulder"], 6) \
        if sweep and sweep.get("target_shoulder") is not None else None
    coach_state = st.get("coach") or {}
    out["coach_in_tolerance"] = coach_state.get("in_tolerance")
    out["coach_countdown_next"] = coach_state.get("countdown_next")
    perr = st.get("pose_error")
    out["pose_error"] = round(perr["error"], 6) \
        if perr and perr.get("error") is not None else None
    out["pose_remedy"] = perr.get("remedy") if perr else None
    return out


def round_cmd(cmd):
    out = dict(cmd)
    out["tilt_deg"] = round(float(cmd["tilt_deg"]), 6)
    return out


def generate():
    rows = []
    for case in build_cases():
        st = copy.deepcopy(case["state"])
        for warm in case["warmup"]:
            _cmd, st = decide.step(warm, st, CONFIG)

        # Freeze the warmed state through JSON so the file is self-contained
        # and every case is exactly one call to decide.step().
        st = json.loads(json.dumps(st))
        cmd, st_out = decide.step(case["obs"], st, CONFIG)

        rows.append({
            "name": case["name"],
            "why": case["why"],
            "state_in": st,
            "obs": case["obs"],
            "cmd_out": round_cmd(cmd),
            "state_out": state_digest(st_out),
        })
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--check", action="store_true",
                        help="verify the committed corpus without writing it")
    args = parser.parse_args()

    rows = generate()
    names = [r["name"] for r in rows]
    assert len(names) == len(set(names)), "duplicate case names"

    path = ROOT / "tests" / "golden" / "cases.json"
    payload = {
        "generated_by": "tools/gen_golden.py",
        "config_hash": __import__("hashlib").sha256(
            (ROOT / "config.json").read_bytes()).hexdigest()[:16],
        "count": len(rows),
        "cases": rows,
    }
    text = json.dumps(payload, indent=1, sort_keys=False) + "\n"

    if args.check:
        if not path.exists():
            print("no corpus committed", file=sys.stderr)
            return 1
        same = path.read_text() == text
        print("corpus is up to date" if same else
              "CORPUS DIFFERS - decide.py changed behaviour")
        return 0 if same else 1

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    print(f"wrote {path.relative_to(ROOT)}: {len(rows)} cases, "
          f"config {payload['config_hash']}")

    moving = sum(1 for r in rows if r["cmd_out"]["left"] or r["cmd_out"]["right"])
    speaking = sum(1 for r in rows if r["cmd_out"]["say"])
    print(f"  {moving} cases command motion, {len(rows) - moving} command a hold")
    print(f"  {speaking} cases produce a spoken line")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
