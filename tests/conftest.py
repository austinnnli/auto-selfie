"""Shared fixtures.  Everything here builds plain dictionaries - the pure
modules are unit-tested with nothing else, which is the point of keeping them
pure in the first place."""

import json
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


@pytest.fixture(scope="session")
def config():
    return json.loads((ROOT / "config.json").read_text())


@pytest.fixture
def cfg(config):
    """A per-test mutable copy, so a test that tweaks a gain cannot leak."""
    return json.loads(json.dumps(config))


def make_keypoints(**overrides):
    """A neutral standing subject, facing the camera, arms down.

    Image coordinates, y DOWN.  Left/right are the SUBJECT'S left and right,
    so ``left_shoulder`` sits at the larger x - the phone faces them.
    """
    kps = {
        "nose": (0.500, 0.300, 0.95),
        "left_eye": (0.515, 0.290, 0.93),
        "right_eye": (0.485, 0.290, 0.93),
        "left_ear": (0.530, 0.295, 0.80),
        "right_ear": (0.470, 0.295, 0.80),
        "left_shoulder": (0.560, 0.380, 0.92),
        "right_shoulder": (0.440, 0.380, 0.92),
        "left_elbow": (0.575, 0.480, 0.88),
        "right_elbow": (0.425, 0.480, 0.88),
        "left_wrist": (0.585, 0.575, 0.85),
        "right_wrist": (0.415, 0.575, 0.85),
        "left_hip": (0.545, 0.600, 0.90),
        "right_hip": (0.455, 0.600, 0.90),
        "left_knee": (0.545, 0.760, 0.85),
        "right_knee": (0.455, 0.760, 0.85),
        "left_ankle": (0.545, 0.920, 0.80),
        "right_ankle": (0.455, 0.920, 0.80),
    }
    kps.update(overrides)
    return kps


def make_obs(t=1.0, subject=(0.333, 0.55, 0.18, 0.62), obj=(0.667, 0.40, 0.14, 0.32),
             shoulder_px=0.120, keypoints=None, bg_flow=(0.0, 0.0),
             rate_z=0.0, pitch=0.0, roll=0.0, lost_for=0.0, telemetry=None,
             eyeline=None):
    """One Observation, in the exact shape perceive.py emits.

    ``eyeline`` places the eyes at a given y, which is what the tilt axis
    controls.  Pass ``eyeline=0.333`` for a subject already on the upper third.
    """
    if keypoints is None and eyeline is not None:
        keypoints = make_keypoints(
            left_eye=(0.515, eyeline, 0.93), right_eye=(0.485, eyeline, 0.93),
            nose=(0.500, eyeline + 0.010, 0.95))
    obs = {
        "t": t,
        "t_frame": t - 0.18,
        "t_frame_local": t,
        "subject": None if subject is None else {
            "cx": subject[0], "cy": subject[1], "w": subject[2], "h": subject[3],
            "conf": 0.9},
        "keypoints": keypoints if keypoints is not None else make_keypoints(),
        "shoulder_px": shoulder_px,
        "object": None if obj is None else {
            "cx": obj[0], "cy": obj[1], "w": obj[2], "h": obj[3], "conf": 0.85},
        "bg_flow": bg_flow,
        "gyro": {"rate_z": rate_z, "pitch": pitch, "roll": roll},
        "lost_for": lost_for,
    }
    if subject is None:
        obs["keypoints"] = None
        obs["shoulder_px"] = None
    if telemetry is not None:
        obs["telemetry"] = telemetry
    return obs


def framed_obs(t=1.0, **overrides):
    """An Observation in which every controlled axis is already satisfied:
    subject and object on their thirds, shoulder_px at the swept target, and
    the eyeline on the upper third.  Use it with a state whose sweep is done.
    """
    params = dict(t=t, subject=(0.333, 0.55, 0.18, 0.62),
                  obj=(0.667, 0.40, 0.14, 0.32), shoulder_px=0.120,
                  eyeline=0.333)
    params.update(overrides)
    return make_obs(**params)


def framing_goal(**overrides):
    spec = {
        "raw": "frame me next to the tree",
        "object_phrase": "the tree",
        "object_ref": None,
        "kind": "framing",
        "pose_goal": None,
        "limb": None,
        "composition": {},
        "tier": 1,
        "locked": True,
    }
    spec.update(overrides)
    return spec


def pose_goal(pose="point_at", limb=None, **overrides):
    spec = framing_goal(kind="pose", pose_goal=pose, limb=limb,
                        raw=f"make me {pose} the tree")
    spec.update(overrides)
    return spec


@pytest.fixture
def obs_factory():
    return make_obs


@pytest.fixture
def kp_factory():
    return make_keypoints
