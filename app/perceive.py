"""perceive.py - bytes in, one Observation out.

Everything here is allowed to use libraries; nothing here makes a decision
(D3a preamble).  The contract is narrow on purpose: ``perceive.py`` emits an
Observation and nothing else, and every downstream module consumes only that.

P-3 DETECTION.  YOLO-pose nano at 480 input.  On a CPU-only laptop, export to
OpenVINO or NCNN first - a 2-3x speedup is the difference between 8 and 20 fps.
``tools/export_model.py`` does the export; this module prefers an exported
model and says so loudly if it falls back to PyTorch.

P-7 CONFIDENCE AND STALENESS.  Perception never substitutes a guess for a
missing measurement - it emits ``None`` and lets the decision layer choose what
to do.  Every ``None`` in this file is deliberate.

Import policy: ultralytics, opencv and numpy are optional.  Without them the
module still imports and still builds Observations from supplied detections,
which is what ``replay.py`` and the tests need.  A missing model is reported
once, clearly, rather than crashing the control app on line one.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field

log = logging.getLogger("perceive")

try:
    import numpy as np
except ImportError:                                    # pragma: no cover
    np = None

try:
    import cv2
except ImportError:                                    # pragma: no cover
    cv2 = None

# COCO-17 keypoint order, as emitted by YOLO-pose.
KEYPOINT_NAMES = (
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
)


def blank_observation(t: float) -> dict:
    """An Observation with every measurement absent.

    Used when a frame cannot be decoded or the detector is unavailable.  It is
    a valid Observation: ``decide()`` reads ``subject is None`` and halts,
    which is the correct response to "I cannot see".
    """
    return {
        "t": t,
        "t_frame": t,
        "subject": None,
        "keypoints": None,
        "shoulder_px": None,
        "object": None,
        "bg_flow": None,
        "gyro": {"rate_z": 0.0, "pitch": 0.0, "roll": 0.0},
        "lost_for": 0.0,
    }


def shoulder_px(keypoints: dict, min_conf: float = 0.4) -> float | None:
    """P-4.  The distance between the two shoulder keypoints, normalised.

    This is the system's distance coordinate.  Do NOT use bounding-box height:
    it changes when the subject raises their arms, which the control loop would
    read as 40 cm of travel.

    Returns None when either shoulder is missing or low-confidence.  A guessed
    distance is worse than no distance - the loop can hold, but it cannot
    un-drive.
    """
    if not keypoints:
        return None
    left = keypoints.get("left_shoulder")
    right = keypoints.get("right_shoulder")
    if left is None or right is None:
        return None
    if left[2] < min_conf or right[2] < min_conf:
        return None
    return math.hypot(right[0] - left[0], right[1] - left[1])


# ---------------------------------------------------------------------------
# P-5  object selection and tracking
# ---------------------------------------------------------------------------

@dataclass
class ObjectTrack:
    """P-5, in the order of cost the PRD sets out.

    1. selection by click in the preview, or by name against numbered detections
    2. frame-to-frame tracker (CSRT) on the selected box
    3. re-detection every N frames, snapping the tracker to the best-matching box
    4. an appearance fingerprint saved at selection, to re-find after occlusion
    5. last-known bearing carried forward with gyro integration during a loss

    Levels 2-4 need OpenCV.  Without it the track degrades to level 1 plus 5,
    which is enough to fly the rest of the system in replay.
    """

    redetect_every: int = 10
    max_coast_s: float = 1.5

    box: tuple | None = field(default=None, init=False)       # cx, cy, w, h
    label: str = field(default="", init=False)
    fingerprint: object = field(default=None, init=False)
    confidence: float = field(default=0.0, init=False)
    lost_since: float | None = field(default=None, init=False)
    _tracker = None
    _frames_since_detect: int = field(default=0, init=False)

    def select(self, frame_bgr, box: tuple, label: str = ""):
        """Level 1 + 4: lock on, and remember what it looked like."""
        self.box = tuple(box)
        self.label = label
        self.confidence = 1.0
        self.lost_since = None
        self._frames_since_detect = 0
        self.fingerprint = self._hist(frame_bgr, box)
        self._tracker = self._make_tracker(frame_bgr, box)

    # -- level 2 --------------------------------------------------------
    def _make_tracker(self, frame_bgr, box):
        if cv2 is None or frame_bgr is None:
            return None
        create = (getattr(cv2, "TrackerCSRT_create", None)
                  or getattr(getattr(cv2, "legacy", None), "TrackerCSRT_create", None)
                  or getattr(cv2, "TrackerKCF_create", None))
        if create is None:
            log.info("no CSRT/KCF tracker in this OpenCV build; "
                     "falling back to re-detection only")
            return None
        try:
            tracker = create()
            tracker.init(frame_bgr, self._to_xywh(frame_bgr, box))
            return tracker
        except Exception as exc:                        # pragma: no cover
            log.warning("tracker init failed: %s", exc)
            return None

    @staticmethod
    def _to_xywh(frame_bgr, box):
        h, w = frame_bgr.shape[:2]
        cx, cy, bw, bh = box
        return (int((cx - bw / 2) * w), int((cy - bh / 2) * h),
                max(1, int(bw * w)), max(1, int(bh * h)))

    @staticmethod
    def _to_norm(frame_bgr, xywh):
        h, w = frame_bgr.shape[:2]
        x, y, bw, bh = xywh
        return ((x + bw / 2) / w, (y + bh / 2) / h, bw / w, bh / h)

    # -- level 4 --------------------------------------------------------
    def _hist(self, frame_bgr, box):
        """A cheap appearance fingerprint: a hue-saturation histogram.

        Good enough to pick the right tree out of three after an occlusion,
        and it costs a fraction of a millisecond.  Anything heavier would eat
        the frame budget that P-3 is fighting for.
        """
        if cv2 is None or np is None or frame_bgr is None:
            return None
        x, y, w, h = self._to_xywh(frame_bgr, box)
        x, y = max(0, x), max(0, y)
        patch = frame_bgr[y:y + h, x:x + w]
        if patch.size == 0:
            return None
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [24, 24], [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
        return hist

    def _match(self, frame_bgr, candidates):
        """Level 3 + 4: snap to the detection that best matches position and
        appearance."""
        if not candidates:
            return None
        best, best_score = None, -1.0
        for cand in candidates:
            score = 0.0
            if self.box is not None:
                dist = math.hypot(cand["cx"] - self.box[0], cand["cy"] - self.box[1])
                score += max(0.0, 1.0 - dist * 2.0)
            if self.fingerprint is not None and cv2 is not None:
                hist = self._hist(frame_bgr, (cand["cx"], cand["cy"], cand["w"], cand["h"]))
                if hist is not None:
                    score += max(0.0, cv2.compareHist(
                        self.fingerprint, hist, cv2.HISTCMP_CORREL))
            score += 0.2 * cand.get("conf", 0.0)
            if score > best_score:
                best, best_score = cand, score
        return best if best_score > 0.3 else None

    def update(self, frame_bgr, candidates, now: float, gyro_dyaw: float = 0.0):
        """One frame of tracking.  Returns the object dict or None."""
        self._frames_since_detect += 1

        tracked = None
        if self._tracker is not None and frame_bgr is not None:
            try:
                ok, xywh = self._tracker.update(frame_bgr)
                if ok:
                    tracked = self._to_norm(frame_bgr, xywh)
            except Exception:                           # pragma: no cover
                tracked = None

        if self._frames_since_detect >= self.redetect_every or tracked is None:
            snapped = self._match(frame_bgr, candidates)
            self._frames_since_detect = 0
            if snapped is not None:
                self.box = (snapped["cx"], snapped["cy"], snapped["w"], snapped["h"])
                self.confidence = snapped.get("conf", 0.8)
                self.lost_since = None
                self._tracker = self._make_tracker(frame_bgr, self.box)
                return self._as_dict()

        if tracked is not None:
            self.box = tracked
            self.confidence = max(0.3, self.confidence * 0.98)
            self.lost_since = None
            return self._as_dict()

        # Level 5: carry the last known bearing forward with gyro integration
        # during a brief loss.  The object has not moved; the rover has.
        if self.box is not None:
            if self.lost_since is None:
                self.lost_since = now
            if now - self.lost_since <= self.max_coast_s:
                cx = self.box[0] - gyro_dyaw
                self.box = (cx, self.box[1], self.box[2], self.box[3])
                self.confidence *= 0.9
                return self._as_dict()
            self.box = None
        return None

    def _as_dict(self):
        cx, cy, w, h = self.box
        return {"cx": cx, "cy": cy, "w": w, "h": h, "conf": round(self.confidence, 3)}


# ---------------------------------------------------------------------------
# P-6  background flow
# ---------------------------------------------------------------------------

class BackgroundFlow:
    """Sparse Lucas-Kanade optical flow on corners OUTSIDE the subject mask.

    This is the system's only self-motion cue and it resolves the central
    ambiguity (D-6): background shifted and subject size changed means the
    rover moved; background static and subject size changed means the subject
    moved.  Without it, a subject walking toward the camera and a rover driving
    forward are the same measurement.
    """

    def __init__(self, max_corners: int = 300, quality: float = 0.01, min_distance: int = 8):
        self.max_corners = max_corners
        self.quality = quality
        self.min_distance = min_distance
        self._prev_gray = None
        self._prev_pts = None

    def update(self, frame_bgr, subject_box=None):
        if cv2 is None or np is None or frame_bgr is None:
            return None

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]

        mask = np.full((h, w), 255, dtype=np.uint8)
        if subject_box is not None:
            cx, cy, bw, bh = subject_box
            # Pad the mask: a tight box still leaks the subject's outline into
            # the corner set, and those corners move with the subject.
            pad = 0.06
            x0 = int(max(0, (cx - bw / 2 - pad) * w))
            x1 = int(min(w, (cx + bw / 2 + pad) * w))
            y0 = int(max(0, (cy - bh / 2 - pad) * h))
            y1 = int(min(h, (cy + bh / 2 + pad) * h))
            mask[y0:y1, x0:x1] = 0

        flow = None
        if self._prev_gray is not None and self._prev_pts is not None and len(self._prev_pts):
            nxt, status, _err = cv2.calcOpticalFlowPyrLK(
                self._prev_gray, gray, self._prev_pts, None,
                winSize=(21, 21), maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
            if nxt is not None:
                good_new = nxt[status.flatten() == 1]
                good_old = self._prev_pts[status.flatten() == 1]
                if len(good_new) >= 8:
                    delta = good_new - good_old
                    # Median, not mean: a handful of corners always land on a
                    # moving leaf or a passing car, and a mean follows them.
                    dx = float(np.median(delta[:, 0])) / w
                    dy = float(np.median(delta[:, 1])) / h
                    flow = (dx, dy)

        self._prev_pts = cv2.goodFeaturesToTrack(
            gray, maxCorners=self.max_corners, qualityLevel=self.quality,
            minDistance=self.min_distance, mask=mask)
        self._prev_gray = gray
        return flow


# ---------------------------------------------------------------------------
# the detector
# ---------------------------------------------------------------------------

class PoseDetector:
    """YOLO-pose nano wrapper (P-3).

    Prefers an OpenVINO or NCNN export, because on a CPU-only laptop the
    PyTorch path runs at 6-8 fps and the target is 10+ sustained.  If only the
    .pt is present it still runs, with one loud warning - a slow system that
    works beats a fast one that is not built yet.
    """

    def __init__(self, weights: str = "yolo11n-pose.pt", imgsz: int = 480,
                 conf: float = 0.35, device: str = "cpu"):
        self.imgsz = imgsz
        self.conf = conf
        self.available = False
        self._model = None
        try:
            from ultralytics import YOLO
        except ImportError:
            log.warning("ultralytics is not installed - perception will emit empty "
                        "Observations.  pip install -r requirements.txt")
            return
        try:
            self._model = YOLO(weights)
            self.available = True
            if weights.endswith(".pt"):
                log.warning("running the PyTorch model (%s).  On a CPU-only laptop, "
                            "export to OpenVINO first - python tools/export_model.py - "
                            "for the 2-3x that gets you from 8 to 20 fps (P-3).", weights)
        except Exception as exc:                        # pragma: no cover
            log.error("could not load %s: %s", weights, exc)

    def detect(self, frame_bgr):
        """Return ``(people, objects)`` in normalised coordinates."""
        if not self.available or frame_bgr is None:
            return ([], [])
        results = self._model.predict(frame_bgr, imgsz=self.imgsz, conf=self.conf,
                                      verbose=False)
        if not results:
            return ([], [])
        res = results[0]
        h, w = frame_bgr.shape[:2]

        people, objects = [], []
        boxes = getattr(res, "boxes", None)
        kps = getattr(res, "keypoints", None)

        if boxes is None:
            return ([], [])

        for i in range(len(boxes)):
            xywh = boxes.xywh[i].tolist()
            conf = float(boxes.conf[i])
            cls = int(boxes.cls[i]) if boxes.cls is not None else 0
            det = {"cx": xywh[0] / w, "cy": xywh[1] / h,
                   "w": xywh[2] / w, "h": xywh[3] / h,
                   "conf": conf, "cls": cls,
                   "label": res.names.get(cls, str(cls)) if hasattr(res, "names") else str(cls)}

            if cls == 0 and kps is not None and i < len(kps):
                named = {}
                data = kps.data[i]
                for j, name in enumerate(KEYPOINT_NAMES):
                    if j < len(data):
                        x, y, c = float(data[j][0]), float(data[j][1]), float(data[j][2])
                        named[name] = (x / w, y / h, c)
                det["keypoints"] = named
                people.append(det)
            else:
                objects.append(det)

        return (people, objects)


class Perceiver:
    """Turns frames into Observations.  One instance per run.

    Holds the only mutable perception state there is: the object track, the
    flow estimator and the time the subject was last seen.
    """

    def __init__(self, config: dict, detector: PoseDetector | None = None):
        self.config = config
        cam = config.get("camera", {})
        self.detector = detector if detector is not None else PoseDetector(
            imgsz=cam.get("work_width", 480))
        self.track = ObjectTrack()
        self.flow = BackgroundFlow()
        self.last_seen: float | None = None
        self.min_conf = config.get("pose", {}).get("min_keypoint_conf", 0.4)
        self.frame_count = 0
        self.detect_ms = 0.0
        # The detections behind the last Observation, kept so the goal
        # resolver can read them without paying for a second inference pass.
        # At 480 input on a CPU laptop that pass is 60-100 ms, which is the
        # whole difference between hitting P-3's 10 fps target and missing it.
        self.last_people: list = []
        self.last_objects: list = []

    def select_object(self, frame_bgr, box, label=""):
        self.track.select(frame_bgr, box, label)

    def observe(self, frame_bgr, t_local: float, t_frame: float, motion: dict) -> dict:
        """One frame -> one Observation.  This is the only public entry point."""
        obs = blank_observation(t_local)
        obs["t_frame"] = t_frame
        obs["gyro"] = {
            "rate_z": float(motion.get("rate_z", 0.0)),
            "pitch": float(motion.get("pitch", 0.0)),
            "roll": float(motion.get("roll", 0.0)),
        }

        t0 = time.perf_counter()
        people, objects = self.detector.detect(frame_bgr)
        self.detect_ms = (time.perf_counter() - t0) * 1000.0
        self.frame_count += 1
        self.last_people, self.last_objects = people, objects

        # The subject is the largest confident person.  v1 is one subject
        # (out of scope: multiple subjects), so this needs no identity model.
        subject = max(people, key=lambda p: p["w"] * p["h"], default=None)

        if subject is not None:
            self.last_seen = t_local
            obs["subject"] = {k: subject[k] for k in ("cx", "cy", "w", "h", "conf")}
            obs["keypoints"] = subject.get("keypoints")
            obs["shoulder_px"] = shoulder_px(subject.get("keypoints"), self.min_conf)
            obs["lost_for"] = 0.0
        else:
            obs["lost_for"] = (t_local - self.last_seen) if self.last_seen else 0.0

        gyro_dyaw = 0.0
        hfov = self.config.get("camera", {}).get("hfov_deg", 66.0)
        if hfov:
            gyro_dyaw = obs["gyro"]["rate_z"] / hfov * (1.0 / max(1.0, self.config
                        .get("camera", {}).get("frame_hz", 12)))

        obs["object"] = self.track.update(
            frame_bgr, objects, t_local, gyro_dyaw)

        subject_box = None
        if obs["subject"]:
            s = obs["subject"]
            subject_box = (s["cx"], s["cy"], s["w"], s["h"])
        obs["bg_flow"] = self.flow.update(frame_bgr, subject_box)

        return obs

    def resolve_object(self, phrase: str, objects: list) -> list:
        """G-6: match the noun phrase against current detections.

        Returns the candidates in preference order.  The caller draws numbered
        boxes and asks which number; ties are broken by size and centrality.
        Resolution happens BEFORE confirmation, so the confirmation is about a
        thing the rover can actually see.
        """
        words = {w for w in phrase.lower().split() if len(w) > 2}
        scored = []
        for det in objects:
            label = str(det.get("label", "")).lower()
            name_hit = any(w in label or label in w for w in words)
            size = det["w"] * det["h"]
            centrality = 1.0 - math.hypot(det["cx"] - 0.5, det["cy"] - 0.5)
            score = (2.0 if name_hit else 0.0) + size + 0.3 * centrality + 0.2 * det.get("conf", 0)
            scored.append((score, det))
        scored.sort(key=lambda pair: pair[0], reverse=True)
        return [det for _score, det in scored]


def decode_jpeg(data: bytes):
    """JPEG bytes -> BGR array, or None.  Kept here so the rest of the module
    never touches the wire format."""
    if cv2 is None or np is None:
        return None
    try:
        buf = np.frombuffer(data, dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)
    except Exception:                                   # pragma: no cover
        return None
