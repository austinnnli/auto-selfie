"""A kinematic rover, so the control loops can be closed without hardware.

Not a physics engine and not trying to be.  It models exactly the things the
control law has opinions about:

  - differential drive with a real L298N dead band, so a command below
    min_duty moves nothing, which is what makes D-2's compensation necessary
  - the pinhole projection of a subject and an object into a normalised frame,
    so image coordinates behave the way they do on the rover
  - shoulder width by similar triangles, which is the distance coordinate
  - a 28BYJ-48 that slews at a finite rate with backlash, so the tilt loop has
    something to close against
  - gyro rate and background optical flow, the two self-motion cues

Everything is in SI units inside and normalised image coordinates out.  The
subject stands at the world origin, which is also the rover's only reference.
"""

import math

# The world, in metres.
SUBJECT_HEIGHT_M = 1.72
SUBJECT_EYE_M = 1.60
SUBJECT_SHOULDER_M = 0.40
CAMERA_HEIGHT_M = 0.55
TRACK_M = 0.22            # wheel separation
MAX_SPEED_MS = 0.35       # at max_duty, before the deadband


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class RoverSim:
    def __init__(self, config, x=-2.6, y=0.0, heading=None,
                 object_xy=(4.5, 1.8), object_height_m=2.2):
        self.cfg = config
        hw = config["hardware"]
        cam = config["camera"]

        self.x, self.y = x, y
        # Point at the subject unless told otherwise.
        self.theta = heading if heading is not None else math.atan2(-y, -x)

        self.object_xy = object_xy
        self.object_height_m = object_height_m

        self.hfov = math.radians(cam["hfov_deg"])
        self.vfov = math.radians(cam["vfov_deg"])

        self.min_duty = (hw["min_duty_left"] + hw["min_duty_right"]) / 2.0
        self.max_duty = hw["max_duty"]
        self.steps_per_deg = hw["steps_per_degree"]
        self.max_step_rate = hw["max_step_rate"]
        self.backlash_steps = hw["backlash_steps"]

        self.tilt_deg = 0.0          # the true camera pitch, what the phone sees
        self.tilt_target = 0.0
        self.tilt_slack = 0.0        # where we sit inside the gear backlash
        self.last_dir = 1

        self.rate_z = 0.0
        self.prev_bearings = None

    # -- physics ---------------------------------------------------------
    def _wheel_speed(self, duty):
        """A duty below the bridge's dead band turns nothing at all."""
        if abs(duty) < self.min_duty:
            return 0.0
        span = max(1.0, self.max_duty - self.min_duty)
        mag = (abs(duty) - self.min_duty) / span
        return math.copysign(min(1.0, mag) * MAX_SPEED_MS, duty)

    def step(self, cmd, dt):
        if not cmd.get("enable"):
            left = right = 0.0
        else:
            left = self._wheel_speed(cmd["left"])
            right = self._wheel_speed(cmd["right"])

        v = (left + right) / 2.0
        # left faster than right turns the rover RIGHT, which is a decreasing
        # heading in a counter-clockwise-positive frame.
        omega = (right - left) / TRACK_M

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = wrap(self.theta + omega * dt)
        # Reported in the convention config.json's rate_z_sign = 1.0 assumes:
        # POSITIVE means turning right, matching decide.py's positive yaw
        # command.  omega is counter-clockwise-positive, hence the negation.
        # On real hardware this is a calibration (CF-2), because the phone's z
        # axis depends on how the mount is built - and getting it backwards
        # turns the yaw inner loop from a damper into an oscillator, which
        # tests/test_integration.py demonstrates.
        self.rate_z = -math.degrees(omega)

        self._step_tilt(cmd.get("tilt_deg", 0.0), dt)

    def _step_tilt(self, target_deg, dt):
        """A 28BYJ-48 with a finite slew rate and 1-2 degrees of gear slop."""
        hw = self.cfg["hardware"]
        target_deg = max(hw["tilt_min_deg"], min(hw["tilt_max_deg"], target_deg))
        self.tilt_target = target_deg

        max_deg = (self.max_step_rate / self.steps_per_deg) * dt
        error = target_deg - self.tilt_deg
        if abs(error) < 1e-9:
            return

        direction = 1 if error > 0 else -1
        move = min(abs(error), max_deg)

        # Reversing eats the backlash before anything moves at the output.
        if direction != self.last_dir:
            slack = self.backlash_steps / self.steps_per_deg
            eaten = min(move, slack)
            move -= eaten
            self.last_dir = direction
        self.tilt_deg += direction * move

    # -- what the camera sees --------------------------------------------
    def _project(self, wx, wy, height_m, base_m=0.0):
        dx, dy = wx - self.x, wy - self.y
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            return None

        bearing = wrap(math.atan2(dy, dx) - self.theta)
        # A target clockwise of the heading (negative bearing) appears on the
        # RIGHT of the image.
        cx = 0.5 - bearing / self.hfov

        centre_m = base_m + height_m / 2.0
        elevation = math.atan2(centre_m - CAMERA_HEIGHT_M, distance) \
            - math.radians(self.tilt_deg)
        cy = 0.5 - elevation / self.vfov

        h = height_m / (2.0 * distance * math.tan(self.vfov / 2.0))
        w = h * 0.35
        return {"cx": cx, "cy": cy, "w": w, "h": h, "conf": 0.9,
                "distance": distance, "bearing": bearing}

    def observation(self, t):
        """One Observation, in the exact shape perceive.py emits."""
        subject = self._project(0.0, 0.0, SUBJECT_HEIGHT_M)
        obj = self._project(self.object_xy[0], self.object_xy[1],
                            self.object_height_m, base_m=0.4)

        visible = subject is not None and 0.0 < subject["cx"] < 1.0
        obs = {
            "t": t, "t_frame": t - 0.15, "t_frame_local": t,
            "subject": None, "keypoints": None, "shoulder_px": None,
            "object": None, "bg_flow": None,
            "gyro": {"rate_z": self.rate_z, "pitch": self.tilt_deg, "roll": 0.0},
            "lost_for": 0.0,
        }

        if visible:
            distance = subject["distance"]
            shoulder = SUBJECT_SHOULDER_M / (2.0 * distance * math.tan(self.hfov / 2.0))

            eye_elev = math.atan2(SUBJECT_EYE_M - CAMERA_HEIGHT_M, distance) \
                - math.radians(self.tilt_deg)
            eye_y = 0.5 - eye_elev / self.vfov

            obs["subject"] = {k: subject[k] for k in ("cx", "cy", "w", "h", "conf")}
            obs["shoulder_px"] = shoulder
            obs["keypoints"] = self._keypoints(subject, shoulder, eye_y)

        if obj is not None and 0.0 < obj["cx"] < 1.0:
            obs["object"] = {k: obj[k] for k in ("cx", "cy", "w", "h", "conf")}

        # P-6: background flow, from how far the world slid between frames.
        bearings = (subject["bearing"] if subject else None,
                    obj["bearing"] if obj else None)
        if self.prev_bearings and bearings[1] is not None and self.prev_bearings[1] is not None:
            obs["bg_flow"] = (-(bearings[1] - self.prev_bearings[1]) / self.hfov, 0.0)
        self.prev_bearings = bearings

        return obs

    def _keypoints(self, subject, shoulder, eye_y):
        cx = subject["cx"]
        half = shoulder / 2.0
        return {
            "nose": (cx, eye_y + 0.012, 0.95),
            "left_eye": (cx + 0.012, eye_y, 0.93),
            "right_eye": (cx - 0.012, eye_y, 0.93),
            "left_shoulder": (cx + half, eye_y + 0.085, 0.92),
            "right_shoulder": (cx - half, eye_y + 0.085, 0.92),
            "left_elbow": (cx + half + 0.010, eye_y + 0.185, 0.88),
            "right_elbow": (cx - half - 0.010, eye_y + 0.185, 0.88),
            "left_wrist": (cx + half + 0.018, eye_y + 0.280, 0.85),
            "right_wrist": (cx - half - 0.018, eye_y + 0.280, 0.85),
            "left_hip": (cx + half * 0.7, eye_y + 0.330, 0.90),
            "right_hip": (cx - half * 0.7, eye_y + 0.330, 0.90),
        }

    # -- diagnostics ------------------------------------------------------
    @property
    def distance_to_subject(self):
        return math.hypot(self.x, self.y)
