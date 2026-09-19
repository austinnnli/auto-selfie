"""record.py - log everything, one clock.

LG-1: frames as JPEG with timestamps, every Observation, every Command, all
telemetry, state transitions, spoken lines, and the active config hash.
Written on the laptop at receipt, with the phone's own frame timestamp carried
alongside.

Format is JSONL plus a ``frames/`` directory.  Not a database, not a binary
container: a run log you can grep, tail and diff at 3 a.m. in a field is worth
more than one you can query.  Each line is one record with a ``kind``.

The writer runs on its own thread with a bounded queue.  If logging ever falls
behind the control loop, records are DROPPED rather than blocking - a disk
stall must not stall the motors.  The count of dropped records is itself
logged, so a quiet loss cannot masquerade as a quiet run.
"""

from __future__ import annotations

import json
import logging
import pathlib
import queue
import threading
import time
from datetime import datetime

log = logging.getLogger("record")


class Recorder:
    """One run, one directory."""

    def __init__(self, config: dict, config_hash: str = "", root: str | None = None):
        rcfg = config.get("record", {})
        base = pathlib.Path(root or rcfg.get("dir", "logs"))
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.dir = base / stamp
        self.frames_dir = self.dir / "frames"
        self.save_frames = rcfg.get("save_frames", True)
        self.frame_every = max(1, int(rcfg.get("frame_every", 1)))

        self.dir.mkdir(parents=True, exist_ok=True)
        if self.save_frames:
            self.frames_dir.mkdir(exist_ok=True)

        self.t0 = time.monotonic()
        self.dropped = 0
        self.written = 0
        self._frame_index = 0
        self._q: queue.Queue = queue.Queue(maxsize=4096)
        self._stop = threading.Event()
        self._file = (self.dir / "run.jsonl").open("w", buffering=1 << 16)
        self._thread = threading.Thread(target=self._writer, name="record", daemon=True)
        self._thread.start()

        # LG-1: the active config hash, so a log can always be tied back to the
        # numbers that produced it.  A run whose gains cannot be reconstructed
        # is an anecdote, not a measurement.
        self.write("meta", {
            "started": datetime.now().isoformat(timespec="seconds"),
            "config_hash": config_hash,
            "config": config,
        })
        log.info("recording to %s", self.dir)

    # -- writing --------------------------------------------------------
    def write(self, kind: str, payload: dict):
        record = {"kind": kind, "t": time.monotonic() - self.t0}
        record.update(payload)
        try:
            self._q.put_nowait(record)
        except queue.Full:
            self.dropped += 1

    def observation(self, obs: dict):
        self.write("obs", {"obs": _jsonable(obs)})

    def command(self, cmd: dict, state_phase: str = ""):
        self.write("cmd", {"cmd": _jsonable(cmd), "phase": state_phase})

    def telemetry(self, tlm):
        if tlm is None:
            return
        self.write("tlm", {"tlm": {
            "status": tlm.status, "seq_echo": tlm.seq_echo,
            "tilt_steps": tlm.tilt_steps, "range_mm": tlm.range_mm,
            "vbat_mv": tlm.vbat_mv, "loop_us": tlm.loop_us}})

    def transition(self, frm: str, to: str, why: str):
        self.write("phase", {"from": frm, "to": to, "why": why})

    def said(self, text: str):
        self.write("say", {"text": text})

    def goal(self, spec: dict, tier: int, objected: bool = False):
        """G-9: every run logs the raw transcript, which tier parsed it, the
        resulting GoalSpec and whether the user objected.  This is the only way
        to measure parse quality offline and improve the tier-1 grammar."""
        self.write("goal", {"spec": _jsonable(spec), "tier": tier, "objected": objected})

    def frame(self, jpeg: bytes, t_phone: float, t_local: float) -> str | None:
        """Save a frame and return its filename, so the Observation record can
        point at the exact image it was computed from."""
        self._frame_index += 1
        if not self.save_frames or (self._frame_index % self.frame_every):
            return None
        name = f"{self._frame_index:06d}.jpg"
        try:
            (self.frames_dir / name).write_bytes(jpeg)
        except OSError as exc:
            log.warning("could not write frame: %s", exc)
            return None
        self.write("frame", {"file": name, "t_phone": t_phone, "t_local": t_local})
        return name

    def capture(self, jpeg: bytes, meta: dict):
        name = f"capture-{int(time.time())}.jpg"
        (self.dir / name).write_bytes(jpeg)
        self.write("capture", {"file": name, **_jsonable(meta)})
        log.info("saved %s (%.1f kB)", name, len(jpeg) / 1024)
        return name

    # -- lifecycle ------------------------------------------------------
    def _writer(self):
        while not self._stop.is_set() or not self._q.empty():
            try:
                record = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._file.write(json.dumps(record, separators=(",", ":")) + "\n")
                self.written += 1
            except (OSError, TypeError) as exc:
                log.warning("could not write record: %s", exc)

    def close(self):
        self.write("end", {"written": self.written, "dropped": self.dropped})
        self._stop.set()
        self._thread.join(timeout=3.0)
        try:
            self._file.close()
        except OSError:
            pass
        if self.dropped:
            log.warning("%d log records were dropped - the disk could not keep up",
                        self.dropped)
        log.info("run log closed: %d records in %s", self.written, self.dir)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def _jsonable(value):
    """Tuples become lists, numpy scalars become floats, everything else is
    left alone.  Keeps the log readable without a custom decoder."""
    if isinstance(value, dict):
        return {k: _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if hasattr(value, "item"):
        try:
            return value.item()
        except Exception:
            pass
    return str(value)


def read_log(path: str | pathlib.Path):
    """Iterate a run log.  Malformed lines are skipped, not fatal - a log
    truncated by a flat battery is still worth replaying."""
    path = pathlib.Path(path)
    if path.is_dir():
        path = path / "run.jsonl"
    with path.open() as handle:
        for line_no, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except ValueError:
                log.warning("%s:%d is not valid JSON, skipping", path, line_no)
