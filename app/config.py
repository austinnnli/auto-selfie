"""config.py - load config.json and hot-reload it on change.

CF-1: everything tunable lives in config.json, hot-reloaded.  No gain is ever a
literal in code.

Hot reload matters more than it sounds: the whole tuning workflow in the PRD's
working rules is "one variable per tuning run, logged".  Restarting the control
app between runs would mean re-acquiring the subject and re-running the sweep
every time, which is how a two-hour tuning session becomes a day.

The reload is atomic from the loop's point of view: a bad edit leaves the
previous config in place and logs the parse error, so a stray comma cannot stop
a rover mid-manoeuvre.
"""

from __future__ import annotations

import hashlib
import json
import logging
import pathlib
import threading

log = logging.getLogger("config")

DEFAULT_PATH = pathlib.Path(__file__).resolve().parents[1] / "config.json"


class Config:
    """A dict-alike that re-reads the file when its mtime changes."""

    def __init__(self, path: pathlib.Path | str = DEFAULT_PATH, poll_s: float = 0.5):
        self.path = pathlib.Path(path)
        self.poll_s = poll_s
        self._lock = threading.Lock()
        self._mtime = 0.0
        self._data: dict = {}
        self._hash = ""
        self._last_error: str | None = None
        self.reload(force=True)

    # -- access ---------------------------------------------------------
    @property
    def data(self) -> dict:
        with self._lock:
            return self._data

    @property
    def hash(self) -> str:
        """Short digest of the active config, logged with every run (LG-1) so a
        recorded session can be tied back to the numbers that produced it."""
        with self._lock:
            return self._hash

    def __getitem__(self, key):
        return self.data[key]

    def get(self, key, default=None):
        return self.data.get(key, default)

    # -- reload ---------------------------------------------------------
    def reload(self, force: bool = False) -> bool:
        """Re-read if the file changed.  Returns True if the data changed."""
        try:
            mtime = self.path.stat().st_mtime
        except OSError as exc:
            if self._last_error != str(exc):
                log.warning("config unreadable (%s); keeping the loaded copy", exc)
                self._last_error = str(exc)
            return False

        if not force and mtime == self._mtime:
            return False

        try:
            raw = self.path.read_bytes()
            data = json.loads(raw)
        except (OSError, ValueError) as exc:
            # A stray comma must not stop a rover mid-manoeuvre.
            if self._last_error != str(exc):
                log.error("config.json is not valid JSON (%s); keeping the previous "
                          "values - fix the file and it will reload itself", exc)
                self._last_error = str(exc)
            self._mtime = mtime
            return False

        digest = hashlib.sha256(raw).hexdigest()[:16]
        with self._lock:
            changed = digest != self._hash
            self._data = data
            self._hash = digest
            self._mtime = mtime
        self._last_error = None
        if changed and not force:
            log.info("config reloaded (%s)", digest)
        return changed

    def start_watch(self) -> threading.Thread:
        """Poll in the background.  Polling rather than inotify: it is four
        lines, works the same on Windows, and half a second of latency on a
        tuning edit is imperceptible."""
        stop = threading.Event()
        self._stop = stop

        def loop():
            while not stop.wait(self.poll_s):
                self.reload()

        thread = threading.Thread(target=loop, name="config-watch", daemon=True)
        thread.start()
        return thread

    def stop_watch(self) -> None:
        stop = getattr(self, "_stop", None)
        if stop is not None:
            stop.set()


def load(path: pathlib.Path | str = DEFAULT_PATH) -> Config:
    return Config(path)
