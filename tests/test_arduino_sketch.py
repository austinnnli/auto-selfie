"""The Arduino sketch's copies of the shared firmware must not drift.

firmware/main/ is the single source of truth.  Arduino IDE only compiles files
beside the .ino and will not reach outside the sketch folder, so the shared
sources are copied in by tools/gen_arduino_sketch.py.

A duplicate that nothing checks is a duplicate that will differ by next month -
and the failure mode is the worst kind, because both builds keep working while
slowly disagreeing about what the firmware does.  Same argument as protocol.h
against protocol.py.
"""

import pathlib
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SKETCH = ROOT / "firmware_arduino" / "framing_rover"

sys.path.insert(0, str(ROOT))
from tools.gen_arduino_sketch import SHARED, SRC, rendered  # noqa: E402


def test_the_sketch_folder_exists():
    assert SKETCH.is_dir()
    # Arduino IDE requires the sketch folder and the .ino to share a name.
    assert (SKETCH / f"{SKETCH.name}.ino").is_file(), (
        f"Arduino IDE needs {SKETCH.name}/{SKETCH.name}.ino")


@pytest.mark.parametrize("name", SHARED)
def test_each_shared_file_is_current(name):
    target = SKETCH / name
    assert target.exists(), f"{name} is missing from the sketch"
    assert target.read_text() == rendered(name), (
        f"{name} in the Arduino sketch differs from firmware/main/{name}.\n"
        f"Edit the original, then: python tools/gen_arduino_sketch.py")


@pytest.mark.parametrize("name", SHARED)
def test_each_copy_says_it_is_a_copy(name):
    head = (SKETCH / name).read_text()[:400]
    assert "DO NOT EDIT" in head and "gen_arduino_sketch" in head


def test_the_build_specific_files_are_not_copied():
    """Each build has its own entry point, network layer and HAL.  Copying one
    build's version into the other would not compile, and the generator must
    not be quietly widened to include them."""
    for name in ("main.c", "net.c", "hal_esp32.c"):
        assert name not in SHARED
        assert not (SKETCH / name).exists(), \
            f"{name} is ESP-IDF-specific and must not be in the sketch"


def test_the_arduino_specific_files_are_present():
    for name in ("framing_rover.ino", "hal_arduino.cpp", "net_arduino.cpp",
                 "secrets_example.h"):
        assert (SKETCH / name).is_file(), f"{name} is missing"


def test_real_credentials_are_not_committed():
    """secrets.h is gitignored; secrets_example.h is not, so it must stay a
    template."""
    example = (SKETCH / "secrets_example.h").read_text()
    assert "your-network" in example and "your-password" in example

    gitignore = (ROOT / ".gitignore").read_text()
    assert "firmware_arduino/framing_rover/secrets.h" in gitignore


def test_the_generator_check_mode_agrees():
    """The same check the Makefile runs, so `make check` and pytest cannot
    disagree about whether the sketch is current."""
    result = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "gen_arduino_sketch.py"), "--check"],
        capture_output=True, text=True, cwd=ROOT)
    assert result.returncode == 0, result.stderr


def test_both_builds_share_one_copy_of_the_task_logic():
    """The refactor that made the Arduino port safe: the five task bodies live
    in tasks.c, which both entry points call.  If main.c ever grows a task
    again, the two builds will drift in exactly the place it matters most."""
    main_c = (SRC / "main.c").read_text()
    assert "xTaskCreatePinnedToCore" not in main_c, \
        "main.c is creating tasks again; that belongs in tasks.c"
    assert "rover_start_tasks" in main_c

    ino = (SKETCH / "framing_rover.ino").read_text()
    assert "xTaskCreatePinnedToCore" not in ino
    assert "rover_start_tasks" in ino
