"""record.py + replay.py round trip.

M7's acceptance test: "a logged run re-runs at 10x with identical output".

LG-2 is emphatic that replay is built during the video-pipeline milestone, not
at the end - built early it accelerates every later milestone, built late it is
never built at all.  This test is what keeps it working once it exists.

No hardware, no phone, no model: a synthetic run is written to a temporary
directory and fed straight back through the same decide() the live loop uses.
"""

import json
import pathlib
import shutil
import tempfile

import pytest

from app import decide, record, replay, session
from tests.conftest import framing_goal, make_obs


def write_run(tmp, cfg, ticks=60):
    """Record a synthetic run: a subject drifting across the frame while the
    rover frames them, which exercises the yaw axis, the rate limiter and the
    tilt loop over a realistic number of ticks."""
    rec = record.Recorder(cfg, config_hash="test", root=str(tmp))
    goal = framing_goal()
    rec.goal(goal, tier=1)

    # Start exactly where replay.py starts: a clean state with the goal
    # locked and the phase taken from the log.  The log carries observations,
    # commands and transitions - not the decision layer's internals - so the
    # only way a replay can be exact is for both sides to begin from the same
    # initial state and be driven purely by what was recorded.  That is also
    # true of a real run, which always starts from Idle.
    state = decide.initial_state(0.0, goal, decide.PHASE_FRAME)
    rec.transition(session.IDLE, session.FRAME, "synthetic")

    commands = []
    for i in range(ticks):
        t = i * 0.04
        obs = make_obs(t=t,
                       subject=(0.20 + 0.008 * i, 0.55, 0.18, 0.62),
                       obj=(0.60 + 0.004 * i, 0.40, 0.14, 0.32),
                       shoulder_px=0.120 + 0.0004 * i,
                       eyeline=0.30 + 0.001 * i,
                       rate_z=1.5)
        rec.observation(obs)
        cmd, state = decide.step(obs, state, cfg)
        rec.command(cmd, state["phase"])
        commands.append(cmd)

    rec.close()
    return rec.dir, commands


@pytest.fixture
def tmp_logs():
    path = pathlib.Path(tempfile.mkdtemp(prefix="rover-replay-"))
    yield path
    shutil.rmtree(path, ignore_errors=True)


def test_a_logged_run_replays_to_identical_output(tmp_logs, cfg):
    """M7.  Byte-for-byte the same commands, from the log alone."""
    log_dir, expected = write_run(tmp_logs, cfg)

    stats, rows = replay.replay(log_dir, speed=0)
    assert stats["decisions"] == len(expected)

    for i, (row, want) in enumerate(zip(rows, expected)):
        assert row["cmd"] == want, f"tick {i} replayed differently"


def test_the_diff_mode_reports_no_mismatches(tmp_logs, cfg):
    """The everyday workflow: change a gain, see exactly which frames now
    behave differently.  With nothing changed, nothing should differ."""
    log_dir, _expected = write_run(tmp_logs, cfg)
    stats, _rows = replay.replay(log_dir, speed=0, diff=True)
    assert stats["mismatches"] == 0
    assert stats["recorded_commands"] == stats["decisions"]


def test_a_changed_gain_shows_up_as_mismatches(tmp_logs, cfg):
    """The other half of the workflow: if a gain moves, the diff must say so.
    A replay that silently agreed with everything would be useless."""
    log_dir, _expected = write_run(tmp_logs, cfg)

    tuned = json.loads(json.dumps(cfg))
    tuned["control"]["yaw"]["kp"] = cfg["control"]["yaw"]["kp"] * 3.0
    tuned_path = log_dir / "config.tuned.json"
    tuned_path.write_text(json.dumps(tuned))

    stats, _rows = replay.replay(log_dir, config=str(tuned_path), speed=0, diff=True)
    assert stats["mismatches"] > 0, "tripling the yaw gain changed nothing"


def test_replay_uses_the_config_that_produced_the_run(tmp_logs, cfg):
    """LG-1 logs the active config so a run can be tied back to the numbers
    that produced it.  Replaying with today's config by accident is how you
    spend an afternoon chasing a difference you introduced yourself."""
    log_dir, _expected = write_run(tmp_logs, cfg)
    stats, _rows = replay.replay(log_dir, speed=0)
    assert stats["config_hash"] == "test"


def test_the_log_is_readable_line_by_line(tmp_logs, cfg):
    """A run log you can grep and tail in a field is worth more than one you
    can query."""
    log_dir, _expected = write_run(tmp_logs, cfg, ticks=10)
    lines = (log_dir / "run.jsonl").read_text().strip().split("\n")
    kinds = [json.loads(line)["kind"] for line in lines]

    assert kinds[0] == "meta"
    assert kinds[-1] == "end"
    assert kinds.count("obs") == 10
    assert kinds.count("cmd") == 10
    assert "goal" in kinds and "phase" in kinds


def test_a_truncated_log_still_replays(tmp_logs, cfg):
    """A log cut short by a flat battery is still worth replaying, so a
    malformed final line must be skipped rather than fatal."""
    log_dir, _expected = write_run(tmp_logs, cfg, ticks=20)
    path = log_dir / "run.jsonl"
    path.write_text(path.read_text() + '{"kind": "obs", "t": 9.9, "obs": {tru')

    stats, rows = replay.replay(log_dir, speed=0)
    assert len(rows) == 20


def test_the_recorder_logs_the_goal_for_parse_quality_measurement(tmp_logs, cfg):
    """G-9: every run logs the raw transcript, which tier parsed it, the
    resulting GoalSpec and whether the user objected.  This is the only way to
    measure parse quality offline and improve the tier-1 grammar over time."""
    rec = record.Recorder(cfg, config_hash="test", root=str(tmp_logs))
    spec = framing_goal(raw="frame me next to the big tree")
    rec.goal(spec, tier=1, objected=False)
    rec.close()

    goals = [r for r in record.read_log(rec.dir) if r["kind"] == "goal"]
    assert len(goals) == 1
    assert goals[0]["tier"] == 1
    assert goals[0]["objected"] is False
    assert goals[0]["spec"]["raw"] == "frame me next to the big tree"


def test_dropped_records_are_themselves_recorded(tmp_logs, cfg):
    """If logging ever falls behind the control loop, records are dropped
    rather than blocking - a disk stall must not stall the motors.  The count
    is logged so a quiet loss cannot masquerade as a quiet run."""
    rec = record.Recorder(cfg, config_hash="test", root=str(tmp_logs))
    rec.close()
    end = [r for r in record.read_log(rec.dir) if r["kind"] == "end"][0]
    assert "dropped" in end and "written" in end
