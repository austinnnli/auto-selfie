"""The purity guard.

Design rule 1: ``decide(obs, state, config)`` imports nothing but ``math``.  No
imaging library, no I/O, no globals.

That rule is what makes the golden corpus meaningful, what lets the decision
layer be replayed at 10x against logs, and what makes the later transliteration
to JavaScript a two-day job instead of a two-week one.  A rule that valuable
should not rest on everyone remembering it, so this test reads the source.

It is a static check on purpose: a runtime check would only catch the import
paths a test happened to execute.
"""

import ast
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]

# The pure modules, and what each is allowed to import.  Pure siblings are
# allowed because importing one cannot introduce impurity - the whole set is
# checked by this same test.
PURE_MODULES = {
    "decide.py": {"math", "app.pose", "app.coach"},
    "pose.py": {"math"},
    "coach.py": {"math"},
    "goal.py": {"math"},
    "session.py": {"math", "app.coach", "app.decide", "app.goal"},
}

# Names that mean this module has stopped being pure.
FORBIDDEN_CALLS = {
    "open", "input", "print", "exec", "eval", "compile", "__import__",
    "globals", "locals", "vars", "breakpoint",
}


def parse(name):
    path = ROOT / "app" / name
    return path, ast.parse(path.read_text(), filename=str(path))


@pytest.mark.parametrize("module", sorted(PURE_MODULES))
def test_imports_are_on_the_allowlist(module):
    allowed = PURE_MODULES[module]
    path, tree = parse(module)
    found = set()

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                found.add(alias.name)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                pytest.fail(f"{module}: relative import at line {node.lineno}")
            base = node.module or ""
            for alias in node.names:
                found.add(f"{base}.{alias.name}" if base else alias.name)

    extra = found - allowed
    assert not extra, (
        f"{module} imports {sorted(extra)}, which breaks design rule 1.\n"
        f"Allowed: {sorted(allowed)}.\n"
        f"If a new dependency is genuinely needed, it belongs in an impure "
        f"module and the value should be passed in as an argument.")


@pytest.mark.parametrize("module", sorted(PURE_MODULES))
def test_no_forbidden_calls(module):
    path, tree = parse(module)
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            assert node.func.id not in FORBIDDEN_CALLS, (
                f"{module}:{node.lineno} calls {node.func.id}() - "
                f"the pure layer does no I/O")


@pytest.mark.parametrize("module", sorted(PURE_MODULES))
def test_no_global_statements(module):
    """"No globals" in the PRD means no mutable module state.  Module-level
    CONSTANTS are fine and necessary; rebinding one at runtime is not."""
    path, tree = parse(module)
    for node in ast.walk(tree):
        assert not isinstance(node, ast.Global), (
            f"{module}:{node.lineno} uses `global` - the pure layer keeps its "
            f"state in the State dict the caller owns")
        assert not isinstance(node, ast.Nonlocal), (
            f"{module}:{node.lineno} uses `nonlocal`")


@pytest.mark.parametrize("module", sorted(PURE_MODULES))
def test_module_level_state_is_constant(module):
    """Every module-level binding is either a CONSTANT, a function or a class."""
    path, tree = parse(module)
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.ClassDef, ast.Import,
                             ast.ImportFrom, ast.Expr)):
            continue
        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            names = []
            for target in targets:
                # `A, B = "a", "b"` is still a pair of constants.
                if isinstance(target, (ast.Tuple, ast.List)):
                    names.extend(target.elts)
                else:
                    names.append(target)
            for name in names:
                assert isinstance(name, ast.Name), (
                    f"{module}:{node.lineno} has a module-level assignment that is "
                    f"not a plain name")
                assert name.id.isupper() or name.id.startswith("_"), (
                    f"{module}:{node.lineno} binds module-level `{name.id}`. "
                    f"Module state must be a CONSTANT (upper case).")
            continue
        pytest.fail(f"{module}:{node.lineno} has unexpected module-level "
                    f"{type(node).__name__}")


def test_decide_runs_with_plain_dictionaries():
    """The practical form of the rule: the whole decision layer must work on
    dictionaries a test can type out, with no objects and no setup."""
    import json
    import sys
    sys.path.insert(0, str(ROOT))
    from app import decide

    cfg = json.loads((ROOT / "config.json").read_text())
    obs = {"t": 0.0, "t_frame": 0.0, "subject": None, "keypoints": None,
           "shoulder_px": None, "object": None, "bg_flow": None,
           "gyro": {"rate_z": 0.0, "pitch": 0.0, "roll": 0.0}, "lost_for": 0.0}
    state = decide.initial_state(0.0)
    cmd = decide.decide(obs, state, cfg)
    assert set(cmd) == {"left", "right", "tilt_deg", "enable", "capture", "say"}


def test_decide_does_not_mutate_its_arguments():
    """``step()`` returns a new state; the caller's copy must be untouched.

    Mutating in place would work fine right up to the first time something
    replays a log or forks a state to compare two gains, and then it would
    produce results that are quietly wrong."""
    import copy
    import json
    import sys
    sys.path.insert(0, str(ROOT))
    from app import decide
    from tests.conftest import framing_goal, make_obs

    cfg = json.loads((ROOT / "config.json").read_text())
    obs = make_obs(subject=(0.6, 0.55, 0.18, 0.62))
    state = decide.initial_state(0.0, framing_goal(), decide.PHASE_FRAME)

    obs_before = copy.deepcopy(obs)
    state_before = copy.deepcopy(state)
    cfg_before = copy.deepcopy(cfg)

    decide.step(obs, state, cfg)

    assert obs == obs_before, "decide.step mutated the Observation"
    assert state == state_before, "decide.step mutated the State it was given"
    assert cfg == cfg_before, "decide.step mutated the Config"
