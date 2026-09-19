#!/usr/bin/env python3
"""Run the test suite with nothing installed.

    python tools/run_tests.py                 # everything
    python tools/run_tests.py -k backlash     # by name
    python tools/run_tests.py -v

pytest is the primary runner and everything in ``tests/`` is written for it -
``pytest -q`` from the repository root does the same thing.  This exists
because the control laptop is often a fresh machine on a field network with no
package index reachable, and "I cannot run the tests until pip works" is a bad
place to be when the rover is behaving oddly.

It implements the small slice of pytest the suite actually uses: fixtures from
``conftest.py`` (including ``scope="session"``), ``@pytest.mark.parametrize``,
``pytest.raises``, ``pytest.approx``, ``pytest.fail`` and ``pytest.skip``.
Anything beyond that should go through real pytest.
"""

from __future__ import annotations

import argparse
import importlib.util
import inspect
import pathlib
import sys
import traceback
import types

ROOT = pathlib.Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------------------
# the shim
# ---------------------------------------------------------------------------

class _Approx:
    def __init__(self, expected, rel=None, abs=None):
        self.expected = expected
        self.rel = rel if rel is not None else 1e-6
        self.abs = abs if abs is not None else 1e-12

    def _close(self, a, b):
        return abs(a - b) <= max(self.abs, self.rel * max(abs(a), abs(b)))

    def __eq__(self, other):
        if isinstance(self.expected, (list, tuple)):
            return (len(other) == len(self.expected)
                    and all(self._close(o, e) for o, e in zip(other, self.expected)))
        if isinstance(self.expected, dict):
            return (set(other) == set(self.expected)
                    and all(self._close(other[k], v) for k, v in self.expected.items()))
        return self._close(other, self.expected)

    def __repr__(self):
        return f"approx({self.expected!r}, rel={self.rel})"


class _Failed(AssertionError):
    pass


class _Skipped(Exception):
    pass


class _Raises:
    def __init__(self, expected, match=None):
        self.expected = expected
        self.match = match
        self.value = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        if exc_type is None:
            raise _Failed(f"DID NOT RAISE {self.expected}")
        if not issubclass(exc_type, self.expected):
            return False
        self.value = exc
        if self.match:
            import re
            if not re.search(self.match, str(exc)):
                raise _Failed(f"{exc!r} does not match {self.match!r}")
        return True


def _make_shim() -> types.ModuleType:
    mod = types.ModuleType("pytest")

    def fixture(func=None, *, scope="function", **_kw):
        def wrap(f):
            f.__fixture__ = {"scope": scope}
            return f
        return wrap(func) if func is not None else wrap

    class _Mark:
        @staticmethod
        def parametrize(argnames, argvalues, **_kw):
            names = [n.strip() for n in argnames.split(",")] \
                if isinstance(argnames, str) else list(argnames)

            def wrap(func):
                cases = getattr(func, "__parametrize__", [])
                func.__parametrize__ = cases + [(names, list(argvalues))]
                return func
            return wrap

        def __getattr__(self, _name):
            def noop(*_a, **_k):
                return lambda f: f
            return noop

    mod.fixture = fixture
    mod.mark = _Mark()
    mod.approx = lambda expected, rel=None, abs=None: _Approx(expected, rel, abs)
    mod.raises = lambda expected, match=None: _Raises(expected, match)
    mod.fail = lambda msg="": (_ for _ in ()).throw(_Failed(msg))
    mod.skip = lambda msg="": (_ for _ in ()).throw(_Skipped(msg))
    mod.Failed = _Failed
    mod.__shim__ = True
    return mod


# ---------------------------------------------------------------------------
# collection and running
# ---------------------------------------------------------------------------

def _rel(path: pathlib.Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def load_module(path: pathlib.Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class Runner:
    def __init__(self, verbose=False, keyword=None):
        self.verbose = verbose
        self.keyword = keyword
        self.fixtures: dict = {}
        self.session_cache: dict = {}
        self.passed = self.failed = self.skipped = 0
        self.failures: list = []

    def load_fixtures(self, module):
        for name, obj in vars(module).items():
            if callable(obj) and hasattr(obj, "__fixture__"):
                self.fixtures[name] = obj

    def resolve(self, name, seen=None):
        if name not in self.fixtures:
            raise LookupError(f"no fixture named {name!r}")
        func = self.fixtures[name]
        scope = func.__fixture__["scope"]
        if scope == "session" and name in self.session_cache:
            return self.session_cache[name]

        seen = (seen or set()) | {name}
        kwargs = {}
        for param in inspect.signature(func).parameters:
            if param in seen:
                raise LookupError(f"fixture cycle at {param!r}")
            kwargs[param] = self.resolve(param, seen)

        value = func(**kwargs)
        if inspect.isgenerator(value):
            value = next(value)
        if scope == "session":
            self.session_cache[name] = value
        return value

    def cases_for(self, func):
        """Expand parametrize marks into concrete argument sets."""
        marks = getattr(func, "__parametrize__", [])
        if not marks:
            return [({}, "")]
        cases = [({}, "")]
        for names, values in reversed(marks):
            expanded = []
            for kwargs, label in cases:
                for value in values:
                    tup = value if isinstance(value, tuple) else (value,)
                    if len(names) == 1 and not isinstance(value, tuple):
                        tup = (value,)
                    merged = dict(kwargs)
                    merged.update(dict(zip(names, tup)))
                    ident = "-".join(str(v) for v in tup)
                    expanded.append((merged, f"{label}[{ident}]" if label else f"[{ident}]"))
            cases = expanded
        return cases

    def run_module(self, path: pathlib.Path):
        module = load_module(path, f"tests_{path.stem}")
        self.load_fixtures(module)

        tests = [(n, o) for n, o in sorted(vars(module).items())
                 if n.startswith("test_") and callable(o)]
        if not tests:
            return

        print(f"\n{_rel(path)}")
        for name, func in tests:
            for kwargs, label in self.cases_for(func):
                full = name + label
                if self.keyword and self.keyword not in full:
                    continue
                self.run_one(path, full, func, kwargs)

    def run_one(self, path, full, func, kwargs):
        call_kwargs = dict(kwargs)
        try:
            for param in inspect.signature(func).parameters:
                if param not in call_kwargs:
                    call_kwargs[param] = self.resolve(param)
        except LookupError as exc:
            self.failed += 1
            self.failures.append((path, full, f"fixture error: {exc}"))
            print(f"  ERROR {full}  ({exc})")
            return

        try:
            func(**call_kwargs)
        except _Skipped as exc:
            self.skipped += 1
            print(f"  skip  {full}  ({exc})")
        except Exception:
            self.failed += 1
            tb = traceback.format_exc()
            self.failures.append((path, full, tb))
            print(f"  FAIL  {full}")
        else:
            self.passed += 1
            if self.verbose:
                print(f"  ok    {full}")

    def report(self):
        if self.failures:
            print("\n" + "=" * 72)
            for path, name, tb in self.failures:
                print(f"\nFAILED {_rel(path)}::{name}")
                print("-" * 72)
                print(tb.rstrip() if "\n" in tb else tb)
        total = self.passed + self.failed + self.skipped
        print("\n" + "=" * 72)
        print(f"{total} tests: {self.passed} passed, {self.failed} failed, "
              f"{self.skipped} skipped")
        return 1 if self.failed else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("-k", dest="keyword", default=None)
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("paths", nargs="*", default=None)
    args = parser.parse_args()

    if "pytest" not in sys.modules:
        try:
            import pytest  # noqa: F401
            print("real pytest is installed - `pytest -q` is the better runner\n")
        except ImportError:
            sys.modules["pytest"] = _make_shim()

    sys.path.insert(0, str(ROOT))
    runner = Runner(verbose=args.verbose, keyword=args.keyword)

    conftest = ROOT / "tests" / "conftest.py"
    if conftest.exists():
        runner.load_fixtures(load_module(conftest, "tests.conftest"))

    paths = [pathlib.Path(p).resolve() for p in args.paths] if args.paths \
        else sorted((ROOT / "tests").glob("test_*.py"))
    for path in paths:
        runner.run_module(path)
    return runner.report()


if __name__ == "__main__":
    raise SystemExit(main())
