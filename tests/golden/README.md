# tests/golden

LG-3's corpus: recorded observations with the command produced for each.

```
python tools/gen_golden.py          # re-record
python tools/gen_golden.py --check  # verify without writing
```

## What this is for

Any refactor, re-tune or future port to JavaScript must reproduce these
commands. A mismatch is then a code bug found at a desk instead of a mystery
found in a field. When `decide.py`, `pose.py` or `coach.py` is transliterated
to JavaScript for a laptop-free build, this file is the acceptance test: load
it, run the JS decision layer over `state_in` and `obs`, compare `cmd_out`.

## What this is *not* for

A golden corpus records what the code **does**. It cannot know what the code
**should** do. Running the generator after a change will always make the tests
pass again, so on its own the corpus proves only that nothing changed by
accident.

What the code should do lives in `tests/test_decide_semantics.py`, where the
expected values are worked out by hand from the geometry and the PRD. The two
files divide the work:

| | says what | fails when |
|---|---|---|
| `test_decide_semantics.py` | the behaviour is *correct* | a sign flips, a gain is misapplied, a rule is dropped |
| `test_golden.py` | the behaviour is *unchanged* | anything at all moves, including things no one thought to assert |

**The rule: never re-bless a golden mismatch until the semantic tests still
pass and the failing case's `why` line has been read and agreed with.** The
failure message prints that line for exactly this reason.

## Structure

Each case is one call to `decide.step()` with no hidden setup:

| field | meaning |
|---|---|
| `name` | stable identifier; also the test id |
| `why` | what this case exists to pin, in plain English |
| `state_in` | the full session state, already warmed up |
| `obs` | one Observation, in the shape `perceive.py` emits |
| `cmd_out` | the expected Command |
| `state_out` | the slice of the evolved state that is pinned |

Cases that need history — a rate limit part-way through a ramp, a sweep
mid-collection, a feasibility window that has already filled — are warmed up by
the generator and the warmed state is frozen into `state_in`. Nothing in this
file depends on being replayed in order.

`state_out` pins a deliberate subset (see `STATE_KEYS` in the generator):
the sweep's progress, the tilt setpoint, the rate-limit memory, the coach's
hysteresis latch and the pose error. Internal bookkeeping that no downstream
module reads is left out, so the corpus is not brittle for its own sake.

## Coverage

The corpus is checked for coverage as well as agreement: every session phase
appears, motion and holding are both represented, speech is represented, and
no case may command a duty inside the L298N's dead band. Those checks live in
`test_golden.py` so that a future edit cannot quietly narrow the corpus.
