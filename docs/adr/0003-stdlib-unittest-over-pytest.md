# ADR-0003: stdlib unittest + ffmpeg subprocess over pytest/imageio

## Status
Accepted — 2026-06-29

## Context
`AGENTS.md` declares "No tests, no CI, no package manifests." We wanted to
drive the pure-logic design test-first (TDD) without contradicting that
stance, and we needed a way to verify the MP4 encoder.

Available runtimes on this host:

| Runtime | numpy | PIL | imageio | pytest | ffmpeg |
|---|---|---|---|---|---|
| system python (miniconda base, 3.13) | no | no | no | no | yes (4.4.2) |
| Isaac Sim bundled python (3.12) | yes | yes | yes (2.37.2) | no | yes |

`imageio` is only importable inside Isaac Sim's python, which takes ~30s and
a GPU to launch — far too heavy for a fast unit-test loop. `pytest` is not
installed in either python.

## Decision
- Use the **stdlib `unittest` module** for tests. No pytest, no conftest, no
  manifest. Run with `python3 -m unittest discover -s tests -v` under system
  python — ~1s, no GPU, no Isaac Sim.
- Pure logic lives in **dependency-free submodules**
  (`common/geometry.py`, `common/stability.py`) — plain tuples and `math`,
  no numpy/Isaac imports — so tests can import them without launching the app.
- `common/video.py` shells out to **`ffmpeg` via `subprocess`** instead of
  using `imageio`. Works in any python, unit-testable with stdlib, and
  produces verified MP4s (tests round-trip through `ffprobe`).
- Isaac-Sim-bound code (camera/RP/writer/loop/asset) is **integration-tested
  by running the scenario scripts** (`$ISAAC_SIM/python.sh
  samples/simulations/<script>.py`), not by unit tests.

## Consequences
- The `tests/` directory and the stdlib runner are the only test
  infrastructure. This bends "no tests" but preserves "no framework, no
  manifest, no CI" — `unittest` is in the standard library, so there is
  nothing to install.
- `common/video.py` cannot use `imageio`'s convenience helpers; the ffmpeg
  command lines are explicit and slightly more verbose. In exchange the
  encoder is testable from system python and has no Isaac Sim dependency.
- The pure-logic / Isaac-bound split is a durable convention: any future
  `common/` logic worth unit-testing goes in a no-deps submodule with a
  matching `tests/test_*.py`; Isaac-bound glue stays in the parent module
  and is verified by running a scenario.
- Running `python3 -m unittest discover -s tests` is the verification step
  for pure logic; running each scenario script is the integration step.
  Both are documented in `AGENTS.md`.
