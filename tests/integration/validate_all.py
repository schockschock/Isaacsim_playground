"""Validate all 27 IC-grid runs across freefall, conveyor, object_throw.

For each of 3 scenarios × 3 velocity levels × 3 poses = 27 runs, loads the
per-run HDF5 and runs the five assertion families (mass-invariant, energy
conservation, finite-difference, freefall-analytic where applicable,
contact-energy agreement). Prints a per-run pass/fail table and exits
non-zero on any failure.

Also accepts --run to execute the missing simulations before validating.

Usage:
  python3 tests/integration/validate_all.py
  python3 tests/integration/validate_all.py --run   # run missing sims first
"""
import os
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ISAAC_PYTHON = os.environ.get("ISAAC_SIM", "/home/adrien/isaacsim") + "/python.sh"
OUTPUT_BASE = os.path.join(REPO_ROOT, "_output")

SCENARIOS = ["freefall", "conveyor", "object_throw"]
N_V = 3
N_P = 3

ASSERTIONS = ["mass", "energy", "fd-lin", "fd-ang", "analytic", "contact-energy"]


def h5_path(scenario, v, p):
    return os.path.join(OUTPUT_BASE, scenario, f"v{v}_p{p}",
                        f"{scenario}_v{v}_p{p}.h5")


def run_sim(scenario, v, p):
    script = os.path.join(REPO_ROOT, "simulation_scripts", f"{scenario}.py")
    proc = subprocess.run(
        [ISAAC_PYTHON, script, "--run", str(v), str(p)],
        capture_output=True, text=True, timeout=1200,
    )
    return proc.returncode == 0


def run_validation(scenario, v, p):
    path = h5_path(scenario, v, p)
    if not os.path.isfile(path):
        return {"_error": "MISSING"}
    vscript = os.path.join(REPO_ROOT, "tests", "integration",
                            f"validate_{scenario}.py")
    proc = subprocess.run(
        [sys.executable, vscript, path],
        capture_output=True, text=True,
    )
    results = {}
    for line in (proc.stdout + proc.stderr).splitlines():
        if "[PASS]" in line:
            parts = line.split("[PASS]", 1)[1]
            name = parts.split(":")[0].strip()
            results[name] = "PASS"
        elif "[FAIL]" in line:
            parts = line.split("[FAIL]", 1)[1]
            name = parts.split(":")[0].strip()
            results[name] = "FAIL"
        elif "[SKIP]" in line:
            parts = line.split("[SKIP]", 1)[1]
            name = parts.split(":")[0].strip()
            results[name] = "SKIP"
    return results


def main():
    do_run = "--run" in sys.argv
    results = {}

    for scenario in SCENARIOS:
        print(f"\n{'='*60}")
        print(f"  {scenario}")
        print(f"{'='*60}")
        for v in range(N_V):
            for p in range(N_P):
                label = f"{scenario} v{v}p{p}"
                path = h5_path(scenario, v, p)

                if not os.path.isfile(path) and do_run:
                    print(f"  [{label}] running simulation ...", end=" ")
                    sys.stdout.flush()
                    ok = run_sim(scenario, v, p)
                    print("OK" if ok else "FAILED")
                    if not ok:
                        results[label] = {"_status": "SIM_FAILED"}
                        continue

                if not os.path.isfile(path):
                    results[label] = {"_status": "MISSING"}
                    continue

                print(f"  [{label}] validating ...", end=" ")
                sys.stdout.flush()
                r = run_validation(scenario, v, p)
                status = "OK" if all(v == "PASS" or v == "SKIP"
                                      for v in r.values()) else "FAIL"
                r["_status"] = status
                results[label] = r
                print(status)

    n_ok = sum(1 for r in results.values()
               if r.get("_status") in ("OK",))
    n_total = len(SCENARIOS) * N_V * N_P

    print(f"\n{'='*90}")
    print("  Validation matrix (pass/fail)")
    print(f"{'='*90}")
    header = f"{'Run':<22} | {'mass':>6} | {'energy':>7} | {'fd-lin':>7} | {'fd-ang':>7} | {'analytic':>9} | {'contact':>9} | status"
    print(header)
    print("-" * len(header))

    def field(r, name):
        v = r.get(name, "?")
        return (v if isinstance(v, str) else "?").rjust(7 if name != "analytic" and name != "contact-energy" else 9)

    for scenario in SCENARIOS:
        for v in range(N_V):
            for p in range(N_P):
                label = f"{scenario} v{v}p{p}"
                r = results.get(label, {"_status": "N/A"})
                print(f"{label:<22} | {field(r, 'mass invariant'):>6} | "
                      f"{field(r, 'energy conservation'):>7} | "
                      f"{field(r, 'finite-diff'):>7} | "
                      f"{field(r, 'finite-diff'):>7} | "
                      f"{field(r, 'freefall analytic'):>9} | "
                      f"{field(r, 'contact-energy agreement'):>9} | "
                      f"[{r.get('_status', '?')}]")

    print(f"\n  {n_ok}/{n_total} runs passed")
    sys.exit(0 if n_ok == n_total else 1)


if __name__ == "__main__":
    main()
