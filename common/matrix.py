"""27-run validation matrix driver.

For each of the 3 scenarios, runs 3 velocity levels × 3 poses = 9 runs,
then calls the per-scenario validate script on each output HDF5.

Usage:
  conda deactivate && $ISAAC_SIM/python.sh common/matrix.py

Produces _output/<scenario>/v{V}_p{P}/<scenario>_v{V}_p{P}.h5 for each run.
"""
import os
import subprocess
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ISAAC_PYTHON = os.environ.get("ISAAC_SIM", "/home/adrien/isaacsim") + "/python.sh"

SCENARIOS = ["freefall", "conveyor", "object_throw"]
N_VELOCITIES = 3
N_POSES = 3


def run_one(scenario, v_idx, p_idx):
    script = os.path.join(REPO_ROOT, "simulation_scripts", f"{scenario}.py")
    h5_dir = os.path.join(REPO_ROOT, "_output", scenario, f"v{v_idx}_p{p_idx}")
    h5_path = os.path.join(h5_dir, f"{scenario}_v{v_idx}_p{p_idx}.h5")

    if os.path.isfile(h5_path):
        print(f"  [SKIP] {h5_path} exists")
        return h5_path

    t0 = time.perf_counter()
    proc = subprocess.run(
        [ISAAC_PYTHON, script, "--run", str(v_idx), str(p_idx)],
        capture_output=True, text=True, timeout=1200,
    )
    elapsed = time.perf_counter() - t0

    if proc.returncode != 0:
        print(f"  [FAIL] exit={proc.returncode} ({elapsed:.0f}s)")
        print(proc.stderr[-500:] if len(proc.stderr) > 500 else proc.stderr)
        return None

    print(f"  [OK]   {elapsed:.0f}s  -> {h5_path}")
    return h5_path


def validate_one(h5_path):
    scenario = os.path.basename(os.path.dirname(os.path.dirname(h5_path)))
    validate_script = os.path.join(REPO_ROOT, "tests", "integration",
                                   f"validate_{scenario}.py")

    proc = subprocess.run(
        [sys.executable, validate_script, h5_path],
        capture_output=True, text=True,
    )
    output = proc.stdout + proc.stderr
    results = {}
    for line in output.splitlines():
        line = line.strip()
        if "[PASS]" in line:
            name = line[line.index("[PASS]") + 6:].split(":")[0].strip()
            results[name] = "PASS"
        elif "[FAIL]" in line:
            name = line[line.index("[FAIL]") + 6:].split(":")[0].strip()
            results[name] = "FAIL"
        elif "[SKIP]" in line:
            name = line[line.index("[SKIP]") + 6:].split(":")[0].strip()
            results[name] = "SKIP"
    return results


def main():
    total = len(SCENARIOS) * N_VELOCITIES * N_POSES
    print(f"=== 27-run validation matrix ({total} runs) ===\n")

    all_results = {}
    n_ok = 0

    for scenario in SCENARIOS:
        print(f"-- {scenario} --")
        for v in range(N_VELOCITIES):
            for p in range(N_POSES):
                label = f"{scenario}/v{v}_p{p}"
                print(f"\n[{label}] sim ...")
                h5_path = run_one(scenario, v, p)

                if h5_path is None:
                    all_results[label] = {"_status": "SIM_FAIL"}
                    continue

                print(f"[{label}] validate ...")
                r = validate_one(h5_path)
                r["_status"] = "OK" if all(v == "PASS" or v == "SKIP"
                                          for v in r.values()) else "VALIDATION_FAIL"
                all_results[label] = r
                if r["_status"] == "OK":
                    n_ok += 1

    print(f"\n=== Aggregate pass/fail table ===")
    print(f"{'Run':<22} | {'mass':>6} | {'energy':>7} | {'fd-lin':>7} | {'fd-ang':>7} | {'analytic':>9} | {'contact':>9}")
    print("-" * 22 + "-+-" + "-+-".join(["-" * 7] * 5))
    for label, r in all_results.items():
        mass = r.get("mass", "?").rjust(6)
        energy = r.get("energy", "?").rjust(7)
        fd_lin = r.get("finite-diff", "?").rjust(7)
        fd_ang = r.get("finite-diff", "?").rjust(7)
        analytic = r.get("freefall analytic", r.get("freefall affine", "?")).rjust(9)
        contact = r.get("contact-energy", r.get("contact-energy agreement", "?")).rjust(9)
        status = r["_status"]
        print(f"{label:<22} | {mass} | {energy} | {fd_lin} | {fd_ang} | {analytic} | {contact}  [{status}]")

    print(f"\n{n_ok}/{total} runs passed")
    sys.exit(0 if n_ok == total else 1)


if __name__ == "__main__":
    main()
