"""Shared dataset runner — runs all IC combos on all objects with resume.

Used by run_freefall_dataset.py, run_conveyor_dataset.py, run_object_throw_dataset.py.
"""
import os
import subprocess
import sys
import time

from pathlib import Path

ISAAC_PYTHON = "/home/adrien/isaacsim/python.sh"
DATASET_DIR = "/data2/adrien/clean_dataset"
OUTPUT_BASE = "/data2/adrien/physXdataset"

N_V = 3
N_P = 3


def get_object_ids():
    ids = sorted(
        d for d in os.listdir(DATASET_DIR)
        if os.path.isdir(os.path.join(DATASET_DIR, d))
        and os.path.isfile(os.path.join(DATASET_DIR, d, f"{d}_centered.usd"))
    )
    return ids


def h5_exists(scenario, object_id, v, p):
    path = os.path.join(
        OUTPUT_BASE, scenario, object_id,
        f"v{v}_p{p}", f"{scenario}_{object_id}_v{v}_p{p}.h5"
    )
    return os.path.isfile(path)


def run_one(scenario, object_id, v, p, script_path):
    tag = f"{scenario}/{object_id}/v{v}_p{p}"
    if h5_exists(scenario, object_id, v, p):
        return tag, "SKIP"

    t0 = time.perf_counter()
    proc = subprocess.run(
        [ISAAC_PYTHON, script_path,
         "--object", object_id, "--run", str(v), str(p)],
        capture_output=True, text=True, timeout=1200,
        env={**os.environ, "CONDA_PREFIX": ""},
    )
    elapsed = time.perf_counter() - t0

    if proc.returncode != 0:
        return tag, f"FAIL({elapsed:.0f}s)"

    return tag, f"OK({elapsed:.0f}s)"


def run_all(scenario, script_name, log_file):
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    script_path = os.path.join(repo, "simulation_scripts", script_name)

    object_ids = get_object_ids()
    n_total = len(object_ids) * N_V * N_P
    n_done = sum(1 for oid in object_ids
                 for v in range(N_V) for p in range(N_P)
                 if h5_exists(scenario, oid, v, p))

    print(f"=== {scenario} dataset ===")
    print(f"  objects: {len(object_ids)}")
    print(f"  IC combos per object: {N_V * N_P} ({N_V} vel × {N_P} pos)")
    print(f"  total runs: {len(object_ids)} × {N_V * N_P} = {n_total}")
    print(f"  already done: {n_done}")
    print(f"  output base: {OUTPUT_BASE}")
    print()

    results = {}
    start_time = time.perf_counter()
    n_run = 0

    for o_idx, oid in enumerate(object_ids):
        for v in range(N_V):
            for p in range(N_P):
                tag, status = run_one(scenario, oid, v, p, script_path)
                results[tag] = status
                n_run += 1

                elapsed = time.perf_counter() - start_time
                eta = (elapsed / max(n_run - n_done, 1)) * (n_total - n_run) if n_run > n_done else 0
                done_str = f"[{n_run}/{n_total}]"
                print(f"  {done_str} {tag}  {status}  "
                      f"({elapsed / 60:.0f}m elapsed, {eta / 60:.0f}m eta)")

                with open(log_file, "a") as f:
                    f.write(f"{tag}\t{status}\t{time.time()}\n")

    total_elapsed = time.perf_counter() - start_time
    ok = sum(1 for _s in results.values()
             if _s.startswith("OK") or _s == "SKIP")
    fail = sum(1 for _s in results.values() if _s.startswith("FAIL"))

    print(f"\n=== {scenario} done ===")
    print(f"  passed/skipped: {ok}")
    print(f"  failed: {fail}")
    print(f"  total time: {total_elapsed / 3600:.1f}h")
    print()

    return fail == 0
