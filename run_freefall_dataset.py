"""Run freefall dataset: all 339 objects × 9 IC combos.

Usage:  python3 run_freefall_dataset.py
        python3 run_freefall_dataset.py --dry-run   # just count
"""
import os
import sys
from common._dataset_runner import run_all, get_object_ids, N_V, N_P, h5_exists

SCENARIO = "freefall"
SCRIPT = "freefall.py"

if __name__ == "__main__":
    if "--dry-run" in sys.argv:
        ids = get_object_ids()
        n_total = len(ids) * N_V * N_P
        n_done = sum(1 for oid in ids
                     for v in range(N_V) for p in range(N_P)
                     if h5_exists(SCENARIO, oid, v, p))
        print(f"Objects: {len(ids)}, total runs: {n_total}, done: {n_done}, todo: {n_total - n_done}")
        sys.exit(0)

    repo = os.path.dirname(os.path.abspath(__file__))
    log = os.path.join(repo, "_output", "freefall_dataset.log")
    os.makedirs(os.path.dirname(log), exist_ok=True)
    ok = run_all(SCENARIO, SCRIPT, log)
    sys.exit(0 if ok else 1)
