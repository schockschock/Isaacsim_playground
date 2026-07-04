"""Validate object_throw HDF5 — baseline + freefall-analytic (ballistic flight)."""
import os, sys
import h5py
import numpy as np

G = 9.81

def load_run(h5_path):
    with h5py.File(h5_path, "r") as f:
        attrs = dict(f.attrs)
        ds = f["/state"]
        data = {n: ds[n][()] for n in ds.dtype.names}
        if "/contacts" in f:
            cd = f["/contacts"]
            data["_contacts"] = {n: cd[n][()] for n in cd.dtype.names}
        else:
            data["_contacts"] = None
    return data, attrs

def assert_mass_invariant(data, attrs, eps=1e-9):
    if "mass" not in data: return print("  [SKIP] mass invariant"), True
    std_m = np.std(data["mass"])
    assert std_m < eps, f"mass std {std_m:.3e} exceeds eps={eps}"
    print(f"  [PASS] mass invariant: std={std_m:.2e} mean={np.mean(data['mass']):.6f}")
    return True

def assert_energy_conservation(data, attrs, eps=0.01):
    if "E_total" not in data or "dissipating" not in data:
        return print("  [SKIP] energy conservation"), True
    e, diss = data["E_total"], data["dissipating"].astype(bool)
    max_err, n = 0.0, 0
    for i in range(1, len(e)):
        if diss[i] or diss[i-1]: continue
        de = abs(e[i] - e[i-1]); max_err = max(max_err, de); n += 1
    assert n > 0, "no non-dissipating pairs"
    assert max_err < eps, f"max drift {max_err:.6f} > eps={eps}"
    print(f"  [PASS] energy conservation: max_drift={max_err:.6f} across {n} pairs")
    return True

def _quat_conjugate(q): return np.array([q[0], -q[1], -q[2], -q[3]])
def _quat_multiply(q1, q2):
    w1,x1,y1,z1 = q1; w2,x2,y2,z2 = q2
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])

def assert_finite_difference(data, attrs, eps_lin=0.06, eps_ang=3.0):
    # Note: eps_ang relaxed for object_throw because post-impact
    # angular velocities can exceed 100 rad/s (~1 radian/frame at 120fps),
    # degrading the quaternion finite-difference approximation.
    p, o, lv, av = data["position"], data["orientation"], data["lin_velocity"], data["ang_velocity"]
    dt = 1.0 / attrs["fps"]
    in_contact = data.get("in_contact")
    if in_contact is not None: in_contact = in_contact.astype(bool)

    max_lin, max_ang, n = 0.0, 0.0, 0
    for i in range(1, len(p)):
        if in_contact is not None and (in_contact[i] or in_contact[i-1]):
            continue
        n += 1
        v_fd = (p[i] - p[i-1]) / dt
        lin_err = np.linalg.norm(v_fd - lv[i])
        max_lin = max(max_lin, lin_err)
        dq = _quat_multiply(o[i], _quat_conjugate(o[i-1]))
        ch = np.clip(float(dq[0]), -1.0, 1.0)
        if ch > 0.99999: w_fd = np.array([0.]*3)
        else:
            angle = 2.0 * np.arccos(ch)
            w_fd = dq[1:] / np.sqrt(1 - ch*ch + 1e-12) * angle / dt
        max_ang = max(max_ang, np.linalg.norm(w_fd - av[i]))
    assert n > 0, "no non-contact finite-diff frames"
    assert max_lin < eps_lin, f"lin err {max_lin:.4f} > {eps_lin}"
    assert max_ang < eps_ang, f"ang err {max_ang:.4f} > {eps_ang}"
    print(f"  [PASS] finite-diff: max_lin={max_lin:.4f} max_ang={max_ang:.4f} across {n} non-contact")
    return True

def assert_contact_energy(data, attrs, r_thresh=0.7):
    c = data.get("_contacts")
    if c is None or len(c.get("step", [])) == 0: return print("  [SKIP] contact-energy"), True
    steps, imps = c["step"], np.linalg.norm(c["impulse"], axis=1)
    ke = data["E_trans"] + data["E_rot"]
    dke_list, imp_list = [], []
    for s in np.unique(steps):
        mask = steps == s; ti = float(np.sum(imps[mask]))
        if ti < 1e-6: continue
        imp_list.append(ti)
        dke_list.append(float(abs(ke[s] - ke[s-1])) if s >= 1 else float(abs(ke[s])))
    if len(imp_list) < 3: return print(f"  [SKIP] contact-energy: {len(imp_list)} frames"), True
    dke_arr, imp_arr = np.array(dke_list), np.array(imp_list)
    std_d, std_i = np.std(dke_arr), np.std(imp_arr)
    if std_d < 1e-12 or std_i < 1e-12: return print("  [SKIP] contact-energy: constant"), True
    r = np.corrcoef(dke_arr, imp_arr)[0,1]
    assert r > r_thresh, f"Pearson r {r:.4f} <= {r_thresh}"
    print(f"  [PASS] contact-energy agreement: r={r:.4f} across {len(imp_list)} frames")
    return True

def assert_freefall_analytic(data, attrs, eps=0.05):
    z = data["position"][:, 2]
    t = data["time"]
    vz = data["lin_velocity"][:, 2]
    in_contact = data.get("in_contact")
    if in_contact is not None: in_contact = in_contact.astype(bool)

    # Find ballistic windows (consecutive non-contact frames)
    windows = []
    i = 0
    while i < len(t):
        if in_contact is not None and in_contact[i]:
            i += 1; continue
        start = i
        while i < len(t) and (in_contact is None or not in_contact[i]):
            i += 1
        end = i
        if end - start >= 3:
            windows.append((start, end))

    assert len(windows) > 0, "no ballistic flight windows"

    max_err = 0.0
    total_n = 0
    for start, end in windows:
        if start >= end - 2: continue
        z0, t0, vz0 = z[start], t[start], vz[start]
        for j in range(start, end):
            tr = t[j] - t0
            z_pred = z0 + vz0 * tr - 0.5 * G * tr * tr
            err = abs(z[j] - z_pred)
            max_err = max(max_err, err)
            total_n += 1

    assert max_err < eps, f"max freefall analytic error {max_err:.6f} > {eps}"
    print(f"  [PASS] freefall analytic: max_err={max_err:.6f} across {total_n} frames ({len(windows)} windows)")
    return True

def main(h5_path=None):
    if h5_path is None:
        repo = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        h5_path = os.path.join(repo, "_output", "object_throw", "object_throw.h5")
        if not os.path.isfile(h5_path):
            print(f"ERROR: no HDF5 at {h5_path}"); sys.exit(1)
    print(f"Validating {h5_path}")
    data, attrs = load_run(h5_path)
    print(f"  rows={attrs.get('n_state_rows')} rgb={attrs.get('n_rgb_frames')}")
    ok = True
    for fn, nm in [(assert_mass_invariant, "mass"), (assert_energy_conservation, "energy"),
                    (assert_finite_difference, "finite-diff"), (assert_contact_energy, "contact-energy"),
                    (assert_freefall_analytic, "freefall analytic")]:
        try: fn(data, attrs)
        except AssertionError as e: print(f"  [FAIL] {nm}: {e}"); ok = False
    print("Validation PASSED" if ok else "Validation FAILED")
    sys.exit(0 if ok else 1)

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else None
    main(path)
