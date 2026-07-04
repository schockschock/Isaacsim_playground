"""Validate freefall HDF5 against physics ground-truth assertions.

Loads a per-run HDF5 produced by freefall.py and verifies:
  1. Freefall analytic: z(t) = z0 - 0.5*g*t^2 for non-contact frames.
  2. Finite-difference consistency: v_lin ~ delta_pos / dt,
     omega ~ numerical quaternion diff / dt.

Runs without Isaac Sim — numpy + h5py only.
"""
import os
import sys

import h5py
import numpy as np

G = 9.81


def load_run(h5_path):
    with h5py.File(h5_path, "r") as f:
        attrs = dict(f.attrs)
        ds = f["/state"]
        data = {name: ds[name][()] for name in ds.dtype.names}
        if "/contacts" in f:
            contacts_ds = f["/contacts"]
            data["_contacts"] = {name: contacts_ds[name][()] for name in contacts_ds.dtype.names}
        else:
            data["_contacts"] = None
    return data, attrs


def _find_non_contact_mask(position_z, lin_vel_z, dt):
    n = len(position_z)
    mask = np.ones(n, dtype=bool)
    accel_threshold = 2.0 * G
    for i in range(1, n):
        accel_z = abs(lin_vel_z[i] - lin_vel_z[i - 1]) / dt
        if accel_z > accel_threshold:
            mask[i:] = False
            break
        if position_z[i] < 0.035:
            mask[i:] = False
            break
    return mask


def _quat_conjugate(q_wxyz):
    w, x, y, z = q_wxyz[0], q_wxyz[1], q_wxyz[2], q_wxyz[3]
    return np.array([w, -x, -y, -z])


def _quat_multiply(q1_wxyz, q2_wxyz):
    w1, x1, y1, z1 = q1_wxyz
    w2, x2, y2, z2 = q2_wxyz
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def assert_freefall_analytic(data, attrs, eps=0.02):
    position = data["position"]
    time = data["time"]
    lin_vel = data["lin_velocity"]
    z0_spawn = 0.4

    dt = 1.0 / attrs["fps"]
    nc = _find_non_contact_mask(position[:, 2], lin_vel[:, 2], dt)
    t_offset = attrs.get("t_offset_frames", 5)

    first_capture = max(t_offset, 0)
    indices = [i for i in range(first_capture, len(time)) if nc[i]]
    assert len(indices) > 0, "no non-contact frames after warmup"

    max_err = 0.0
    for i in indices:
        z_pred = z0_spawn - 0.5 * G * time[i] ** 2
        err = abs(position[i, 2] - z_pred)
        max_err = max(max_err, err)

    assert max_err < eps, (
        f"max freefall analytic error {max_err:.6f} exceeds eps={eps}"
    )
    print(f"  [PASS] freefall analytic: max_err={max_err:.6f} across "
          f"{len(indices)} non-contact frames")
    return True


def assert_finite_difference(data, attrs, eps_lin=0.05, eps_ang=0.5):
    position = data["position"]
    orientation = data["orientation"]
    lin_vel = data["lin_velocity"]
    ang_vel = data["ang_velocity"]

    dt = 1.0 / attrs["fps"]
    nc = _find_non_contact_mask(position[:, 2], lin_vel[:, 2], dt)

    max_lin_err = 0.0
    max_ang_err = 0.0
    n_checked = 0

    for i in range(1, len(data["time"])):
        if not nc[i] or not nc[i - 1]:
            continue
        n_checked += 1

        dp = position[i] - position[i - 1]
        v_fd = dp / dt
        v_tensor = lin_vel[i]
        lin_err = np.linalg.norm(v_fd - v_tensor)
        max_lin_err = max(max_lin_err, lin_err)

        q_prev_wxyz = orientation[i - 1]
        q_curr_wxyz = orientation[i]
        dq_wxyz = _quat_multiply(q_curr_wxyz, _quat_conjugate(q_prev_wxyz))
        cos_half = np.clip(float(dq_wxyz[0]), -1.0, 1.0)
        if cos_half > 0.99999:
            w_fd = np.array([0.0, 0.0, 0.0])
        else:
            angle = 2.0 * np.arccos(cos_half)
            axis = dq_wxyz[1:] / np.sqrt(1.0 - cos_half * cos_half + 1e-12)
            w_fd = axis * angle / dt
        w_tensor = ang_vel[i]
        ang_err = np.linalg.norm(w_fd - w_tensor)
        max_ang_err = max(max_ang_err, ang_err)

    assert n_checked > 0, "no valid finite-diff frames"
    assert max_lin_err < eps_lin, (
        f"max finite-diff linear error {max_lin_err:.4f} exceeds eps_lin={eps_lin}"
    )
    assert max_ang_err < eps_ang, (
        f"max finite-diff angular error {max_ang_err:.4f} exceeds eps_ang={eps_ang}"
    )
    print(f"  [PASS] finite-diff: max_lin_err={max_lin_err:.4f}, "
          f"max_ang_err={max_ang_err:.4f} across {n_checked} pairs")
    return True


def assert_mass_invariant(data, attrs, eps=1e-9):
    if "mass" not in data:
        print("  [SKIP] mass invariant: no mass field (pre-Slice-B HDF5)")
        return True

    mass = data["mass"]
    assert len(mass) > 0, "empty mass series"
    std_mass = np.std(mass)
    assert std_mass < eps, (
        f"mass std {std_mass:.3e} exceeds eps={eps}"
    )
    print(f"  [PASS] mass invariant: std(mass)={std_mass:.2e} "
          f"(mean={np.mean(mass):.6f}) across {len(mass)} frames")
    return True


def assert_energy_conservation(data, attrs, eps=0.01):
    if "E_total" not in data or "dissipating" not in data:
        print("  [SKIP] energy conservation: no energy fields (pre-Slice-B HDF5)")
        return True

    e_total = data["E_total"]
    dissipating = data["dissipating"].astype(bool)

    max_err = 0.0
    n_checked = 0
    for i in range(1, len(e_total)):
        if dissipating[i] or dissipating[i - 1]:
            continue
        de = abs(e_total[i] - e_total[i - 1])
        max_err = max(max_err, de)
        n_checked += 1

    assert n_checked > 0, "no non-dissipating consecutive frames"
    assert max_err < eps, (
        f"max energy drift {max_err:.6f} exceeds eps={eps}"
    )
    print(f"  [PASS] energy conservation: max_drift={max_err:.6f} "
          f"across {n_checked} non-dissipating pairs")
    return True


def assert_contact_energy_agreement(data, attrs, r_threshold=0.7):
    contacts = data.get("_contacts")
    if contacts is None or len(contacts.get("step", [])) == 0:
        print("  [SKIP] contact-energy agreement: no /contacts data")
        return True

    contact_steps = contacts["step"]
    contact_impulses = contacts["impulse"]
    impulse_mags = np.linalg.norm(contact_impulses, axis=1)

    E_trans = data["E_trans"]
    E_rot = data["E_rot"]
    ke = E_trans + E_rot

    unique_steps = np.unique(contact_steps)

    dke_list = []
    imp_list = []
    for step in unique_steps:
        mask = contact_steps == step
        total_impulse = float(np.sum(impulse_mags[mask]))
        if total_impulse < 1e-6:
            continue
        imp_list.append(total_impulse)

        if step >= 1:
            dke = float(abs(ke[step] - ke[step - 1]))
        else:
            dke = float(abs(ke[step]))
        dke_list.append(dke)

    if len(imp_list) < 3:
        print(f"  [SKIP] contact-energy agreement: only {len(imp_list)} impact frames")
        return True

    dke_arr = np.array(dke_list, dtype=np.float64)
    imp_arr = np.array(imp_list, dtype=np.float64)

    std_dke = np.std(dke_arr)
    std_imp = np.std(imp_arr)
    if std_dke < 1e-12 or std_imp < 1e-12:
        print("  [SKIP] contact-energy agreement: constant values")
        return True

    r = np.corrcoef(dke_arr, imp_arr)[0, 1]
    assert r > r_threshold, (
        f"Pearson r {r:.4f} does not exceed threshold {r_threshold} across {len(imp_list)} impact frames"
    )
    print(f"  [PASS] contact-energy agreement: Pearson r={r:.4f} "
          f"across {len(imp_list)} impact frames")
    return True


def main(h5_path=None):
    if h5_path is None:
        repo_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        h5_path = os.path.join(
            repo_root, "_output", "freefall_high_angular_velocity",
            "freefall.h5",
        )
        if not os.path.isfile(h5_path):
            print(f"ERROR: no HDF5 at {h5_path}. Run freefall.py first or pass --h5.")
            sys.exit(1)

    print(f"Validating {h5_path}")
    data, attrs = load_run(h5_path)
    print(f"  n_state_rows={attrs.get('n_state_rows')}, "
          f"n_rgb_frames={attrs.get('n_rgb_frames')}, "
          f"t_offset_frames={attrs.get('t_offset_frames')}")
    print(f"  fps={attrs.get('fps')}, dt_phys={attrs.get('dt_phys')}")

    all_pass = True
    try:
        assert_freefall_analytic(data, attrs)
    except AssertionError as e:
        print(f"  [FAIL] freefall analytic: {e}")
        all_pass = False

    try:
        assert_finite_difference(data, attrs)
    except AssertionError as e:
        print(f"  [FAIL] finite-diff: {e}")
        all_pass = False

    try:
        assert_mass_invariant(data, attrs)
    except AssertionError as e:
        print(f"  [FAIL] mass invariant: {e}")
        all_pass = False

    try:
        assert_energy_conservation(data, attrs)
    except AssertionError as e:
        print(f"  [FAIL] energy conservation: {e}")
        all_pass = False

    try:
        assert_contact_energy_agreement(data, attrs)
    except AssertionError as e:
        print(f"  [FAIL] contact-energy agreement: {e}")
        all_pass = False

    if all_pass:
        print("Validation PASSED")
        sys.exit(0)
    else:
        print("Validation FAILED")
        sys.exit(1)


if __name__ == "__main__":
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else None
    main(path)
