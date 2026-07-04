"""PhysX-informed pure-logic measurement computations.

Operates on numpy arrays produced by common.readback. No Isaac deps.
Used by the capture loop to compute energy, momentum, and dissipating flags.
"""
import numpy as np


def body_inertia_from_column_major(I9):
    return np.asarray(I9, dtype=np.float64).reshape(3, 3, order="F")


def _rot_matrix_from_quat_wxyz(q):
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return np.array([
        [1 - 2 * (y * y + z * z),     2 * (x * y - w * z),     2 * (x * z + w * y)],
        [    2 * (x * y + w * z), 1 - 2 * (x * x + z * z),     2 * (y * z - w * x)],
        [    2 * (x * z - w * y),     2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def world_inertia(I_body, q_wxyz):
    R = _rot_matrix_from_quat_wxyz(q_wxyz)
    return R @ I_body @ R.T


def kinetic_energies(m, v_lin, I_world, w_world):
    ke_trans = 0.5 * float(m) * float(np.dot(v_lin, v_lin))
    Iw = I_world @ w_world
    ke_rot = 0.5 * float(np.dot(w_world, Iw))
    return ke_trans, ke_rot


def potential_energy(m, h, g=9.81):
    return float(m) * float(g) * float(h)


def momenta(m, v_lin, I_world, w_world, I_body):
    p_lin = float(m) * np.asarray(v_lin, dtype=np.float64)
    L_ang_world = I_world @ np.asarray(w_world, dtype=np.float64)
    L_ang_body = I_body @ np.asarray(w_world, dtype=np.float64)
    return tuple(p_lin.flatten()), tuple(L_ang_world.flatten()), tuple(L_ang_body.flatten())


def dissipating_flag(E_total_series, eps):
    E = np.asarray(E_total_series, dtype=np.float64)
    if len(E) == 0:
        return np.array([], dtype=bool)
    result = np.zeros(len(E), dtype=bool)
    for i in range(1, len(E)):
        if abs(E[i] - E[i - 1]) > eps:
            result[i] = True
    return result
