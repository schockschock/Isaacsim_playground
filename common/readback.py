"""PhysX Tensor API readback for rigid-body state.

Uses isaacsim.core.experimental.prims.RigidPrim (backed by the PhysX tensor API)
to read the solver's internal state directly — no finite differences, no
USD-attribute indirection. This is the ground-truth data source.

Adapted from the issue spec's low-level Tensor API approach to the stable
RigidPrim wrapper used by the Isaac Sim experimental prims layer.

See ADR-0006 for why Tensor API over USD-attribute readback.
"""
import numpy as np

from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.experimental.prims import RigidPrim


def setup_rigid_body_view(prim_path):
    SimulationManager.initialize_physics()
    rp = RigidPrim(prim_path)
    if not rp.is_physics_tensor_entity_valid():
        raise RuntimeError(
            f"Physics tensor entity not valid for {prim_path}. "
            "Ensure the timeline has been played at least once."
        )
    return rp


def read_state(rp):
    (pos_wp, quat_wp) = rp.get_world_poses()
    (lin_vel_wp, ang_vel_wp) = rp.get_velocities()

    return {
        "position": pos_wp.numpy().reshape(3).astype(np.float64).copy(),
        "orientation": quat_wp.numpy().reshape(4).astype(np.float64).copy(),
        "lin_velocity": lin_vel_wp.numpy().reshape(3).astype(np.float64).copy(),
        "ang_velocity": ang_vel_wp.numpy().reshape(3).astype(np.float64).copy(),
    }


def read_full_state(rp):
    state = read_state(rp)

    mass_wp = rp.get_masses()
    state["mass"] = float(mass_wp.numpy().flat[0])

    (com_pos_wp, com_quat_wp) = rp.get_coms()
    state["com_offset"] = com_pos_wp.numpy().reshape(3).astype(np.float64).copy()

    inertia_wp = rp.get_inertias()
    state["inertia_body_col9"] = inertia_wp.numpy().reshape(9).astype(np.float64).copy()

    return state
