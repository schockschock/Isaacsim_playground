from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.timeline
import omni.replicator.core as rep
from pxr import Gf, UsdPhysics

from common.app import shutdown
from common.world import setup_stage
from common.asset import load, add_colliders, make_rigid
from common.readback import setup_rigid_body_view, read_state


def test_read_state_on_falling_potato():
    stage = setup_stage()

    init_quat = Gf.Quatd(1.0, Gf.Vec3d(0, 0, 0))
    potato = load(name="Potato", position=(0, 0, 0.5), rotation=init_quat)
    make_rigid(potato)
    mass_api = UsdPhysics.MassAPI.Apply(potato)
    mass_api.CreateDensityAttr(1000.0)
    add_colliders(potato, approximation="convexHull")

    rp = setup_rigid_body_view("/World/Potato")
    assert rp.is_physics_tensor_entity_valid(), "physics tensor entity not valid"

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    for _ in range(10):
        simulation_app.update()

    state = read_state(rp)

    assert isinstance(state, dict)
    for key in ("position", "orientation", "lin_velocity", "ang_velocity"):
        assert key in state, f"missing key: {key}"

    assert state["position"].shape == (3,), f"bad position shape: {state['position'].shape}"
    assert state["orientation"].shape == (4,), f"bad orientation shape: {state['orientation'].shape}"
    assert state["lin_velocity"].shape == (3,)
    assert state["ang_velocity"].shape == (3,)

    assert state["position"].dtype in (np.float32, np.float64)
    assert state["position"][2] < 0.49, (
        f"expected z below 0.49 after 10 frames of freefall, got {state['position'][2]:.4f}"
    )

    timeline.stop()
    print("  [PASS] test_read_state_on_falling_potato")


test_read_state_on_falling_potato()
shutdown(simulation_app)
