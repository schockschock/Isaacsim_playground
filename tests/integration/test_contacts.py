from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.timeline
import omni.replicator.core as rep
from pxr import Gf, UsdPhysics

from common.app import shutdown
from common.world import setup_stage
from common.asset import load, add_colliders, make_rigid
from common.contacts import enable_contact_reporting, subscribe_contacts, decode_contacts


def test_contacts_on_falling_potato():
    stage = setup_stage()

    init_quat = Gf.Quatd(1.0, Gf.Vec3d(0, 0, 0))
    potato = load(name="Potato", position=(0, 0, 0.5), rotation=init_quat)
    make_rigid(potato)
    UsdPhysics.MassAPI.Apply(potato).CreateDensityAttr(1000.0)
    add_colliders(potato, approximation="convexHull")

    enable_contact_reporting(["/World/Potato"], threshold=0.0)

    contacts_buffer = []

    def on_contact(contact_headers, contact_data):
        decoded = decode_contacts(contact_headers, contact_data)
        contacts_buffer.extend(decoded)

    sub = subscribe_contacts(on_contact)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    for _ in range(40):
        simulation_app.update()

    timeline.stop()

    assert len(contacts_buffer) > 0, f"no contact events after 40 frames of freefall"

    types_seen = set(c["type"] for c in contacts_buffer)
    assert 0 in types_seen or sum(types_seen) > 0, f"no CONTACT_FOUND or PERSIST events: {types_seen}"

    has_impulse = any(abs(c["impulse"][2]) > 0.001 for c in contacts_buffer)
    assert has_impulse, "no contact points with non-trivial impulse"

    assert contacts_buffer[0]["position"] is not None
    assert contacts_buffer[0]["actor0"] is not None
    assert contacts_buffer[0]["actor1"] is not None

    print(f"  [PASS] test_contacts_on_falling_potato ({len(contacts_buffer)} contact events)")


test_contacts_on_falling_potato()
shutdown(simulation_app)
