"""PhysX contact reporting via PhysxContactReportAPI subscription.

Applies PhysxContactReportAPI to body prims, subscribes to per-step contact
events, decodes them into Python dicts, and provides the per-frame in_contact
flag and ragged contact lists for HDF5 storage.

See ADR-0007 for the contact reporting design.
"""
import omni.physx
import omni.usd
from pxr import PhysxSchema, PhysicsSchemaTools
from omni.physx.bindings._physx import ContactEventType


CONTACT_FOUND = 0
CONTACT_LOST = 1
CONTACT_PERSIST = 2


def enable_contact_reporting(rigid_paths, threshold=0.0):
    stage = omni.usd.get_context().get_stage()
    for path in rigid_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise ValueError(f"Invalid prim path: {path}")
        api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
        api.CreateThresholdAttr().Set(threshold)


def set_kinematic_pairs_flags(physics_scene_path="/PhysicsScene"):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(physics_scene_path)
    if not prim.IsValid():
        raise ValueError(f"Invalid physics scene path: {physics_scene_path}")
    api = PhysxSchema.PhysxSceneAPI.Apply(prim)
    api.CreateReportKinematicKinematicPairsAttr().Set(True)
    api.CreateReportKinematicStaticPairsAttr().Set(True)


def subscribe_contacts(on_step_callback):
    return omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(
        on_step_callback
    )


def decode_contacts(contact_headers, contact_data):
    results = []
    for header in contact_headers:
        actor0 = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
        actor1 = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
        contact_type = int(header.type)

        for i in range(header.contact_data_offset,
                       header.contact_data_offset + header.num_contact_data):
            d = contact_data[i]
            results.append({
                "type": contact_type,
                "actor0": actor0,
                "actor1": actor1,
                "position": (float(d.position.x), float(d.position.y),
                             float(d.position.z)),
                "normal": (float(d.normal.x), float(d.normal.y),
                           float(d.normal.z)),
                "impulse": (float(d.impulse.x), float(d.impulse.y),
                            float(d.impulse.z)),
                "separation": float(d.separation),
                "face_index0": int(d.face_index0),
                "face_index1": int(d.face_index1),
            })
    return results


def has_active_contacts(contacts):
    for c in contacts:
        if c["type"] in (CONTACT_FOUND, CONTACT_PERSIST):
            return True
    return False
