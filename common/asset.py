"""Load the potato USD asset and apply collision / rigid body."""
import omni.replicator.core as rep
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

ASSET_PATH = "/data1/adrien/clean_dataset/2R1-1/2R1-1_centered.usd"


def load(name="Potato", position=(0, 0, 0.5), parent="/World"):
    prim = rep.functional.create.reference(
        usd_path=ASSET_PATH,
        parent=parent,
        name=name,
        position=position,
    )
    return prim


def add_colliders(root_prim, approximation="convexHull"):
    for desc_prim in Usd.PrimRange(root_prim):
        if desc_prim.IsA(UsdGeom.Mesh) or desc_prim.IsA(UsdGeom.Gprim):
            UsdPhysics.CollisionAPI.Apply(desc_prim).CreateCollisionEnabledAttr(True)
            physx = PhysxSchema.PhysxCollisionAPI.Apply(desc_prim)
            physx.CreateContactOffsetAttr(0.001)
            physx.CreateRestOffsetAttr(0.0)
        if desc_prim.IsA(UsdGeom.Mesh):
            mesh_api = UsdPhysics.MeshCollisionAPI.Apply(desc_prim)
            mesh_api.GetApproximationAttr().Set(approximation)
            if approximation == "sdf":
                PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(desc_prim)


def make_rigid(prim, disable_gravity=False):
    rep.functional.physics.apply_rigid_body(prim, disableGravity=disable_gravity)
