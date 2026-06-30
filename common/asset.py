"""Load the potato USD asset and apply collision / rigid body."""
import omni
import omni.replicator.core as rep
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, Sdf

from common.geometry import angular_velocity_rad_s_to_deg_s


def load(name="Potato",
         prim_path="/data2/adrien/clean_dataset/2R1-1/2R1-1_centered.usd",
         position=(0, 0, 0.5),
         parent="/World",
         rotation=(0, 0, 0),
         scale_quaternion=(1, 1, 1)):
    prim = rep.functional.create.reference(
        usd_path=prim_path,
        parent=parent,
        name=name,
        position=position,
        rotation=rotation,
        scale=scale_quaternion
    )
    return prim


def add_colliders(root_prim, approximation="convexHull"):
    """Add PhysX collision to all meshes under a prim.
       Attribute "approximation" can be "convexHull" or "sdf" (signed distance field).
    """
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


def apply_physics_material(prim, restitution, static_friction, dynamic_friction):
    """Create a physics Material prim and bind it to a collision prim.

    Physics materials control restitution (bounciness) and friction. They are
    bound via the "material:binding:physics" relationship — NOT "material:binding"
    (which is for rendering shaders). The combine mode is set to "maximum" so
    the higher coefficient between two colliding bodies wins.

    Args:
        prim: The collision prim to bind the material to.
        restitution: Coefficient of restitution (0 = inelastic, 1 = elastic).
        static_friction: Static friction coefficient.
        dynamic_friction: Dynamic (kinetic) friction coefficient.
    """
    stage = omni.usd.get_context().get_stage()
    material_path = f"/World/PhysicsMaterial_{prim.GetName()}"

    material_prim = stage.DefinePrim(material_path, "Material")
    material_api = UsdPhysics.MaterialAPI.Apply(material_prim)
    material_api.CreateRestitutionAttr().Set(restitution)
    material_api.CreateStaticFrictionAttr().Set(static_friction)
    material_api.CreateDynamicFrictionAttr().Set(dynamic_friction)

    # PhysX combine modes: "maximum" = use the higher of the two colliding materials
    try:
        physx_mat = PhysxSchema.PhysxMaterialAPI.Apply(material_prim)
        physx_mat.CreateRestitutionCombineModeAttr().Set("maximum")
        physx_mat.CreateFrictionCombineModeAttr().Set("maximum")
    except Exception:
        pass  # Not critical — defaults work

    # Bind the material to the prim using the physics binding namespace
    prim.CreateRelationship("material:binding:physics", False).AddTarget(
        material_prim.GetPath()
    )

def switch_off_shadows(prim):
    """Switch off shadows for all meshes under a prim."""
    for mesh_prim in Usd.PrimRange(prim):
            if mesh_prim.IsA(UsdGeom.Mesh):
                # Create and set the doNotCastShadows attribute to True
                attr = mesh_prim.CreateAttribute("primvars:doNotCastShadows", 
                                                Sdf.ValueTypeNames.Bool, 
                                                False)
                attr.Set(True)
                print(f"Shadows disabled for: {mesh_prim.GetPath()}")

def read_rigid_body_state(prim):
    """Read position, orientation, and velocities from a USD rigid body prim.

    PhysX writes the current state to USD attributes every physics step. We
    read these directly — no finite differences. This gives us the solver's
    internal values, which is what the dataset spec requires.

    Args:
        prim: The rigid body USD prim.

    Returns:
        Tuple of (position, quaternion, linear_velocity, angular_velocity).
        position: Gf.Vec3d (x, y, z) in meters
        quaternion: Gf.Quatf or Gf.Quatd (w, x, y, z)
        linear_velocity: Gf.Vec3f (vx, vy, vz) in m/s
        angular_velocity: Gf.Vec3f (wx, wy, wz) in rad/s
    """
    pos = prim.GetAttribute("xformOp:translate").Get()
    quat = prim.GetAttribute("xformOp:orient").Get()
    lv = prim.GetAttribute("physics:velocity").Get()
    av = prim.GetAttribute("physics:angularVelocity").Get()
    return pos, quat, lv, av

def instantiate_prim(prim_path, position=(0, 0, 0),
                     rotation=(0, 0, 0),
                     scale=(1, 1, 1),
                     parent="/World",
                     rigid=True,
                     collider=True,
                     restitution=0.0,
                     static_friction=0.5,
                     dynamic_friction=0.5,
                     approximation="convexHull",
                     shadows=True
                     ):
    """Instantiate a USD prim from a given path with specified transform."""
    prim = rep.functional.create.reference(
        usd_path=prim_path,
        parent=parent,
        position=position,
        rotation=rotation,
        scale=scale
    )

    # Setup the prim
    if not shadows:
        switch_off_shadows(prim)
    if rigid:
        make_rigid(prim)
    if collider:
        add_colliders(prim, approximation=approximation)
    if restitution != 0.0 or static_friction != 0.5 or dynamic_friction != 0.5:
        apply_physics_material(prim, restitution, static_friction, dynamic_friction)

    # Return the prim for further manipulation if needed
    return prim


def set_initial_state(prim, linear_velocity=(0.0, 0.0, 0.0),
                      angular_velocity_rad_s=(0.0, 0.0, 0.0)):
    """Set initial linear and angular velocity on a rigid body prim.

    The USD Physics schema defines ``physics:angularVelocity`` in
    degrees/second, so angular values in rad/s are converted via
    ``common.geometry.angular_velocity_rad_s_to_deg_s`` before setting.
    Linear velocity is in m/s (no conversion needed).

    Uses ``RigidBodyAPI.CreateVelocityAttr()`` / ``CreateAngularVelocityAttr()``
    to guarantee the attributes are created before setting.

    Args:
        prim: The rigid body prim.
        linear_velocity: (vx, vy, vz) in m/s.
        angular_velocity_rad_s: (wx, wy, wz) in rad/s.
    """
    rb = UsdPhysics.RigidBodyAPI(prim)
    rb.CreateVelocityAttr().Set(Gf.Vec3f(*linear_velocity))
    av_deg = angular_velocity_rad_s_to_deg_s(angular_velocity_rad_s)
    rb.CreateAngularVelocityAttr().Set(Gf.Vec3f(*av_deg))