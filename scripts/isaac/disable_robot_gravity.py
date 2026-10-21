# Isaac Sim Script Editor (simulation stopped): disable gravity on all FR3 rigid bodies.
# Mirrors the real FR3, which compensates gravity internally (libfranka torques exclude it).
import omni.usd
from pxr import UsdPhysics, Sdf
ROBOT_PREFIX = "/fr3"
stage = omni.usd.get_context().get_stage()
n = 0
for p in stage.Traverse():
    if str(p.GetPath()).startswith(ROBOT_PREFIX) and p.HasAPI(UsdPhysics.RigidBodyAPI):
        try:
            from pxr import PhysxSchema
            PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr(True)
        except Exception:
            p.AddAppliedSchema("PhysxRigidBodyAPI")
            p.CreateAttribute("physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool).Set(True)
        n += 1
print("gravity disabled on", n, "links  (save the scene with Ctrl+S)")
