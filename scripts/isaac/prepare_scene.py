# Fix up the FR3 scene USD for the impedance controller (run with Isaac's Python, no SimulationApp needed):
#   ~/IsaacLab/.venv/bin/python -I prepare_scene.py [scene.usd] [fr3.urdf]
# 1. Select the "physx" Physics variant. Without a selection no physics payload is composed:
#    no articulation root, no joints, no rigid bodies.
# 2. Set PhysX joint armature = motor_inertia * gear_ratio^2 from the URDF <dynamics> tags (FR3 rotor
#    inertia reflected through the gearbox). Without it the light wrist (joint 7 drives only the hand)
#    chatters at Nyquist under the controller's damping at 240 Hz and the EE orientation wanders.
# 3. Set joint friction (PhysxJointAxisAPI "angular") from the friction identification used by the
#    controller's friction compensation (cartesian_impedance_controller.hpp: static_friction,
#    coulomb_friction, beta; identified on the upstream author's FR3). Not representable in PhysX and
#    left out: the per-joint offset_friction (asymmetric bias) and the LuGre bristle states (sigma_0/1).
#    Franka's URDF <dynamics> friction (0.2 for every joint) is not used: it is a placeholder.
#    --no-friction removes it again.
import math
import sys
import xml.etree.ElementTree as ET

from pxr import Sdf, Usd, UsdPhysics

args = [a for a in sys.argv[1:] if not a.startswith("--")]
friction = "--no-friction" not in sys.argv
scene = args[0] if len(args) > 0 else "/home/njohnson/ci_mpc_assets/fr3_scene.usd"
urdf = args[1] if len(args) > 1 else "/home/njohnson/ci_mpc_assets/fr3.urdf"

# joints 1..7: static [Nm], Coulomb [Nm], viscous beta [Nm s/rad]
STATIC = [1.025412896, 1.259913793, 0.8380147058, 1.005214968, 1.2928, 0.41525, 0.5341655]
COULOMB = [1.025412896, 1.259913793, 0.8380147058, 0.96, 1.2928, 0.41525, 0.5341655]
BETA = [1.18, 0.0, 0.55, 0.87, 0.935, 0.54, 0.45]

stage = Usd.Stage.Open(scene)
vs = stage.GetPrimAtPath("/fr3").GetVariantSets().GetVariantSet("Physics")
if vs.GetVariantSelection() != "physx":
    print("Physics variant:", repr(vs.GetVariantSelection()), "-> physx")
    vs.SetVariantSelection("physx")

for joint in ET.parse(urdf).getroot().findall("joint"):
    dyn = joint.find("dynamics")
    if dyn is None or not dyn.get("motor_inertia"):
        continue
    armature = float(dyn.get("motor_inertia")) * float(dyn.get("gear_ratio")) ** 2
    prim = stage.GetPrimAtPath(f"/fr3/Physics/{joint.get('name')}")
    attr = prim.GetAttribute("physxJoint:armature")
    if not attr:
        attr = prim.CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float)
    attr.Set(armature)
    print(f"{joint.get('name')}: armature {armature:.4f} kg m^2")

AXIS_API = "PhysxJointAxisAPI:angular"
for i in range(7):
    prim = stage.GetPrimAtPath(f"/fr3/Physics/fr3_joint{i + 1}")
    spec = stage.GetRootLayer().GetPrimAtPath(prim.GetPath()) or Sdf.CreatePrimInLayer(stage.GetRootLayer(), prim.GetPath())
    schemas = spec.GetInfo("apiSchemas") if spec.HasInfo("apiSchemas") else Sdf.TokenListOp()
    items = list(schemas.prependedItems)
    values = {
        "staticFrictionEffort": STATIC[i],
        "dynamicFrictionEffort": COULOMB[i],
        "viscousFrictionCoefficient": BETA[i] * math.pi / 180.0,  # PhysX unit: Nm s/deg
    }
    if friction:
        if AXIS_API not in items:
            spec.SetInfo("apiSchemas", Sdf.TokenListOp.Create(prependedItems=items + [AXIS_API]))
        for name, value in values.items():
            attr = prim.GetAttribute(f"physxJointAxis:angular:{name}")
            if not attr:
                attr = prim.CreateAttribute(f"physxJointAxis:angular:{name}", Sdf.ValueTypeNames.Float)
            attr.Set(value)
        print(f"fr3_joint{i + 1}: static {STATIC[i]:.3f} Nm, Coulomb {COULOMB[i]:.3f} Nm, viscous {BETA[i]:.3f} Nm s/rad")
    else:
        for name in values:
            prim.RemoveProperty(f"physxJointAxis:angular:{name}")
        if AXIS_API in items:
            spec.SetInfo("apiSchemas", Sdf.TokenListOp.Create(prependedItems=[x for x in items if x != AXIS_API]))
        print(f"fr3_joint{i + 1}: friction removed")

stage.GetRootLayer().Save()
roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print("articulation roots:", roots)
