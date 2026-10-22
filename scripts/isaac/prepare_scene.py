# Fix up the FR3 scene USD for the impedance controller (run with Isaac's Python, no SimulationApp needed):
#   ~/IsaacLab/.venv/bin/python -I prepare_scene.py [scene.usd] [fr3.urdf]
# 1. Select the "physx" Physics variant. Without a selection no physics payload is composed:
#    no articulation root, no joints, no rigid bodies.
# 2. Set PhysX joint armature = motor_inertia * gear_ratio^2 from the URDF <dynamics> tags (FR3 rotor
#    inertia reflected through the gearbox). Without it the light wrist (joint 7 drives only the hand)
#    chatters at Nyquist under the controller's damping at 240 Hz and the EE orientation wanders.
import sys
import xml.etree.ElementTree as ET

from pxr import Sdf, Usd, UsdPhysics

scene = sys.argv[1] if len(sys.argv) > 1 else "/home/njohnson/ci_mpc_assets/fr3_scene.usd"
urdf = sys.argv[2] if len(sys.argv) > 2 else "/home/njohnson/ci_mpc_assets/fr3.urdf"

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

stage.GetRootLayer().Save()
roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print("articulation roots:", roots)
