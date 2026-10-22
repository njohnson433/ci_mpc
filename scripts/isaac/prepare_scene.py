# Fix up the FR3 scene USD for the impedance controller (run with Isaac's Python, no SimulationApp needed):
#   ~/IsaacLab/.venv/bin/python -I prepare_scene.py [scene.usd] [--params joint_params.yaml] [--no-friction]
# 1. Select the "physx" Physics variant. Without a selection no physics payload is composed:
#    no articulation root, no joints, no rigid bodies.
# 2. Set PhysX joint armature (FR3 rotor inertia reflected through the gearbox). Without it the light
#    wrist (joint 7 drives only the hand) chatters at Nyquist under the controller's damping at 240 Hz
#    and the EE orientation wanders.
# 3. Set joint friction (PhysxJointAxisAPI "angular": static, Coulomb, viscous). Not representable in
#    PhysX: the direction-independent offset (ignored here). --no-friction removes the friction again.
# Values come from --params (default: ci_mpc_sysid/config/fr3_joint_params_sim.yaml, see its `source`).
import math
import os
import sys

import yaml
from pxr import Sdf, Usd, UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__))
argv = sys.argv[1:]
params_path = os.path.join(HERE, "..", "..", "ci_mpc_sysid", "config", "fr3_joint_params_sim.yaml")
if "--params" in argv:
    i = argv.index("--params")
    params_path = argv[i + 1]
    del argv[i:i + 2]
friction = "--no-friction" not in argv
args = [a for a in argv if not a.startswith("--")]
scene = args[0] if args else "/home/njohnson/ci_mpc_assets/fr3_scene.usd"

params = yaml.safe_load(open(params_path))["joints"]
print("joint parameters from", os.path.realpath(params_path))

stage = Usd.Stage.Open(scene)
vs = stage.GetPrimAtPath("/fr3").GetVariantSets().GetVariantSet("Physics")
if vs.GetVariantSelection() != "physx":
    print("Physics variant:", repr(vs.GetVariantSelection()), "-> physx")
    vs.SetVariantSelection("physx")


def set_attr(prim, name, value):
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.Float)
    attr.Set(value)


AXIS_API = "PhysxJointAxisAPI:angular"
FRICTION_ATTRS = ("staticFrictionEffort", "dynamicFrictionEffort", "viscousFrictionCoefficient")
for name, p in sorted(params.items()):
    prim = stage.GetPrimAtPath(f"/fr3/Physics/{name}")
    set_attr(prim, "physxJoint:armature", p["armature"])
    layer = stage.GetRootLayer()
    spec = layer.GetPrimAtPath(prim.GetPath()) or Sdf.CreatePrimInLayer(layer, prim.GetPath())
    items = list(spec.GetInfo("apiSchemas").prependedItems) if spec.HasInfo("apiSchemas") else []
    if friction:
        if AXIS_API not in items:
            spec.SetInfo("apiSchemas", Sdf.TokenListOp.Create(prependedItems=items + [AXIS_API]))
        set_attr(prim, "physxJointAxis:angular:staticFrictionEffort", p["static"])
        set_attr(prim, "physxJointAxis:angular:dynamicFrictionEffort", p["coulomb"])
        # PhysX unit: Nm s/deg
        set_attr(prim, "physxJointAxis:angular:viscousFrictionCoefficient", p["viscous"] * math.pi / 180.0)
        print(f"{name}: armature {p['armature']:.4f} kg m^2, static {p['static']:.3f} Nm, "
              f"Coulomb {p['coulomb']:.3f} Nm, viscous {p['viscous']:.3f} Nm s/rad")
    else:
        for attr in FRICTION_ATTRS:
            prim.RemoveProperty(f"physxJointAxis:angular:{attr}")
        if AXIS_API in items:
            spec.SetInfo("apiSchemas", Sdf.TokenListOp.Create(prependedItems=[x for x in items if x != AXIS_API]))
        print(f"{name}: armature {p['armature']:.4f} kg m^2, no friction")

stage.GetRootLayer().Save()
roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print("articulation roots:", roots)
