# Isaac Sim Script Editor: print the prim(s) carrying the ArticulationRootAPI.
import omni.usd
from pxr import UsdPhysics
stage = omni.usd.get_context().get_stage()
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.ArticulationRootAPI):
        print("Articulation root:", p.GetPath())
