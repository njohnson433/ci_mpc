# Factory gear mesh (held-out benchmark H2; Isaac Lab Factory assets, SDF collision) for the headless sim (loaded
# with runpy by headless_impedance_sim.py --gear): the gear base with the small and large gears fixed in their
# assembled places, the medium gear lying on the table to be picked; it has to go onto the middle shaft and mesh.
# Asset frames: every gear's origin is the base origin in the assembled pose (the medium gear's axis is at
# x = +20.25 mm, its body spans z 5-30 mm); the base plate spans 150 x 75 mm.
#   add_objects(stage, base_xy, gear_xy, base_yaw_deg)  before Play
#   Poller(sync_dir)  writes <sync>/object.json with "peg" = the medium gear (center, size 42 x 42 x 25 mm) and
#                     "hole" = the middle shaft's seat (opening = where the gear's bottom face starts on the shaft,
#                     size z = travel to seated), so the peg demo's strategies run unchanged; "origin" of both is
#                     the assembled-pose origin (success: gear origin within 1 mm of the base origin).
import json
import os

import numpy as np
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

ASSETS = "/home/njohnson/ci_mpc_assets/factory"
TABLE_CENTER, TABLE_SIZE = (0.5, 0.0, 0.1), (0.4, 0.6, 0.2)
TOP = TABLE_CENTER[2] + TABLE_SIZE[2] / 2
MEDIUM_AXIS = np.array([0.02025, 0.0])
GEAR_Z0, GEAR_Z1 = 0.005, 0.030      # medium gear body, in the asset frame
SHAFT_TRAVEL = 0.020                 # assumed travel from first contact with the shaft top to seated
FRICTION, ITERATIONS = 0.75, 32
BASE, MEDIUM = "/Gear/Base", "/Gear/Medium"
BASE_BODY, MEDIUM_BODY = BASE + "/factory_gear_base_loose/factory_gear_base_loose", MEDIUM + "/factory_gear_medium"


def _material(stage, path, mu):
    m = UsdShade.Material.Define(stage, path)
    pm = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
    pm.CreateStaticFrictionAttr(mu)
    pm.CreateDynamicFrictionAttr(mu)
    pm.CreateRestitutionAttr(0.0)
    return m


def _bind(prim, m):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(m, UsdShade.Tokens.weakerThanDescendants, "physics")


def _body(prim, kinematic):
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(bool(kinematic))
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateSolverPositionIterationCountAttr(ITERATIONS)
    rb.CreateSolverVelocityIterationCountAttr(1)
    rb.CreateMaxDepenetrationVelocityAttr(5.0)


def add_objects(stage, base_xy=(0.5, 0.05), gear_xy=(0.45, -0.12), base_yaw_deg=0.0):
    mat = _material(stage, "/Gear/Material", FRICTION)
    table = UsdGeom.Cube.Define(stage, "/Gear/Table")
    table.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(table.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*TABLE_CENTER))
    xf.AddScaleOp().Set(Gf.Vec3f(*TABLE_SIZE))
    table.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.45, 0.35)])
    UsdPhysics.CollisionAPI.Apply(table.GetPrim())
    _bind(table.GetPrim(), mat)
    # base and the two fixed gears, all in the assembled pose at the base origin
    for path, asset, body in ((BASE, "factory_gear_base", BASE_BODY),
                              ("/Gear/Small", "factory_gear_small", "/Gear/Small/factory_gear_small"),
                              ("/Gear/Large", "factory_gear_large", "/Gear/Large/factory_gear_large")):
        prim = stage.DefinePrim(path, "Xform")
        prim.GetReferences().AddReference(f"{ASSETS}/{asset}.usd")
        x = UsdGeom.Xformable(prim)
        x.AddTranslateOp().Set(Gf.Vec3d(base_xy[0], base_xy[1], TOP))
        x.AddRotateZOp().Set(float(base_yaw_deg))
        b = stage.GetPrimAtPath(body)
        if b.HasAPI(UsdPhysics.ArticulationRootAPI):
            b.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        j = stage.GetPrimAtPath(path + "/factory_gear_base_loose/root_joint")
        if j.IsValid():
            j.SetActive(False)
        _body(b, kinematic=True)
        _bind(b, mat)
    # medium gear lying on the table: its axis at gear_xy, its bottom face on the table top
    med = stage.DefinePrim(MEDIUM, "Xform")
    med.GetReferences().AddReference(f"{ASSETS}/factory_gear_medium.usd")
    UsdGeom.Xformable(med).AddTranslateOp().Set(Gf.Vec3d(gear_xy[0] - MEDIUM_AXIS[0], gear_xy[1] - MEDIUM_AXIS[1],
                                                         TOP - GEAR_Z0))
    mb = stage.GetPrimAtPath(MEDIUM_BODY)
    mb.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    _body(mb, kinematic=False)
    _bind(mb, mat)
    for joint in ("fr3_finger_joint1", "fr3_finger_joint2"):  # force-controlled 40 N grasp (as the peg scene)
        p = stage.GetPrimAtPath(f"/fr3/Physics/{joint}")
        if p.IsValid():
            drive = UsdPhysics.DriveAPI.Apply(p, "linear")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(20000.0)
            drive.CreateDampingAttr(200.0)
            drive.CreateMaxForceAttr(20.0)
            drive.CreateTargetPositionAttr(0.04)
    print(f"[sim] gear mesh (Factory, SDF): base at {base_xy} yaw {base_yaw_deg} deg, medium gear at {gear_xy}",
          flush=True)


class Poller:
    def __init__(self, sync_dir, root=None):
        from isaacsim.core.experimental.prims import XformPrim

        self.sync_dir = sync_dir
        self.gear = XformPrim(MEDIUM_BODY)
        self.base = XformPrim(BASE_BODY)
        self.frame = 0

    @staticmethod
    def _pose(prim):
        pos, quat = prim.get_world_poses()
        p = np.asarray(pos.numpy() if hasattr(pos, "numpy") else pos).reshape(-1)[:3]
        q = np.asarray(quat.numpy() if hasattr(quat, "numpy") else quat).reshape(-1)[:4]
        w, x, y, z = q
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        return p, q, R

    def poll(self):
        self.frame += 1
        if self.frame % 5:
            return
        gp, gq, gR = self._pose(self.gear)
        bp, bq, bR = self._pose(self.base)
        gear_center = gp + gR @ np.array([MEDIUM_AXIS[0], MEDIUM_AXIS[1], (GEAR_Z0 + GEAR_Z1) / 2])
        # seat opening: where the gear's bottom face is when it first meets the shaft top (travel SHAFT_TRAVEL above
        # the seated bottom face)
        seat = bp + bR @ np.array([MEDIUM_AXIS[0], MEDIUM_AXIS[1], GEAR_Z0 + SHAFT_TRAVEL])
        objs = [{"name": "peg", "cube": gear_center.tolist(), "quat_wxyz": gq.tolist(),
                 "size": [0.042, 0.042, GEAR_Z1 - GEAR_Z0], "origin": gp.tolist()},
                {"name": "hole", "cube": seat.tolist(), "quat_wxyz": bq.tolist(),
                 "size": [0.008, 0.008, SHAFT_TRAVEL], "origin": bp.tolist()}]
        tmp = os.path.join(self.sync_dir, "object.json.tmp")
        json.dump({**{k: objs[0][k] for k in ("cube", "quat_wxyz", "size")}, "objects": objs}, open(tmp, "w"))
        os.replace(tmp, os.path.join(self.sync_dir, "object.json"))
