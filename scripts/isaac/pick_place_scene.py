# Pick-and-place additions for the headless sim (loaded with runpy by headless_impedance_sim.py --pick-place):
# a static table, a 4 cm / 100 g cube on it, a position-controlled gripper and the cube pose for the ROS side.
#   add_objects(stage)                    before Play and before the Controller Manager starts: table top at
#                                         z 0.2 (x 0.5), cube at the pick spot, finger joints as position
#                                         drives (stiffness 2000 N/m, damping 100, max 40 N). isaacsim.ros2.control
#                                         decides per joint from the USD drive: stiffness > 0 = position command,
#                                         else effort, so this has to be in USD (tensor-API gains are not enough).
#                                         The ci_mpc_sim_gripper controller (ROS topic /gripper/width) then
#                                         commands the finger positions.
#   Gripper(sync_dir, articulation_root)  after Play: poll() writes <sync>/object.json with the cube position
# The real robot uses the franka_gripper action server instead; this is a simulation stand-in.
import json
import os

import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

TABLE_CENTER, TABLE_SIZE = (0.5, 0.0, 0.1), (0.4, 0.6, 0.2)
CUBE_EDGE, CUBE_MASS = 0.04, 0.1
PICK = (0.45, -0.12)
CUBE_PATH = "/PickPlace/Cube"


def _box(stage, path, center, size, color):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube.GetPrim()


def add_objects(stage):
    material = UsdShade.Material.Define(stage, "/PickPlace/Material")
    pm = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    pm.CreateStaticFrictionAttr(1.0)
    pm.CreateDynamicFrictionAttr(1.0)
    pm.CreateRestitutionAttr(0.0)
    table = _box(stage, "/PickPlace/Table", TABLE_CENTER, TABLE_SIZE, (0.55, 0.45, 0.35))
    top = TABLE_CENTER[2] + TABLE_SIZE[2] / 2
    cube = _box(stage, CUBE_PATH, (PICK[0], PICK[1], top + CUBE_EDGE / 2), (CUBE_EDGE,) * 3, (0.85, 0.1, 0.1))
    UsdPhysics.RigidBodyAPI.Apply(cube)
    UsdPhysics.MassAPI.Apply(cube).CreateMassAttr(CUBE_MASS)
    for prim in (table, cube):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
    for joint in ("fr3_finger_joint1", "fr3_finger_joint2"):
        drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(f"/fr3/Physics/{joint}"), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(2000.0)
        drive.CreateDampingAttr(100.0)
        drive.CreateMaxForceAttr(40.0)
        drive.CreateTargetPositionAttr(0.04)
    print(f"[sim] pick-place: table top z {top:.3f}, cube {CUBE_EDGE * 100:.0f} cm at {PICK} ", flush=True)


class Gripper:
    """Cube pose for the ROS side (the fingers are driven by the ci_mpc_sim_gripper controller)."""

    def __init__(self, sync_dir, root):
        from isaacsim.core.experimental.prims import XformPrim

        self.sync_dir = sync_dir
        self.cube = XformPrim(CUBE_PATH)
        self.frame = 0

    def poll(self):
        self.frame += 1
        if self.frame % 5 == 0:
            pos, _ = self.cube.get_world_poses()
            p = np.asarray(pos).reshape(-1)[:3].tolist()
            tmp = os.path.join(self.sync_dir, "object.json.tmp")
            json.dump({"cube": p}, open(tmp, "w"))
            os.replace(tmp, os.path.join(self.sync_dir, "object.json"))
