# Pick-and-place additions for the headless sim (loaded with runpy by headless_impedance_sim.py --pick-place):
# a static table, a 4 cm / 100 g cube on it, a position-controlled gripper and the cube pose for the ROS side.
#   add_objects(stage)                    before Play and before the Controller Manager starts: table top at
#                                         z 0.2 (x 0.5), cube at the pick spot, finger joints as position
#                                         drives (stiffness 2000 N/m, damping 100, max 40 N). isaacsim.ros2.control
#                                         decides per joint from the USD drive: stiffness > 0 = position command,
#                                         else effort, so this has to be in USD (tensor-API gains are not enough).
#                                         The ci_mpc_sim_gripper controller (ROS topic /gripper/width) then
#                                         commands the finger positions.
#                                         boxes=[(x, y, yaw_deg), ...] adds more boxes of the same size (stacking:
#                                         box0 at the pick spot, box1.. at these spots; colors differ).
#   Gripper(sync_dir, articulation_root)  after Play: poll() writes <sync>/object.json with the cube position
#                                         (first box; "objects": all boxes with name, cube, quat_wxyz, size)
# The real robot uses the franka_gripper action server instead; this is a simulation stand-in.
import json
import os

import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

TABLE_CENTER, TABLE_SIZE = (0.5, 0.0, 0.1), (0.4, 0.6, 0.2)
CUBE_EDGE, CUBE_MASS = 0.04, 0.1  # default object: 4 cm cube
PICK = (0.45, -0.12)
CUBE_PATH = "/PickPlace/Cube"
OBJECT_SIZE = [CUBE_EDGE] * 3


def _box(stage, path, center, size, color, yaw_deg=0.0):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddRotateZOp().Set(float(yaw_deg))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube.GetPrim()


COLORS = [(0.85, 0.1, 0.1), (0.1, 0.6, 0.15), (0.1, 0.3, 0.85), (0.9, 0.7, 0.1), (0.6, 0.2, 0.7)]
BOX_PATHS = [CUBE_PATH]


def add_objects(stage, size=(CUBE_EDGE,) * 3, yaw_deg=0.0, boxes=()):
    """size: object box extents [m] (x, y, z in its own frame), yaw_deg: rotation about z;
    boxes: more boxes [(x, y, yaw_deg)] of the same size."""
    material = UsdShade.Material.Define(stage, "/PickPlace/Material")
    pm = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    pm.CreateStaticFrictionAttr(1.0)
    pm.CreateDynamicFrictionAttr(1.0)
    pm.CreateRestitutionAttr(0.0)
    table = _box(stage, "/PickPlace/Table", TABLE_CENTER, TABLE_SIZE, (0.55, 0.45, 0.35))
    top = TABLE_CENTER[2] + TABLE_SIZE[2] / 2
    spots = [(PICK[0], PICK[1], yaw_deg)] + [tuple(b) for b in boxes]
    prims = [table]
    for i, (x, y, yaw) in enumerate(spots):
        path = CUBE_PATH if i == 0 else f"/PickPlace/Box{i}"
        cube = _box(stage, path, (x, y, top + size[2] / 2), size, COLORS[i % len(COLORS)], yaw)
        UsdPhysics.RigidBodyAPI.Apply(cube)
        UsdPhysics.MassAPI.Apply(cube).CreateMassAttr(CUBE_MASS)
        prims.append(cube)
        if i > 0:
            BOX_PATHS.append(path)
    for prim in prims:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
    for joint in ("fr3_finger_joint1", "fr3_finger_joint2"):
        drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(f"/fr3/Physics/{joint}"), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(2000.0)
        drive.CreateDampingAttr(100.0)
        drive.CreateMaxForceAttr(40.0)
        drive.CreateTargetPositionAttr(0.04)
    global OBJECT_SIZE
    OBJECT_SIZE = list(size)
    print(f"[sim] pick-place: table top z {top:.3f}, object {[round(x * 100, 1) for x in size]} cm, yaw {yaw_deg} deg "
          f"at {PICK}" + (f", more boxes at {list(boxes)}" if boxes else ""), flush=True)


class Gripper:
    """Cube pose for the ROS side (the fingers are driven by the ci_mpc_sim_gripper controller)."""

    def __init__(self, sync_dir, root):
        from isaacsim.core.experimental.prims import XformPrim

        self.sync_dir = sync_dir
        self.boxes = [XformPrim(p) for p in BOX_PATHS]
        self.frame = 0

    def poll(self):
        self.frame += 1
        if self.frame % 5 == 0:
            objects = []
            for i, box in enumerate(self.boxes):
                pos, quat = box.get_world_poses()  # quaternion w, x, y, z
                objects.append({"name": f"box{i}", "cube": np.asarray(pos).reshape(-1)[:3].tolist(),
                                "quat_wxyz": np.asarray(quat).reshape(-1)[:4].tolist(), "size": OBJECT_SIZE})
            tmp = os.path.join(self.sync_dir, "object.json.tmp")
            json.dump({**{k: objects[0][k] for k in ("cube", "quat_wxyz", "size")}, "objects": objects}, open(tmp, "w"))
            os.replace(tmp, os.path.join(self.sync_dir, "object.json"))
