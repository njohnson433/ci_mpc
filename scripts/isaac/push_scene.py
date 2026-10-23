# Planar pushing additions for the headless sim (loaded with runpy by headless_impedance_sim.py --push):
# a static table, a square slider box on it and a pusher rod on the hand; the slider pose for the ROS side.
#   add_objects(stage, size, yaw_deg, xy)  before Play: table top at z 0.2 (x 0.5), slider (default 8 x 8 x 5 cm,
#                                          300 g) at xy, pusher rod = capsule collider on fr3_hand along the tool
#                                          axis, radius 6 mm, from 7 to 19 cm below the hand frame (tip 8.66 cm
#                                          below the TCP, so the fingertips stay above the slider).
#                                          Friction: table-slider 0.5, pusher-slider 0.3 (pusher material with
#                                          combine mode "min").
#   Poller(sync_dir)                       after Play: poll() writes <sync>/object.json (same format as the
#                                          pick-place scene: cube, quat_wxyz, size).
# On the real robot the rod would be a pin held in the closed gripper.
import json
import os

import numpy as np
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

TABLE_CENTER, TABLE_SIZE = (0.5, 0.0, 0.1), (0.4, 0.6, 0.2)
SLIDER_PATH = "/Push/Slider"
SLIDER_MASS = 0.3
MU_TABLE, MU_PUSHER = 0.5, 0.3
ROD_RADIUS, ROD_Z0, ROD_Z1 = 0.006, 0.07, 0.19  # in the fr3_hand frame (TCP at z 0.1034)
HAND = ("/fr3/Geometry/base/fr3_link0/fr3_link1/fr3_link2/fr3_link3/fr3_link4/fr3_link5/fr3_link6/fr3_link7/"
        "fr3_link8/fr3_hand")
OBJECT_SIZE = [0.08, 0.08, 0.05]


def _material(stage, path, mu, combine=None):
    material = UsdShade.Material.Define(stage, path)
    pm = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    pm.CreateStaticFrictionAttr(mu)
    pm.CreateDynamicFrictionAttr(mu)
    pm.CreateRestitutionAttr(0.0)
    if combine:
        PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr(combine)
    return material


def _bind(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")


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


def add_objects(stage, size=(0.08, 0.08, 0.05), yaw_deg=90.0, xy=(0.55, -0.15)):
    surface = _material(stage, "/Push/SurfaceMaterial", MU_TABLE)
    pusher = _material(stage, "/Push/PusherMaterial", MU_PUSHER, combine="min")
    table = _box(stage, "/Push/Table", TABLE_CENTER, TABLE_SIZE, (0.55, 0.45, 0.35))
    top = TABLE_CENTER[2] + TABLE_SIZE[2] / 2
    slider = _box(stage, SLIDER_PATH, (xy[0], xy[1], top + size[2] / 2), size, (0.1, 0.35, 0.85), yaw_deg)
    UsdPhysics.RigidBodyAPI.Apply(slider)
    UsdPhysics.MassAPI.Apply(slider).CreateMassAttr(SLIDER_MASS)
    _bind(table, surface)
    _bind(slider, surface)
    rod = UsdGeom.Capsule.Define(stage, HAND + "/pusher_rod")
    rod.CreateAxisAttr("Z")
    rod.CreateRadiusAttr(ROD_RADIUS)
    rod.CreateHeightAttr(ROD_Z1 - ROD_Z0 - 2 * ROD_RADIUS)
    UsdGeom.Xformable(rod.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, (ROD_Z0 + ROD_Z1) / 2))
    rod.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.9, 0.2)])
    UsdPhysics.CollisionAPI.Apply(rod.GetPrim())
    UsdPhysics.MassAPI.Apply(rod.GetPrim()).CreateMassAttr(0.01)
    _bind(rod.GetPrim(), pusher)
    global OBJECT_SIZE
    OBJECT_SIZE = list(size)
    print(f"[sim] push: table top z {top:.3f}, slider {[round(x * 100, 1) for x in size]} cm {SLIDER_MASS} kg, "
          f"yaw {yaw_deg} deg at {xy}; mu table {MU_TABLE}, pusher {MU_PUSHER}; rod r {ROD_RADIUS} m", flush=True)


class Poller:
    def __init__(self, sync_dir, root=None):
        from isaacsim.core.experimental.prims import XformPrim

        self.sync_dir = sync_dir
        self.slider = XformPrim(SLIDER_PATH)
        self.rod = XformPrim(HAND + "/pusher_rod")
        self.tcp = XformPrim(HAND + "/fr3_hand_tcp")
        self.frame = 0

    def poll(self):
        self.frame += 1
        if self.frame == 3:
            rod = np.asarray(self.rod.get_world_poses()[0]).reshape(-1)[:3]
            tcp = np.asarray(self.tcp.get_world_poses()[0]).reshape(-1)[:3]
            print(f"[sim] rod center {np.round(rod, 4).tolist()}, TCP {np.round(tcp, 4).tolist()}", flush=True)
        if self.frame % 5 == 0:
            pos, quat = self.slider.get_world_poses()  # quaternion w, x, y, z
            p = np.asarray(pos).reshape(-1)[:3].tolist()
            q = np.asarray(quat).reshape(-1)[:4].tolist()
            tmp = os.path.join(self.sync_dir, "object.json.tmp")
            json.dump({"cube": p, "quat_wxyz": q, "size": OBJECT_SIZE}, open(tmp, "w"))
            os.replace(tmp, os.path.join(self.sync_dir, "object.json"))
