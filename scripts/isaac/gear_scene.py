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
SHAFT_TRAVEL = 0.020                 # shaft top z 25 mm (r 4.75 mm, 0.5 mm chamfer) to seated; bore r 5.0 mm
FRICTION, ITERATIONS = 0.75, 32
BASE, MEDIUM = "/Gear/Base", "/Gear/Medium"
BASE_BODY, MEDIUM_BODY = BASE + "/factory_gear_base_loose", MEDIUM + "/factory_gear_medium"
DROP_GEAR = "/Gear/DropGear{}/factory_gear_medium"


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


def _ref(stage, path, asset, pos, yaw_deg=0.0):
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(f"{ASSETS}/{asset}.usd")
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()  # the assets carry identity translate/orient/scale ops
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x.AddRotateZOp().Set(float(yaw_deg))
    return prim


def _assembly(stage, root, xy, yaw_deg, mat):
    """Kinematic base with the small and large gears in their assembled places, origin at xy on the table."""
    for name, asset in (("Base", "factory_gear_base"), ("Small", "factory_gear_small"), ("Large", "factory_gear_large")):
        _ref(stage, f"{root}/{name}", asset, (xy[0], xy[1], TOP), yaw_deg)
        b = stage.GetPrimAtPath(f"{root}/{name}/{_BODY[name]}")
        if b.HasAPI(UsdPhysics.ArticulationRootAPI):
            b.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        j = stage.GetPrimAtPath(f"{root}/{name}/root_joint")
        if j.IsValid():
            j.SetActive(False)
        _body(b, kinematic=True)
        _bind(b, mat)


def _free_gear(stage, path, origin, yaw_deg, mat):
    _ref(stage, path, "factory_gear_medium", origin, yaw_deg)
    mb = stage.GetPrimAtPath(path + "/factory_gear_medium")
    mb.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    _body(mb, kinematic=False)
    PhysxSchema.PhysxRigidBodyAPI(mb).CreateSleepThresholdAttr(0.0)
    _bind(mb, mat)


_BODY = {"Base": "factory_gear_base_loose", "Small": "factory_gear_small", "Large": "factory_gear_large"}
# The assets' medium gear frame is not in mesh with the fixed gears at yaw 0: its 40 teeth (pitch 9 deg) face the
# small (20) and large (60) gears tooth on tooth there (drop test: stuck 10 mm above the seat), at +4.5 deg they mesh.
MESH_YAW = 4.5
DROPS = ((0.0, 0.0), (0.0002, 0.0), (0.0, 2.0))  # drop test: (x offset [m], yaw offset from MESH_YAW [deg])
DROP_XY = ((0.38, 0.2), (0.58, 0.2), (0.62, -0.2))


def add_objects(stage, base_xy=(0.5, 0.05), gear_xy=(0.45, -0.12), base_yaw_deg=0.0, drop_test=False):
    """drop_test: also drop free medium gears 3 mm above the shaft top of extra assemblies (DROPS), no robot."""
    mat = _material(stage, "/Gear/Material", FRICTION)
    table = UsdGeom.Cube.Define(stage, "/Gear/Table")
    table.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(table.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*TABLE_CENTER))
    xf.AddScaleOp().Set(Gf.Vec3f(*TABLE_SIZE))
    table.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.45, 0.35)])
    UsdPhysics.CollisionAPI.Apply(table.GetPrim())
    _bind(table.GetPrim(), mat)
    _assembly(stage, "/Gear", base_xy, base_yaw_deg, mat)
    # medium gear lying on the table: its axis at gear_xy, its bottom face on the table top
    _free_gear(stage, MEDIUM, (gear_xy[0] - MEDIUM_AXIS[0], gear_xy[1] - MEDIUM_AXIS[1], TOP - GEAR_Z0), 0.0, mat)
    if drop_test:
        for i, ((dx, dyaw), xy) in enumerate(zip(DROPS, DROP_XY)):
            _assembly(stage, f"/Gear/Drop{i}", xy, 0.0, mat)
            # rotate about the medium gear's own axis: shift the origin so the axis stays on the shaft
            dyaw += MESH_YAW
            a = np.radians(dyaw)
            axis_off = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]) @ MEDIUM_AXIS
            o = np.array(xy) + MEDIUM_AXIS - axis_off + np.array([dx, 0.0])
            _free_gear(stage, f"/Gear/DropGear{i}", (o[0], o[1], TOP + SHAFT_TRAVEL + 0.003), dyaw, mat)
    for joint in ("fr3_finger_joint1", "fr3_finger_joint2"):  # force-controlled 40 N grasp (as the peg scene)
        p = stage.GetPrimAtPath(f"/fr3/Physics/{joint}")
        if p.IsValid():
            drive = UsdPhysics.DriveAPI.Apply(p, "linear")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(20000.0)
            drive.CreateDampingAttr(200.0)
            drive.CreateMaxForceAttr(20.0)
            drive.CreateTargetPositionAttr(0.04)
    print(f"[sim] gear mesh (Factory, SDF): base at {base_xy} yaw {base_yaw_deg} deg, medium gear at {gear_xy}"
          + (", drop test gears" if drop_test else ""), flush=True)


class Poller:
    def __init__(self, sync_dir, root=None):
        from isaacsim.core.experimental.prims import XformPrim

        self.sync_dir = sync_dir
        self.gear = XformPrim(MEDIUM_BODY)
        self.base = XformPrim(BASE_BODY)
        self.frame = 0
        from pxr import Usd  # noqa: F401
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        self.drops = [XformPrim(DROP_GEAR.format(i)) for i in range(len(DROPS))
                      if stage.GetPrimAtPath(DROP_GEAR.format(i)).IsValid()]

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
        if self.drops and self.frame % 240 == 0:  # drop test: seated = origin z at the table top (0 mm)
            for i, d in enumerate(self.drops):
                p, q, R = self._pose(d)
                yaw = np.degrees(2 * np.arctan2(q[3], q[0])) - MESH_YAW
                print(f"[sim] drop gear {i} {DROPS[i]}: {(p[2] - TOP) * 1000:+.2f} mm above seated, "
                      f"axis xy {np.round((p[:2] + R[:2, :2] @ MEDIUM_AXIS - np.array(DROP_XY[i]) - MEDIUM_AXIS) * 1000, 2).tolist()} mm, yaw {yaw:+.1f} deg", flush=True)
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
