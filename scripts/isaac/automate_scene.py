# AutoMate assemblies (held-out benchmark H3; Isaac Lab AutoMate assets, 100 plug/socket pairs, SDF collision) for
# the headless sim (loaded with runpy by headless_impedance_sim.py --automate ID). Assets: ~/ci_mpc_assets/automate/
# <ID>/{plug,socket}.usd + .obj, plug_grasps.json (hand pose in the plug frame, seated), disassembly_dist.json
# (insertion travel). Plug and socket share one frame in the assembled pose (origin of both = the seated pose).
#   add_objects(stage, assembly, socket_xy, plug_xy)  before Play: kinematic target socket on the table; the plug
#                                                     seated in a second (source) socket at plug_xy (take it out, put it in)
#   Poller(sync_dir)  writes <sync>/object.json with "peg" = the plug (center xy = the plug's center at the grasp
#                     height, size = [grasp width, grasp width, height]) and "hole" = the socket opening (where the
#                     plug's bottom is when it starts to enter); "origin" = the plug's frame at its bottom height and
#                     the seated position of that point, so depth / xy error are as for the peg.
#   info(assembly)    the geometry the demo needs: grasp depth below the top, grasp width, travel, contact model.
import json
import os

import numpy as np
try:
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade
except ImportError:  # info() only (demo_automate.sh, outside a running Isaac app): no PhysxSchema there
    pass

ASSETS = os.environ.get("AUTOMATE_ASSETS", "/home/njohnson/ci_mpc_assets/automate")  # NIST (H5): ~/ci_mpc_assets/nist
TABLE_CENTER, TABLE_SIZE = (0.5, 0.0, 0.1), (0.4, 0.6, 0.2)
TOP = TABLE_CENTER[2] + TABLE_SIZE[2] / 2
HAND_TO_TCP = 0.1034  # fr3_hand -> fr3_hand_tcp
FRICTION, ITERATIONS = 0.75, 32
PLUG, SOCKET, SOURCE = "/Asm/Plug", "/Asm/Socket", "/Asm/SourceSocket"


def _obj(path):
    return np.array([ln.split()[1:4] for ln in open(path) if ln.startswith("v ")], float)


def info(assembly):
    pv, sv = _obj(f"{ASSETS}/{assembly}/plug.obj"), _obj(f"{ASSETS}/{assembly}/socket.obj")
    g = json.load(open(f"{ASSETS}/plug_grasps.json"))[f"asset_{assembly}"]
    travel = json.load(open(f"{ASSETS}/disassembly_dist.json"))[f"asset_{assembly}"]
    tcp_z = g[2] - HAND_TO_TCP
    band = pv[np.abs(pv[:, 2] - tcp_z) < 0.005]
    center = (band[:, :2].min(0) + band[:, :2].max(0)) / 2
    tip = pv[pv[:, 2] < pv[:, 2].min() + 0.003]
    a = min(np.ptp(tip[:, 0]), np.ptp(tip[:, 1])) / 2
    # entry plane: the socket's top face around the hole. z0 + travel (the disassembly distance) lies 1.1-2.0 mm
    # above it (00700: 5.1 mm, so the plug "engaged" on the top face and no strategy searched; H3 run 1, 2026-10-24)
    c_tip = (tip[:, :2].min(0) + tip[:, :2].max(0)) / 2
    near = sv[np.hypot(sv[:, 0] - c_tip[0], sv[:, 1] - c_tip[1]) < max(np.ptp(tip[:, :2], axis=0)) / 2 + 0.003]
    entry = near[:, 2].max() - pv[:, 2].min()  # travel from first contact with the socket to seated
    return {"z0": pv[:, 2].min(), "z1": pv[:, 2].max(), "grasp_depth": pv[:, 2].max() - tcp_z,
            "grasp_width": np.ptp(band[:, 1]), "center_xy": center, "travel": travel, "entry": entry,
            "socket_top": sv[:, 2].max(),
            # round-peg contact model guess (the true features are not round): plug tip half width, 0.25 mm clearance
            "geo": (a + 0.00025, 0.0005, entry, a, 0.0003)}


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
    if not kinematic:
        rb.CreateSleepThresholdAttr(0.0)


def _ref(stage, path, usd, pos):
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(usd)
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    return prim


def add_objects(stage, assembly, socket_xy=(0.5, 0.05), plug_xy=(0.45, -0.12)):
    i = info(assembly)
    mat = _material(stage, "/Asm/Material", FRICTION)
    table = UsdGeom.Cube.Define(stage, "/Asm/Table")
    table.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(table.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*TABLE_CENTER))
    xf.AddScaleOp().Set(Gf.Vec3f(*TABLE_SIZE))
    table.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.45, 0.35)])
    UsdPhysics.CollisionAPI.Apply(table.GetPrim())
    _bind(table.GetPrim(), mat)
    # the plug starts seated in a second (source) socket: AutoMate plugs do not stand on their own, so the task is
    # take it out of the source socket and put it into the target socket
    src = (plug_xy[0] - i["center_xy"][0], plug_xy[1] - i["center_xy"][1], TOP)
    for path, name, pos, kinematic in ((SOCKET, "socket", (socket_xy[0], socket_xy[1], TOP), True),
                                       (SOURCE, "socket", src, True), (PLUG, "plug", src, False)):
        _ref(stage, path, f"{ASSETS}/{assembly}/{name}.usd", pos)
        body = stage.GetPrimAtPath(f"{path}/a_{assembly}_{name}")
        if body.HasAPI(UsdPhysics.ArticulationRootAPI):
            body.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        j = stage.GetPrimAtPath(f"{path}/root_joint")
        if j.IsValid():
            j.SetActive(False)
        _body(body, kinematic)
        _bind(body, mat)
        # as Isaac Lab's AutoMate config: mass 0.05 kg (the assets carry 0: PhysX would use density 1000, a few
        # grams, and 20 N fingers then sink into the SDF), contact offset 5 mm, rest offset 0
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(0.05)
        for p in Usd.PrimRange(body):
            if p.IsInstance():
                p.SetInstanceable(False)
        for p in Usd.PrimRange(body):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                pc = PhysxSchema.PhysxCollisionAPI.Apply(p)
                pc.CreateContactOffsetAttr(0.005)
                pc.CreateRestOffsetAttr(0.0)
    for joint in ("fr3_finger_joint1", "fr3_finger_joint2"):  # force-controlled 40 N grasp (as the peg scene)
        p = stage.GetPrimAtPath(f"/fr3/Physics/{joint}")
        if p.IsValid():
            drive = UsdPhysics.DriveAPI.Apply(p, "linear")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(20000.0)
            drive.CreateDampingAttr(200.0)
            drive.CreateMaxForceAttr(20.0)
            drive.CreateTargetPositionAttr(0.04)
    print(f"[sim] AutoMate {assembly} (SDF): socket at {socket_xy}, plug seated in a source socket at {plug_xy}; grasp width "
          f"{i['grasp_width'] * 1000:.1f} mm, {i['grasp_depth'] * 1000:.1f} mm below the top, travel "
          f"{i['travel'] * 1000:.1f} mm", flush=True)


class Poller:
    def __init__(self, sync_dir, assembly):
        from isaacsim.core.experimental.prims import XformPrim

        self.sync_dir, self.i = sync_dir, info(assembly)
        self.plug = XformPrim(f"{PLUG}/a_{assembly}_plug")
        self.socket = XformPrim(f"{SOCKET}/a_{assembly}_socket")
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
        i = self.i
        pp, pq, pR = self._pose(self.plug)
        sp, sq, sR = self._pose(self.socket)
        c = np.array([*i["center_xy"], 0.0])
        plug_center = pp + pR @ (c + [0, 0, (i["z0"] + i["z1"]) / 2])
        opening = sp + sR @ (c + [0, 0, i["z0"] + i["entry"]])
        w = i["grasp_width"]
        objs = [{"name": "peg", "cube": plug_center.tolist(), "quat_wxyz": pq.tolist(),
                 "size": [w, w, i["z1"] - i["z0"]], "origin": (pp + pR @ (c + [0, 0, i["z0"]])).tolist()},
                {"name": "hole", "cube": opening.tolist(), "quat_wxyz": sq.tolist(),
                 "size": [w, w, i["entry"]], "origin": (sp + sR @ (c + [0, 0, i["z0"]])).tolist()}]
        tmp = os.path.join(self.sync_dir, "object.json.tmp")
        json.dump({**{k: objs[0][k] for k in ("cube", "quat_wxyz", "size")}, "objects": objs}, open(tmp, "w"))
        os.replace(tmp, os.path.join(self.sync_dir, "object.json"))
