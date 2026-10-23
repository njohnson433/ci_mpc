# Factory peg-in-hole (NVIDIA Isaac Lab Factory / FORGE assets, 8 mm round peg) for the headless sim (loaded with
# runpy by headless_impedance_sim.py --peg). Assets: ~/ci_mpc_assets/factory/factory_{peg,hole}_8mm.usd (from the
# Isaac 6.2 asset server, Isaac/IsaacLab/Factory). Both meshes carry PhysX SDF collision (physics:approximation
# "sdf"), which tight mesh-mesh contact needs (convex decomposition cannot represent a 0.06 mm radial clearance).
#   add_objects(stage, hole_xy, peg_xy)  before Play:
#     table as in the pick-place scene (top z 0.2), the hole block on it (kinematic, bottom on the table, hole top
#     z 0.228), the peg standing upright on the table (dynamic, 19 g); gripper finger joints as position drives
#     (like pick_place_scene). Contact settings as in Isaac Lab Factory: friction 0.75, contact offset 5 mm, rest
#     offset 0, ITERATIONS (32; Factory: 192) position iterations on peg and hole, max depenetration velocity 5 m/s.
#   Poller(sync_dir)                     after Play: appends the total contact force on the hole block (PhysX contact
#                                        report, mean over the physics steps since the last write) to
#                                        <sync>/hole_contact.csv (sim time, fx, fy, fz [N], world frame); writes
#                                        <sync>/object.json: objects "peg" and "hole" (center of
#                                        the hole opening at its top face; peg center), same format as the other
#                                        scenes (cube, quat_wxyz, size).
# Geometry (meshes, metres, z up): peg  r 3.993 mm, z 0..50 mm (origin at its bottom face)
#                                  hole r 4.05 mm, depth 25 mm, block 40 x 40 mm, z -3..25 mm (origin at the hole
#                                  bottom), radial clearance 0.057 mm.
import json
import os

import numpy as np
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

ASSETS = "/home/njohnson/ci_mpc_assets/factory"
TABLE_CENTER, TABLE_SIZE = (0.5, 0.0, 0.1), (0.4, 0.6, 0.2)
TOP = TABLE_CENTER[2] + TABLE_SIZE[2] / 2
PEG, HOLE = "/Peg/Peg", "/Peg/Hole"
PEG_BODY, HOLE_BODY = PEG + "/forge_round_peg_8mm", HOLE + "/forge_hole_8mm/forge_hole_8mm"
PEG_LEN, PEG_D, HOLE_D, HOLE_DEPTH, HOLE_BASE = 0.050, 0.007986, 0.0081, 0.025, 0.003
FRICTION = 0.75


def _material(stage, path, mu):
    m = UsdShade.Material.Define(stage, path)
    pm = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
    pm.CreateStaticFrictionAttr(mu)
    pm.CreateDynamicFrictionAttr(mu)
    pm.CreateRestitutionAttr(0.0)
    return m


def _bind(prim, m):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(m, UsdShade.Tokens.weakerThanDescendants, "physics")


ITERATIONS = 32  # Factory uses 192; holding the peg puts the robot in the same island (192: RTF 0.19)


def _factory_body(prim, iterations=None):
    iterations = iterations or ITERATIONS
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateSolverPositionIterationCountAttr(iterations)
    rb.CreateSolverVelocityIterationCountAttr(1)
    rb.CreateMaxDepenetrationVelocityAttr(5.0)
    rb.CreateLinearDampingAttr(0.0)
    rb.CreateAngularDampingAttr(0.0)


def add_objects(stage, hole_xy=(0.5, 0.1), peg_xy=(0.45, -0.12), drop_test=False, friction=None, hole_tilt_deg=0.0):
    """friction: contact friction of table / peg / hole (default FRICTION); hole_tilt_deg: the hole block tilted
    about the world x axis (held-out variations, unknown to the controller)."""
    """drop_test: also drop free pegs into the hole (no robot needed) to check the contact setup."""
    mat = _material(stage, "/Peg/Material", FRICTION if friction is None else friction)
    table = UsdGeom.Cube.Define(stage, "/Peg/Table")
    table.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(table.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*TABLE_CENTER))
    xf.AddScaleOp().Set(Gf.Vec3f(*TABLE_SIZE))
    table.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.45, 0.35)])
    UsdPhysics.CollisionAPI.Apply(table.GetPrim())
    _bind(table.GetPrim(), mat)
    # hole block: bottom of the block on the table (mesh z -3 mm), kinematic (fixed in place); its articulation
    # fixed joint is deactivated (its anchor is in the asset's own frame)
    holes = [(HOLE, hole_xy)]
    pegs = [(PEG, (peg_xy[0], peg_xy[1], TOP))]
    if drop_test:  # extra holes 6 cm apart, a free peg 5 mm above each: aligned, 0.5 mm and 1.5 mm off
        for i, dx in enumerate((0.0, 0.0005, 0.0015)):
            xy = (hole_xy[0], hole_xy[1] - 0.06 * (i + 1))
            holes.append((f"/Peg/DropHole{i}", xy))
            pegs.append((f"/Peg/Drop{i}", (xy[0] + dx, xy[1], TOP + HOLE_BASE + HOLE_DEPTH + 0.005)))
    for path, xy in holes:
        hole = stage.DefinePrim(path, "Xform")
        hole.GetReferences().AddReference(f"{ASSETS}/factory_hole_8mm.usd")
        UsdGeom.Xformable(hole).AddTranslateOp().Set(Gf.Vec3d(xy[0], xy[1], TOP + HOLE_BASE))
        if hole_tilt_deg and path == HOLE:
            UsdGeom.Xformable(hole).AddRotateXOp().Set(float(hole_tilt_deg))
        stage.GetPrimAtPath(path + "/forge_hole_8mm/FixedJoint").SetActive(False)
        hb = stage.GetPrimAtPath(path + "/forge_hole_8mm/forge_hole_8mm")
        UsdPhysics.RigidBodyAPI(hb).CreateKinematicEnabledAttr(True)
        _factory_body(hb)
        _bind(hb, mat)
        # contact forces on the hole blocks (ground truth for the external wrench estimate; drop holes: calibration)
        PhysxSchema.PhysxContactReportAPI.Apply(hb).CreateThresholdAttr().Set(0.0)
    for path, pos in pegs:
        peg = stage.DefinePrim(path, "Xform")
        peg.GetReferences().AddReference(f"{ASSETS}/factory_peg_8mm.usd")
        UsdGeom.Xformable(peg).AddTranslateOp().Set(Gf.Vec3d(*pos))
        body = stage.GetPrimAtPath(path + "/forge_round_peg_8mm")
        body.RemoveAPI(UsdPhysics.ArticulationRootAPI)  # a free rigid body, not a one-link articulation
        UsdPhysics.MassAPI(body).CreateMassAttr(0.019)
        _factory_body(body)
        PhysxSchema.PhysxRigidBodyAPI(body).CreateSleepThresholdAttr(0.0)
        _bind(body, mat)
    for joint in ("fr3_finger_joint1", "fr3_finger_joint2"):
        p = stage.GetPrimAtPath(f"/fr3/Physics/{joint}")
        if p.IsValid():
            drive = UsdPhysics.DriveAPI.Apply(p, "linear")
            drive.CreateTypeAttr("force")
            # force-controlled grasp like the Franka Hand (see pick_place_scene.py): 20 N per finger = 40 N grasp
            drive.CreateStiffnessAttr(20000.0)
            drive.CreateDampingAttr(200.0)
            drive.CreateMaxForceAttr(20.0)
            drive.CreateTargetPositionAttr(0.04)
    print(f"[sim] friction {FRICTION if friction is None else friction}, hole tilt {hole_tilt_deg} deg", flush=True)
    print(f"[sim] peg-in-hole (Factory 8 mm, SDF): hole at {hole_xy} (top z {TOP + HOLE_BASE + HOLE_DEPTH:.3f}), "
          f"peg at {peg_xy}" + (", drop test pegs" if drop_test else ""), flush=True)


class Poller:
    def __init__(self, sync_dir, root=None):
        from isaacsim.core.experimental.prims import XformPrim

        self.sync_dir = sync_dir
        self.peg = XformPrim(PEG_BODY)
        self.hole = XformPrim(HOLE_BODY)
        from pxr import Usd
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        self.drops = [XformPrim(f"/Peg/Drop{i}/forge_round_peg_8mm") for i in range(3)
                      if stage.GetPrimAtPath(f"/Peg/Drop{i}").IsValid()]
        self.frame = 0
        # contact report: impulses on the hole block per physics step
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools
        import omni.timeline
        self._to_path = PhysicsSchemaTools.intToSdfPath
        self.timeline = omni.timeline.get_timeline_interface()
        self.impulse, self.steps, self.total_steps = np.zeros(3), 0, 0
        self.drop_imp, self.drop_headers, self.drop_steps0 = [np.zeros(3) for _ in range(3)], [0, 0, 0], None
        self.contact_csv = open(os.path.join(sync_dir, "hole_contact.csv"), "w")
        self.contact_csv.write("t,fx,fy,fz\n")
        self.contact_sub = get_physx_simulation_interface().subscribe_contact_report_events(self.on_contacts)

    def on_contacts(self, headers, data):
        self.steps += 1
        self.total_steps += 1
        if self.drops:  # calibration: per drop hole, the force of the free peg resting in it (= its weight)
            for k in range(3):
                hp = f"/Peg/DropHole{k}/forge_hole_8mm/forge_hole_8mm"
                for h in headers:
                    a0, a1 = str(self._to_path(h.actor0)), str(self._to_path(h.actor1))
                    if hp in (a0, a1):
                        sign = 1.0 if a0 == hp else -1.0
                        for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                            imp = data[i].impulse
                            self.drop_imp[k] += sign * np.array([imp[0], imp[1], imp[2]])
                        self.drop_headers[k] += 1
        for h in headers:
            a0, a1 = str(self._to_path(h.actor0)), str(self._to_path(h.actor1))
            if HOLE_BODY not in (a0, a1):
                continue
            sign = 1.0 if a0 == HOLE_BODY else -1.0  # impulse is reported on actor0
            for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                imp = data[i].impulse
                self.impulse += sign * np.array([imp[0], imp[1], imp[2]])

    def poll(self):
        if self.steps:
            f = self.impulse / self.steps * 240.0  # mean impulse per step / dt (physics at 240 Hz)
            self.contact_csv.write(f"{self.timeline.get_current_time():.5f},{f[0]:.4f},{f[1]:.4f},{f[2]:.4f}\n")
            self.contact_csv.flush()
            self.impulse, self.steps = np.zeros(3), 0
        self.frame += 1
        if self.drops and self.frame == 300:
            self.drop_imp, self.drop_headers, self.drop_steps0 = [np.zeros(3) for _ in range(3)], [0, 0, 0], self.total_steps
        if self.drops and self.frame == 600:
            n = self.total_steps - self.drop_steps0
            for k in range(3):
                print(f"[sim] contact report calibration, drop hole {k}: {n} callbacks, {self.drop_headers[k]} headers, "
                      f"mean force {np.round(self.drop_imp[k] / n * 240.0, 4).tolist()} N (resting peg weight "
                      f"{0.019 * 9.81:.3f} N)", flush=True)
        if self.drops and self.frame in (60, 240, 600):  # app updates (~60 per sim second)
            hole_bottom = np.asarray(self.hole.get_world_poses()[0]).reshape(-1)[2]  # all holes at the same z
            for i, d in enumerate(self.drops):
                p = np.asarray(d.get_world_poses()[0]).reshape(-1)[:3]
                print(f"[sim] drop test peg {i} ({[0, 0.5, 1.5][i]} mm off): bottom {(p[2] - hole_bottom) * 1000:+.2f} mm above the hole bottom, "
                      f"xy {np.round(p[:2] * 1000, 2).tolist()} mm (frame {self.frame})", flush=True)
        if self.frame % 5:
            return
        objs = []
        for name, prim, offset, size in (("peg", self.peg, PEG_LEN / 2, [PEG_D, PEG_D, PEG_LEN]),
                                         ("hole", self.hole, HOLE_DEPTH, [HOLE_D, HOLE_D, HOLE_DEPTH])):
            pos, quat = prim.get_world_poses()
            p = np.asarray(pos).reshape(-1)[:3]
            q = np.asarray(quat).reshape(-1)[:4]
            w, x, y, z = q
            axis = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])  # body z in world
            objs.append({"name": name, "cube": (p + axis * offset).tolist(), "quat_wxyz": q.tolist(), "size": size,
                         "origin": p.tolist()})
        tmp = os.path.join(self.sync_dir, "object.json.tmp")
        json.dump({**{k: objs[0][k] for k in ("cube", "quat_wxyz", "size")}, "objects": objs}, open(tmp, "w"))
        os.replace(tmp, os.path.join(self.sync_dir, "object.json"))
