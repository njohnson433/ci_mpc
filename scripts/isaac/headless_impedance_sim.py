# Headless Isaac Sim side of the impedance-controller test (run via isaac_ros.sh --python).
# Opens the FR3 scene, adds the ros2_control graph (not saved to the scene file), plays, puts the
# arm at home, then keeps stepping until the ROS-side checker writes <sync_dir>/done.
#   ~/isaac_ros.sh --python headless_impedance_sim.py --sync-dir /tmp/x [--timeout 600]
import argparse
import os
import runpy
import time

parser = argparse.ArgumentParser()
parser.add_argument("--scene", default="/home/njohnson/ci_mpc_assets/fr3_scene.usd")
parser.add_argument("--sync-dir", required=True)
parser.add_argument("--timeout", type=float, default=7200.0, help="wall-clock seconds (safety net; the ROS side ends the run)")
parser.add_argument("--config", default="", help="controller YAML (default: the installed isaac_sim_controllers.yaml)")
parser.add_argument("--gui", action="store_true", help="open the Isaac Sim window (needs DISPLAY)")
parser.add_argument("--pick-place", action="store_true", help="add table, cube and a position-controlled gripper")
parser.add_argument("--object-size", default="0.04,0.04,0.04", help="pick-place object box extents x,y,z [m]")
parser.add_argument("--object-yaw", type=float, default=0.0, help="pick-place object yaw [deg]")
parser.add_argument("--boxes", default="", help="pick-place: more boxes 'x,y,yaw;x,y,yaw' (same size)")
parser.add_argument("--wrist-ft", action="store_true", help="log the simulated wrist F/T sensor (wrist_ft.csv)")
parser.add_argument("--camera", action="store_true", help="add the simulated Gemini 305 wrist camera")
parser.add_argument("--peg", action="store_true", help="add the Factory 8 mm peg-in-hole (SDF collision)")
parser.add_argument("--peg-hole-xy", default="0.5,0.1", help="peg: hole x,y [m]")
parser.add_argument("--peg-drop-test", action="store_true", help="peg: also drop free pegs into the hole")
parser.add_argument("--peg-friction", type=float, default=None, help="peg: contact friction (default 0.75)")
parser.add_argument("--gear", action="store_true", help="add the Factory gear mesh (held-out H2, SDF)")
parser.add_argument("--gear-base-xy", default="0.5,0.05", help="gear: base x,y [m]")
parser.add_argument("--gear-drop-test", action="store_true", help="gear: also drop free gears onto extra shafts")
parser.add_argument("--peg-hole-tilt", type=float, default=0.0, help="peg: hole block tilt about x [deg]")
parser.add_argument("--push", action="store_true", help="add table, slider box and a pusher rod on the hand")
parser.add_argument("--push-xy", default="0.55,-0.15", help="push slider start x,y [m]")
parser.add_argument("--push-yaw", type=float, default=90.0, help="push slider yaw [deg]")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

# camera: real-time ray tracing without DLSS (the default path tracer + DLSS at 640x400 ran out of RTX
# resource descriptors and hung the renderer)
app = SimulationApp({"headless": not args.gui,
                     **({"renderer": "RaytracedLighting", "anti_aliasing": 0} if args.camera else {})})

import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402

ext = omni.kit.app.get_app().get_extension_manager()
for name in ("isaacsim.ros2.bridge", "isaacsim.ros2.nodes", "isaacsim.ros2.control"):
    ext.set_extension_enabled_immediate(name, True)

here = os.path.dirname(os.path.abspath(__file__))
graph_lib = runpy.run_path(os.path.join(here, "add_ros2_control_graph.py"), run_name="lib")
home_lib = runpy.run_path(os.path.join(here, "set_home_pose.py"), run_name="lib")
pick_lib = runpy.run_path(os.path.join(here, "pick_place_scene.py"), run_name="lib") if args.pick_place else None
push_lib = runpy.run_path(os.path.join(here, "push_scene.py"), run_name="lib") if args.push else None
peg_lib = runpy.run_path(os.path.join(here, "peg_scene.py"), run_name="lib") if args.peg else None
gear_lib = runpy.run_path(os.path.join(here, "gear_scene.py"), run_name="lib") if args.gear else None
cam_lib = runpy.run_path(os.path.join(here, "wrist_camera.py"), run_name="lib") if args.camera else None

omni.usd.get_context().open_stage(args.scene)
for _ in range(20):
    app.update()
# Before Play, turn off the old topic-based graph (it writes to the same joints) and any
# ros2_control graph saved in the scene file; add_ros2_control_graph() recreates the latter below.
stage = omni.usd.get_context().get_stage()
for path in (graph_lib["OLD_GRAPH"], graph_lib["GRAPH"]):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        prim.SetActive(False)
if pick_lib:
    boxes = [[float(v) for v in b.split(",")] for b in args.boxes.split(";") if b]
    pick_lib["add_objects"](stage, [float(x) for x in args.object_size.split(",")], args.object_yaw, boxes)
if cam_lib:
    cam_lib["add_camera"](stage)
if gear_lib:
    gear_lib["add_objects"](stage, [float(v) for v in args.gear_base_xy.split(",")], drop_test=args.gear_drop_test)
if peg_lib:
    peg_lib["add_objects"](stage, [float(v) for v in args.peg_hole_xy.split(",")], drop_test=args.peg_drop_test,
                           friction=args.peg_friction, hole_tilt_deg=args.peg_hole_tilt)
if push_lib:
    push_lib["add_objects"](stage, yaw_deg=args.push_yaw, xy=[float(x) for x in args.push_xy.split(",")])

# Play first and let SimulationManager initialize physics. If the ros2_control graph already
# exists at Play, its first tick runs before physics is set up and createArticulationView fails.
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(10):
    app.update()
home_lib["set_home_pose"]()
for _ in range(5):
    app.update()
graph_lib["add_ros2_control_graph"](with_manager=False)  # /clock only
if not graph_lib["setup_controller_manager"](app.update, config=args.config or graph_lib["CONFIG"]):
    print("[sim] Controller Manager setup failed, giving up", flush=True)
    os._exit(1)
if cam_lib:
    cam_lib["add_camera_graph"]()
for _ in range(5):
    app.update()
# External wrench on the hand, requested by the ROS side through <sync_dir>/wrench.json:
# {"force": [fx, fy, fz], "torque": [tx, ty, tz]} (world frame, force acting at the TCP), applied on
# every physics step while the file exists; <sync_dir>/wrench_ack echoes the wrench once it is active.
import json  # noqa: E402

import omni.physx  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

HAND = "/fr3/Geometry/base/fr3_link0/fr3_link1/fr3_link2/fr3_link3/fr3_link4/fr3_link5/fr3_link6/fr3_link7/fr3_link8/fr3_hand"
hand = RigidPrim(HAND)
tcp_xform = UsdGeom.Xformable(stage.GetPrimAtPath(HAND + "/fr3_hand_tcp"))
wrench = {"active": None, "mtime": None}


def apply_wrench(_dt):
    w = wrench["active"]
    if w is None:
        return
    p = tcp_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    hand.apply_forces_and_torques_at_pos(forces=[w["force"]], torques=[w["torque"]], positions=[list(p)],
                                         local_frame=False)


wrench_sub = omni.physx.get_physx_interface().subscribe_physics_on_step_events(apply_wrench, pre_step=True, order=0)
wrench_path = os.path.join(args.sync_dir, "wrench.json")


def poll_wrench():
    if not os.path.exists(wrench_path):
        if wrench["active"] is not None:
            print("[sim] wrench released", flush=True)
        wrench["active"], wrench["mtime"] = None, None
        return
    mtime = os.path.getmtime(wrench_path)
    if mtime != wrench["mtime"]:
        try:
            w = json.load(open(wrench_path))
        except (OSError, ValueError):
            return  # being written, retry next frame
        wrench["active"] = {"force": list(w.get("force", [0, 0, 0])), "torque": list(w.get("torque", [0, 0, 0]))}
        wrench["mtime"] = mtime
        json.dump(wrench["active"], open(os.path.join(args.sync_dir, "wrench_ack"), "w"))
        print(f"[sim] wrench applied: {wrench['active']}", flush=True)


gripper = pick_lib["Gripper"](args.sync_dir, home_lib["ROOT"]) if pick_lib else None
if push_lib:
    gripper = push_lib["Poller"](args.sync_dir)
if peg_lib:
    gripper = peg_lib["Poller"](args.sync_dir)
if gear_lib:
    gripper = gear_lib["Poller"](args.sync_dir)
wrist_ft = runpy.run_path(os.path.join(here, "wrist_ft.py"), run_name="lib")["WristFT"](args.sync_dir, home_lib["ROOT"]) \
    if args.wrist_ft else None
open(os.path.join(args.sync_dir, "home_done"), "w").close()

t0_wall, t0_sim = time.time(), timeline.get_current_time()
last = t0_wall
done = os.path.join(args.sync_dir, "done")
while not os.path.exists(done) and time.time() - t0_wall < args.timeout:
    poll_wrench()
    if gripper:
        gripper.poll()
    if wrist_ft:
        wrist_ft.poll()
    app.update()
    now = time.time()
    if now - last > 5.0:
        sim = timeline.get_current_time() - t0_sim
        print(f"[sim] wall {now - t0_wall:6.1f} s  sim {sim:6.1f} s  RTF {sim / (now - t0_wall):.2f}", flush=True)
        last = now

sim = timeline.get_current_time() - t0_sim
print(f"[sim] finished: sim {sim:.1f} s in {time.time() - t0_wall:.1f} s wall", flush=True)
from isaacsim.ros2.control import Ros2ControlManager  # noqa: E402

Ros2ControlManager.teardown_all()
timeline.stop()
for _ in range(5):
    app.update()
# app.close() can hang on RTX sync scopes in headless mode; nothing left to save
os._exit(0)
