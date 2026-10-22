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
parser.add_argument("--timeout", type=float, default=600.0, help="wall-clock seconds")
parser.add_argument("--config", default="", help="controller YAML (default: the installed isaac_sim_controllers.yaml)")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402

ext = omni.kit.app.get_app().get_extension_manager()
for name in ("isaacsim.ros2.bridge", "isaacsim.ros2.nodes", "isaacsim.ros2.control"):
    ext.set_extension_enabled_immediate(name, True)

here = os.path.dirname(os.path.abspath(__file__))
graph_lib = runpy.run_path(os.path.join(here, "add_ros2_control_graph.py"), run_name="lib")
home_lib = runpy.run_path(os.path.join(here, "set_home_pose.py"), run_name="lib")

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
for _ in range(5):
    app.update()
open(os.path.join(args.sync_dir, "home_done"), "w").close()

t0_wall, t0_sim = time.time(), timeline.get_current_time()
last = t0_wall
done = os.path.join(args.sync_dir, "done")
while not os.path.exists(done) and time.time() - t0_wall < args.timeout:
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
