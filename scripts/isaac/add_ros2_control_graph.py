# Isaac Sim Script Editor (simulation stopped): add the in-process ros2_control graph to the scene.
#   On Playback Tick -> ROS2 Control Manager (targetPrim = articulation root, our controllers YAML)
#   On Playback Tick -> ROS2 Publish Clock (sim time; the Controller Manager uses use_sim_time)
# The Controller Manager starts on the first tick after Play and is torn down on Stop.
# Also deactivates the old topic-based graph (/Graph/ROS_JointStates): its ArticulationController
# would write to the same joints and its /joint_states publisher collides with joint_state_broadcaster.
# Idempotent. Save the scene afterwards (Ctrl+S) to keep it.
# Isaac must be started with ~/isaac_ros.sh so it finds our controller plugin.
import omni.graph.core as og
import omni.usd
from usdrt import Sdf as UsdrtSdf

ROOT = "/fr3/Geometry/base/fr3_link0"  # articulation root
# targetPrim for the Controller Manager: the URDF synthesizer collects joints under this prim and
# finds the articulation root below it. Our joints live in /fr3/Physics, not under fr3_link0
# (with fr3_link0 as target the URDF had no arm joints and joint_state_broadcaster found no interfaces).
TARGET = "/fr3"
CONFIG = ("/home/njohnson/franka_ros2_ws/install/cartesian_impedance_control/share/"
          "cartesian_impedance_control/config/isaac_sim_controllers.yaml")
GRAPH = "/Graph/ROS2Control"
OLD_GRAPH = "/Graph/ROS_JointStates"
PHYSICS_HZ = 240  # >= controller_manager update_rate


def add_ros2_control_graph(root=TARGET, config=CONFIG, graph_path=GRAPH, physics_hz=PHYSICS_HZ,
                           with_manager=True):
    """with_manager=False: clock publisher only; start the Controller Manager with
    setup_controller_manager() instead (the graph node gives up after one failed attempt)."""
    stage = omni.usd.get_context().get_stage()

    old = stage.GetPrimAtPath(OLD_GRAPH)
    if old.IsValid() and old.IsActive():
        old.SetActive(False)
        print("deactivated", OLD_GRAPH)

    if stage.GetPrimAtPath(graph_path).IsValid():
        stage.RemovePrim(graph_path)
    nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
    ]
    values, connect = [], [
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
    ]
    if with_manager:
        nodes.append(("ControlManager", "isaacsim.ros2.control.ROS2ControlManager"))
        values += [
            ("ControlManager.inputs:targetPrim", [UsdrtSdf.Path(root)]),
            ("ControlManager.inputs:controllerConfig", config),
        ]
        connect.append(("OnPlaybackTick.outputs:tick", "ControlManager.inputs:execIn"))
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: nodes,
            og.Controller.Keys.SET_VALUES: values,
            og.Controller.Keys.CONNECT: connect,
        },
    )

    for p in stage.Traverse():
        if p.GetTypeName() == "PhysicsScene":
            attr = p.GetAttribute("physxScene:timeStepsPerSecond")
            if attr and attr.Get() != physics_hz:
                attr.Set(physics_hz)
            print(p.GetPath(), "timeStepsPerSecond =", attr.Get() if attr else "?")
    print("ros2_control graph at", graph_path, "->", root if with_manager else "(clock only)", "|", config)


def setup_controller_manager(step, root=TARGET, config=CONFIG, attempts=5, frames_between=10):
    """Start the in-process Controller Manager while the simulation is playing, with retries.
    The first createArticulationView in the plugin fails ("did not match any articulations")
    and a later attempt succeeds; the ROS2ControlManager graph node latches that first failure
    until the next Stop->Play. step() advances the app by one frame."""
    from isaacsim.ros2.control import Ros2ControlManager

    for i in range(1, attempts + 1):
        try:
            Ros2ControlManager.setup(root, config)
            print(f"Controller Manager up for {root} (attempt {i})", flush=True)
            return True
        except Exception as e:
            print(f"Controller Manager setup attempt {i} failed: {e}", flush=True)
            try:
                Ros2ControlManager.teardown(root)
            except Exception:
                pass
            for _ in range(frames_between):
                step()
    return False


if __name__ in ("__main__", "builtins"):
    add_ros2_control_graph()
