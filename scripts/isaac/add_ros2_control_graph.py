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

ROOT = "/fr3/Geometry/base/fr3_link0"
CONFIG = ("/home/njohnson/franka_ros2_ws/install/cartesian_impedance_control/share/"
          "cartesian_impedance_control/config/isaac_sim_controllers.yaml")
GRAPH = "/Graph/ROS2Control"
OLD_GRAPH = "/Graph/ROS_JointStates"
PHYSICS_HZ = 240  # >= controller_manager update_rate


def add_ros2_control_graph(root=ROOT, config=CONFIG, graph_path=GRAPH, physics_hz=PHYSICS_HZ):
    stage = omni.usd.get_context().get_stage()

    old = stage.GetPrimAtPath(OLD_GRAPH)
    if old.IsValid() and old.IsActive():
        old.SetActive(False)
        print("deactivated", OLD_GRAPH)

    if stage.GetPrimAtPath(graph_path).IsValid():
        stage.RemovePrim(graph_path)
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ControlManager", "isaacsim.ros2.control.ROS2ControlManager"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ControlManager.inputs:targetPrim", [UsdrtSdf.Path(root)]),
                ("ControlManager.inputs:controllerConfig", config),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ControlManager.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )

    for p in stage.Traverse():
        if p.GetTypeName() == "PhysicsScene":
            attr = p.GetAttribute("physxScene:timeStepsPerSecond")
            if attr and attr.Get() != physics_hz:
                attr.Set(physics_hz)
            print(p.GetPath(), "timeStepsPerSecond =", attr.Get() if attr else "?")
    print("ros2_control graph at", graph_path, "->", root, "|", config)


if __name__ in ("__main__", "builtins"):
    add_ros2_control_graph()
