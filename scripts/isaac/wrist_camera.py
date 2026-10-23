# Simulated Orbbec Gemini 305 on the FR3 hand (loaded with runpy by headless_impedance_sim.py --camera).
# Config: ci_mpc_perception/config/gemini305_sim.yaml (mount pose, resolution, FOV; shared with the ROS nodes).
#   add_camera(stage, cfg)      before Play: camera body (box collider, 68 g) + camera prim under fr3_hand.
#                               Color and depth come from one camera pose (= the driver's depth-to-color alignment).
#   add_camera_graph(cfg)       after Play: render product + ROS 2 publishers (sim time):
#                                 /camera/color/image_raw (rgb8), /camera/color/camera_info,
#                                 /sim/camera/depth (32FC1 [m], noise-free; ci_mpc_perception depth_noise turns it
#                                 into the driver's /camera/depth/image_raw, 16UC1 [mm], with stereo noise)
# Topic names follow OrbbecSDK_ROS2 (camera_name "camera").
import math

import numpy as np
import omni.graph.core as og
import yaml
from usdrt import Sdf as UsdrtSdf
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

HAND = ("/fr3/Geometry/base/fr3_link0/fr3_link1/fr3_link2/fr3_link3/fr3_link4/fr3_link5/fr3_link6/fr3_link7/"
        "fr3_link8/fr3_hand")
CONFIG = "/home/njohnson/franka_ros2_ws/src/ci_mpc/ci_mpc_perception/config/gemini305_sim.yaml"
GRAPH = "/Graph/WristCamera"


def load(path=CONFIG):
    return yaml.safe_load(open(path))


def optical_in_hand(cam):
    """4x4 pose of the optical frame (z forward, x right, y down) in the hand frame."""
    a = -math.radians(cam["tilt_deg"])  # rotate the view direction from the tool axis towards -x (the fingers)
    z = np.array([math.sin(a), 0.0, math.cos(a)])
    x = np.array([0.0, 1.0, 0.0])
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, y, z])
    T[:3, 3] = cam["position"]
    return T


def _xform(prim, T):
    # USD uses row vectors: the matrix is the transpose of the column-vector convention
    UsdGeom.Xformable(prim).AddTransformOp().Set(Gf.Matrix4d(*T.T.flatten().tolist()))


def add_lights(stage):
    """The FR3 scene has no lights (fine for physics, black for a camera): soft dome + a tilted distant light."""
    if not stage.GetPrimAtPath("/Lights/Dome").IsValid():
        UsdLux.DomeLight.Define(stage, "/Lights/Dome").CreateIntensityAttr(400.0)
        sun = UsdLux.DistantLight.Define(stage, "/Lights/Sun")
        sun.CreateIntensityAttr(1500.0)
        sun.CreateAngleAttr(2.0)
        UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 20.0, 0.0))


def add_camera(stage, cfg=None):
    add_lights(stage)
    cam = (cfg or load())["camera"]
    T_opt = optical_in_hand(cam)
    root = UsdGeom.Xform.Define(stage, HAND + "/gemini305")
    _xform(root.GetPrim(), T_opt)
    # USD camera looks along -Z with +Y up: optical frame with y and z flipped
    usd_cam = UsdGeom.Camera.Define(stage, HAND + "/gemini305/camera")
    _xform(usd_cam.GetPrim(), np.diag([1.0, -1.0, -1.0, 1.0]))
    ha = 20.955
    usd_cam.CreateHorizontalApertureAttr(ha)
    usd_cam.CreateVerticalApertureAttr(ha * cam["height"] / cam["width"])
    usd_cam.CreateFocalLengthAttr(ha / (2 * math.tan(math.radians(cam["hfov_deg"]) / 2)))
    usd_cam.CreateClippingRangeAttr(Gf.Vec2f(cam["near"] * 0.5, cam["far"]))
    # body: behind the optical center along -z
    body = UsdGeom.Cube.Define(stage, HAND + "/gemini305/body")
    sx, sy, sz = cam["body_size"]
    T_body = np.diag([sx, sy, sz, 1.0])
    T_body[2, 3] = -sz / 2 - 0.001
    _xform(body.GetPrim(), T_body)
    body.CreateSizeAttr(1.0)
    body.CreateDisplayColorAttr([Gf.Vec3f(0.15, 0.15, 0.15)])
    UsdPhysics.CollisionAPI.Apply(body.GetPrim())
    UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(cam["body_mass"])
    f_px = cam["width"] / (2 * math.tan(math.radians(cam["hfov_deg"]) / 2))
    print(f"[sim] wrist camera (Gemini 305 sim): {cam['width']}x{cam['height']}, HFOV {cam['hfov_deg']} deg "
          f"(f {f_px:.1f} px), at {cam['position']} in the hand frame, tilt {cam['tilt_deg']} deg", flush=True)
    return HAND + "/gemini305/camera"


def add_camera_graph(cfg=None, render_hz=60):
    """render_hz: app updates per sim second (each renders once and steps physics 4 x at 240 Hz; measured)."""
    cam = (cfg or load())["camera"]
    skip = max(0, int(round(render_hz / cam["rate"])) - 1)
    frame = cam["frame_id"]
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("Color", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("Depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("Info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", [UsdrtSdf.Path(HAND + "/gemini305/camera")]),
                ("RenderProduct.inputs:width", cam["width"]),
                ("RenderProduct.inputs:height", cam["height"]),
                ("Color.inputs:type", "rgb"),
                ("Color.inputs:topicName", "camera/color/image_raw"),
                ("Color.inputs:frameId", frame),
                ("Color.inputs:frameSkipCount", skip),
                ("Depth.inputs:type", "depth"),
                ("Depth.inputs:topicName", "sim/camera/depth"),
                ("Depth.inputs:frameId", frame),
                ("Depth.inputs:frameSkipCount", skip),
                ("Info.inputs:topicName", "camera/color/camera_info"),
                ("Info.inputs:frameId", frame),
                ("Info.inputs:frameSkipCount", skip),
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "Color.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "Depth.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "Info.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "Color.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "Depth.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "Info.inputs:renderProductPath"),
            ],
        },
    )
    print(f"[sim] wrist camera graph {GRAPH}: color/depth/camera_info at ~{render_hz / (skip + 1):.0f} Hz", flush=True)
