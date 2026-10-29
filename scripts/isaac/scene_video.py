# Off-screen scene video of a headless run (loaded with runpy by headless_impedance_sim.py --record-video PATH):
# a fixed camera on the work area, rendered at 1280x720 through a replicator RGB annotator and piped into ffmpeg
# (H.264 mp4). Frames are taken at 30 fps of SIM time, so the video plays at real speed whatever the RTF.
#   SceneVideo(stage, path, eye, target)  before Play: lights + camera prim
#   .attach()                             after Play: render product + annotator
#   .poll(sim_t)                          every app.update(): grab a frame when sim time passes the next 1/30 s
#   .close()                              finish the mp4
import math
import subprocess

import numpy as np
from pxr import Gf, UsdGeom, UsdLux

CAM = "/World/SceneVideoCamera"


def _look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """Camera-to-world 4x4 (USD camera looks along its -z, y up) from an eye point and a target."""
    eye, target, up = (np.asarray(v, float) for v in (eye, target, up))
    z = eye - target
    z /= np.linalg.norm(z)
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, eye
    return T


class SceneVideo:
    def __init__(self, stage, path, eye=(1.05, -0.55, 0.55), target=(0.48, -0.02, 0.24), res=(1280, 720), fps=30.0):
        self.path, self.res, self.dt = path, res, 1.0 / fps
        if not stage.GetPrimAtPath("/Lights/SceneDome").IsValid():
            UsdLux.DomeLight.Define(stage, "/Lights/SceneDome").CreateIntensityAttr(600.0)
            sun = UsdLux.DistantLight.Define(stage, "/Lights/SceneSun")
            sun.CreateIntensityAttr(2000.0)
            UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-40.0, 25.0, 0.0))
        cam = UsdGeom.Camera.Define(stage, CAM)
        cam.CreateFocalLengthAttr(28.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 20.0))
        T = _look_at(eye, target)
        UsdGeom.Xformable(cam.GetPrim()).AddTransformOp().Set(Gf.Matrix4d(*T.T.flatten().tolist()))
        self.next_t, self.frames, self.proc, self.annot = None, 0, None, None

    def attach(self):
        import omni.replicator.core as rep

        self.rp = rep.create.render_product(CAM, self.res)
        self.annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.annot.attach([self.rp])
        w, h = self.res
        self.proc = subprocess.Popen(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
             "-r", str(round(1.0 / self.dt)), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
             self.path], stdin=subprocess.PIPE)

    def poll(self, sim_t):
        if self.proc is None:
            return
        if self.next_t is None:
            self.next_t = sim_t
        if sim_t + 1e-9 < self.next_t:
            return
        data = self.annot.get_data()
        if data is None or not hasattr(data, "shape") or data.size == 0:
            return  # renderer warming up
        img = np.asarray(data)[..., :3].astype(np.uint8)
        # one frame per elapsed 1/30 s of sim time (duplicates when the app updates less often than 30 Hz sim)
        n = max(1, int(math.floor((sim_t - self.next_t) / self.dt)) + 1)
        for _ in range(n):
            self.proc.stdin.write(img.tobytes())
        self.frames += n
        self.next_t += n * self.dt

    def close(self):
        if self.proc is not None:
            self.proc.stdin.close()
            self.proc.wait()
            print(f"[sim] video: {self.frames} frames ({self.frames * self.dt:.1f} s) -> {self.path}", flush=True)
