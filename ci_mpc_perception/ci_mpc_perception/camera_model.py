"""Wrist camera mount geometry (same convention as scripts/isaac/wrist_camera.py, which builds the sim camera)."""
import math

import numpy as np


def optical_in_hand(cam):
    """4x4 pose of the optical frame (z forward, x right, y down) in the hand frame: at cam["position"], viewing
    along the tool axis tilted by cam["tilt_deg"] towards -x (the fingers), image x along the hand y axis."""
    a = -math.radians(cam["tilt_deg"])
    z = np.array([math.sin(a), 0.0, math.cos(a)])
    x = np.array([0.0, 1.0, 0.0])
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, y, z])
    T[:3, 3] = cam["position"]
    return T
