#!/usr/bin/env python3
# GPU sampling-MPC server (weekend track B): runs ci_mpc_planner.sampling_mpc.SamplingMPC in the Isaac venv (PyTorch +
# CUDA) and answers action requests over a local TCP socket, so the ROS side (system python 3.10, no PyTorch) can
# use it without installing PyTorch there. One JSON object per line:
#   request:  {"x": [5], "hole_xy": [2], "wrench": [5], "hole_samples": [[2], ...] (optional, belief samples),
#              "reset": bool (optional)}
#   reply:    {"u": [5], "source": str, "score": float, "ms": float}
#   ~/IsaacLab/.venv/bin/python scripts/gpu_mpc_server.py [--port 47011] [--w-force 0.5]
import argparse
import json
import socket
import sys
import time

import numpy as np

sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0] + "/ci_mpc_planner")
from ci_mpc_planner.peg_hole import PegHoleGeometry  # noqa: E402
from ci_mpc_planner.sampling_mpc import SamplingMPC  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("--port", type=int, default=47011)
p.add_argument("--w-force", type=float, default=0.5)
p.add_argument("--compliance", default="1500,1500,500,130,130", help="controller stiffness (rotational: K/2)")
p.add_argument("--arm-friction", default="2.0,2.0,3.5,0.05,0.05")
p.add_argument("--geo", default="", help="R,c,depth,a,pc,tip_below_tcp [m] (default: the Factory 8 mm peg)")
args = p.parse_args()
comp = 1.0 / np.array([float(v) for v in args.compliance.split(",")])
fa = np.array([float(v) for v in args.arm_friction.split(",")])
geo = PegHoleGeometry(**dict(zip(("R", "c", "depth", "a", "pc", "tip_below_tcp"), map(float, args.geo.split(",")))))\
    if args.geo else None
mpc = SamplingMPC(comp, fa, w_force=args.w_force, geo=geo)
mpc.act(np.array([0, 0, 0.0405, 0, 0]), np.zeros(2), np.zeros(5))  # compile / warm up before accepting clients
mpc.act(np.array([0, 0, 0.0405, 0, 0]), np.zeros(2), np.zeros(5), hole_samples=np.zeros((8, 2)))
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", args.port))
srv.listen(1)
print(f"gpu_mpc_server ready on 127.0.0.1:{args.port}", flush=True)
while True:
    conn, _ = srv.accept()
    f = conn.makefile("rw")
    for line in f:
        req = json.loads(line)
        if req.get("reset"):
            mpc.best = None
        t0 = time.perf_counter()
        hs = np.array(req["hole_samples"]) if req.get("hole_samples") else None
        u, info = mpc.act(np.array(req["x"]), np.array(req["hole_xy"]), np.array(req["wrench"]), hole_samples=hs)
        f.write(json.dumps({"u": u.tolist(), "source": info["source"], "score": info["score"],
                            "ms": (time.perf_counter() - t0) * 1000}) + "\n")
        f.flush()
    conn.close()
