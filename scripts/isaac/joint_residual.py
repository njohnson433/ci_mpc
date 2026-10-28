"""Item 3 (pipeline observer over-read in contact, sim side): per joint, which effort-state torque reads high in a
probe hold? From a step4d episode (bag of /joint_states with effort + wrist_ft.csv + trace.npz): per hold (as
hold_forces.py), tared change of the measured effort vs J_hand^T (tared true hand wrench, wrist F/T, Franka sign).
    python3 scripts/isaac/joint_residual.py EP_DIR [EP_DIR ...] [--out x.npz]   (ROS env, as bag_trace.py)
"""
import argparse
import os

import numpy as np
import pinocchio as pin
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

p = argparse.ArgumentParser()
p.add_argument("dirs", nargs="+")
p.add_argument("--out", default="")
args = p.parse_args()
full = pin.buildModelFromUrdf("/home/njohnson/ci_mpc_assets/fr3.urdf")
ARM = [f"fr3_joint{i}" for i in range(1, 8)]
model = pin.buildReducedModel(full, [j for j in range(1, full.njoints) if full.names[j] not in ARM], pin.neutral(full))
data = model.createData()
hand = model.getFrameId("fr3_hand")


def quat_R(w, x, y, z):
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], 1)


rows = []
for d in args.dirs:
    bag = os.path.join(d, "bag")
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    T, Q, E = [], [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic != "/joint_states":
            continue
        m = deserialize_message(raw, types[topic])
        if all(n in m.name for n in ARM):
            T.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
            Q.append([m.position[m.name.index(n)] for n in ARM])
            E.append([m.effort[m.name.index(n)] for n in ARM])
    T, Q, E = np.array(T), np.array(Q), np.array(E)
    tr = np.load(os.path.join(d, "trace.npz"))
    t = tr["t"] + tr["t0"]
    ff, K = tr["ff"][:, 2], tr["K"][:, 2]
    wf = np.loadtxt(os.path.join(d, "wrist_ft.csv"), delimiter=",", skiprows=1)
    R = quat_R(*(wf[:, i] for i in (7, 8, 9, 10)))
    Ww = np.concatenate([np.einsum("nij,nj->ni", R, wf[:, 1:4]), np.einsum("nij,nj->ni", R, wf[:, 4:7])], 1)
    W = np.stack([np.interp(T, wf[:, 0], Ww[:, i]) for i in range(6)], 1)
    G = np.array([pin.computeGeneralizedGravity(model, data, q) for q in Q])
    m = np.interp(T, t, ((np.abs(ff + 3.0) < 1e-6) & (K == 500.0)).astype(float)) > 0.5
    e = np.flatnonzero(np.diff(np.r_[0, m.astype(int), 0]))
    n_seg = 0
    for a, b in zip(e[::2], e[1::2]):
        if not 0.15 < T[b - 1] - T[a] < 1.5:
            continue
        pre, hold = slice(max(a - 60, 0), a), slice(max(b - 72, a), b)
        q = Q[hold].mean(0)
        pin.computeJointJacobians(model, data, q)
        pin.framesForwardKinematics(model, data, q)
        J = pin.getFrameJacobian(model, data, hand, pin.LOCAL_WORLD_ALIGNED)
        dW = W[hold].mean(0) - W[pre].mean(0)
        dE = (E[hold].mean(0) - G[hold].mean(0)) - (E[pre].mean(0) - G[pre].mean(0))
        rows.append(np.r_[dE, J.T @ dW, dW])
        n_seg += 1
    print(f"{d}: {n_seg} holds, effort samples {len(T)}")
R = np.array(rows)
dE, tJ, dW = R[:, :7], R[:, 7:14], R[:, 14:20]
print(f"holds {len(R)}; per joint median [Nm]: effort change | J^T true | effort - J^T true (sd) | corr")
for j in range(7):
    res = dE[:, j] - tJ[:, j]
    print(f"  j{j + 1}: {np.median(dE[:, j]):+.3f} | {np.median(tJ[:, j]):+.3f} | {np.median(res):+.3f} ({res.std():.3f})"
          f" | {np.corrcoef(dE[:, j], tJ[:, j])[0, 1]:+.2f}")
s, *_ = np.linalg.lstsq(tJ, dE, rcond=None)
print("sign check: median dW true Fz %.2f N" % np.median(dW[:, 2]))
if args.out:
    np.savez(args.out, dE=dE, tJ=tJ, dW=dW)
