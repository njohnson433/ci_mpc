"""CI-MPC B2: full 6D trace from a rosbag2 of /joint_states, the MPC commands and the external wrench (as
scripts/isaac/bag_trace.py, which keeps only the translational part). Per joint-state stamp (sim time): TCP position +
rotation (FK), the commanded pose (position + quaternion xyzw), desired twist (6), feedforward wrench (force + torque),
stiffness (6), damping (6), integral flag (zero-order hold), the observer wrench (force + torque). Run in the ROS environment.
    python3 scripts/b2_trace.py BAG_DIR out.npz
"""
import sys

import numpy as np
import pinocchio as pin
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, out = sys.argv[1], sys.argv[2]
full = pin.buildModelFromUrdf("/home/njohnson/ci_mpc_assets/fr3.urdf")
ARM = [f"fr3_joint{i}" for i in range(1, 8)]
model = pin.buildReducedModel(full, [j for j in range(1, full.njoints) if full.names[j] not in ARM], pin.neutral(full))
data = model.createData()
ee = model.getFrameId("fr3_hand_tcp")

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
js, cmd, wr = [], [], []
while r.has_next():
    topic, raw, t_rec = r.read_next()
    m = deserialize_message(raw, types[topic])
    t = t_rec * 1e-9
    if topic == "/joint_states":
        if all(n in m.name for n in ARM):
            q = np.array([m.position[m.name.index(n)] for n in ARM])
            pin.framesForwardKinematics(model, data, q)
            T = data.oMf[ee]
            qt = pin.Quaternion(T.rotation)
            js.append((t, m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, *T.translation, qt.x, qt.y, qt.z, qt.w))
    elif topic.endswith("mpc_command"):
        p, o, f, tq = m.pose.position, m.pose.orientation, m.wrench.force, m.wrench.torque
        tl, ta = m.twist.linear, m.twist.angular
        cmd.append((t, p.x, p.y, p.z, o.x, o.y, o.z, o.w, f.x, f.y, f.z, tq.x, tq.y, tq.z, *m.stiffness, *m.damping,
                    float(m.integral_enabled), tl.x, tl.y, tl.z, ta.x, ta.y, ta.z))
    else:
        f, tq = m.wrench.force, m.wrench.torque
        wr.append((t, f.x, f.y, f.z, tq.x, tq.y, tq.z))
js, cmd, wr = np.array(js), np.array(cmd), np.array(wr)
cmd[:, 0] = np.interp(cmd[:, 0], js[:, 0], js[:, 1])  # wall-clock record times -> sim time via the joint-state stamps
wr[:, 0] = np.interp(wr[:, 0], js[:, 0], js[:, 1])
js = js[:, 1:]
ic = np.clip(np.searchsorted(cmd[:, 0], js[:, 0], side="right") - 1, 0, None)
iw = np.clip(np.searchsorted(wr[:, 0], js[:, 0], side="right") - 1, 0, None)
np.savez(out, t0=js[0, 0], t=js[:, 0] - js[0, 0], x=js[:, 1:4], q_xyzw=js[:, 4:8],
         cmd=cmd[ic, 1:4], cmd_q_xyzw=cmd[ic, 4:8], ff=cmd[ic, 8:11], ff_torque=cmd[ic, 11:14],
         K=cmd[ic, 14:20], D=cmd[ic, 20:26], integral=cmd[ic, 26], twist_d=cmd[ic, 27:33], cmd_age=js[:, 0] - cmd[ic, 0],
         wrench=wr[iw, 1:4], wrench_torque=wr[iw, 4:7], n_cmd=len(cmd), cmd_t=cmd[:, 0] - js[0, 0])
print(f"joint states {len(js)} ({len(js) / (js[-1, 0] - js[0, 0]):.0f} Hz), commands {len(cmd)}, wrench {len(wr)}")
