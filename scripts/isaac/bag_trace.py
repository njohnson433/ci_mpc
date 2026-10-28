"""Pipeline side of the fidelity trace (vectorized step 4c): a rosbag2 of /joint_states, the MPC commands and the
controller's external wrench -> npz with the TCP (FK of the measured joints), the commanded pose / feedforward /
stiffness / integral flag (zero-order hold at the joint-state stamps) and the wrench. Run in the ROS environment
(ros_franka). Compare with fidelity_probe_replay.py --trace.
    python3 scripts/isaac/bag_trace.py BAG_DIR out.npz
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
            js.append((t, m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, *data.oMf[ee].translation))
    elif topic.endswith("mpc_command"):
        p, f = m.pose.position, m.wrench.force
        cmd.append((t, p.x, p.y, p.z, f.x, f.y, f.z, *m.stiffness[:3], float(m.integral_enabled)))
    else:
        f = m.wrench.force
        wr.append((t, f.x, f.y, f.z))
js, cmd, wr = np.array(js), np.array(cmd), np.array(wr)
# record times are wall clock (the sim runs at RTF ~0.6): map all to sim time via the joint-state stamps
cmd[:, 0] = np.interp(cmd[:, 0], js[:, 0], js[:, 1])
wr[:, 0] = np.interp(wr[:, 0], js[:, 0], js[:, 1])
js = js[:, 1:]
# command and wrench held at the joint-state times
ic = np.clip(np.searchsorted(cmd[:, 0], js[:, 0], side="right") - 1, 0, None)
iw = np.clip(np.searchsorted(wr[:, 0], js[:, 0], side="right") - 1, 0, None)
cmd_age = js[:, 0] - cmd[ic, 0]
np.savez(out, t0=js[0, 0], t=js[:, 0] - js[0, 0], x=js[:, 1:4], cmd=cmd[ic, 1:4], ff=cmd[ic, 4:7], K=cmd[ic, 7:10],
         integral=cmd[ic, 10], cmd_age=cmd_age, wrench=wr[iw, 1:4], n_cmd=len(cmd), cmd_t=cmd[:, 0] - js[0, 0])
print(f"joint states {len(js)} ({len(js) / (js[-1, 0] - js[0, 0]):.0f} Hz), commands {len(cmd)}, wrench {len(wr)}")
