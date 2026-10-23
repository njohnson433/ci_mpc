# Simulated wrist force/torque sensor (loaded with runpy by headless_impedance_sim.py --wrist-ft): the 6D force
# PhysX transmits through the fixed joint between fr3_link8 and fr3_hand (articulation link incoming joint force),
# i.e. everything acting on the hand, fingers and what they hold, friction included. Ground truth for the
# controller's external wrench estimate (a contact report only gives normal impulses).
#   WristFT(sync_dir, root)   after Play; poll() appends to <sync>/wrist_ft.csv:
#     t (sim time), raw force (3) and torque (3) as returned by the tensor API, hand orientation quaternion w,x,y,z
#     (world). Frame and sign are resolved offline against a known external wrench (scripts/.../wrench check).
import os

import numpy as np

HAND = ("/fr3/Geometry/base/fr3_link0/fr3_link1/fr3_link2/fr3_link3/fr3_link4/fr3_link5/fr3_link6/fr3_link7/"
        "fr3_link8/fr3_hand")


def _np(x):
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


class WristFT:
    def __init__(self, sync_dir, root):
        import omni.timeline
        from isaacsim.core.experimental.prims import Articulation, RigidPrim

        self.art = Articulation(root)
        names = list(self.art.link_names)
        self.link = names.index("fr3_hand")
        self.hand = RigidPrim(HAND)
        self.timeline = omni.timeline.get_timeline_interface()
        self.csv = open(os.path.join(sync_dir, "wrist_ft.csv"), "w")
        self.csv.write("t,fx,fy,fz,tx,ty,tz,qw,qx,qy,qz\n")
        print(f"[sim] wrist F/T: link {self.link} of {names}", flush=True)

    def poll(self):
        f, t = self.art.get_link_incoming_joint_force(link_indices=[self.link])
        f, t = _np(f).reshape(-1)[:3], _np(t).reshape(-1)[:3]
        q = _np(self.hand.get_world_poses()[1]).reshape(-1)[:4]
        self.csv.write(f"{self.timeline.get_current_time():.5f}," + ",".join(f"{v:.5f}" for v in (*f, *t, *q)) + "\n")
        self.csv.flush()
