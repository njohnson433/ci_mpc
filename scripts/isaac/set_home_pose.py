# Isaac Sim Script Editor (simulation playing): put the FR3 arm at the libfranka home pose, at rest.
# Run this BEFORE activating cartesian_impedance_controller: the scene starts at q = 0 (arm
# straight up), which is close to singular, and the controller holds whatever pose it starts in.
import numpy as np
from isaacsim.core.experimental.prims import Articulation

ROOT = "/fr3/Geometry/base/fr3_link0"
Q_HOME = [0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4]


def set_home_pose(root=ROOT, q_home=Q_HOME):
    art = Articulation(root)
    names = list(art.dof_names)
    arm = [names.index(f"fr3_joint{i}") for i in range(1, 8)]
    art.set_dof_positions(np.array([q_home]), dof_indices=arm)
    art.set_dof_velocities(np.zeros((1, len(names))))
    print("arm set to home:", np.round(q_home, 4).tolist())


if __name__ in ("__main__", "builtins"):
    set_home_pose()
