"""Isaac Lab Factory PegInsert with our FR3 instead of the stock Panda (ROADMAP 9, step 1, scene parity).

The stock task (isaaclab_tasks/contrib/factory) spawns franka_mimic.usd (Panda) with zero joint friction / armature.
Here: the FR3 from ~/ci_mpc_assets/fr3.urdf (same model as Pinocchio / the ros2_control sim) through Isaac Lab's URDF
importer, armature and Coulomb friction per joint from ci_mpc_sysid/config/fr3_joint_params_sim.yaml (the values
prepare_scene.py puts into our Isaac scene), FR3 effort / velocity limits.
The Factory env looks bodies up by Panda names, so a generated copy of the URDF (fr3_factory.urdf next to it) names
the fingers panda_left/rightfinger and adds panda_fingertip_centered fixed to fr3_hand at the fr3_hand_tcp offset
(TODO: compare with franka_mimic's fingertip frame; it sets where the held peg sits in the fingers).
Import registers the gym id TASK.
"""
import os
import re

import gymnasium as gym
import yaml

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.utils import configclass

from isaaclab_tasks.contrib.factory.factory_env_cfg import FactoryTaskPegInsertCfg

TASK = "CiMpc-Factory-PegInsert-FR3-Direct"
URDF = os.path.expanduser("~/ci_mpc_assets/fr3.urdf")
URDF_FACTORY = os.path.expanduser("~/ci_mpc_assets/fr3_factory.urdf")
HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, "..", "..", "ci_mpc_sysid", "config", "fr3_joint_params_sim.yaml")
Q0 = (0.00871, -0.10368, -0.00794, -1.49139, -0.00083, 1.38774, 0.0)  # the stock task's start pose


def factory_urdf():
    """fr3.urdf with the Panda body names FactoryEnv looks up."""
    s = open(URDF).read()
    s = re.sub(r'"fr3_(left|right)finger"', r'"panda_\1finger"', s)
    # massless "base" + fixed fr3_base_joint: the importer then leaves the articulation floating (Jacobian with 6 base
    # columns, NaN states at the first reset); with fr3_link0 as the root, fix_base anchors it
    s = re.sub(r'\s*<link name="base">\s*</link>\s*<joint name="fr3_base_joint".*?</joint>', "", s, flags=re.S)
    tcp = re.search(r'<joint name="fr3_hand_tcp_joint".*?<origin ([^/]*)/>', s, re.S).group(1)
    s = s.replace("</robot>", f'''  <link name="panda_fingertip_centered">
    <inertial><mass value="0.01"/><origin xyz="0 0 0"/><inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <joint name="panda_fingertip_centered_joint" type="fixed">
    <parent link="fr3_hand"/>
    <child link="panda_fingertip_centered"/>
    <origin {tcp}/>
  </joint>
</robot>''')
    # a link without <inertial> is dropped by the importer (no body): give the frame a negligible mass
    if not os.path.exists(URDF_FACTORY) or open(URDF_FACTORY).read() != s:
        open(URDF_FACTORY, "w").write(s)
    return URDF_FACTORY


def _joint_params():
    j = yaml.safe_load(open(PARAMS))["joints"]
    return {k: v["coulomb"] for k, v in j.items()}, {k: v["armature"] for k, v in j.items()}


def _arm(joints, effort, velocity, friction, armature):
    # effort control (stiffness / damping 0: the task-space controller computes the torques), identified Coulomb
    # friction and reflected rotor inertia as in our ros2_control scene
    return ImplicitActuatorCfg(joint_names_expr=joints, stiffness=0.0, damping=0.0, joint_effort_limit=effort,
                               joint_velocity_limit=velocity,
                               friction={k: v for k, v in friction.items() if re.fullmatch(joints[0], k)},
                               armature={k: v for k, v in armature.items() if re.fullmatch(joints[0], k)})


@configclass
class FactoryFR3PegInsertCfg(FactoryTaskPegInsertCfg):
    def __post_init__(self):
        friction, armature = _joint_params()
        # the FR3 collision meshes give more broad-phase pairs than the Panda: at N 1024 PhysX asked for 2.3 M
        # (default 2**21) and the states went NaN at the first reset
        self.sim.physics.gpu_total_aggregate_pairs_capacity = int(os.environ.get("FR3_AGG_PAIRS", 2**21))
        stock = self.scene.robot.spawn
        self.scene.robot = self.scene.robot.replace(
            spawn=sim_utils.UrdfFileCfg(
                asset_path=factory_urdf(), fix_base=True, fix_root_link=True, merge_fixed_joints=False,  # fix_base alone
                # left the articulation floating (6 base columns in the Jacobian) once articulation_props are passed
                convert_mimic_joints_to_normal_joints=True, activate_contact_sensors=True,
                rigid_props=stock.rigid_props, articulation_props=stock.articulation_props,
                collision_props=stock.collision_props,
                joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                    gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)),
            ),
            init_state=self.scene.robot.init_state.replace(
                joint_pos={**{f"fr3_joint{i + 1}": q for i, q in enumerate(Q0)}, "fr3_finger_joint.*": 0.04}),
            actuators={
                "fr3_arm1": _arm(["fr3_joint[1-4]"], 87.0, 2.62, friction, armature),
                "fr3_arm2": _arm(["fr3_joint[5-7]"], 12.0, 5.26, friction, armature),
                "fr3_hand": ImplicitActuatorCfg(
                    joint_names_expr=["fr3_finger_joint[1-2]"], joint_effort_limit=40.0, joint_velocity_limit=0.04,
                    stiffness=7500.0, damping=173.0, friction=0.1, armature=0.0),
            },
        )


gym.register(id=TASK, entry_point="isaaclab_tasks.contrib.factory.factory_env:FactoryEnv", disable_env_checker=True,
             kwargs={"env_cfg_entry_point": f"{__name__}:FactoryFR3PegInsertCfg"})
