"""Isaac Lab Factory PegInsert with our FR3 instead of the stock Panda (ROADMAP 9, step 1, scene parity).

The stock task (isaaclab_tasks/contrib/factory) spawns franka_mimic.usd (Panda) with zero joint friction / armature.
Here: the FR3 from ~/ci_mpc_assets/fr3.urdf (same model as Pinocchio / the ros2_control sim) through Isaac Lab's URDF
importer, armature and friction (static / Coulomb / viscous) per joint from ci_mpc_sysid/config/fr3_joint_params_sim.yaml (the values
prepare_scene.py puts into our Isaac scene), FR3 effort / velocity limits.
The Factory env looks bodies up by Panda names, so a generated copy of the URDF (fr3_factory.urdf next to it) names
the fingers panda_left/rightfinger and adds panda_fingertip_centered fixed to fr3_hand at the fr3_hand_tcp offset
(TODO: compare with franka_mimic's fingertip frame; it sets where the held peg sits in the fingers).
TASK_CI: the same scene driven by our controller (ci_ctrl_torch.BatchedImpedance) every physics step instead of
Factory's; the env's `ctrl` holds the per-env command (pos_d, rot_d, twist_d, F_ff, K, D_cmd, integral_enabled) and
the wrench estimate (ctrl.F_ext); step() actions are ignored. Import registers both gym ids.
"""
import os
import re

import gymnasium as gym
import torch
import warp as wp
import yaml

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.utils import configclass

from isaaclab.utils import math as torch_utils
from isaaclab_tasks.contrib.factory import factory_utils
from isaaclab_tasks.contrib.factory.factory_env import FactoryEnv
from isaaclab_tasks.contrib.factory.factory_env_cfg import FactoryTaskPegInsertCfg

from ci_ctrl_torch import BatchedImpedance

TASK = "CiMpc-Factory-PegInsert-FR3-Direct"
TASK_CI = "CiMpc-Factory-PegInsert-FR3-CiCtrl-Direct"
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
    if os.environ.get("FR3_DIAG_INERTIA"):  # test switch: drop the off-diagonal inertia terms
        s = re.sub(r'(i(?:xy|xz|yz))="[^"]*"', r'\1="0.0"', s)
    if not os.path.exists(URDF_FACTORY) or open(URDF_FACTORY).read() != s:
        open(URDF_FACTORY, "w").write(s)
    return URDF_FACTORY


def _joint_params():
    return yaml.safe_load(open(PARAMS))["joints"]


def _arm(joints, effort, velocity, params):
    # effort control (stiffness / damping 0: the task-space controller computes the torques), identified friction
    # (static breakaway, Coulomb while moving, viscous; tensor API in rad, prepare_scene.py converts to the USD's
    # per-degree viscous) and reflected rotor inertia as in our ros2_control scene
    def pick(key):
        scale = 0.0 if key != "armature" and os.environ.get("FR3_NO_FRICTION") else 1.0  # test switches
        scale = 0.0 if key == "armature" and os.environ.get("FR3_NO_ARMATURE") else scale
        return {k: scale * v[key] for k, v in params.items() if re.fullmatch(joints[0], k)}
    return ImplicitActuatorCfg(joint_names_expr=joints, stiffness=0.0, damping=0.0, joint_effort_limit=effort,
                               joint_velocity_limit=velocity, friction=pick("static"), dynamic_friction=pick("coulomb"),
                               viscous_friction=pick("viscous"), armature=pick("armature"))


@configclass
class FactoryFR3PegInsertCfg(FactoryTaskPegInsertCfg):
    def __post_init__(self):
        params = _joint_params()
        # the FR3 collision meshes give more broad-phase pairs than the Panda: at N 1024 PhysX asked for 2.3 M
        # (default 2**21) and the states went NaN at the first reset
        self.sim.physics.gpu_total_aggregate_pairs_capacity = int(os.environ.get("FR3_AGG_PAIRS", 2**21))
        stock = self.scene.robot.spawn
        art = stock.articulation_props
        if os.environ.get("FR3_ITERS"):  # test switch "pos,vel": robot solver iterations (stock 192, 1)
            pi, vi = (int(v) for v in os.environ["FR3_ITERS"].split(","))
            art = [a.replace(solver_position_iteration_count=pi, solver_velocity_iteration_count=vi)
                   if hasattr(a, "solver_velocity_iteration_count") else a for a in art]
        self.scene.robot = self.scene.robot.replace(
            spawn=sim_utils.UrdfFileCfg(
                asset_path=factory_urdf(), fix_base=True, fix_root_link=True, merge_fixed_joints=False,  # fix_base alone
                # left the articulation floating (6 base columns in the Jacobian) once articulation_props are passed
                convert_mimic_joints_to_normal_joints=True, activate_contact_sensors=True,
                rigid_props=stock.rigid_props, articulation_props=art,
                collision_props=stock.collision_props,
                joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                    gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)),
            ),
            init_state=self.scene.robot.init_state.replace(
                joint_pos={**{f"fr3_joint{i + 1}": q for i, q in enumerate(Q0)}, "fr3_finger_joint.*": 0.04}),
            actuators={
                "fr3_arm1": _arm(["fr3_joint[1-4]"], 87.0, 2.62, params),
                "fr3_arm2": _arm(["fr3_joint[5-7]"], 12.0, 5.26, params),
                "fr3_hand": ImplicitActuatorCfg(
                    joint_names_expr=["fr3_finger_joint[1-2]"], joint_effort_limit=40.0, joint_velocity_limit=0.04,
                    stiffness=7500.0, damping=173.0, friction=0.1, armature=0.0),
            },
        )


class CiCtrlFactoryEnv(FactoryEnv):
    """Factory PegInsert (FR3) with our impedance controller + observer at the physics rate (like ros2_control at
    240 Hz). Factory's own controller still runs the reset (IK, grasp); our controller then starts from the current
    pose (on_activate)."""

    def __init__(self, cfg, render_mode=None, **kwargs):
        # FactoryEnv adds 0.01 kg m^2 to every body's inertia (an IsaacGymEnvs armature workaround): M then differs
        # from Pinocchio / our scene / the FR3 by 30 % (test_ci_ctrl.py model check); our FR3 has the real armature
        orig, factory_utils.set_body_inertias = factory_utils.set_body_inertias, lambda robot: None
        try:
            super().__init__(cfg, render_mode, **kwargs)
        finally:
            factory_utils.set_body_inertias = orig
        self.ctrl = BatchedImpedance(self.num_envs, self.device, self.physics_dt)
        self.tcp_idx = self._robot.body_names.index("panda_fingertip_centered")
        self.hand_idx = self._robot.body_names.index("fr3_hand")
        self.ci_qdd = torch.zeros(self.num_envs, 7, device=self.device)
        self.ci_dq_prev = None
        self.ci_active = False
        self.ctrl_arm = torch.tensor([p["armature"] for p in _joint_params().values()], device=self.device)
        self.ci_reset_ctrl()

    # Link-side joint torques (the FR3 torque sensors; Isaac's ros2_control effort state in our pipeline):
    #   "projected": PhysX get_dof_projected_joint_forces. In this env it is wrong on the proximal joints (known force
    #     at the TCP at rest: joint 2 reads -0.6 instead of -2.6 Nm; the link incoming forces are not even consistent
    #     along the chain, 9.7 N at links 6-7, 4.2 N at links 2-3), independent of friction, armature, solver
    #     iterations, GPU / CPU, inertia (test_ci_ctrl.py, PROGRESS 2026-10-27). Joints 5-7 and the hand are right.
    #   "wrist": M qdd + C dq + J_hand^T W_hand, W_hand = the hand's incoming joint wrench (world-aligned, about the hand
    #     origin), i.e. the link-side torque for everything acting through the hand (fingers, held part: all our
    #     contacts); neglects the hand subtree's inertial force (~0.1 N at our accelerations), forces on links 1-7.
    ci_tau_source = os.environ.get("CI_TAU_SOURCE", "wrist")

    def ci_state(self):
        """Inputs of the C++ controller from PhysX: q, dq, M (no armature), C dq, J at the TCP (world-aligned), TCP
        pose, link-side joint torques (ci_tau_source)."""
        d, view = self._robot.data, self._robot.root_view
        q, dq = d.joint_pos.torch[:, :7], d.joint_vel.torch[:, :7]
        M = d.mass_matrix.torch[:, :7, :7] - torch.diag_embed(self.ctrl_arm) * self.ci_M_has_armature
        cor = wp.to_torch(view.get_coriolis_and_centrifugal_compensation_forces())[:, :7]
        J = d.body_link_jacobian_w.torch[:, self.tcp_idx - 1, 0:6, 0:7]  # fixed base: no root body in the Jacobian
        pos = d.body_pos_w.torch[:, self.tcp_idx] - self.scene.env_origins
        rot = torch_utils.matrix_from_quat(d.body_quat_w.torch[:, self.tcp_idx])
        if self.ci_tau_source == "projected":
            tau = wp.to_torch(view.get_dof_projected_joint_forces())[:, :7]
        else:
            tau = (M @ self.ci_qdd.unsqueeze(-1)).squeeze(-1) + cor + (
                self.ci_hand_jacobian().transpose(1, 2) @ self.ci_hand_wrench().unsqueeze(-1)).squeeze(-1)
        return q, dq, M, cor, J, pos, rot, tau

    def ci_hand_jacobian(self):
        return self._robot.data.body_link_jacobian_w.torch[:, self.hand_idx - 1, 0:6, 0:7]

    def ci_hand_wrench(self):
        """Wrench the arm exerts on the hand subtree, world-aligned, torque about the hand origin (Franka sign: at
        rest = the force of the robot on the environment); the pipeline's simulated wrist F/T (wrist_ft.py)."""
        raw = wp.to_torch(self._robot.root_view.get_link_incoming_joint_force())[:, self.hand_idx]
        R = torch_utils.matrix_from_quat(self._robot.data.body_quat_w.torch[:, self.hand_idx])
        return torch.cat(((R @ raw[:, :3, None]).squeeze(-1), (R @ raw[:, 3:, None]).squeeze(-1)), 1)

    def ci_reset_ctrl(self, env_ids=None):
        env_ids = torch.arange(self.num_envs, device=self.device) if env_ids is None else env_ids
        _, _, _, _, _, pos, rot, _ = self.ci_state()
        self.ctrl.reset(env_ids, pos, rot)
        self.ci_qdd[env_ids], self.ci_dq_prev = 0.0, None
        self.ci_active = True

    ci_M_has_armature = 0.0  # PhysX's mass matrix excludes the joint armature (= Pinocchio, test_ci_ctrl.py)

    def _pre_physics_step(self, action):
        if not self.ci_active:
            return super()._pre_physics_step(action)

    def _apply_action(self):
        if not self.ci_active:
            return super()._apply_action()
        if self.last_update_timestamp < self._robot._data._sim_timestamp:
            self._compute_intermediate_values(dt=self.physics_dt)
        dq = self._robot.data.joint_vel.torch[:, :7]
        if self.ci_dq_prev is not None:
            self.ci_qdd = (dq - self.ci_dq_prev) / self.physics_dt
        self.ci_dq_prev = dq.clone()
        tau = self.ctrl.compute(*self.ci_state())
        self.joint_torque = torch.zeros_like(self.joint_torque) if hasattr(self, "joint_torque") else torch.zeros(
            (self.num_envs, self._robot.num_joints), device=self.device)
        self.joint_torque[:, :7] = tau
        self.ctrl_target_joint_pos[:, 7:9] = 0.0  # gripper closed (position target, PhysX PD)
        self._robot.set_joint_position_target_index(target=self.ctrl_target_joint_pos)
        self._robot.set_joint_effort_target_index(target=self.joint_torque)

    def _get_rewards(self):
        if not self.ci_active:
            return super()._get_rewards()
        return torch.zeros(self.num_envs, device=self.device)

    def _reset_idx(self, env_ids):
        self.ci_active = False  # Factory's reset uses its own controller
        super()._reset_idx(env_ids)
        self.ci_reset_ctrl()


gym.register(id=TASK, entry_point="isaaclab_tasks.contrib.factory.factory_env:FactoryEnv", disable_env_checker=True,
             kwargs={"env_cfg_entry_point": f"{__name__}:FactoryFR3PegInsertCfg"})
gym.register(id=TASK_CI, entry_point=f"{__name__}:CiCtrlFactoryEnv", disable_env_checker=True,
             kwargs={"env_cfg_entry_point": f"{__name__}:FactoryFR3PegInsertCfg"})
