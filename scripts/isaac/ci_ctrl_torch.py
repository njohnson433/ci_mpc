"""Our Cartesian impedance controller + momentum observer, batched in torch (ROADMAP 9, step 1).

A line-by-line port of CartesianImpedanceController::update (cartesian_impedance_control, branch isaac-sim-pinocchio)
with the parameters of config/isaac_sim_controllers.yaml, for N envs at once:
  error = [p - p_d, -R vec(q^-1 q_d)],  D = 2 sqrt(K) sqrt(diag Lambda) (rot x1.3, D_z >= 2 sqrt(K_z)) or the commanded D,
  conditional integral (windows, EE speed, freeze on saturation, leak), F = -(D (J dq - twist) + K e + I) + F_ff clamped
  (18 N / 9 Nm / 2 Nm about z), tau = J^T F + C dq + velocity friction compensation, 1 Nm/ms torque-rate limit.
Observer (observer_torque_source measured): p = M dq, r = K_o (p - int(tau_meas + C^T dq + r) - p0),
F_ext = -(J J^T)^-1 J r (Franka sign: force of the robot on the environment). Torque sensor model: bias + N(0, noise).
Differences to the C++ (none in the control law):
  - C^T dq comes from M_dot dq - C dq (finite-difference M_dot; Pinocchio computes it exactly);
  - M, C dq, J come from PhysX instead of Pinocchio (test_ci_ctrl.py compares them);
  - no MPC-command timeout (the batched strategy writes the command every control step).
"""
import torch

# isaac_sim_controllers.yaml / cartesian_impedance_controller.hpp
COULOMB = (1.025412896, 1.259913793, 0.8380147058, 0.96, 1.2928, 0.41525, 0.5341655)
BETA = (1.18, 0.0, 0.55, 0.87, 0.935, 0.54, 0.45)
TORQUE_BIAS = (0.10, -0.15, 0.08, 0.12, -0.05, 0.06, -0.03)


class BatchedImpedance:
    def __init__(self, num_envs, device, dt, observer_gain=30.0, torque_noise=0.05, torque_bias=TORQUE_BIAS,
                 integrator_weights=(3000.0, 3000.0, 3000.0, 50.0, 50.0, 50.0),
                 integrator_limits=(8.0, 8.0, 8.0, 1.5, 1.5, 0.5),
                 integrator_windows=(0.003, 0.003, 0.003, 0.02, 0.02, 0.02), integrator_velocity_window=0.01,
                 integrator_leak=2.0, integral_off_leak=5.0, friction_scale=1.0, friction_vel_eps=0.02,
                 delta_tau_max=1.0, generator=None):
        f = dict(device=device, dtype=torch.float32)
        self.n, self.device, self.dt = num_envs, device, dt
        self.gen = generator
        # sensor model and observer gain per env, so they can be randomized (step 2)
        self.obs_gain, self.noise = torch.full((num_envs, 1), observer_gain, **f), torch.full((num_envs, 1), torque_noise, **f)
        self.bias = torch.tensor(torque_bias, **f).repeat(num_envs, 1)
        self.wI, self.limI = torch.tensor(integrator_weights, **f), torch.tensor(integrator_limits, **f)
        self.winI, self.vwin = torch.tensor(integrator_windows, **f), integrator_velocity_window
        self.leak, self.off_leak = integrator_leak, integral_off_leak
        self.coulomb = friction_scale * torch.tensor(COULOMB, **f)
        self.beta = friction_scale * torch.tensor(BETA, **f)
        self.vel_eps, self.max_step = friction_vel_eps, delta_tau_max * dt / 0.001
        self.f_clamp = torch.tensor((18.0, 18.0, 18.0, 9.0, 9.0, 2.0), **f)
        z6, z7 = torch.zeros(num_envs, 6, **f), torch.zeros(num_envs, 7, **f)
        # command (MpcCommand fields), per env
        self.pos_d, self.rot_d = torch.zeros(num_envs, 3, **f), torch.eye(3, **f).repeat(num_envs, 1, 1)
        self.twist_d, self.F_ff = z6.clone(), z6.clone()
        self.K = torch.tensor((2500.0, 2500.0, 1000.0, 130.0, 130.0, 45.0), **f).repeat(num_envs, 1)  # C++ default K
        self.D_cmd = z6.clone()  # all zero = critically damped from K and Lambda
        self.integral_enabled = torch.ones(num_envs, dtype=torch.bool, device=device)
        # controller state
        self.tau_prev, self.I, self.saturated = z7.clone(), z6.clone(), torch.zeros(num_envs, 6, dtype=torch.bool,
                                                                                         device=device)
        self.obs_int, self.obs_p0, self.tau_ext = z7.clone(), z7.clone(), z7.clone()
        self.M_prev, self.obs_init = torch.zeros(num_envs, 7, 7, **f), torch.zeros(num_envs, dtype=torch.bool,
                                                                                   device=device)
        self.F_ext, self.F, self.error = z6.clone(), z6.clone(), z6.clone()

    def reset(self, env_ids, pos, rot):
        """on_activate: hold the current pose, zero torque / integral, restart the observer."""
        self.pos_d[env_ids], self.rot_d[env_ids] = pos[env_ids], rot[env_ids]
        for t in (self.twist_d, self.F_ff, self.D_cmd, self.tau_prev, self.I, self.obs_int, self.tau_ext, self.F_ext):
            t[env_ids] = 0.0
        self.saturated[env_ids] = False
        self.integral_enabled[env_ids] = True
        self.obs_init[env_ids] = False

    def _observer(self, M, dq, coriolis, J, tau_meas):
        tau = tau_meas + self.bias
        tau = tau + self.noise * torch.randn(tau.shape, device=self.device, generator=self.gen)
        p = (M @ dq.unsqueeze(-1)).squeeze(-1)
        new = ~self.obs_init
        if new.any():
            self.obs_p0[new], self.obs_int[new], self.tau_ext[new], self.M_prev[new] = p[new], 0.0, 0.0, M[new]
        ct_dq = (((M - self.M_prev) / self.dt) @ dq.unsqueeze(-1)).squeeze(-1) - coriolis  # C^T dq = M_dot dq - C dq
        old = self.obs_init.unsqueeze(-1)
        self.obs_int = torch.where(old, self.obs_int + (tau + ct_dq + self.tau_ext) * self.dt, self.obs_int)
        self.tau_ext = torch.where(old, self.obs_gain * (p - self.obs_int - self.obs_p0), self.tau_ext)
        self.M_prev, self.obs_init = M.clone(), torch.ones_like(self.obs_init)
        JJt = J @ J.transpose(1, 2)
        self.F_ext = -torch.linalg.solve(JJt, (J @ self.tau_ext.unsqueeze(-1)).squeeze(-1))

    def compute(self, q, dq, M, coriolis, J, pos, rot, tau_meas):
        """One control cycle. q, dq (N, 7); M (N, 7, 7) without armature; coriolis = C dq (N, 7); J (N, 6, 7) at the
        TCP, world-aligned; pos (N, 3), rot (N, 3, 3) of the TCP; tau_meas: link-side joint torques (N, 7).
        Returns the joint torque command (N, 7); F_ext (N, 6) holds the wrench estimate."""
        del q
        self._observer(M, dq, coriolis, J, tau_meas)
        e = torch.empty_like(self.error)
        e[:, :3] = pos - self.pos_d
        E = rot.transpose(1, 2) @ self.rot_d  # orientation^-1 * orientation_d, as a matrix; quaternion with w >= 0
        w = 0.5 * torch.sqrt(torch.clamp(1.0 + E[:, 0, 0] + E[:, 1, 1] + E[:, 2, 2], min=1e-12))
        v = torch.stack((E[:, 2, 1] - E[:, 1, 2], E[:, 0, 2] - E[:, 2, 0], E[:, 1, 0] - E[:, 0, 1]), 1) / (4.0 * w)[:, None]
        e[:, 3:] = -(rot @ v.unsqueeze(-1)).squeeze(-1)
        self.error = e

        Minv = torch.linalg.inv(M)
        Lam = torch.linalg.inv(J @ Minv @ J.transpose(1, 2))
        lam = torch.diagonal(Lam, dim1=1, dim2=2).clamp(min=0.0)
        Kc = self.K.clamp(min=0.0)
        D = 2.0 * Kc.sqrt() * lam.sqrt()
        D[:, 3:] *= 1.3
        D[:, 5] = torch.maximum(D[:, 5], 2.0 * Kc[:, 5].sqrt())
        D = torch.where((self.D_cmd != 0.0).any(1, keepdim=True), self.D_cmd, D)

        v_ee = (J @ dq.unsqueeze(-1)).squeeze(-1)
        slow = (v_ee[:, :3].norm(dim=1) < self.vwin) if self.vwin > 0 else torch.ones_like(self.integral_enabled)
        decay = torch.exp(-torch.where(self.integral_enabled, self.leak, self.off_leak) * self.dt)[:, None]
        in_win = torch.where(self.winI > 0, e.abs() < self.winI, torch.ones_like(e, dtype=torch.bool))
        act = (self.integral_enabled & slow)[:, None] & in_win & ~self.saturated
        self.I = torch.where(act, self.I + self.wI * e * self.dt, self.I * decay)
        self.I = torch.maximum(torch.minimum(self.I, self.limI), -self.limI)

        F_un = -(D * (v_ee - self.twist_d) + self.K * e + self.I) + self.F_ff
        F = torch.maximum(torch.minimum(F_un, self.f_clamp), -self.f_clamp)
        self.saturated, self.F = (F_un - F).abs() > 1e-9, F

        tau_imp = (J.transpose(1, 2) @ F.unsqueeze(-1)).squeeze(-1)
        tau_fric = self.coulomb * torch.tanh(dq / self.vel_eps) + self.beta * dq
        tau = tau_imp + coriolis + tau_fric
        tau = self.tau_prev + torch.clamp(tau - self.tau_prev, -self.max_step, self.max_step)
        self.tau_prev = tau
        return tau
