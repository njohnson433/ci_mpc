"""End-effector translational MPC: a double integrator per axis, solved as a QP with OSQP.

State x = [p (3), v (3)], input a (3) = EE acceleration, exact discretization with step dt:
    p+ = p + dt v + dt^2/2 a,   v+ = v + dt a
Cost over the horizon N:
    sum_k |p_k - r_k|^2_Qp + |v_k - w_k|^2_Qv + |a_k|^2_R + |a_k - a_{k-1}|^2_Rd   (terminal p, v weighted x term)
Constraints: x_0 = measured state, dynamics, |v| <= v_max, |a| <= a_max, p in [p_min, p_max] (per axis).
Only the initial state, the reference and a_{-1} change between solves, so the matrices are built once and
each solve updates the linear cost and the bounds.
"""
from dataclasses import dataclass, field

import numpy as np
import osqp
import scipy.sparse as sp


@dataclass
class MpcConfig:
    dt: float = 0.02
    horizon: int = 50
    q_pos: float = 1000.0
    q_vel: float = 50.0  # 10: 3.9 mm overshoot on a 15 cm reach; 50: none, 1.2 s to < 1 mm (sweep 2026-10-23)
    r_acc: float = 1.0
    r_dacc: float = 0.1
    terminal: float = 10.0
    v_max: float = 0.3
    a_max: float = 2.0
    p_min: np.ndarray = field(default_factory=lambda: np.array([0.2, -0.4, 0.15]))
    p_max: np.ndarray = field(default_factory=lambda: np.array([0.7, 0.4, 0.8]))


class TranslationalMpc:
    def __init__(self, cfg: MpcConfig):
        self.cfg = cfg
        N, dt = cfg.horizon, cfg.dt
        nx, nu = 6, 3
        self.nx, self.nu, self.N = nx, nu, N
        self.n_x = nx * (N + 1)
        self.n_var = self.n_x + nu * N

        I3 = np.eye(3)
        A = np.block([[I3, dt * I3], [np.zeros((3, 3)), I3]])
        B = np.vstack([0.5 * dt ** 2 * I3, dt * I3])

        # cost: states x_1..x_N, inputs a_0..a_{N-1}, acceleration changes a_k - a_{k-1}
        qx = np.concatenate([np.full(3, cfg.q_pos), np.full(3, cfg.q_vel)])
        Px = [np.zeros(nx)] + [qx] * (N - 1) + [cfg.terminal * qx]
        P_x = sp.diags(np.concatenate(Px))
        D = sp.eye(N) * 2 - sp.eye(N, k=-1) - sp.eye(N, k=1)  # second difference, ends corrected below
        D = D.tolil()
        D[N - 1, N - 1] = 1.0  # last a_k has only one neighbour (a_{k-1})
        P_u = sp.kron(sp.eye(N) * cfg.r_acc + cfg.r_dacc * D.tocsc(), sp.eye(nu))
        self.P = sp.block_diag([P_x, P_u], format="csc")
        self.qx_weights = Px

        # equality: x_0 = x_meas; x_{k+1} - A x_k - B a_k = 0
        Ax_dyn = sp.kron(sp.eye(N + 1), -sp.eye(nx)) + sp.kron(sp.eye(N + 1, k=-1), sp.csc_matrix(A))
        Bu_dyn = sp.kron(sp.vstack([sp.csc_matrix((1, N)), sp.eye(N)]), sp.csc_matrix(B))
        A_eq = sp.hstack([Ax_dyn, Bu_dyn])
        # inequality: box on all states (p and v) and inputs
        A_ineq = sp.eye(self.n_var)
        self.A = sp.vstack([A_eq, A_ineq], format="csc")

        x_lo = np.concatenate([cfg.p_min, np.full(3, -cfg.v_max)])
        x_hi = np.concatenate([cfg.p_max, np.full(3, cfg.v_max)])
        self.l_ineq = np.concatenate([np.tile(x_lo, N + 1), np.full(nu * N, -cfg.a_max)])
        self.u_ineq = np.concatenate([np.tile(x_hi, N + 1), np.full(nu * N, cfg.a_max)])
        self.n_eq = nx * (N + 1)

        self.solver = osqp.OSQP()
        l0, u0 = self._bounds(np.zeros(nx))
        self.solver.setup(self.P, np.zeros(self.n_var), self.A, l0, u0, verbose=False, warm_start=True,
                          polishing=False, eps_abs=1e-5, eps_rel=1e-5, max_iter=4000)
        self.a_prev = np.zeros(nu)

    def _bounds(self, x0):
        beq = np.zeros(self.n_eq)
        beq[:self.nx] = -x0
        l = np.concatenate([beq, self.l_ineq])
        u = np.concatenate([beq, self.u_ineq])
        # the measured state may be outside the box (e.g. faster than v_max after a push): relax x_0's box
        l[self.n_eq:self.n_eq + self.nx] = -np.inf
        u[self.n_eq:self.n_eq + self.nx] = np.inf
        return l, u

    def solve(self, x0, p_ref, v_ref=None):
        """x0: measured [p, v]; p_ref, v_ref: (N+1, 3) references for x_0..x_N (v_ref default 0).
        Returns (states (N+1, 6), accelerations (N, 3), status string)."""
        N, nx = self.N, self.nx
        if v_ref is None:
            v_ref = np.zeros_like(p_ref)
        ref = np.hstack([p_ref, v_ref])  # (N+1, 6)
        q = np.zeros(self.n_var)
        q[:self.n_x] = -(np.concatenate(self.qx_weights) * ref.reshape(-1))
        # -2 r_dacc a_{-1} a_0 term of the first acceleration change
        q[self.n_x:self.n_x + self.nu] = -self.cfg.r_dacc * self.a_prev
        l, u = self._bounds(np.asarray(x0, dtype=float))
        self.solver.update(q=q, l=l, u=u)
        res = self.solver.solve(raise_error=False)
        status = res.info.status
        if res.x is None or not np.all(np.isfinite(res.x)):
            return None, None, status
        X = res.x[:self.n_x].reshape(N + 1, nx)
        U = res.x[self.n_x:].reshape(N, self.nu)
        self.a_prev = U[0].copy()
        return X, U, status
