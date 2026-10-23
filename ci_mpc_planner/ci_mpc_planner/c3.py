"""Contact-implicit MPC for the planar peg-in-hole with C3+ (ADMM over complementarity constraints, OSQP).

Model (peg_hole.PegHole, linearized at the current state x0 with its contact set):
    x_{k+1} = u_k + C W lam_k                                  (quasi-static impedance, C = K^-1)
    eta_n   = phi + N (x_{k+1} - x0)                           (normal gap at the next state)
    eta_t+- = gamma_k +- T (x_{k+1} - x_k)                     (sliding)
    eta_g   = mu lam_n - lam_t+ - lam_t-                       (friction cone)
    complementarity  0 <= (lam_n, lam_t+, lam_t-, gamma) _|_ (eta_n, eta_t+, eta_t-, eta_g) >= 0
C3+ (Bui et al. 2024): the complementarity is enforced by alternating
    QP:      min cost(z) + rho/2 |S z - delta + omega|^2  s.t. the linear equations above + convex limits
    project: (lam_i, eta_i) <- (max(a, 0), 0) if a > b else (0, max(b, 0))   with (a, b) = S z + omega, per pair
    dual:    omega += S z - delta,  rho *= rho_growth
so the solver chooses contact modes (open / stick / slide) itself.
Cost: TCP pose to the goal (tip at the hole bottom on the hole axis, upright), spring deflection u - x (= contact
force / K, keeps forces low), command rate. Convex limits: grasp wrench (axial / lateral force, torque about the
TCP: the peg must not slip or turn in the fingers), command step per stage, lam >= 0.
Units inside the QP: mm and mrad for displacements / angles (conditioning), N for forces, N m for torque.
"""
from dataclasses import dataclass, field

import numpy as np
import osqp
import scipy.sparse as sp

from ci_mpc_planner.peg_hole import PegHole


@dataclass
class C3Config:
    horizon: int = 8
    q_pos: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 0.05]))   # x, z [1/mm^2], phi [1/mrad^2]
    q_terminal: float = 5.0
    r_force: np.ndarray = field(default_factory=lambda: np.array([0.05, 0.05, 0.01]))  # on u - x+ (mm / mrad)
    r_rate: np.ndarray = field(default_factory=lambda: np.array([0.1, 0.1, 0.01]))    # on u_k - u_{k-1}
    du_max: np.ndarray = field(default_factory=lambda: np.array([0.5, 0.5, 20.0]))   # per stage, mm / mrad
    grasp_axial: float = 10.0    # [N] peg axial load before it slips in the fingers (2 x 0.75 x ~8 N grip, margin)
    grasp_lateral: float = 6.0   # [N]
    grasp_torque: float = 0.08   # [N m] about the TCP before the peg turns in the fingers
    rho0: float = 10.0          # 1: complementarity violation 3.0 after 12 iterations, 10: 0.14 (offline)
    rho_growth: float = 1.8
    admm_iters: int = 12
    contact_radius: float = 0.004


class PegHoleC3:
    def __init__(self, model: PegHole, compliance, cfg=C3Config()):
        self.model, self.cfg = model, cfg
        self.compliance = np.asarray(compliance, float)
        self.u_prev = None
        self.last = {}

    def solve(self, x0, goal, u_prev=None):
        """x0: measured TCP pose (m, m, rad); goal: TCP goal pose; u_prev: last command.
        Returns (u0 [m, m, rad], info)."""
        cfg, mdl = self.cfg, self.model
        S1 = np.array([1000.0, 1000.0, 1000.0])  # m, m, rad -> mm, mm, mrad
        x0s, goals = x0 * S1, goal * S1
        u_prev = self.u_prev if u_prev is None else u_prev
        u_prev_s = (u_prev if u_prev is not None else x0) * S1
        phi, N, T, W = mdl.matrices(x0, self.compliance, cfg.contact_radius)
        m = len(phi)
        Nh = cfg.horizon
        C_s = np.diag(self.compliance) * S1[:, None]  # (mm|mrad) per (N|N m)
        CW = C_s @ W  # 3 x 3m
        # contact rows in scaled units: gap [mm] per scaled state; N, T have units m per (m, m, rad)
        Ns = N * 1000.0 / S1[None, :]
        Ts = T * 1000.0 / S1[None, :]
        phis = phi * 1000.0
        mu = mdl.g.mu
        # variable layout per stage k: x_{k+1} (3), u_k (3), lam_k (3m), gam_k (m), eta_k (4m)
        nl, ng, ne = 3 * m, m, 4 * m
        nv = 3 + 3 + nl + ng + ne
        n = Nh * nv
        ix = lambda k: k * nv  # noqa: E731
        iu = lambda k: k * nv + 3  # noqa: E731
        il = lambda k: k * nv + 6  # noqa: E731
        ig = lambda k: k * nv + 6 + nl  # noqa: E731
        ie = lambda k: k * nv + 6 + nl + ng  # noqa: E731
        # --- cost
        P = sp.lil_matrix((n, n))
        q = np.zeros(n)
        for k in range(Nh):
            w = cfg.q_pos * (cfg.q_terminal if k == Nh - 1 else 1.0)
            for j in range(3):
                P[ix(k) + j, ix(k) + j] += 2 * w[j]
                q[ix(k) + j] += -2 * w[j] * goals[j]
                # spring deflection (u - x+)
                rf = cfg.r_force[j]
                P[iu(k) + j, iu(k) + j] += 2 * rf
                P[ix(k) + j, ix(k) + j] += 2 * rf
                P[iu(k) + j, ix(k) + j] += -2 * rf
                P[ix(k) + j, iu(k) + j] += -2 * rf
                # command rate
                rr = cfg.r_rate[j]
                P[iu(k) + j, iu(k) + j] += 2 * rr
                if k == 0:
                    q[iu(k) + j] += -2 * rr * u_prev_s[j]
                else:
                    P[iu(k - 1) + j, iu(k - 1) + j] += 2 * rr
                    P[iu(k) + j, iu(k - 1) + j] += -2 * rr
                    P[iu(k - 1) + j, iu(k) + j] += -2 * rr
        sel = []  # indices of the complementarity variables (lam, gam) and their partners (eta)
        for k in range(Nh):
            a_idx = list(range(il(k), il(k) + nl)) + list(range(ig(k), ig(k) + ng))
            # pair order: (ln_i, eta_n_i), (lt+_i, eta_t+_i), (lt-_i, eta_t-_i), (gam_i, eta_g_i)
            lam_order = []
            for i in range(m):
                lam_order += [il(k) + 3 * i]
            for i in range(m):
                lam_order += [il(k) + 3 * i + 1]
            for i in range(m):
                lam_order += [il(k) + 3 * i + 2]
            lam_order += list(range(ig(k), ig(k) + ng))
            eta_order = list(range(ie(k), ie(k) + ne))
            sel += list(zip(lam_order, eta_order))
            del a_idx
        sel = np.array(sel, dtype=int).reshape(-1, 2)
        for a, b in sel:  # explicit diagonal entries for the rho term (same sparsity for every rho)
            P[a, a] += 1e-9
            P[b, b] += 1e-9
        P = P.tocsc()
        # --- constraints
        rows, lo, hi = [], [], []
        A = sp.lil_matrix((0, n))
        Al = []

        def add(coeffs, l, h):
            Al.append((coeffs, l, h))

        for k in range(Nh):
            for j in range(3):  # dynamics x_{k+1} - u_k - CW lam_k = 0
                c = {ix(k) + j: 1.0, iu(k) + j: -1.0}
                for col in range(nl):
                    if abs(CW[j, col]) > 0:
                        c[il(k) + col] = -CW[j, col]
                add(c, 0.0, 0.0)
            for i in range(m):
                # eta_n - N (x_{k+1} - x0) = phi
                c = {ie(k) + i: 1.0}
                for j in range(3):
                    c[ix(k) + j] = c.get(ix(k) + j, 0.0) - Ns[i, j]
                rhs = phis[i] - Ns[i] @ x0s
                add(c, rhs, rhs)
                # eta_t+ - gam - T (x_{k+1} - x_k) = 0 ; eta_t- - gam + T (x_{k+1} - x_k) = 0
                for sgn, off in ((1.0, m), (-1.0, 2 * m)):
                    c = {ie(k) + off + i: 1.0, ig(k) + i: -1.0}
                    rhs = 0.0
                    for j in range(3):
                        c[ix(k) + j] = c.get(ix(k) + j, 0.0) - sgn * Ts[i, j]
                        if k == 0:
                            rhs += -sgn * Ts[i, j] * x0s[j]
                        else:
                            c[ix(k - 1) + j] = c.get(ix(k - 1) + j, 0.0) + sgn * Ts[i, j]
                    add(c, rhs, rhs)
                # eta_g - mu ln + lt+ + lt- = 0
                add({ie(k) + 3 * m + i: 1.0, il(k) + 3 * i: -mu, il(k) + 3 * i + 1: 1.0, il(k) + 3 * i + 2: 1.0},
                    0.0, 0.0)
            # lam, gamma >= 0
            for col in range(nl + ng):
                add({il(k) + col: 1.0}, 0.0, np.inf)
            # grasp limits on the generalized contact force W lam (Fx, Fz, torque)
            for j, lim in enumerate((cfg.grasp_lateral, cfg.grasp_axial, cfg.grasp_torque)):
                c = {il(k) + col: W[j, col] for col in range(nl) if abs(W[j, col]) > 0}
                if c:
                    add(c, -lim, lim)
            # command step
            for j in range(3):
                if k == 0:
                    add({iu(k) + j: 1.0}, u_prev_s[j] - cfg.du_max[j], u_prev_s[j] + cfg.du_max[j])
                else:
                    add({iu(k) + j: 1.0, iu(k - 1) + j: -1.0}, -cfg.du_max[j], cfg.du_max[j])
        r, cidx, vals, l, h = [], [], [], [], []
        for ri, (c, lo_, hi_) in enumerate(Al):
            for col, v in c.items():
                r.append(ri)
                cidx.append(col)
                vals.append(v)
            l.append(lo_)
            h.append(hi_)
        A = sp.csc_matrix((vals, (r, cidx)), shape=(len(Al), n))
        l, h = np.array(l), np.array(h)
        # --- C3+ ADMM
        D = sp.csc_matrix((np.ones(2 * len(sel)), (sel.reshape(-1), sel.reshape(-1))), shape=(n, n))
        rho = cfg.rho0
        delta = np.zeros(2 * len(sel))
        omega = np.zeros(2 * len(sel))
        solver = osqp.OSQP()
        Pk = sp.triu(P + rho * D).tocsc()
        Pk.sort_indices()
        solver.setup(Pk, q, A, l, h, verbose=False, eps_abs=1e-5, eps_rel=1e-5, polish=False, max_iter=4000,
                     warm_starting=True)
        z = None
        flat = sel.reshape(-1)
        for it in range(cfg.admm_iters):
            target = delta - omega
            qk = q.copy()
            np.add.at(qk, flat, -rho * target)
            if it > 0:
                Pn = sp.triu(P + rho * D).tocsc()
                Pn.sort_indices()
                solver.update(Px=Pn.data, q=qk)
            else:
                solver.update(q=qk)
            res = solver.solve(raise_error=False)
            if res.x is None:
                break
            z = res.x
            v = z[flat] + omega
            a, b = v[0::2], v[1::2]
            da = np.where(a > b, np.maximum(a, 0.0), 0.0)
            db = np.where(a > b, 0.0, np.maximum(b, 0.0))
            delta = np.empty_like(v)
            delta[0::2], delta[1::2] = da, db
            omega = omega + z[flat] - delta
            rho *= cfg.rho_growth
        if z is None:
            return (u_prev_s / S1), {"status": "failed", "m": m}
        u0 = z[iu(0):iu(0) + 3] / S1
        X = np.array([z[ix(k):ix(k) + 3] for k in range(Nh)]) / S1
        lam0 = z[il(0):il(0) + nl]
        viol = np.abs(z[flat] - delta).max() if len(flat) else 0.0
        self.u_prev = u0.copy()
        self.last = {"status": "ok", "m": m, "X": X, "lam0": lam0, "wrench0": W @ lam0, "viol": viol}
        return u0, self.last
