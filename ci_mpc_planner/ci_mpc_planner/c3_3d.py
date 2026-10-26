"""C3+ contact-implicit MPC on the 3D peg-hole model (stage 3, step 2): centering and insertion once the peg is
engaged (after probing found the hole).

Model (peg_hole_3d.PegHole3D, linearized at the current state x0, its contact set fixed over the horizon):
    x_{k+1} = u_k + C (W lam_k + f+_k - f-_k)            quasi-static impedance, x = (x, y, z, rx, ry)
    eta_n    = phi + N (x_{k+1} - x0)                    gap
    eta_j+-  = gamma_k +- T_j (x_{k+1} - x_k), j = 1, 2  sliding (friction pyramid, 4 directions)
    eta_c    = mu lam_n - sum lam_t                      friction cone
    arm stiction per TCP coordinate: eta_a+- = gamma_a +- (x_{k+1} - x_k), eta_ac = F_s - f+ - f-
    complementarity (lam, gamma, f, gamma_a) _|_ eta, solved by C3+ ADMM (OSQP QP + closed-form projection).
Warm start (the planar C3 sat in the "stick" local minimum): the projection targets of the first ADMM iteration
come from a physics rollout of a nominal command (straight to the goal, step-limited) through the convex plant
(PegHole3D.step), so the first QP already sees the arm sliding and the contacts the rollout predicts; later solves
start from the previous solution's targets, shifted by one stage.
Cost: TCP pose to the goal, spring deflection (contact force / K), command rate. Convex limits: grasp wrench
(lateral, axial, torque), command step, lam / gamma / f >= 0.
Units in the QP: mm and mrad, N, N m.
"""
from dataclasses import dataclass, field

import numpy as np
import osqp
import scipy.sparse as sp


@dataclass
class C33Config:
    horizon: int = 4
    q_pos: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0, 0.05, 0.05]))
    q_terminal: float = 5.0
    r_force: np.ndarray = field(default_factory=lambda: np.array([0.05, 0.05, 0.05, 0.01, 0.01]))
    r_rate: np.ndarray = field(default_factory=lambda: np.array([0.1, 0.1, 0.1, 0.01, 0.01]))
    du_max: np.ndarray = field(default_factory=lambda: np.array([0.5, 0.5, 0.5, 10.0, 10.0]))
    grasp_lateral: float = 10.0
    grasp_axial: float = 20.0   # 40 N force-controlled grasp, mu 0.75 per finger pad: ~60 N, margin
    grasp_torque: float = 0.3
    arm_friction: np.ndarray = field(default_factory=lambda: np.array([2.0, 2.0, 3.5, 0.05, 0.05]))
    rho0: float = 10.0
    rho_growth: float = 1.8
    admm_iters: int = 10
    dual_rescale: bool = True    # omega /= rho_growth when rho grows (scaled dual, as dairlib C3: w /= rho_scale).
                                 # False (before 2026-10-26) inflates the dual 1.8x per iteration: stuck at the opening
    contact_radius: float = 0.0005  # 2 mm: 40 contacts in the bore, ~1 s per solve
    warm_start: str = "rollout"  # rollout | zero
    rollout_every: int = 4       # fresh rollout targets every n solves (and when the contact set changes), shifted
                                 # previous targets in between (always shifting: stale targets, stuck at the opening)


class PegHoleC3_3D:
    def __init__(self, model, compliance, cfg=C33Config()):
        self.model, self.cfg = model, cfg
        self.compliance = np.asarray(compliance, float)
        self.prev = None

    def solve(self, x0, goal, u_prev, nominal=None):
        """nominal: optional (horizon, 5) command sequence for the warm-start rollout (sampled teachers try several);
        default: straight to the goal, step-limited. info["U"] is the planned command sequence."""
        cfg, mdl = self.cfg, self.model
        S1 = np.full(5, 1000.0)
        x0s, goals, ups = x0 * S1, goal * S1, u_prev * S1
        phi, N, T1, T2, W = mdl.matrices(x0, cfg.contact_radius)
        m = len(phi)
        Nh, nq = cfg.horizon, 5
        C_s = np.diag(self.compliance * 1000.0)
        CW = C_s @ W
        Ns, T1s, T2s, phis = N, T1, T2, phi * 1000.0  # (m per m) = (mm per mm); rotations: m/rad -> mm/mrad: same
        mu, Fs = mdl.g.mu, np.asarray(cfg.arm_friction, float)
        nl, ng, ne, na = 5 * m, m, 6 * m, 30  # arm: f+ (5), f- (5), gamma_a (5), eta_a (15)
        nv = 2 * nq + nl + ng + ne + na
        n = Nh * nv
        ix = lambda k: k * nv  # noqa: E731
        iu = lambda k: k * nv + nq  # noqa: E731
        il = lambda k: k * nv + 2 * nq  # noqa: E731
        ig = lambda k: il(k) + nl  # noqa: E731
        ie = lambda k: ig(k) + ng  # noqa: E731
        ia = lambda k: ie(k) + ne  # noqa: E731
        # --- cost (triplets)
        Pr, Pc, Pv = [], [], []
        q = np.zeros(n)

        def addP(i, j, v):
            Pr.append(i), Pc.append(j), Pv.append(v)

        for k in range(Nh):
            w = cfg.q_pos * (cfg.q_terminal if k == Nh - 1 else 1.0)
            for j in range(nq):
                addP(ix(k) + j, ix(k) + j, 2 * w[j])
                q[ix(k) + j] -= 2 * w[j] * goals[j]
                rf, rr = cfg.r_force[j], cfg.r_rate[j]
                addP(iu(k) + j, iu(k) + j, 2 * rf + 2 * rr)
                addP(ix(k) + j, ix(k) + j, 2 * rf)
                addP(iu(k) + j, ix(k) + j, -2 * rf)
                addP(ix(k) + j, iu(k) + j, -2 * rf)
                if k == 0:
                    q[iu(k) + j] -= 2 * rr * ups[j]
                else:
                    addP(iu(k - 1) + j, iu(k - 1) + j, 2 * rr)
                    addP(iu(k) + j, iu(k - 1) + j, -2 * rr)
                    addP(iu(k - 1) + j, iu(k) + j, -2 * rr)
        # complementarity pairs (variable, partner)
        pairs = []
        for k in range(Nh):
            for i in range(m):
                pairs.append((il(k) + 5 * i, ie(k) + i))  # ln, eta_n
                for t in range(4):
                    pairs.append((il(k) + 5 * i + 1 + t, ie(k) + m + 4 * i + t))  # lam_t, eta_t
                pairs.append((ig(k) + i, ie(k) + 5 * m + i))  # gamma, eta_c
            for j in range(10):
                pairs.append((ia(k) + j, ia(k) + 15 + j))  # f+-, eta_a+-
            for j in range(5):
                pairs.append((ia(k) + 10 + j, ia(k) + 25 + j))  # gamma_a, eta_ac
        pairs = np.array(pairs, dtype=int)
        flat = pairs.reshape(-1)
        for v in flat:
            addP(v, v, 1e-9)
        P = sp.csc_matrix((Pv, (Pr, Pc)), shape=(n, n))
        D = sp.csc_matrix((np.ones(len(flat)), (flat, flat)), shape=(n, n))
        # --- constraints (triplets)
        Ar, Ac, Av, lo, hi = [], [], [], [], []
        row = [0]

        def add(coeffs, l, h):
            for col, v in coeffs.items():
                Ar.append(row[0]), Ac.append(col), Av.append(v)
            lo.append(l), hi.append(h)
            row[0] += 1

        Tj = [T1s, -T1s, T2s, -T2s]
        for k in range(Nh):
            for j in range(nq):  # dynamics
                c = {ix(k) + j: 1.0, iu(k) + j: -1.0, ia(k) + j: -C_s[j, j], ia(k) + 5 + j: C_s[j, j]}
                for col in np.flatnonzero(np.abs(CW[j]) > 0):
                    c[il(k) + col] = -CW[j, col]
                add(c, 0.0, 0.0)
            for i in range(m):
                c = {ie(k) + i: 1.0}
                for j in range(nq):
                    c[ix(k) + j] = -Ns[i, j]
                rhs = phis[i] - Ns[i] @ x0s
                add(c, rhs, rhs)
                for t in range(4):  # eta_t - gamma - Tj (x_{k+1} - x_k) = 0
                    c = {ie(k) + m + 4 * i + t: 1.0, ig(k) + i: -1.0}
                    rhs = 0.0
                    for j in range(nq):
                        c[ix(k) + j] = c.get(ix(k) + j, 0.0) - Tj[t][i, j]
                        if k == 0:
                            rhs -= Tj[t][i, j] * x0s[j]
                        else:
                            c[ix(k - 1) + j] = c.get(ix(k - 1) + j, 0.0) + Tj[t][i, j]
                    add(c, rhs, rhs)
                c = {ie(k) + 5 * m + i: 1.0, il(k) + 5 * i: -mu}
                for t in range(4):
                    c[il(k) + 5 * i + 1 + t] = 1.0
                add(c, 0.0, 0.0)
            for j in range(nq):  # arm stiction
                for sgn, off in ((1.0, 0), (-1.0, 5)):
                    c = {ia(k) + 15 + off + j: 1.0, ia(k) + 10 + j: -1.0, ix(k) + j: -sgn}
                    rhs = 0.0
                    if k == 0:
                        rhs = -sgn * x0s[j]
                    else:
                        c[ix(k - 1) + j] = sgn
                    add(c, rhs, rhs)
                add({ia(k) + 25 + j: 1.0, ia(k) + j: 1.0, ia(k) + 5 + j: 1.0}, Fs[j], Fs[j])
            for col in range(nl + ng):
                add({il(k) + col: 1.0}, 0.0, np.inf)
            for col in range(15):
                add({ia(k) + col: 1.0}, 0.0, np.inf)
            for j, lim in enumerate((cfg.grasp_lateral, cfg.grasp_lateral, cfg.grasp_axial, cfg.grasp_torque,
                                     cfg.grasp_torque)):
                c = {il(k) + col: W[j, col] for col in np.flatnonzero(np.abs(W[j]) > 0)}
                if c:
                    add(c, -lim, lim)
            for j in range(nq):
                if k == 0:
                    add({iu(k) + j: 1.0}, ups[j] - cfg.du_max[j], ups[j] + cfg.du_max[j])
                else:
                    add({iu(k) + j: 1.0, iu(k - 1) + j: -1.0}, -cfg.du_max[j], cfg.du_max[j])
        A = sp.csc_matrix((Av, (Ar, Ac)), shape=(row[0], n))
        lo, hi = np.array(lo), np.array(hi)
        # --- warm start of the projection targets
        delta = np.zeros(len(flat))
        if cfg.warm_start == "rollout":
            self.count = getattr(self, "count", 0) + 1
            if self.prev is not None and self.prev[0] == m and self.count % cfg.rollout_every:
                d_prev = self.prev[1]
                per_stage = len(flat) // Nh
                delta[:-per_stage] = d_prev[per_stage:]
                delta[-per_stage:] = d_prev[-per_stage:]
            else:
                delta = self._rollout_targets(x0, goal, u_prev, pairs, ix, il, ig, ie, ia, m, Nh, Fs, nominal)
        omega = np.zeros(len(flat))
        rho = cfg.rho0
        solver = osqp.OSQP()
        Pk = sp.triu(P + rho * D).tocsc()
        Pk.sort_indices()
        solver.setup(Pk, q, A, lo, hi, verbose=False, eps_abs=1e-5, eps_rel=1e-5, polish=False, max_iter=4000)
        z = None
        for it in range(cfg.admm_iters):
            qk = q.copy()
            np.add.at(qk, flat, -rho * (delta - omega))
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
            delta = np.empty_like(v)
            delta[0::2] = np.where(a > b, np.maximum(a, 0.0), 0.0)
            delta[1::2] = np.where(a > b, 0.0, np.maximum(b, 0.0))
            omega = omega + z[flat] - delta
            rho *= cfg.rho_growth
            if cfg.dual_rescale:
                omega /= cfg.rho_growth
        if z is None:
            return u_prev.copy(), {"status": "failed", "m": m}
        self.prev = (m, delta.copy())
        u0 = z[iu(0):iu(0) + nq] / S1
        X = np.array([z[ix(k):ix(k) + nq] for k in range(Nh)]) / S1
        viol = np.abs(z[flat] - delta).max() if len(flat) else 0.0
        U = np.array([z[iu(k):iu(k) + nq] for k in range(Nh)]) / S1
        return u0, {"status": "ok", "m": m, "X": X, "U": U, "viol": viol, "wrench0": W @ z[il(0):il(0) + nl]}

    def _rollout_targets(self, x0, goal, u_prev, pairs, ix, il, ig, ie, ia, m, Nh, Fs, nominal=None):
        """Projection targets from a rollout of a nominal command through the convex plant."""
        mdl, cfg = self.model, self.cfg
        x, u = x0.copy(), u_prev.copy()
        S1 = 1000.0
        target = {}
        for k in range(Nh):
            u = nominal[k] if nominal is not None else u + np.clip(goal - u, -cfg.du_max / S1, cfg.du_max / S1)
            xn, lam, nc = mdl.step(x, u, self.compliance, Fs, eps=1e-5, max_iter=2000)
            dx = (xn - x) * S1
            # arm friction: sliding where the rollout moved (f at the limit against the motion, gamma = |dx|)
            for j in range(5):
                if abs(dx[j]) > 1e-3:
                    target[ia(k) + (5 if dx[j] > 0 else 0) + j] = Fs[j]
                    target[ia(k) + 10 + j] = abs(dx[j])
            if nc == m and nc:  # same contact set as the model's linearization: normal forces from the rollout
                for i in range(m):
                    target[il(k) + 5 * i] = lam[5 * i]
            x = xn
        delta = np.zeros(pairs.size)
        flat = pairs.reshape(-1)
        for p, v in enumerate(flat):
            if v in target:
                delta[p] = target[v]
        return delta
