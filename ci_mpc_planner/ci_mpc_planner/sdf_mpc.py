"""CI-MPC stage B (docs/ci_mpc_design.md section 2), B2 skeleton: smooth quasi-static plant on a primitive SDF, and a
Gauss-Newton / Levenberg-Marquardt MPC over a short setpoint horizon. Offline only; not wired into the demo.

Geometry from dimensions (no meshes):
  HoleSpec   block top face at z = 0 (hole frame), bore of radius R (round) or half widths (hx, hy) (box), depth D,
             optional 45-degree chamfer c, all edges rounded by rho (the SDF is C1: gap and normal are smooth across
             face / rim / chamfer / bore / bottom).
  PegSpec    cylinder of radius a (or box half widths (ax, ay)), end chamfer pc; surface points on the bottom face,
             the chamfer edge and the side up to `side_len`.
State x = TCP pose (x, y, z, rx, ry, rz) in the hole frame; the peg is rigid in the hand (known in-hand pose).
Plant (quasi-static, one step): find x+ with
    K (u - x+) + f_arm(x+ - x) + sum_i J_i(x+)^T f_i(x+) = 0
  f_i   contact force at peg point i: normal k_c eps softplus(-phi_i / eps) n_i, friction -mu f_n v_t / sqrt(|v_t|^2 + v_eps^2)
        with v_t the point's tangential displacement over the step;
  f_arm smooth Coulomb on the TCP displacement, -F_s tanh(dx / d_eps) per axis (the arm's stiction band, no box).
  Solved by damped Newton on the 6 pose coordinates (finite-difference Jacobian).
MPC: decision u_0..u_{H-1}; residuals over the rollout: contact normal force - F_ref, lateral force cap, peg / hole axis
  alignment, depth progress, setpoint rate. Levenberg-Marquardt, finite-difference Jacobian, warm start by shifting.
"""
from dataclasses import dataclass, field

import numpy as np


def rot(rx, ry, rz):
    cx, sx, cy, sy, cz, sz = np.cos(rx), np.sin(rx), np.cos(ry), np.sin(ry), np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


@dataclass
class HoleSpec:
    R: float = 0.0045                 # bore radius (round); ignored if half_widths is set
    half_widths: tuple = None         # (hx, hy) for a box bore
    depth: float = 0.025
    chamfer: float = 0.001
    rho: float = 0.0002               # edge rounding [m]
    block_half: float = 0.02


@dataclass
class PegSpec:
    a: float = 0.003993               # radius (round); ignored if half_widths is set
    half_widths: tuple = None
    pc: float = 0.0005
    tip_below_tcp: float = 0.040
    side_len: float = 0.010
    n_ring: int = 24


def hole_sdf_value(p, h: HoleSpec):
    """Gap of points p (n x 3, hole frame) to the solid block with the bore: > 0 in free space, < 0 inside the solid.
    Free space = above the top face (z > 0), or the bore cavity (|xy| < bore, z > -depth), or above the 45-degree
    chamfer cone; the gap is a smooth maximum (log-sum-exp, temperature rho) of the three regions' signed distances,
    which rounds the rims by ~rho and is C-infinity."""
    p = np.atleast_2d(p)
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    if h.half_widths is None:
        rb = np.hypot(x, y) - h.R                                  # > 0 outside the bore wall
    else:
        qx, qy = np.abs(x) - h.half_widths[0], np.abs(y) - h.half_widths[1]
        rb = np.where((qx > 0) & (qy > 0), np.hypot(qx, qy), np.maximum(qx, qy))
    terms = [z, -np.maximum(rb, -(z + h.depth))]                   # above the face; inside the cavity
    if h.chamfer > 0:
        terms.append((z + h.chamfer - rb) / np.sqrt(2.0))          # above the chamfer cone
    T = np.stack(terms, 1)
    m = T.max(1)
    return m + h.rho * np.log(np.exp((T - m[:, None]) / h.rho).sum(1))


def hole_sdf(p, h: HoleSpec, eps=1e-6):
    """-> gap (n,), unit normal = gradient direction (n x 3), central differences."""
    p = np.atleast_2d(p)
    d = hole_sdf_value(p, h)
    g = np.stack([(hole_sdf_value(p + e, h) - hole_sdf_value(p - e, h)) / (2 * eps) for e in np.eye(3) * eps], 1)
    return d, g / np.maximum(np.linalg.norm(g, axis=1, keepdims=True), 1e-12)


def peg_points(s: PegSpec):
    """Peg surface points in the TCP frame (z up the peg from the tip; the tip is tip_below_tcp below the TCP)."""
    ang = np.linspace(0, 2 * np.pi, s.n_ring, endpoint=False)
    if s.half_widths is None:
        ux, uy, ra, rb = np.cos(ang), np.sin(ang), s.a - s.pc, s.a
        rxy = lambda r: (r * ux, r * uy)  # noqa: E731
    else:
        m = np.maximum(np.abs(np.cos(ang)) / s.half_widths[0], np.abs(np.sin(ang)) / s.half_widths[1])
        ux, uy = np.cos(ang) / m, np.sin(ang) / m
        rxy = lambda r: (ux * r, uy * r)  # noqa: E731  (r = scale of the outline: 1 = the side)
        ra, rb = 1.0 - s.pc / min(s.half_widths), 1.0
    pts = []
    for r, z in [(ra, 0.0), (rb, s.pc), (0.5 * ra, 0.0)] + [(rb, zz) for zz in np.linspace(2 * s.pc, s.side_len, 5)]:
        X, Y = rxy(r)
        pts.append(np.column_stack([X, Y, np.full(len(X), z)]))
    pts.append([[0.0, 0.0, 0.0]])
    P = np.vstack(pts)
    P[:, 2] -= s.tip_below_tcp  # into the TCP frame (tip below the TCP)
    return P


@dataclass
class PlantParams:
    K: np.ndarray = field(default_factory=lambda: np.array([1500.0, 1500.0, 500.0, 130.0, 130.0, 45.0]))
    F_s: np.ndarray = field(default_factory=lambda: np.array([1.37, 0.86, 1.15, 1.24, 0.68, 0.3]))  # real-FR3 breakaway
    d_eps: np.ndarray = field(default_factory=lambda: np.array([5e-5, 5e-5, 5e-5, 5e-4, 5e-4, 5e-4]))
    k_c: float = 2e4      # contact stiffness [N/m] of the softplus normal force
    eps: float = 1e-5     # softplus width [m] (5e-5 let a flat face carry 2 N at a 0.16 mm gap over ~49 points)
    mu: float = 0.75
    v_eps: float = 2e-5   # friction regularization [m per step]


class SdfPlant:
    def __init__(self, hole: HoleSpec, peg: PegSpec, params: PlantParams = PlantParams()):
        self.h, self.s, self.prm = hole, peg, params
        self.P = peg_points(peg)

    def points(self, x):
        return x[:3] + self.P @ rot(*x[3:6]).T

    def contact(self, x, x_prev):
        """-> generalized contact force (6, about the TCP), per-point normal forces, gaps."""
        W = self.points(x)
        phi, n = hole_sdf(W, self.h)
        prm = self.prm
        fn = prm.k_c * prm.eps * np.logaddexp(0.0, -phi / prm.eps)
        dW = W - self.points(x_prev)
        vt = dW - (np.sum(dW * n, 1, keepdims=True)) * n
        ft = -prm.mu * fn[:, None] * vt / np.sqrt(np.sum(vt * vt, 1, keepdims=True) + prm.v_eps ** 2)
        f = fn[:, None] * n + ft
        r = W - x[:3]
        Fg = np.r_[f.sum(0), np.cross(r, f).sum(0)]
        return Fg, fn, phi

    def residual(self, x, u, x_prev):
        prm = self.prm
        d = u - x
        f_arm = -prm.F_s * np.tanh((x - x_prev) / prm.d_eps)
        Fg, _, _ = self.contact(x, x_prev)
        return prm.K * d + f_arm + Fg

    def step(self, x_prev, u, iters=20, tol=1e-6):
        """Quasi-static x+ by damped Newton from x_prev (finite-difference Jacobian of the 6 force residuals)."""
        x = x_prev.copy()
        S = 1.0 / self.prm.K  # scale: residual in displacement units
        for _ in range(iters):
            r = self.residual(x, u, x_prev) * S
            if np.linalg.norm(r) < tol:
                break
            J = np.zeros((6, 6))
            h = 1e-7
            for k in range(6):
                e = np.zeros(6); e[k] = h
                J[:, k] = (self.residual(x + e, u, x_prev) * S - r) / h
            dx = np.linalg.solve(J - 1e-9 * np.eye(6), -r)
            a = 1.0
            n0 = np.linalg.norm(r)
            while a > 1e-3 and np.linalg.norm(self.residual(x + a * dx, u, x_prev) * S) > n0:
                a *= 0.5
            x = x + a * dx
        return x


@dataclass
class MpcConfig:
    H: int = 6
    F_ref: float = 4.0         # target total normal contact force [N]
    F_lat_max: float = 3.0     # soft cap on the lateral contact force [N]
    w_force: float = 1.0
    w_lat: float = 1.0
    w_align: float = 300.0     # per mrad^2 / mm^2 scale handled below
    w_depth: float = 2.0
    w_rate: float = 20.0
    lm_iters: int = 4
    lm_lambda: float = 1e-2
    du_max: float = 0.002      # per-step setpoint change bound [m] (rotation: 0.02 rad), by clipping


class SdfMpc:
    """Levenberg-Marquardt over the setpoint sequence; hole estimate h_xy shifts the hole frame (the belief mean)."""

    def __init__(self, plant: SdfPlant, cfg: MpcConfig = MpcConfig()):
        self.plant, self.cfg = plant, cfg
        self.U = None

    def rollout(self, x0, U):
        xs, x = [], x0.copy()
        for u in U:
            x = self.plant.step(x, u)
            xs.append(x)
        return xs

    def residuals(self, x0, U, depth_goal):
        cfg, r = self.cfg, []
        xs = self.rollout(x0, U)
        xp, up = x0, x0
        for x, u in zip(xs, U):
            Fg, fn, _ = self.plant.contact(x, xp)
            r.append(np.sqrt(cfg.w_force) * (fn.sum() - cfg.F_ref) / cfg.F_ref)
            lat = np.linalg.norm(Fg[:2])
            r.append(np.sqrt(cfg.w_lat) * max(lat - cfg.F_lat_max, 0.0) / cfg.F_lat_max)
            r += list(np.sqrt(cfg.w_align) * np.r_[x[:2] / 1e-3 * 1e-3, x[3:5]])  # axis on the hole axis, upright
            r.append(np.sqrt(cfg.w_depth) * (x[2] - depth_goal) / 0.01)
            r += list(np.sqrt(cfg.w_rate) * (u - up) / np.array([1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2]) * 1e-2)
            xp, up = x, u
        return np.array(r)

    def solve(self, x0, depth_goal):
        cfg = self.cfg
        if self.U is None:
            self.U = np.tile(x0, (cfg.H, 1))
        else:  # warm start: shift
            self.U = np.vstack([self.U[1:], self.U[-1:]])
        U = self.U.copy()
        lam = cfg.lm_lambda
        r = self.residuals(x0, U, depth_goal)
        for _ in range(cfg.lm_iters):
            n = U.size
            J = np.zeros((len(r), n))
            h = 1e-6
            for k in range(n):
                Uk = U.copy().reshape(-1); Uk[k] += h
                J[:, k] = (self.residuals(x0, Uk.reshape(U.shape), depth_goal) - r) / h
            step = np.linalg.solve(J.T @ J + lam * np.eye(n), -J.T @ r).reshape(U.shape)
            lim = np.array([cfg.du_max] * 3 + [10 * cfg.du_max] * 3)
            step = np.clip(step, -lim, lim)
            r_new = self.residuals(x0, U + step, depth_goal)
            if r_new @ r_new < r @ r:
                U, r, lam = U + step, r_new, lam * 0.3
            else:
                lam *= 10.0
        self.U = U
        return U[0], {"cost": float(r @ r)}
