"""Planar pusher-slider with a sticking point contact (stage 2: contact in the model, mode fixed).

Slider: box of face length `a` on a table, quasi-static, ellipsoidal limit surface
    H(w) = (f_x^2 + f_y^2) / f_max^2 + m^2 / m_max^2,   c = m_max / f_max  (~0.38 a for uniform pressure on a
    square, a / sqrt(2) for four corner contacts),
slider body twist V = [v_x, v_y, omega] parallel to grad H. The pusher is a point on the face x = -a/2 at
y = p_y (slider frame), pushing in +x. While the contact sticks, the pusher velocity u = [v_n, v_t] (world
velocity in the slider frame) determines the slider motion (Lynch, Maekawa, Tanie 1992):
    V = M(p_y) u,  M = 1/(c^2 + x_c^2 + y_c^2) [[c^2 + x_c^2, x_c y_c], [x_c y_c, c^2 + y_c^2], [-y_c, x_c]]
with (x_c, y_c) = (-a/2, p_y). It sticks while u is inside the motion cone gamma_lo <= v_t / v_n <= gamma_hi
(the pusher velocities that need a force inside the pusher friction cone |f_t| <= mu_p f_n). Outside it the
pusher slides along the face (`step` models that, the MPC does not: that is stage 3).

World state s = [x, y, theta]: ds/dt = [R(theta) V_xy; omega].

PushingMpc: linear time-varying MPC around a reference slider trajectory (sticking at a fixed p_y),
OSQP. Variables ds_k = s_k - s_ref_k, du_k = u_k - u_ref_k; constraints: linearized dynamics, v_n in
[v_n_min, v_n_max] (v_n_min > 0 keeps the contact), motion cone shrunk by `cone_margin`.
"""
from dataclasses import dataclass

import numpy as np
import osqp
import scipy.sparse as sp


def rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


@dataclass
class SliderParams:
    a: float = 0.08        # face length [m] (square slider; the pusher is on the face x = -a/2)
    c: float = 0.045       # m_max / f_max [m]; fitted from Isaac pushes (0.040-0.048; 0.03 = uniform pressure
                           # guess: cone too wide -> contact slid, arc 29 mm RMS)
    mu_p: float = 0.3      # pusher-slider friction
    f_max: float = 1.5     # mu_g m g [N]: friction force at pure translation (feedforward only)


class PusherSlider:
    def __init__(self, params: SliderParams):
        self.p = params

    def M(self, py):
        xc, yc, c2 = -self.p.a / 2, py, self.p.c ** 2
        return np.array([[c2 + xc ** 2, xc * yc], [xc * yc, c2 + yc ** 2], [-yc, xc]]) / (c2 + xc ** 2 + yc ** 2)

    def contact_velocity(self, V, py):
        """Velocity of the slider's material point at the contact (slider frame)."""
        xc, yc = -self.p.a / 2, py
        return np.array([V[0] - V[2] * yc, V[1] + V[2] * xc])

    def twist_from_force(self, f, py):
        """Slider twist direction (unscaled) for a contact force f = [f_n, f_t] (slider frame)."""
        xc, yc = -self.p.a / 2, py
        m = xc * f[1] - yc * f[0]
        return np.array([f[0], f[1], m / self.p.c ** 2])

    def motion_cone(self, py):
        """(gamma_lo, gamma_hi): v_t / v_n of the cone edges (force on the friction cone edges)."""
        g = []
        for s in (-1.0, 1.0):
            vc = self.contact_velocity(self.twist_from_force(np.array([1.0, s * self.p.mu_p]), py), py)
            g.append(vc[1] / vc[0])
        return min(g), max(g)

    def force(self, V, py):
        """Quasi-static contact force [f_n, f_t] (slider frame) that moves the slider with twist V."""
        if np.linalg.norm(V) < 1e-9:
            return np.zeros(2)
        w = np.array([V[0], V[1], self.p.c ** 2 * V[2]])  # proportional to the wrench (grad H inverse)
        h = (w[0] ** 2 + w[1] ** 2) + (w[2] / self.p.c) ** 2  # H(w) with f_max = 1
        return self.p.f_max * w[:2] / np.sqrt(h)

    def step(self, s, py, u, dt):
        """Simulate one step of the true contact (sticking, sliding along the face, or separation).
        Returns new state, new p_y, mode ('stick', 'slide_up', 'slide_down', 'free')."""
        if u[0] <= 1e-5:
            return s.copy(), py, "free"
        lo, hi = self.motion_cone(py)
        ratio = u[1] / u[0]
        if lo <= ratio <= hi:
            V, mode = self.M(py) @ u, "stick"
        else:
            sgn = 1.0 if ratio > hi else -1.0
            Vd = self.twist_from_force(np.array([1.0, sgn * self.p.mu_p]), py)
            vc = self.contact_velocity(Vd, py)
            V = Vd * (u[0] / vc[0])  # normal velocity of the contact point matches the pusher's
            mode = "slide_up" if sgn > 0 else "slide_down"
        vc = self.contact_velocity(V, py)
        py_new = py + (u[1] - vc[1]) * dt  # pusher slides along the face
        s_new = s + dt * np.concatenate([rot(s[2]) @ V[:2], [V[2]]])
        return s_new, py_new, mode

    def dynamics(self, s, u, py):
        V = self.M(py) @ u
        return np.concatenate([rot(s[2]) @ V[:2], [V[2]]])

    def linearize(self, s, u, py):
        """Continuous-time A = df/ds, B = df/du at (s, u)."""
        V = self.M(py) @ u
        dR = np.array([[-np.sin(s[2]), -np.cos(s[2])], [np.cos(s[2]), -np.sin(s[2])]])
        A = np.zeros((3, 3))
        A[:2, 2] = dR @ V[:2]
        B = np.vstack([rot(s[2]) @ self.M(py)[:2], self.M(py)[2:3]])
        return A, B

    def contact_for_curvature(self, kappa, margin=0.01):
        """Contact position p_y for pushing along a path of curvature kappa [1/m] (+ = left) while sticking: the
        one whose nominal input (exact twist [v, 0, v kappa]) sits in the middle of the motion cone. Left turns
        need the contact right of the center (p_y < 0)."""
        best = None
        for py in np.linspace(-self.p.a / 2 + margin, self.p.a / 2 - margin, 121):
            V = np.array([1.0, 0.0, kappa])
            u, *_ = np.linalg.lstsq(self.M(py), V, rcond=None)
            res = np.linalg.norm((self.M(py) @ u - V) * [1.0, 1.0, self.p.a])
            lo, hi = self.motion_cone(py)
            mid = abs((u[1] / u[0] - lo) / (hi - lo) - 0.5) if u[0] > 0 else np.inf
            cost = mid + 10.0 * res
            if best is None or cost < best[0]:
                best = (cost, py)
        return best[1]

    def nominal_input(self, s, ds, py):
        """Least-squares pusher velocity for a desired world state rate ds (sticking at p_y)."""
        V = np.concatenate([rot(s[2]).T @ ds[:2], [ds[2]]])
        W = np.diag([1.0, 1.0, self.p.a])  # angular rate in m/s at the face
        u, *_ = np.linalg.lstsq(W @ self.M(py), W @ V, rcond=None)
        return u


@dataclass
class PushingMpcConfig:
    dt: float = 0.1
    horizon: int = 50          # 5 s: steering by heading acts slowly (30: weaker correction of the 10 deg start)
    q_pos: float = 1000.0
    q_theta: float = 3.0       # 0.1: Isaac yaw drifted 12 deg (cheap lateral slip now, heading drift later);
                               # 10: offline 3-5 mm lateral; sweep 2026-10-23 with c 0.045, 30 ms latency
    terminal: float = 10.0
    r_u: float = 1.0
    r_du: float = 10.0
    v_n_min: float = 0.002
    v_n_max: float = 0.08
    v_t_max: float = 0.08
    cone_margin: float = 0.7   # use this fraction of the motion cone (friction is uncertain)


class PushingMpc:
    def __init__(self, model: PusherSlider, cfg: PushingMpcConfig):
        self.model, self.cfg = model, cfg
        self.last_u = None

    def solve(self, s0, py, s_ref, u_ref):
        """s0: measured state (3), py: contact position, s_ref: (N+1, 3), u_ref: (N, 2).
        Returns (U (N, 2) planned pusher velocities, S (N+1, 3) predicted states, status)."""
        cfg, N, dt = self.cfg, self.cfg.horizon, self.cfg.dt
        nx, nu = 3, 2
        n_s, n_u = nx * (N + 1), nu * N
        # linearized dynamics around the reference (theta from the reference)
        Ads, Bds, cs = [], [], []
        for k in range(N):
            A, B = self.model.linearize(s_ref[k], u_ref[k], py)
            Ad, Bd = np.eye(3) + dt * A, dt * B
            # affine term: reference not exactly dynamically consistent
            f = s_ref[k] + dt * self.model.dynamics(s_ref[k], u_ref[k], py) - s_ref[k + 1]
            f[2] = wrap(f[2])
            Ads.append(Ad), Bds.append(Bd), cs.append(f)
        # cost
        q = np.array([cfg.q_pos, cfg.q_pos, cfg.q_theta])
        Pq = [np.zeros(3)] + [q] * (N - 1) + [cfg.terminal * q]
        D = (sp.eye(N) * 2 - sp.eye(N, k=-1) - sp.eye(N, k=1)).tolil()
        D[N - 1, N - 1] = 1.0
        D[0, 0] = 2.0  # du_0 - du_{-1} below (previous plan)
        P = sp.block_diag([sp.diags(np.concatenate(Pq)),
                           sp.kron(sp.eye(N) * cfg.r_u + cfg.r_du * D.tocsc(), sp.eye(nu))], format="csc")
        qv = np.zeros(n_s + n_u)
        if self.last_u is not None:  # smoothness against the previous first input
            qv[n_s:n_s + nu] = -cfg.r_du * (self.last_u - u_ref[0])
        # equality constraints
        rows, cols, vals = [], [], []
        A_eq = sp.lil_matrix((n_s, n_s + n_u))
        b_eq = np.zeros(n_s)
        A_eq[0:3, 0:3] = -np.eye(3)
        e0 = s0 - s_ref[0]
        e0[2] = wrap(e0[2])
        b_eq[0:3] = -e0
        for k in range(N):
            r = 3 * (k + 1)
            A_eq[r:r + 3, 3 * k:3 * k + 3] = Ads[k]
            A_eq[r:r + 3, 3 * (k + 1):3 * (k + 1) + 3] = -np.eye(3)
            A_eq[r:r + 3, n_s + 2 * k:n_s + 2 * k + 2] = Bds[k]
            b_eq[r:r + 3] = -cs[k]
        # inequalities on absolute inputs u = u_ref + du
        lo, hi = self.model.motion_cone(py)
        lo, hi = cfg.cone_margin * lo, cfg.cone_margin * hi
        G = sp.lil_matrix((4 * N, n_s + n_u))
        gl, gu = np.zeros(4 * N), np.zeros(4 * N)
        for k in range(N):
            c = n_s + 2 * k
            ur = u_ref[k]
            G[4 * k, c] = 1.0  # v_n
            v_min = min(cfg.v_n_min, ur[0])  # keep pushing while the reference moves, may stop at its end
            gl[4 * k], gu[4 * k] = v_min - ur[0], cfg.v_n_max - ur[0]
            G[4 * k + 1, c + 1] = 1.0  # v_t
            gl[4 * k + 1], gu[4 * k + 1] = -cfg.v_t_max - ur[1], cfg.v_t_max - ur[1]
            G[4 * k + 2, c + 1], G[4 * k + 2, c] = 1.0, -hi  # v_t - hi v_n <= 0
            gl[4 * k + 2], gu[4 * k + 2] = -np.inf, -(ur[1] - hi * ur[0])
            G[4 * k + 3, c + 1], G[4 * k + 3, c] = -1.0, lo  # lo v_n - v_t <= 0
            gl[4 * k + 3], gu[4 * k + 3] = -np.inf, -(lo * ur[0] - ur[1])
        Acon = sp.vstack([A_eq.tocsc(), G.tocsc()], format="csc")
        l = np.concatenate([b_eq, gl])
        u = np.concatenate([b_eq, gu])
        solver = osqp.OSQP()
        solver.setup(P, qv, Acon, l, u, verbose=False, eps_abs=1e-5, eps_rel=1e-5, polish=True,
                     max_iter=4000, time_limit=0.02)
        res = solver.solve(raise_error=False)
        status = res.info.status
        if res.x is None or res.info.status_val not in (1, 2):
            return None, None, status
        dS = res.x[:n_s].reshape(N + 1, 3)
        dU = res.x[n_s:].reshape(N, 2)
        U = u_ref + dU
        S = s_ref + dS
        self.last_u = U[0].copy()
        return U, S, status


def line_reference(s_start, direction, speed, length, t, horizon, dt):
    """Straight-line slider reference: from s_start[:2] along `direction` (unit, world) at `speed`, theta
    pointing along the line, until `length`. Returns s_ref (N+1, 3), ds_ref (N+1, 3) from time t."""
    ts = t + dt * np.arange(horizon + 1)
    d = np.asarray(direction, float) / np.linalg.norm(direction)
    s_along = np.minimum(speed * ts, length)
    th = np.arctan2(d[1], d[0])
    s_ref = np.column_stack([s_start[0] + d[0] * s_along, s_start[1] + d[1] * s_along, np.full(len(ts), th)])
    moving = (speed * ts < length).astype(float)
    ds_ref = np.column_stack([d[0] * speed * moving, d[1] * speed * moving, np.zeros(len(ts))])
    return s_ref, ds_ref


def arc_reference(s_start, theta0, radius, speed, angle, t, horizon, dt):
    """Arc slider reference: starts at s_start[:2] heading theta0, turns by `angle` (sign = left/right) on a
    circle of `radius`, then continues straight."""
    ts = t + dt * np.arange(horizon + 1)
    sgn = np.sign(angle)
    L_arc = radius * abs(angle)
    out, dout = [], []
    center = np.asarray(s_start[:2]) + radius * sgn * np.array([-np.sin(theta0), np.cos(theta0)])
    for tk in ts:
        s = speed * tk
        if s <= L_arc:
            phi = s / radius
            th = theta0 + sgn * phi
            p = center + radius * sgn * np.array([np.sin(th), -np.cos(th)])
            out.append([p[0], p[1], th])
            dout.append([speed * np.cos(th), speed * np.sin(th), sgn * speed / radius])
        else:
            th = theta0 + angle
            pe = center + radius * sgn * np.array([np.sin(th), -np.cos(th)])
            p = pe + (s - L_arc) * np.array([np.cos(th), np.sin(th)])
            out.append([p[0], p[1], th])
            dout.append([speed * np.cos(th), speed * np.sin(th), 0.0])
    return np.array(out), np.array(dout)
