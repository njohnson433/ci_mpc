"""3D peg-in-hole contact model with an exact axisymmetric hole distance field (stage 3, step 1).

Geometry as peg_hole.PegHoleGeometry (Factory 8 mm peg / hole). The hole is a surface of revolution, so the signed
distance of a point only depends on (r, z): r = distance to the hole axis, z = height above the opening. Its 2D
profile (top face r >= R + c, 45-degree chamfer, bore r = R, bottom z = -depth) gives distance and normal in
closed form; the 3D normal is n_r * (x, y) / r + n_z * e_z.

State x = TCP pose (x, y, z, rx, ry) in the hole frame of the *model* (origin on the estimated hole axis in the
opening plane); the true axis is at `hole_xy`. rx, ry: small rotations about the world x / y axes at the TCP (the
peg is round, yaw does not matter). The peg hangs rigidly below the TCP (tip at tip_below_tcp along -z_tool).

Contacts (within `radius` of touching):
  - peg tip ring (r = a - pc, z = 0) and chamfer edge ring (r = a, z = pc), `n_ring` points each, vs the hole;
  - the hole's two rim circles (r = R + c at z = 0, r = R at z = -c), `n_rim` points each, vs the peg's side.
Each contact: gap phi, normal n (into the peg), tangents t1, t2, and the Jacobian G (3 x 5) of the peg point.

Quasi-static impedance x+ = u + C (W lam + f_arm) as in the planar model, friction pyramid with 4 generators.
  step()        offline plant (convex dual QP, Anitescu relaxation, + arm stiction box)
  likelihood()  static measurement model for localization: at the measured pose a hole position is consistent if
                nothing penetrates and the measured contact wrench is a non-negative combination of its contacts'
                friction-cone generators (NNLS), no dynamics needed.
"""
import numpy as np
from scipy.optimize import nnls

from ci_mpc_planner.peg_hole import PegHoleGeometry


def rotxy(rx, ry):
    cx, sx, cy, sy = np.cos(rx), np.sin(rx), np.cos(ry), np.sin(ry)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    return Ry @ Rx


class PegHole3D:
    def __init__(self, geo=PegHoleGeometry(), hole_xy=(0.0, 0.0), n_ring=12, n_rim=24):
        self.g = geo
        self.hole_xy = np.asarray(hole_xy, float)
        g = geo
        ang = np.linspace(0, 2 * np.pi, n_ring, endpoint=False)
        tip = np.column_stack([(g.a - g.pc) * np.cos(ang), (g.a - g.pc) * np.sin(ang), np.zeros(n_ring)])
        edge = np.column_stack([g.a * np.cos(ang), g.a * np.sin(ang), np.full(n_ring, g.pc)])
        self.peg_points = np.vstack([tip, edge, [[0.0, 0.0, 0.0]]])  # + tip center (bottom contact)
        ar = np.linspace(0, 2 * np.pi, n_rim, endpoint=False)
        R, c = g.R, g.c
        self.rim_points = np.vstack([np.column_stack([(R + c) * np.cos(ar), (R + c) * np.sin(ar), np.zeros(n_rim)]),
                                     np.column_stack([R * np.cos(ar), R * np.sin(ar), np.full(n_rim, -c)])])
        s2 = 1 / np.sqrt(2)
        W = g.top_half_width
        # profile segments in (r, z) with normals into free space
        self.profile = [
            (np.array([R + c, 0.0]), np.array([W, 0.0]), np.array([0.0, 1.0])),
            (np.array([R, -c]), np.array([R + c, 0.0]), np.array([-s2, s2])),
            (np.array([R, -g.depth]), np.array([R, -c]), np.array([-1.0, 0.0])),
            (np.array([0.0, -g.depth]), np.array([R, -g.depth]), np.array([0.0, 1.0])),
        ]
        self.tcp_offset = np.array([0.0, 0.0, g.tip_below_tcp])

    # --- kinematics (points given in the peg frame: tip center origin, axis +z)
    def world(self, x, p_local, hole_xy=None):
        """Point in the frame of the hole at hole_xy (default: the model's)."""
        h = self.hole_xy if hole_xy is None else hole_xy
        tcp = np.array([x[0] - h[0], x[1] - h[1], x[2]])
        return tcp + rotxy(x[3], x[4]) @ (np.asarray(p_local) - self.tcp_offset)

    def jacobian(self, x, p_local, eps=1e-6):
        J = np.zeros((3, 5))
        J[:, :3] = np.eye(3)
        for k in (3, 4):
            d = np.zeros(5)
            d[k] = eps
            J[:, k] = (self.world(x + d, p_local) - self.world(x - d, p_local)) / (2 * eps)
        return J

    def tip(self, x, hole_xy=None):
        return self.world(x, [0.0, 0.0, 0.0], hole_xy)

    # --- hole distance field
    def hole_sdf(self, p):
        """(signed distance, unit normal into free space) of a point p (hole frame) to the hole surface."""
        r = np.hypot(p[0], p[1])
        q = np.array([r, p[2]])
        best = None
        for p0, p1, n in self.profile:
            d = p1 - p0
            s = np.clip((q - p0) @ d / (d @ d), 0.0, 1.0)
            foot = p0 + s * d
            dist = (q - foot) @ n
            off = np.linalg.norm(q - foot - dist * n)
            if off > 1e-9:  # beyond the segment end: distance to its end point (convex edges)
                v = q - foot
                dist = np.linalg.norm(v) * (1.0 if v @ n > 0 else -1.0)
                nn = v / max(np.linalg.norm(v), 1e-12) if dist > 0 else n
            else:
                nn = n
            if best is None or abs(dist) < abs(best[0]):
                best = (dist, nn)
        dist, n2 = best
        radial = np.array([p[0], p[1]]) / r if r > 1e-9 else np.array([1.0, 0.0])
        return dist, np.array([n2[0] * radial[0], n2[0] * radial[1], n2[1]])

    @staticmethod
    def tangents(n):
        a = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        t1 = np.cross(n, a)
        t1 /= np.linalg.norm(t1)
        return t1, np.cross(n, t1)

    def contacts(self, x, radius=0.004, hole_xy=None):
        """List of (phi, n, t1, t2, p_local) within radius."""
        out = []
        for pl in self.peg_points:
            p = self.world(x, pl, hole_xy)
            phi, n = self.hole_sdf(p)
            if -0.002 < phi < radius:
                out.append((phi, n, *self.tangents(n), pl))
        # rim points vs the peg's side (cylinder of radius a from z_l = pc to length along the peg axis)
        h = self.hole_xy if hole_xy is None else hole_xy
        Rm = rotxy(x[3], x[4])
        base = self.world(x, [0, 0, 0], hole_xy)
        axis = Rm @ np.array([0.0, 0.0, 1.0])
        for hp in self.rim_points:
            v = hp - base
            z_l = v @ axis
            if not (self.g.pc <= z_l <= self.g.length):
                continue
            radial = v - z_l * axis
            rr = np.linalg.norm(radial)
            phi = rr - self.g.a
            if -0.002 < phi < radius and rr > 1e-9:
                n = -radial / rr  # force on the peg from the rim point points into the peg
                pl = np.linalg.solve(Rm, hp - (np.array([x[0] - h[0], x[1] - h[1], x[2]]))) + self.tcp_offset
                out.append((phi, n, *self.tangents(n), pl))
        return out

    def matrices(self, x, radius=0.004, hole_xy=None):
        """phi (m), N (m x 5), T1, T2 (m x 5), generalized force map W (5 x 5m) for lam = [ln, t1+, t1-, t2+, t2-]."""
        cs = self.contacts(x, radius, hole_xy)
        m = len(cs)
        phi = np.array([c[0] for c in cs])
        N, T1, T2 = (np.zeros((m, 5)) for _ in range(3))
        W = np.zeros((5, 5 * m))
        for i, (ph, n, t1, t2, pl) in enumerate(cs):
            G = self.jacobian(x, pl)
            N[i], T1[i], T2[i] = n @ G, t1 @ G, t2 @ G
            W[:, 5 * i:5 * i + 5] = np.column_stack([N[i], T1[i], -T1[i], T2[i], -T2[i]])
        return phi, N, T1, T2, W

    # --- offline plant
    def step(self, x, u, compliance, arm_friction=None):
        import osqp
        import scipy.sparse as sp

        C = np.diag(compliance)
        phi, N, T1, T2, W = self.matrices(x)
        m = len(phi)
        du = np.asarray(u, float) - np.asarray(x, float)
        fa = np.zeros(5) if arm_friction is None else np.asarray(arm_friction, float)
        if m == 0 and not fa.any():
            return np.asarray(u, float).copy(), np.zeros(0), 0
        Wf = np.hstack([W, np.eye(5)])
        sc = 1000.0
        nvar = 5 * m + 5
        A = sc * (Wf.T @ C @ Wf) + 1e-9 * np.eye(nvar)
        b = np.zeros(nvar)
        b[0:5 * m:5] = sc * (phi + N @ du)
        b[1:5 * m:5], b[2:5 * m:5] = sc * (T1 @ du), -sc * (T1 @ du)
        b[3:5 * m:5], b[4:5 * m:5] = sc * (T2 @ du), -sc * (T2 @ du)
        b[5 * m:] = sc * du
        G = np.zeros((6 * m + 5, nvar))
        lo, hi = np.zeros(6 * m + 5), np.full(6 * m + 5, np.inf)
        G[:5 * m, :5 * m] = np.eye(5 * m)
        for i in range(m):
            G[5 * m + i, 5 * i:5 * i + 5] = [self.g.mu, -1, -1, -1, -1]
        G[6 * m:, 5 * m:] = np.eye(5)
        lo[6 * m:], hi[6 * m:] = -fa, fa
        solver = osqp.OSQP()
        solver.setup(sp.csc_matrix(np.triu(A)), b, sp.csc_matrix(G), lo, hi, verbose=False, eps_abs=1e-9,
                     eps_rel=1e-9, polish=True, max_iter=20000)
        res = solver.solve(raise_error=False)
        z = res.x if res.x is not None else np.zeros(nvar)
        lam = np.maximum(z[:5 * m], 0.0)
        self._last_W = W
        return np.asarray(u, float) + C @ (W @ lam + z[5 * m:]), lam, m

    def contact_wrench(self, lam):
        """Generalized contact force (Fx, Fy, Fz, Mrx, Mry at the TCP) of the last step."""
        return self._last_W @ lam if len(lam) else np.zeros(5)

    # --- static measurement model for localization
    def likelihood(self, x, wrench, hole_xy, sigma_pen=3e-4, sigma_w=(0.6, 0.6, 0.6, 0.03, 0.03), touch=4e-4,
                   min_force=0.8):
        """log-likelihood of the hole at hole_xy given the measured TCP pose x and contact wrench (5)."""
        cs = self.contacts(x, radius=0.0015, hole_xy=hole_xy)
        pen = sum(min(c[0], 0.0) ** 2 for c in cs)
        ll = -pen / sigma_pen ** 2
        sw = np.asarray(sigma_w)
        if np.linalg.norm(wrench[:3]) < min_force:  # no contact measured: nothing may touch
            ll -= sum(1.0 for c in cs if c[0] < touch * 0.5) * 2.0
            return ll
        # wrench must be a non-negative combination of the touching contacts' friction-cone generators
        gens = []
        for phi, n, t1, t2, pl in cs:
            if phi < touch:
                G = self.jacobian(x, pl)
                for t in (t1, -t1, t2, -t2):
                    gens.append(G.T @ (n + self.g.mu * t))
        if not gens:
            return ll - 0.5 * np.sum((np.asarray(wrench) / sw) ** 2)
        A = np.array(gens).T / sw[:, None]
        _, res = nnls(A, np.asarray(wrench) / sw)
        return ll - 0.5 * res ** 2
