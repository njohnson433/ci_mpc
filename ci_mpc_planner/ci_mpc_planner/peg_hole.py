"""Planar peg-in-hole contact model (x-z plane, tilt about y) for contact-implicit MPC (stage 3).

Geometry (Factory 8 mm peg / hole, metres), hole frame: x across the hole axis, z up, z = 0 at the opening (top
face), the hole axis at x = hole_x (unknown to the controller up to the perception error):
  hole: radius R = 4.5 mm, 1 mm 45-degree chamfer at the opening, depth 25 mm, top face beyond |x| > R + c.
  peg:  half width a = 3.993 mm, 0.5 mm chamfer at the tip; frame at the tip center, axis along +z_peg.
State x = TCP pose (x, z, phi) in the plane; the peg hangs rigidly below the TCP (tip at tip_below_tcp along the
tool axis, which is -z_peg), phi = rotation about +y (positive tilts the tool axis... see tip()).

Contacts (point vs segment, recomputed around the current state):
  - peg vertices (2 tip corners, 2 chamfer-side corners) vs hole segments (top faces, chamfers, walls, bottom);
  - hole vertices (chamfer top edges, wall top edges) vs the peg's long sides.
Each contact: signed distance phi_i, unit normal n_i (pointing from the hole into the peg), tangent t_i, and the
Jacobian G_i (2 x 3) of the peg material point at the contact w.r.t. the TCP pose.

Quasi-static impedance model (the arm is a spring around the commanded pose u, its inertia neglected at mm/s):
  x+ = u + C W(x) lambda,     C = diag(1/kx, 1/kz, 1/kphi) the controller compliance,
  W = [G_i^T n_i, G_i^T t_i, -G_i^T t_i] per contact (generalized force of lambda = [ln, lt+, lt-]),
  0 <= ln  _|_ phi + n^T G (x+ - x) >= 0,
  0 <= lt+ _|_ gamma + t^T G (x+ - x) >= 0,  0 <= lt- _|_ gamma - t^T G (x+ - x) >= 0,
  0 <= gamma _|_ mu ln - lt+ - lt- >= 0          (Stewart-Trinkle, displacement form).
`step()` is the offline plant (convex relaxation of this LCP, see there); the MPC uses the LCP itself.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class PegHoleGeometry:
    R: float = 0.0045            # hole radius
    c: float = 0.001             # hole chamfer (45 deg)
    depth: float = 0.025
    a: float = 0.003993          # peg half width
    pc: float = 0.0005           # peg tip chamfer
    length: float = 0.050
    tip_below_tcp: float = 0.040
    top_half_width: float = 0.02  # hole block half width
    mu: float = 0.75


def rot(phi):
    c, s = np.cos(phi), np.sin(phi)
    # rotation about +y in the x-z plane: (x, z) -> (c x + s z, -s x + c z)
    return np.array([[c, s], [-s, c]])


def drot(phi):
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[-s, c], [-c, -s]])


class PegHole:
    def __init__(self, geo=PegHoleGeometry(), hole_x=0.0):
        self.g = geo
        self.hole_x = hole_x
        g = geo
        # peg points in the peg frame (tip center origin, axis +z), and the TCP at +tip_below_tcp along the axis
        self.peg_vertices = np.array([[-(g.a - g.pc), 0.0], [g.a - g.pc, 0.0], [-g.a, g.pc], [g.a, g.pc]])
        # hole segments (p0, p1) with the free side on the left of p0 -> p1 ... stored with explicit normals
        R, c, D, W = g.R, g.c, g.depth, g.top_half_width
        s2 = 1 / np.sqrt(2)
        self.segments = [  # (p0, p1, normal into free space)
            (np.array([-W, 0.0]), np.array([-(R + c), 0.0]), np.array([0.0, 1.0])),
            (np.array([-(R + c), 0.0]), np.array([-R, -c]), np.array([s2, s2])),
            (np.array([-R, -c]), np.array([-R, -D]), np.array([1.0, 0.0])),
            (np.array([-R, -D]), np.array([R, -D]), np.array([0.0, 1.0])),
            (np.array([R, -D]), np.array([R, -c]), np.array([-1.0, 0.0])),
            (np.array([R, -c]), np.array([R + c, 0.0]), np.array([-s2, s2])),
            (np.array([R + c, 0.0]), np.array([W, 0.0]), np.array([0.0, 1.0])),
        ]
        self.hole_vertices = np.array([[-(R + c), 0.0], [-R, -c], [R, -c], [R + c, 0.0]])

    # --- kinematics
    def peg_point(self, x, p_local):
        """World (hole frame, hole axis at hole_x) position of a peg point given in the peg frame."""
        tcp = np.array([x[0] - self.hole_x, x[1]])
        return tcp + rot(x[2]) @ (np.asarray(p_local) - np.array([0.0, self.g.tip_below_tcp]))

    def point_jacobian(self, x, p_local):
        return np.column_stack([[1.0, 0.0], [0.0, 1.0],
                                drot(x[2]) @ (np.asarray(p_local) - np.array([0.0, self.g.tip_below_tcp]))])

    def tip(self, x):
        return self.peg_point(x, [0.0, 0.0])

    # --- contacts
    def contacts(self, x, radius=0.004):
        """Candidate contacts within `radius` of touching: list of (phi, n, t, G)."""
        out = []
        for v in self.peg_vertices:
            p = self.peg_point(x, v)
            best = None
            for p0, p1, n in self.segments:
                d = p1 - p0
                s = np.clip((p - p0) @ d / (d @ d), 0.0, 1.0)
                q = p0 + s * d
                dist = (p - q) @ n  # signed distance along the segment normal (negative = penetration)
                lateral = np.linalg.norm(p - q - dist * n)
                if lateral > 1e-9 or dist < -0.002:  # beyond the segment end or behind it
                    continue
                if best is None or abs(dist) < abs(best[0]):
                    best = (dist, n)
            if best is None:  # above a convex hole edge: closest feature is the edge vertex itself
                for hv in self.hole_vertices:
                    d = p - hv
                    if np.linalg.norm(d) < radius and d[1] > 0:
                        dn = np.linalg.norm(d)
                        if best is None or dn < best[0]:
                            best = (dn, d / max(dn, 1e-12))
            if best is not None and best[0] < radius:
                n = best[1]
                out.append((best[0], n, np.array([n[1], -n[0]]), self.point_jacobian(x, v)))
        # hole vertices vs the peg's long sides
        R2 = rot(x[2])
        for hv in self.hole_vertices:
            for side in (-1.0, 1.0):
                # peg side line: points (side * a, z_l), z_l in [pc, length]; outward normal (side, 0) in the peg frame
                n_peg = R2 @ np.array([side, 0.0])
                base = self.peg_point(x, [side * self.g.a, 0.0])
                axis = R2 @ np.array([0.0, 1.0])
                z_l = (hv - base) @ axis
                if not (self.g.pc <= z_l <= self.g.length):
                    continue
                dist = (hv - base) @ n_peg  # > 0: the hole vertex is outside the peg side
                if -0.002 < dist < radius:
                    # force on the peg points from the vertex into the peg: -n_peg
                    n = -n_peg
                    p_local = np.array([side * self.g.a, z_l])
                    out.append((dist, n, np.array([n[1], -n[0]]), self.point_jacobian(x, p_local)))
        return out

    def matrices(self, x, compliance, radius=0.004):
        """Contact quantities at x: phi (m), N (m x 3) = n^T G, T (m x 3) = t^T G, W (3 x 3m)."""
        cs = self.contacts(x, radius)
        m = len(cs)
        phi = np.array([c[0] for c in cs])
        N = np.array([c[1] @ c[3] for c in cs]).reshape(m, 3)
        T = np.array([c[2] @ c[3] for c in cs]).reshape(m, 3)
        W = np.zeros((3, 3 * m))
        for i in range(m):
            W[:, 3 * i] = N[i]
            W[:, 3 * i + 1] = T[i]
            W[:, 3 * i + 2] = -T[i]
        return phi, N, T, W

    # --- quasi-static step (offline plant): convex contact model (Anitescu relaxation)
    def step(self, x, u, compliance, arm_friction=None):
        """One quasi-static step from x with commanded pose u: contact forces from the convex dual QP
            min_l 1/2 l^T (W^T C W) l + l^T b,   b = [phi_i + N_i (u - x), +-T_i (u - x)],
            s.t. ln >= 0, lt+- >= 0, lt+ + lt- <= mu ln
        whose KKT conditions are non-penetration with Coulomb friction (sliding contacts separate by mu * slip;
        negligible at quasi-static speeds). Always solvable (Lemke on the Stewart-Trinkle LCP was not).
        arm_friction (Fx, Fz, torque): the arm's static friction in TCP coordinates (box friction: the TCP does not
        move while the spring force stays below it; joint stiction of the FR3, which the velocity-based friction
        compensation does not remove at rest).
        Returns (x+, lambda, number of contacts)."""
        import osqp
        import scipy.sparse as sp

        C = np.diag(compliance)
        phi, N, T, W = self.matrices(x, compliance)
        m = len(phi)
        du = np.asarray(u, float) - np.asarray(x, float)
        fa = np.zeros(3) if arm_friction is None else np.asarray(arm_friction, float)
        if m == 0 and not fa.any():
            return np.asarray(u, float).copy(), np.zeros(0), 0
        # variables [lambda (3m), f_arm (3)]; f_arm acts on the TCP like a contact with T = identity, |f| <= fa
        Wf = np.hstack([W, np.eye(3)])
        sc = 1000.0  # displacements in mm (conditioning)
        A = sc * (Wf.T @ C @ Wf) + 1e-9 * np.eye(3 * m + 3)
        b = np.zeros(3 * m + 3)
        b[0:3 * m:3] = sc * (phi + N @ du)
        b[1:3 * m:3] = sc * (T @ du)
        b[2:3 * m:3] = -sc * (T @ du)
        b[3 * m:] = sc * du
        G = np.zeros((4 * m + 3, 3 * m + 3))
        lo, hi = np.zeros(4 * m + 3), np.full(4 * m + 3, np.inf)
        G[:3 * m, :3 * m] = np.eye(3 * m)
        for i in range(m):
            G[3 * m + i, 3 * i:3 * i + 3] = [self.g.mu, -1.0, -1.0]
        G[4 * m:, 3 * m:] = np.eye(3)
        lo[4 * m:], hi[4 * m:] = -fa, fa
        solver = osqp.OSQP()
        solver.setup(sp.csc_matrix(np.triu(A)), b, sp.csc_matrix(G), lo, hi, verbose=False, eps_abs=1e-9,
                     eps_rel=1e-9, polishing=True, max_iter=20000)
        res = solver.solve(raise_error=False)
        z = res.x if res.x is not None else np.zeros(3 * m + 3)
        lam = np.maximum(z[:3 * m], 0.0)
        return np.asarray(u, float) + C @ (W @ lam + z[3 * m:]), lam, m

    def contact_wrench(self, x, lam, compliance):
        """Generalized force (Fx, Fz, torque about y at the TCP) the contacts apply to the peg."""
        _, _, _, W = self.matrices(x, compliance)
        return W @ lam if len(lam) else np.zeros(3)


