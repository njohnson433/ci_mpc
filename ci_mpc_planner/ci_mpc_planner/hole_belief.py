"""Grid belief over the 2D hole position from contact, and an information-driven probing search (stage 3, step 1).

Finding that motivates it (3D model, peg_hole_3d.py): a peg resting flat on the top face only rules out holes
directly under its tip; a lateral push from the round chamfer only exists while the tip overlaps it (offset up to
~2.5 mm). Localization beyond that needs active probing, so the belief chooses where to probe next.

Belief: log-weights on a grid of hole positions (model frame, around the perception estimate). Measurement model at
the measured TCP pose (vectorized over the grid, exact axisymmetric hole distance field):
  - no penetration of the peg points (tip ring, chamfer edge ring, tip center) into the hole;
  - contact force measured (|F| > f_contact): some peg point must touch (gap < touch); its lateral component must be
    possible: if all touching normals are vertical (flat top face, bottom), the lateral force must be small;
    if the measured lateral force is clear, the touching normals must have a lateral component that way;
  - no force: nothing may touch.
Probing: the next probe goes where the probability of the hole lying within the capture radius is highest.
"""
import numpy as np

from ci_mpc_planner.peg_hole import PegHoleGeometry
from ci_mpc_planner.peg_hole_3d import rotxy


def profile_sdf(r, z, g):
    """Signed distance (free space > 0) and normal (n_r, n_z) of points (r, z) to the hole profile, vectorized."""
    R, c, D, W = g.R, g.c, g.depth, g.top_half_width
    s2 = 1 / np.sqrt(2)
    segs = [((R + c, 0.0), (W, 0.0), (0.0, 1.0)), ((R, -c), (R + c, 0.0), (-s2, s2)),
            ((R, -D), (R, -c), (-1.0, 0.0)), ((0.0, -D), (R, -D), (0.0, 1.0))]
    best_d = np.full(r.shape, np.inf)
    best_nr = np.zeros(r.shape)
    best_nz = np.ones(r.shape)
    for p0, p1, n in segs:
        p0, p1, n = np.array(p0), np.array(p1), np.array(n)
        d = p1 - p0
        s = np.clip(((r - p0[0]) * d[0] + (z - p0[1]) * d[1]) / (d @ d), 0.0, 1.0)
        fr, fz = p0[0] + s * d[0], p0[1] + s * d[1]
        vr, vz = r - fr, z - fz
        dist_n = vr * n[0] + vz * n[1]
        off = np.hypot(vr - dist_n * n[0], vz - dist_n * n[1])
        norm = np.maximum(np.hypot(vr, vz), 1e-12)
        end = off > 1e-9
        dist = np.where(end, np.sign(dist_n + 1e-15) * norm, dist_n)
        nr = np.where(end & (dist > 0), vr / norm, n[0])
        nz = np.where(end & (dist > 0), vz / norm, n[1])
        take = np.abs(dist) < np.abs(best_d)
        best_d = np.where(take, dist, best_d)
        best_nr = np.where(take, nr, best_nr)
        best_nz = np.where(take, nz, best_nz)
    return best_d, best_nr, best_nz


class HoleBelief:
    def __init__(self, geo=PegHoleGeometry(), spread=0.006, res=0.0005, n_ring=24, touch=4e-4, f_contact=0.8,
                 pen_tol=3e-4,
                 f_lateral=1.0, capture=0.002):  # f_lateral ~3 sigma of the wrench estimate (0.5: noise read as a push)
        self.g = geo
        ax = np.arange(-spread, spread + 1e-9, res)
        gx, gy = np.meshgrid(ax, ax)
        self.h = np.column_stack([gx.ravel(), gy.ravel()])
        self.logw = np.zeros(len(self.h))
        ang = np.linspace(0, 2 * np.pi, n_ring, endpoint=False)
        tip = np.column_stack([(geo.a - geo.pc) * np.cos(ang), (geo.a - geo.pc) * np.sin(ang), np.zeros(n_ring)])
        edge = np.column_stack([geo.a * np.cos(ang), geo.a * np.sin(ang), np.full(n_ring, geo.pc)])
        self.peg_points = np.vstack([tip, edge, [[0, 0, 0]]])
        self.touch, self.f_contact, self.f_lateral, self.capture = touch, f_contact, f_lateral, capture
        # penetration tolerance: the measured tip height is only good to ~0.1-0.3 mm (grasp offset calibration,
        # estimate); without it a tip 0.07 mm "inside" the top face favoured holes next to every probe
        self.pen_tol = pen_tol

    def points_world(self, x):
        """Peg points in the model frame for TCP pose x = (x, y, z, rx, ry)."""
        Rm = rotxy(x[3], x[4])
        return x[:3] + (self.peg_points - np.array([0, 0, self.g.tip_below_tcp])) @ Rm.T

    def gaps(self, x):
        """Gap (H x P) and normals (H x P x 3) of every peg point for every hole hypothesis."""
        p = self.points_world(x)  # P x 3
        dx = p[None, :, 0] - self.h[:, None, 0]
        dy = p[None, :, 1] - self.h[:, None, 1]
        r = np.hypot(dx, dy)
        z = np.broadcast_to(p[None, :, 2], r.shape)
        d, nr, nz = profile_sdf(r, z, self.g)
        rr = np.maximum(r, 1e-12)
        return d, np.stack([nr * dx / rr, nr * dy / rr, nz], axis=-1)

    def update(self, x, force):
        """x: measured TCP pose (model frame), force: measured contact force on the peg (3, world)."""
        d, n = self.gaps(x)
        ll = -np.sum(np.minimum(d + self.pen_tol, 0.0) ** 2, axis=1) / (3e-4) ** 2  # penetration beyond tolerance
        touching = d < self.touch + self.pen_tol
        any_touch = touching.any(axis=1)
        F = np.asarray(force, float)
        if np.linalg.norm(F) > self.f_contact:
            ll -= np.where(any_touch, 0.0, 20.0)
            lat = F[:2]
            nl = np.where(touching[..., None], n[..., :2], 0.0)
            lat_cap = np.linalg.norm(nl, axis=2).max(axis=1)  # largest lateral normal component among touches
            if np.linalg.norm(lat) > self.f_lateral:
                along = (nl @ (lat / np.linalg.norm(lat))).max(axis=1)  # some touching normal pushes that way
                ll -= np.where(along > 0.2, 0.0, 4.0)
            else:
                # no lateral push: unless the friction cone can hold it, the touches must be nearly vertical
                ll -= np.where(lat_cap < 0.9, 0.0, 2.0)
        else:
            ll -= np.where(any_touch & (d.min(axis=1) < 0.5 * self.touch), 10.0, 0.0)
        self.logw += ll
        self.logw -= self.logw.max()

    def weights(self):
        w = np.exp(self.logw)
        return w / w.sum()

    def estimate(self):
        w = self.weights()
        mean = w @ self.h
        return mean, np.sqrt(w @ np.sum((self.h - mean) ** 2, axis=1))

    def next_probe(self, exclude=(), current=None):
        """Probe position maximizing P(hole within the capture radius); positions already probed are skipped; among
        near-ties (within 10 %) the one closest to `current` (a flat belief otherwise always picked a grid corner)."""
        w = self.weights()
        cands, ps = [], []
        for c in self.h[::3]:
            if any(np.linalg.norm(c - e) < 0.5 * self.capture for e in exclude):
                continue
            cands.append(c)
            ps.append(w[np.linalg.norm(self.h - c, axis=1) < self.capture].sum())
        cands, ps = np.array(cands), np.array(ps)
        near = ps >= 0.9 * ps.max()
        cur = np.zeros(2) if current is None else np.asarray(current)
        i = np.flatnonzero(near)[np.argmin(np.linalg.norm(cands[near] - cur, axis=1))]
        return cands[i], ps[i]
