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
                 f_lateral=1.0, capture=0.002, likelihood=None, face_rings=(), gap_sigma=None,
                 rect=None):  # f_lateral ~3 sigma of the wrench estimate
        self.g = geo
        # rect = (ax, ay): rectangular peg with these half widths in the model frame (axis aligned with the hole), and a
        # rectangular hole with the round model's radial clearance R - a per side (dev square, H5 rect; 2026-10-28)
        self.rect = None if rect is None else np.asarray(rect, float)
        self.likelihood = likelihood  # learned probe likelihood (probe_likelihood.py) instead of the hand model
        ax = np.arange(-spread, spread + 1e-9, res)
        gx, gy = np.meshgrid(ax, ax)
        self.h = np.column_stack([gx.ravel(), gy.ravel()])
        self.logw = np.zeros(len(self.h))
        ang = np.linspace(0, 2 * np.pi, n_ring, endpoint=False)
        if self.rect is None:
            ux, uy = np.cos(ang), np.sin(ang)
            ra, rb = (geo.a - geo.pc, geo.a - geo.pc), (geo.a, geo.a)
        else:  # rectangle outline: the ray at each angle to the unit square (n_ring divisible by 8 puts rays on corners)
            m = np.maximum(np.abs(np.cos(ang)), np.abs(np.sin(ang)))
            ux, uy = np.cos(ang) / m, np.sin(ang) / m
            ra, rb = self.rect - geo.pc, self.rect
        tip = np.column_stack([ra[0] * ux, ra[1] * uy, np.zeros(n_ring)])
        edge = np.column_stack([rb[0] * ux, rb[1] * uy, np.full(n_ring, geo.pc)])
        # face_rings: extra rings on the flat bottom face (fractions of its radius): a tilted peg meets a sharp hole
        # edge with the face interior, which the tip / edge rings alone do not see (tilt probe, 2026-10-28)
        face = [np.column_stack([f * ra[0] * ux, f * ra[1] * uy, np.zeros(n_ring)]) for f in face_rings]
        self.peg_points = np.vstack([tip, edge, *face, [[0, 0, 0]]])
        self.touch, self.f_contact, self.f_lateral, self.capture = touch, f_contact, f_lateral, capture
        # penetration tolerance: the measured tip height is only good to ~0.1-0.3 mm (grasp offset calibration,
        # estimate); without it a tip 0.07 mm "inside" the top face favoured holes next to every probe
        self.pen_tol = pen_tol
        # gap_sigma: contact model instead of the penetration / touch rules: the peg rests at the measured pose, so for
        # the true hole the smallest peg-hole gap is ~0 (Gaussian, capped at 4 sigma). Tilt probe on sharp holes: the
        # informative drops are 0.2-1.5 mm, inside the 0.3 mm pen_tol of the rules (2026-10-28)
        self.gap_sigma = gap_sigma

    def points_world(self, x):
        """Peg points in the model frame for TCP pose x = (x, y, z, rx, ry)."""
        Rm = rotxy(x[3], x[4])
        return x[:3] + (self.peg_points - np.array([0, 0, self.g.tip_below_tcp])) @ Rm.T

    def gaps(self, x):
        """Gap (H x P) and normals (H x P x 3) of every peg point for every hole hypothesis."""
        p = self.points_world(x)  # P x 3
        dx = p[None, :, 0] - self.h[:, None, 0]
        dy = p[None, :, 1] - self.h[:, None, 1]
        if self.rect is not None:
            # rectangular hole, half widths rect + (R - a): the profile in the coordinate of the nearer wall pair
            # (exact on the face / in the bore away from the corners)
            ex, ey = np.abs(dx) - self.rect[0] + self.g.a, np.abs(dy) - self.rect[1] + self.g.a
            r = np.maximum(ex, ey)
            z = np.broadcast_to(p[None, :, 2], r.shape)
            d, nr, nz = profile_sdf(np.maximum(r, 0.0), z, self.g)
            xs = ex >= ey
            return d, np.stack([np.where(xs, nr * np.sign(dx), 0.0), np.where(xs, 0.0, nr * np.sign(dy)), nz], axis=-1)
        r = np.hypot(dx, dy)
        z = np.broadcast_to(p[None, :, 2], r.shape)
        d, nr, nz = profile_sdf(r, z, self.g)
        rr = np.maximum(r, 1e-12)
        return d, np.stack([nr * dx / rr, nr * dy / rr, nz], axis=-1)

    def update(self, x, force):
        """x: measured TCP pose (model frame), force: measured contact force on the peg (3, world)."""
        if self.likelihood is not None:
            self.logw += self.likelihood(self.h, x[:2], x[2] - self.g.tip_below_tcp, force)
            self.logw -= self.logw.max()
            return
        d, n = self.gaps(x)
        if self.gap_sigma is not None and np.linalg.norm(force) > self.f_contact:
            self.logw += -0.5 * np.minimum((d.min(axis=1) / self.gap_sigma) ** 2, 16.0)
            self.logw -= self.logw.max()
            return
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

    def next_probe_edge(self, offset, radius, exclude=(), current=None, min_spacing=0.0015):
        """Tilt probe: the probe point (the peg's low point) where P(the hole is under it) is closest to 0.5, i.e. on the
        uncertain edge of the belief (a sure drop or a sure miss teaches nothing); offset = low point - TCP is
        irrelevant here, candidates are low-point positions; near-ties (within 0.05) the one closest to `current`."""
        w = self.weights()
        cands, sc = [], []
        for c in self.h[::2]:
            if any(np.linalg.norm(c - e) < min_spacing for e in exclude):
                continue
            pin = w[np.linalg.norm(self.h - c, axis=1) < radius].sum()
            cands.append(c)
            sc.append(min(pin, 1.0 - pin))
        cands, sc = np.array(cands), np.array(sc)
        near = sc >= sc.max() - 0.05
        cur = np.zeros(2) if current is None else np.asarray(current)
        i = np.flatnonzero(near)[np.argmin(np.linalg.norm(cands[near] - cur, axis=1))]
        return cands[i], sc[i]

    def predicted_drop(self, pose, cap=0.0015, n_hyp=400):
        """Tilt probe: expected drop [m] of the peg at `pose` (TCP at the flat-face height) over the bulk of the belief:
        the smallest peg-hole gap per hypothesis, capped, weighted (as next_probe_info's prediction)."""
        w = self.weights()
        idx = np.argsort(w)[::-1][:n_hyp]
        h_all = self.h
        self.h = h_all[idx]
        try:
            d, _ = self.gaps(pose)
        finally:
            self.h = h_all
        return float(w[idx] @ np.clip(d.min(axis=1), 0.0, cap) / w[idx].sum())

    def next_probe_info(self, pose_at, exclude=(), current=None, min_spacing=0.0015, cap=0.0015, n_hyp=400):
        """Tilt probe: the candidate (low-point position) whose predicted measurement varies most over the belief.
        pose_at(c) = the TCP pose that puts the low point at c on the flat face; the prediction per hypothesis is the
        smallest peg-hole gap there (~ how far the peg would drop), capped; score = its weighted variance (a sure drop
        or a sure miss, and a 0.1 mm drop at the very edge, all teach little). Near-ties (10 %): closest to `current`."""
        w = self.weights()
        idx = np.argsort(w)[::-1][:n_hyp]  # the bulk of the belief
        wi = w[idx] / w[idx].sum()
        h_all = self.h
        self.h = h_all[idx]
        cands, sc = [], []
        try:
            for c in h_all[::2]:
                if any(np.linalg.norm(c - e) < min_spacing for e in exclude):
                    continue
                d, _ = self.gaps(pose_at(c))
                y = np.clip(d.min(axis=1), 0.0, cap)
                m = wi @ y
                cands.append(c)
                sc.append(wi @ (y - m) ** 2)
        finally:
            self.h = h_all
        cands, sc = np.array(cands), np.array(sc)
        near = sc >= 0.9 * sc.max()
        cur = np.zeros(2) if current is None else np.asarray(current)
        i = np.flatnonzero(near)[np.argmin(np.linalg.norm(cands[near] - cur, axis=1))]
        return cands[i], sc[i]

    def next_probe(self, exclude=(), current=None, min_spacing=None):
        """Probe position maximizing P(hole within the capture radius); positions already probed are skipped; among
        near-ties (within 10 %) the one closest to `current` (a flat belief otherwise always picked a grid corner)."""
        w = self.weights()
        cands, ps = [], []
        for c in self.h[::3]:
            spacing = 0.5 * self.capture if min_spacing is None else min_spacing
            if any(np.linalg.norm(c - e) < spacing for e in exclude):
                continue
            cands.append(c)
            ps.append(w[np.linalg.norm(self.h - c, axis=1) < self.capture].sum())
        cands, ps = np.array(cands), np.array(ps)
        near = ps >= 0.9 * ps.max()
        cur = np.zeros(2) if current is None else np.asarray(current)
        i = np.flatnonzero(near)[np.argmin(np.linalg.norm(cands[near] - cur, axis=1))]
        return cands[i], ps[i]
