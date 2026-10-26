"""Batched (PyTorch, GPU) version of the 3D peg-hole quasi-static plant (peg_hole_3d.PegHole3D.step), for sampling-
based contact-implicit MPC and a fast teacher (weekend track B).

Same model: TCP pose x = (x, y, z, rx, ry), peg rigidly below the TCP, exact axisymmetric hole SDF, peg points (tip
ring, chamfer edge ring, tip center) vs the hole, hole rim points vs the peg side, arm stiction box, quasi-static
impedance x+ = u + C (W lam + f). Differences for batching:
  - a fixed candidate contact set (all points, masked when farther than `radius`), so every problem has the same size;
  - the friction pyramid as 4 generators per contact, n + mu t_j (lam = sum of non-negative generator weights), so the
    dual QP only has bounds (alpha >= 0, |f| <= F_s); solved with FISTA (projected accelerated gradient), no
    factorization: B problems at once on the GPU.
`check_against_cpu()` compares with the OSQP plant.
"""
import math

import torch

from ci_mpc_planner.peg_hole import PegHoleGeometry


def rot_xy(rx, ry):
    """R = Ry(ry) Rx(rx) and its derivatives, batched (B,) -> (B, 3, 3) x 3."""
    cx, sx, cy, sy = torch.cos(rx), torch.sin(rx), torch.cos(ry), torch.sin(ry)
    z, o = torch.zeros_like(rx), torch.ones_like(rx)
    Rx = torch.stack([o, z, z, z, cx, -sx, z, sx, cx], -1).view(-1, 3, 3)
    Ry = torch.stack([cy, z, sy, z, o, z, -sy, z, cy], -1).view(-1, 3, 3)
    dRx = torch.stack([z, z, z, z, -sx, -cx, z, cx, -sx], -1).view(-1, 3, 3)
    dRy = torch.stack([-sy, z, cy, z, z, z, -cy, z, -sy], -1).view(-1, 3, 3)
    return Ry @ Rx, Ry @ dRx, dRy @ Rx


def profile_sdf(r, z, g):
    """Signed distance (free > 0) and normal (n_r, n_z) to the hole profile, batched."""
    R, c, D, W = g.R, g.c, g.depth, g.top_half_width
    s2 = 1 / math.sqrt(2)
    segs = [((R + c, 0.0), (W, 0.0), (0.0, 1.0)), ((R, -c), (R + c, 0.0), (-s2, s2)),
            ((R, -D), (R, -c), (-1.0, 0.0)), ((0.0, -D), (R, -D), (0.0, 1.0))]
    best_d = torch.full_like(r, float("inf"))
    best_nr, best_nz = torch.zeros_like(r), torch.ones_like(r)
    for p0, p1, n in segs:
        dx, dz = p1[0] - p0[0], p1[1] - p0[1]
        s = (((r - p0[0]) * dx + (z - p0[1]) * dz) / (dx * dx + dz * dz)).clamp(0.0, 1.0)
        vr, vz = r - (p0[0] + s * dx), z - (p0[1] + s * dz)
        dn = vr * n[0] + vz * n[1]
        off = torch.hypot(vr - dn * n[0], vz - dn * n[1])
        norm = torch.hypot(vr, vz).clamp_min(1e-12)
        end = off > 1e-9
        dist = torch.where(end, torch.sign(dn + 1e-15) * norm, dn)
        nr = torch.where(end & (dist > 0), vr / norm, torch.full_like(r, n[0]))
        nz = torch.where(end & (dist > 0), vz / norm, torch.full_like(r, n[1]))
        take = dist.abs() < best_d.abs()
        best_d = torch.where(take, dist, best_d)
        best_nr = torch.where(take, nr, best_nr)
        best_nz = torch.where(take, nz, best_nz)
    return best_d, best_nr, best_nz


_MOMENTUM = None


def _momentum(iters):
    """FISTA momentum coefficients (t_k - 1) / t_{k+1}, precomputed (no python floats inside the compiled loop)."""
    global _MOMENTUM
    if _MOMENTUM is None or len(_MOMENTUM) < iters:
        tk, out = 1.0, []
        for _ in range(max(iters, 2000)):
            tk1 = (1 + math.sqrt(1 + 4 * tk * tk)) / 2
            out.append((tk - 1) / tk1)
            tk = tk1
        _MOMENTUM = out
    return _MOMENTUM


def _fista_block(W, Cs, b, lo, hi, L, z, y, beta):
    """`beta.shape[0]` FISTA iterations of the bound-constrained QP min 1/2 z^T W C W^T z + b^T z (unrolled);
    beta: momentum coefficients as a tensor (python floats would be specialized and recompile per block)."""
    for k in range(beta.shape[0]):
        grad = torch.einsum("bzi,bi->bz", W, Cs * torch.einsum("bzi,bz->bi", W, y)) + b
        z_new = torch.maximum(torch.minimum(y - grad / L, hi), lo)
        y = z_new + beta[k] * (z_new - z)
        z = z_new
    return z, y


_compiled_block = None


def _fista(compiled, W, Cs, b, lo, hi, L, iters, block=25):
    """FISTA in blocks of `block` iterations; with `compiled` each block is one torch.compile'd CUDA graph (the
    plain loop was launch-bound: ~85 ms for 300 iterations at any batch size up to 512)."""
    global _compiled_block
    z = torch.zeros_like(b)
    y = z.clone()
    mom = _momentum(iters)
    fn = _fista_block
    if compiled:
        if _compiled_block is None:
            _compiled_block = torch.compile(_fista_block, mode="reduce-overhead", fullgraph=True, dynamic=False)
        fn = _compiled_block
    for k0 in range(0, iters, block):
        beta = torch.tensor(mom[k0:k0 + block], device=b.device, dtype=b.dtype)
        z, y = fn(W, Cs, b, lo, hi, L, z, y, beta)
        if compiled:
            z, y = z.clone(), y.clone()
    return z


class BatchedPegHole3D:
    def __init__(self, geo=PegHoleGeometry(), n_ring=8, n_rim=12, radius=0.004, device="cuda", dtype=torch.float64,
                 compiled=False):
        self.g, self.radius, self.dev, self.dt = geo, radius, torch.device(device), dtype
        self.compiled = compiled
        kw = dict(device=self.dev, dtype=dtype)
        ang = torch.arange(n_ring, **kw) * (2 * math.pi / n_ring)
        tip = torch.stack([(geo.a - geo.pc) * torch.cos(ang), (geo.a - geo.pc) * torch.sin(ang), torch.zeros_like(ang)], 1)
        edge = torch.stack([geo.a * torch.cos(ang), geo.a * torch.sin(ang), torch.full_like(ang, geo.pc)], 1)
        self.peg_points = torch.cat([tip, edge, torch.zeros(1, 3, **kw)])  # (P1, 3)
        ar = torch.arange(n_rim, **kw) * (2 * math.pi / n_rim)
        R, c = geo.R, geo.c
        self.rim_points = torch.cat([torch.stack([(R + c) * torch.cos(ar), (R + c) * torch.sin(ar), torch.zeros_like(ar)], 1),
                                     torch.stack([R * torch.cos(ar), R * torch.sin(ar), torch.full_like(ar, -c)], 1)])
        self.off = torch.tensor([0.0, 0.0, geo.tip_below_tcp], **kw)

    def t(self, a):
        return torch.as_tensor(a, device=self.dev, dtype=self.dt)

    def contacts(self, x, hole_xy):
        """x (B, 5), hole_xy (B, 2) -> gap (B, P), normal n (B, P, 3), tangents t1, t2, Jacobian G (B, P, 3, 5),
        active mask (B, P). P = peg points + rim points."""
        B = x.shape[0]
        Rm, dRx, dRy = rot_xy(x[:, 3], x[:, 4])
        tcp = torch.cat([x[:, :2] - hole_xy, x[:, 2:3]], 1)  # (B, 3) in the hole frame
        # peg points vs the hole
        pl = self.peg_points - self.off  # (P1, 3) relative to the TCP
        pw = tcp[:, None, :] + torch.einsum("bij,pj->bpi", Rm, pl)
        r = torch.hypot(pw[..., 0], pw[..., 1])
        d1, nr, nz = profile_sdf(r, pw[..., 2], self.g)
        rr = r.clamp_min(1e-9)
        n1 = torch.stack([nr * pw[..., 0] / rr, nr * pw[..., 1] / rr, nz], -1)
        J1 = torch.zeros(B, pl.shape[0], 3, 5, device=self.dev, dtype=self.dt)
        J1[..., :3] = torch.eye(3, device=self.dev, dtype=self.dt)
        J1[..., 3] = torch.einsum("bij,pj->bpi", dRx, pl)
        J1[..., 4] = torch.einsum("bij,pj->bpi", dRy, pl)
        # hole rim points vs the peg side (cylinder of radius a along the peg axis, pc <= z_l <= length)
        base = tcp + torch.einsum("bij,j->bi", Rm, -self.off)  # tip center
        axis = Rm[:, :, 2]
        v = self.rim_points[None] - base[:, None, :]  # (B, P2, 3)
        zl = (v * axis[:, None, :]).sum(-1)
        radial = v - zl[..., None] * axis[:, None, :]
        rad = radial.norm(dim=-1).clamp_min(1e-12)
        d2 = rad - self.g.a
        valid2 = (zl >= self.g.pc) & (zl <= self.g.length)
        d2 = torch.where(valid2, d2, torch.full_like(d2, 1.0))
        n2 = -radial / rad[..., None]
        pl2 = torch.einsum("bji,bpj->bpi", Rm, self.rim_points[None] - tcp[:, None, :])  # R^T (hp - tcp)
        J2 = torch.zeros(B, pl2.shape[1], 3, 5, device=self.dev, dtype=self.dt)
        J2[..., :3] = torch.eye(3, device=self.dev, dtype=self.dt)
        J2[..., 3] = torch.einsum("bij,bpj->bpi", dRx, pl2)
        J2[..., 4] = torch.einsum("bij,bpj->bpi", dRy, pl2)
        d = torch.cat([d1, d2], 1)
        n = torch.cat([n1, n2], 1)
        G = torch.cat([J1, J2], 1)
        active = (d < self.radius) & (d > -0.002)
        a = torch.where(n[..., 0:1].abs() < 0.9, torch.tensor([1.0, 0, 0], device=self.dev, dtype=self.dt),
                        torch.tensor([0, 1.0, 0], device=self.dev, dtype=self.dt))
        t1 = torch.linalg.cross(n, a.expand_as(n))
        t1 = t1 / t1.norm(dim=-1, keepdim=True).clamp_min(1e-12)
        t2 = torch.linalg.cross(n, t1)
        return d, n, t1, t2, G, active

    def step(self, x, u, compliance, arm_friction, hole_xy, iters=300, return_wrench=False):
        """One quasi-static step for B problems. x, u (B, 5), compliance / arm_friction (5,) or (B, 5),
        hole_xy (B, 2). Returns x+ (B, 5) [and the generalized contact wrench (B, 5)]."""
        mu, sc = self.g.mu, 1000.0
        d, n, t1, t2, G, active = self.contacts(x, hole_xy)
        B, P = d.shape
        C = self.t(compliance).expand(B, 5)
        Fs = self.t(arm_friction).expand(B, 5)
        du = u - x
        # generator directions n + mu t for t in (t1, -t1, t2, -t2): (B, P, 4, 3); generalized: G^T dir (B, P, 4, 5)
        dirs = torch.stack([n + mu * t1, n - mu * t1, n + mu * t2, n - mu * t2], 2)
        Wg = torch.einsum("bpgi,bpij->bpgj", dirs, G) * active[..., None, None]  # (B, P, 4, 5)
        Wg = Wg.reshape(B, P * 4, 5)
        # b for the generators: gap + (generalized direction) . du ; for the arm friction: du
        bg = (d[..., None] + torch.einsum("bpgi,bpij,bj->bpg", dirs, G, du)).reshape(B, P * 4)
        bg = torch.where(active.repeat_interleave(4, 1), bg, torch.zeros_like(bg))
        W = torch.cat([Wg, torch.diag_embed(torch.ones(B, 5, device=self.dev, dtype=self.dt))], 1)  # (B, nz, 5)
        b = sc * torch.cat([bg, du], 1)
        Cs = sc * C  # A = W C W^T (in z space): A z = W (C (W^T z))
        lo = torch.cat([torch.zeros(B, P * 4, device=self.dev, dtype=self.dt), -Fs], 1)
        hi = torch.cat([torch.where(active.repeat_interleave(4, 1), torch.full((B, P * 4), float("inf"), device=self.dev,
                                                                                 dtype=self.dt),
                                    torch.zeros(B, P * 4, device=self.dev, dtype=self.dt)), Fs], 1)
        # Lipschitz constant: ||A|| <= trace bound of the 5x5 W^T W C (A has rank <= 5)
        M = torch.einsum("bzi,bzj->bij", W, W) * Cs[:, None, :]
        L = torch.linalg.matrix_norm(M, ord=2).clamp_min(1e-9)[:, None]
        z = _fista(self.compiled, W, Cs, b, lo, hi, L, iters)
        gen = torch.einsum("bzi,bz->bi", W, z)  # generalized force: contacts + arm friction
        x_new = u + C * gen
        if return_wrench:
            wrench = torch.einsum("bzi,bz->bi", Wg, z[:, :P * 4])
            return x_new, wrench
        return x_new


def check_against_cpu(n=200, seed=0, device="cuda", iters=300):
    """Max |x+ difference| between the batched and the OSQP plant on random near-contact states without initial
    penetration (states deep inside the block made the OSQP reference blow up, not the batched solver)."""
    import numpy as np

    from ci_mpc_planner.peg_hole_3d import PegHole3D

    rng = np.random.default_rng(seed)
    comp = np.array([1 / 1500, 1 / 1500, 1 / 500, 1 / 130, 1 / 130])
    fa = np.array([2.0, 2.0, 3.5, 0.05, 0.05])
    bp = BatchedPegHole3D(n_ring=8, n_rim=12, device=device)
    xs, us, hs, ref = [], [], [], []
    t = lambda a: torch.tensor(np.array(a), device=device, dtype=torch.float64)  # noqa: E731
    while len(xs) < n:
        h = rng.uniform(-3e-3, 3e-3, 2)
        x = np.array([0.0, 0.0, 0.040 + rng.uniform(-0.003, 0.001), rng.normal(0, 0.01), rng.normal(0, 0.01)])
        u = x + np.concatenate([rng.normal(0, 8e-4, 3), rng.normal(0, 0.01, 2)])
        dd, _, _, _, _, act = bp.contacts(t([x]), t([h]))
        if (dd[act] < -2e-4).any():
            continue
        cpu = PegHole3D(hole_xy=h, n_ring=8, n_rim=12)
        x1, _, _ = cpu.step(x, u, comp, fa)
        xs.append(x), us.append(u), hs.append(h), ref.append(x1)
    out = bp.step(t(xs), t(us), comp, fa, t(hs), iters=iters).cpu().numpy()
    err = np.abs(out - np.array(ref))
    return err[:, :3].max() * 1000, err[:, 3:].max() * 1000, np.percentile(err[:, :3].max(1), 95) * 1000
