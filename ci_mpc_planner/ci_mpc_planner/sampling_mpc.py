"""Sampling-based contact-implicit MPC on the batched GPU plant (weekend track B): the sampled C3 teacher's strength
was "try many options, evaluate each in the contact model"; here that is done directly with K command sequences
rolled out in parallel in BatchedPegHole3D (contacts and their modes come from the model; the optimizer is
predictive sampling / MPPI instead of C3-ADMM, which does not batch).

Candidates per decision (horizon H, all step-limited like the controller's command stream):
  - structured families (as the CPU teacher): centering with light / firm press, press only, lateral +-x / +-y,
    tilts, each with a few press depths;
  - force-following heuristic variants (different lateral gains / normal forces), using the model's wrench;
  - Gaussian perturbations of the previous best sequence, shifted by one step (warm start).
Score (as the teacher): tip depth gained, tip distance to the (estimated) hole axis, penalty on contact force above
f_soft. Returns the best first command (pose set-point incl. any feedforward as an equivalent offset).
"""
import math

import numpy as np
import torch

from ci_mpc_planner.peg_hole_3d_torch import BatchedPegHole3D


class SamplingMPC:
    def __init__(self, compliance, arm_friction, horizon=4, n_random=192, iters=200, f_soft=3.0, w_depth=1.0,
                 w_center=1.0, w_force=0.5, du_max=(5e-4, 5e-4, 5e-4, 0.01, 0.01), device="cuda",
                 dtype=torch.float32, seed=0):
        self.plant = BatchedPegHole3D(n_ring=8, n_rim=12, device=device, dtype=dtype, compiled=True)
        self.comp, self.fa = np.asarray(compliance), np.asarray(arm_friction)
        self.H, self.n_random, self.iters = horizon, n_random, iters
        self.f_soft, self.w = f_soft, (w_depth, w_center, w_force)
        self.du = torch.tensor(du_max, device=device, dtype=dtype)
        self.dev, self.dt = torch.device(device), dtype
        self.best = None  # previous best sequence (H, 5), absolute set-points
        self.gen = torch.Generator(device=device).manual_seed(seed)
        self.tb = self.plant.g.tip_below_tcp
        self.n_total = 256

    def t(self, a):
        if isinstance(a, torch.Tensor):
            return a.to(device=self.dev, dtype=self.dt)
        return torch.as_tensor(np.asarray(a), device=self.dev, dtype=self.dt)

    def _structured(self, x, goal_xy):
        """(K, H, 5) structured candidate set-point sequences from x, built in numpy and moved to the GPU once (the
        per-element torch version took 159 ms in small kernel launches)."""
        x = np.asarray(x, float)
        g = np.asarray(goal_xy, float)
        du = self.du.double().cpu().numpy()
        seqs = []
        for lat in (None, (0, 0), (3e-4, 0), (-3e-4, 0), (0, 3e-4), (0, -3e-4)):
            for press in (0.001, 0.002, 0.004, 0.006):
                for tilt in ((0, 0), (0.01, 0), (-0.01, 0), (0, 0.01), (0, -0.01)):
                    if tilt != (0, 0) and press != 0.002:
                        continue
                    u, seq = x.copy(), []
                    for _ in range(self.H):
                        u = u.copy()
                        if lat is not None:
                            u[:2] += np.clip(g + np.asarray(lat) - u[:2], -du[0], du[0])
                        u[2] = max(u[2] - du[2], x[2] - press)
                        u[3:] = tilt
                        seq.append(u)
                    seqs.append(seq)
        return self.t(np.array(seqs))

    def _random(self, x):
        if self.best is None:
            return torch.empty(0, self.H, 5, device=self.dev, dtype=self.dt)
        base = torch.cat([self.best[1:], self.best[-1:]], 0)  # shift
        noise = torch.randn(self.n_random, self.H, 5, device=self.dev, dtype=self.dt, generator=self.gen)
        noise = noise * self.t([2e-4, 2e-4, 6e-4, 0.005, 0.005])
        return base[None] + torch.cumsum(noise, 1) * 0.5

    def rollout(self, x0, seqs, hole_xy, heuristic=None, w0=None):
        """Roll out K sequences (or the heuristic with K gain sets); returns score (K,), peak force (K,)."""
        K = seqs.shape[0] if heuristic is None else heuristic.shape[0]
        x = self.t(x0)[None].repeat(K, 1)
        h = self.t(hole_xy)
        h = h[None].repeat(K, 1) if h.dim() == 1 else h  # one hole for all, or one per row (belief samples)
        peak = torch.zeros(K, device=self.dev, dtype=self.dt)
        w = torch.zeros(K, 5, device=self.dev, dtype=self.dt) if w0 is None else self.t(w0)[None].repeat(K, 1)
        first = None
        for k in range(self.H):
            if heuristic is None:
                u = seqs[:, k]
            else:  # force following: gains (K, 3) = lateral step, lateral ff, normal ff
                lat = w[:, :2]
                nl = lat.norm(dim=1, keepdim=True).clamp_min(1e-9)
                d = torch.where(nl > 0.3, lat / nl, torch.zeros_like(lat))
                u = x.clone()
                u[:, :2] = x[:, :2] + heuristic[:, 0:1] * d
                pressing = (nl[:, 0] <= 0.3)
                u[:, 2] = x[:, 2] - torch.where(pressing, 0.004, 0.002)
                u[:, 3:] = 0
                ff = torch.zeros(K, 5, device=self.dev, dtype=self.dt)
                ff[:, :2] = heuristic[:, 1:2] * d
                ff[:, 2] = torch.where(pressing, -3.0 * torch.ones_like(nl[:, 0]), -heuristic[:, 2])
                u = u + self.t(self.comp) * ff
            if first is None:
                first = u.clone()
            x, w = self.plant.step(x, u, self.comp, self.fa, h, iters=self.iters, return_wrench=True)
            peak = torch.maximum(peak, w[:, :3].norm(dim=1))
        tip = x[:, :3] - torch.cat([h, torch.zeros(K, 1, device=self.dev, dtype=self.dt)], 1)
        tip[:, 2] -= self.tb
        wd, wc, wf = self.w
        score = -wd * tip[:, 2] * 1000 - wc * tip[:, :2].norm(dim=1) * 1000 - wf * (peak - self.f_soft).clamp_min(0) ** 2
        return score, peak, first

    @torch.no_grad()
    def act(self, x, hole_xy, wrench=None, hole_samples=None):
        """Best first set-point (numpy, 5) for the TCP pose x given the (estimated) hole xy and the measured contact
        wrench (for the heuristic candidates' first step). hole_samples (M, 2): hole positions sampled from the
        belief; every candidate is then rolled out under each of them and scored by the mean (belief-space MPC: in
        Isaac the MPC engaged before probing had localized the hole and planned against a hole 2 mm off)."""
        seqs = torch.cat([self._structured(x, hole_xy), self._random(x)], 0)
        # fixed batch size (one CUDA graph): pad with copies of the first structured sequence / truncate
        K = self.n_total
        if seqs.shape[0] < K:
            seqs = torch.cat([seqs, seqs[:1].repeat(K - seqs.shape[0], 1, 1)], 0)
        seqs = seqs[:K]
        M = 1 if hole_samples is None else len(hole_samples)
        if M > 1:
            hs = self.t(hole_samples)
            s1, p1, f1 = self.rollout(x, seqs.repeat_interleave(M, 0), hs.repeat(seqs.shape[0], 1))
            s1, f1 = s1.view(-1, M).mean(1), f1.view(-1, M, 5)[:, 0]
        else:
            s1, p1, f1 = self.rollout(x, seqs, hole_xy)
        gains = self.t([[4e-4, 2.5, 2.0], [2e-4, 2.5, 2.0], [4e-4, 1.5, 1.0], [6e-4, 3.0, 2.0], [4e-4, 2.5, 3.0]])
        if M > 1:
            s2, p2, f2 = self.rollout(x, None, hs.repeat(gains.shape[0], 1),
                                      heuristic=gains.repeat_interleave(M, 0), w0=wrench)
            s2, f2 = s2.view(-1, M).mean(1), f2.view(-1, M, 5)[:, 0]
        else:
            s2, p2, f2 = self.rollout(x, None, hole_xy, heuristic=gains, w0=wrench)
        scores = torch.cat([s1, s2])
        i = int(torch.argmax(scores))
        if i < seqs.shape[0]:
            self.best = seqs[i].clone()
            u0 = seqs[i, 0]
        else:
            self.best = None
            u0 = f2[i - seqs.shape[0]]
        return u0.double().cpu().numpy(), {"score": float(scores[i]), "n": int(scores.shape[0]),
                                           "source": "sample" if i < seqs.shape[0] else "heuristic"}
