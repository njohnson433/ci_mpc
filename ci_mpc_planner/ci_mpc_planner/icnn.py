"""Partially input-convex cost-to-go for the peg insertion (2026-10-24, user's idea 2): C(s, a) ~ -score of a 4-step
command sequence, arbitrary in the state s (TCP pose relative to the hole estimate, 5) and convex in the action a,
so one MPC step is a small convex minimization instead of 256 GPU rollouts.

Action a (5): the structured candidates of sampling_mpc.SamplingMPC made continuous:
  a[0:2] lateral set-point step per control step [m] (|.| <= 0.5 mm), a[2] press depth below the current z [m]
  (-1..6 mm; the set-point goes down 0.5 mm per step until it is a[2] below x), a[3:5] tilt set-point [rad] (+-0.01).
Network (Amos et al. 2017, PICNN): x-path tanh MLP; z-path z_{i+1} = softplus(W_z+ z_i + W_u a + W_h h_i + b) with
W_z >= 0 (clamped), output W_o+ z + w_u(h) . a + c(h) + |q(h) * a|^2 (strongly convex in a).
Known limit: convex in a cannot represent the left-or-right choice around the hole; the planner can restrict a to
a box around a sampled mode (hybrid).
"""
import numpy as np
import torch
from torch import nn
from torch.nn import functional as Fn

A_LO = np.array([-5e-4, -5e-4, -1e-3, -0.01, -0.01])
A_HI = np.array([5e-4, 5e-4, 6e-3, 0.01, 0.01])
A_SCALE = (A_HI - A_LO) / 2  # network input: a normalized to [-1, 1]
A_MID = (A_HI + A_LO) / 2
S_SCALE = np.array([1e-3, 1e-3, 1e-3, 0.01, 0.01])  # state (x - hole, tip height) in mm / 10 mrad


def sequence(x, a, H=4, du_z=5e-4):
    """(B, 5) state, (B, 5) action -> (B, H, 5) absolute set-points (torch)."""
    k = torch.arange(1, H + 1, device=x.device, dtype=x.dtype)[None, :, None]
    u = x[:, None, :].repeat(1, H, 1)
    u[:, :, :2] = x[:, None, :2] + k[..., 0:1] * a[:, None, :2]
    press, ramp = a[:, None, 2], k[..., 0] * du_z  # the set-point moves 0.5 mm per step until it is `press` below x
    u[:, :, 2] = x[:, None, 2] - torch.where(press >= 0, torch.minimum(ramp, press), torch.maximum(-ramp, press))
    u[:, :, 3:] = a[:, None, 3:]
    return u


class PICNN(nn.Module):
    def __init__(self, n_s=5, n_a=5, hidden=128, n_layers=3):
        super().__init__()
        self.x_layers = nn.ModuleList([nn.Linear(n_s if i == 0 else hidden, hidden) for i in range(n_layers)])
        self.z_u = nn.ModuleList([nn.Linear(n_a, hidden) for _ in range(n_layers)])
        self.z_h = nn.ModuleList([nn.Linear(hidden, hidden, bias=False) for _ in range(n_layers)])
        self.z_z = nn.ModuleList([nn.Linear(hidden, hidden, bias=False) for _ in range(n_layers - 1)])
        self.out_z = nn.Linear(hidden, 1, bias=False)
        self.out_lin = nn.Linear(hidden, n_a)   # w_u(h): linear term in a
        self.out_c = nn.Linear(hidden, 1)       # c(h)
        self.out_q = nn.Linear(hidden, n_a)     # q(h): quadratic term
        self.clamp()

    def clamp(self):
        with torch.no_grad():
            for m in [*self.z_z, self.out_z]:
                m.weight.clamp_(min=0.0)

    def forward(self, s, a):
        h, z = s, None
        for i, lx in enumerate(self.x_layers):
            h = torch.tanh(lx(h))
            pre = self.z_u[i](a) + self.z_h[i](h)
            if z is not None:
                pre = pre + self.z_z[i - 1](z)
            z = Fn.softplus(pre)
        q = Fn.softplus(self.out_q(h)) * 0.3
        return self.out_z(z)[:, 0] + (self.out_lin(h) * a).sum(1) + self.out_c(h)[:, 0] + ((q * a) ** 2).sum(1)


class ICNNPlanner:
    """argmin_a C(s, a) in the box by projected Adam (batched over restarts / belief samples)."""

    def __init__(self, path, device="cuda", iters=40, lr=0.1):
        ck = torch.load(path, map_location=device, weights_only=False)
        self.net = PICNN(**ck["arch"]).to(device)
        self.net.load_state_dict(ck["model_state_dict"])
        self.net.eval()
        self.dev, self.iters, self.lr = torch.device(device), iters, lr
        self.c_mu, self.c_sd = ck["c_mu"], ck["c_sd"]

    def s_feat(self, x, hole_xy):
        s = np.asarray(x, float).copy()
        s[:2] -= np.asarray(hole_xy, float)
        return s / S_SCALE

    def plan(self, x, hole_xy, tip_below_tcp, hole_samples=None, box=None):
        """-> best action a (5, numpy, physical units), its predicted cost. hole_samples: mean cost over them (still
        convex). box (lo, hi) in physical units restricts a (e.g. around a sampled mode)."""
        holes = [hole_xy] if hole_samples is None else list(hole_samples)
        xs = np.array(x, float).copy()
        xs[2] -= tip_below_tcp  # tip height above the opening
        S = torch.tensor(np.array([self.s_feat(xs, h) for h in holes]), dtype=torch.float32, device=self.dev)
        lo, hi = (A_LO, A_HI) if box is None else (np.maximum(box[0], A_LO), np.minimum(box[1], A_HI))
        lo_n = torch.tensor((lo - A_MID) / A_SCALE, dtype=torch.float32, device=self.dev)
        hi_n = torch.tensor((hi - A_MID) / A_SCALE, dtype=torch.float32, device=self.dev)
        a = ((lo_n + hi_n) / 2).clone().requires_grad_(True)
        opt = torch.optim.Adam([a], lr=self.lr)
        for _ in range(self.iters):
            c = self.net(S, a[None].repeat(len(S), 1)).mean()
            opt.zero_grad()
            c.backward()
            opt.step()
            with torch.no_grad():
                a.data = torch.maximum(torch.minimum(a.data, hi_n), lo_n)
        with torch.no_grad():
            c = self.net(S, a[None].repeat(len(S), 1)).mean().item()
        return a.detach().cpu().numpy() * A_SCALE + A_MID, c * self.c_sd + self.c_mu
