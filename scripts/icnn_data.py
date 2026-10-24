#!/usr/bin/env python3
# Training data for the ICNN cost-to-go (ci_mpc_planner/icnn.py): engaged states from the batched GPU plant (hole
# error r 0-2.5 mm as in the evals plus margin, then 0-25 random-walk steps, so states lie on the chamfer, the rim,
# the top face and inside the bore), and for each state K random actions scored exactly as the sampler scores a
# candidate (4-step rollout: depth, centering, force above 3 N).
#   ~/IsaacLab/.venv/bin/python scripts/icnn_data.py OUT.npz [--states 20000 --k 64 --seed 0]
import argparse
import sys
import time

import numpy as np
import torch

sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0] + "/ci_mpc_planner")
from ci_mpc_planner.icnn import A_HI, A_LO, sequence  # noqa: E402
from ci_mpc_planner.peg_hole import PegHoleGeometry  # noqa: E402
from ci_mpc_planner.peg_hole_3d_torch import BatchedPegHole3D  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("out")
p.add_argument("--states", type=int, default=20000)
p.add_argument("--k", type=int, default=64)
p.add_argument("--batch", type=int, default=4096)
p.add_argument("--seed", type=int, default=0)
args = p.parse_args()
dev, dt = torch.device("cuda"), torch.float32
torch.manual_seed(args.seed)
g = PegHoleGeometry()
plant = BatchedPegHole3D(g, n_ring=8, n_rim=12, device="cuda", dtype=dt, compiled=False)
comp = torch.tensor([1 / 1500, 1 / 1500, 1 / 500, 1 / 130, 1 / 130], device=dev, dtype=dt)
fa = torch.tensor([2.0, 2.0, 3.5, 0.05, 0.05], device=dev, dtype=dt)
tb = g.tip_below_tcp
lo, hi = torch.tensor(A_LO, device=dev, dtype=dt), torch.tensor(A_HI, device=dev, dtype=dt)


def states(n):
    r = torch.rand(n, device=dev) * 2.5e-3
    th = torch.rand(n, device=dev) * 2 * np.pi
    hole = torch.stack([r * torch.cos(th), r * torch.sin(th)], 1)
    x = torch.zeros(n, 5, device=dev, dtype=dt)
    x[:, 2] = tb + 5e-4
    for _ in range(10):  # the evals' engaging press
        u = x.clone()
        u[:, 2] = x[:, 2] - 0.004
        u = u + comp * torch.tensor([0, 0, -3.0, 0, 0], device=dev, dtype=dt)
        x = plant.step(x, u, comp, fa, hole, iters=200)
    walk = torch.randint(0, 26, (n,), device=dev)
    for k in range(25):  # random walk of sampled actions (first command of each), per-state length
        a = lo + (hi - lo) * torch.rand(n, 5, device=dev, dtype=dt)
        xn = plant.step(x, sequence(x, a)[:, 0], comp, fa, hole, iters=200)
        x = torch.where((k < walk)[:, None], xn, x)
    return x, hole


def score(x, hole, a):
    seq = sequence(x, a)
    peak = torch.zeros(len(x), device=dev, dtype=dt)
    for k in range(seq.shape[1]):
        x, w = plant.step(x, seq[:, k], comp, fa, hole, iters=200, return_wrench=True)
        peak = torch.maximum(peak, w[:, :3].norm(dim=1))
    tip = torch.cat([x[:, :2] - hole, x[:, 2:3] - tb], 1)
    return -tip[:, 2] * 1000 - tip[:, :2].norm(dim=1) * 1000 - 0.5 * (peak - 3.0).clamp_min(0) ** 2, peak


S_all, A_all, Y_all, P_all = [], [], [], []
t0 = time.time()
n_per = max(1, args.batch // args.k)
with torch.no_grad():
    for b in range(0, args.states, n_per):
        n = min(n_per, args.states - b)
        x, hole = states(n)
        xr, hr = x.repeat_interleave(args.k, 0), hole.repeat_interleave(args.k, 0)
        a = lo + (hi - lo) * torch.rand(n * args.k, 5, device=dev, dtype=dt)
        y, pk = score(xr, hr, a)
        s = x.clone()
        s[:, :2] -= hole
        s[:, 2] -= tb
        S_all.append(s.cpu().numpy()), A_all.append(a.view(n, args.k, 5).cpu().numpy())
        Y_all.append(y.view(n, args.k).cpu().numpy()), P_all.append(pk.view(n, args.k).cpu().numpy())
        if b % (20 * n_per) == 0:
            print(f"{b + n}/{args.states} states, {time.time() - t0:.0f} s", flush=True)
np.savez(args.out, s=np.concatenate(S_all), a=np.concatenate(A_all), y=np.concatenate(Y_all),
         peak=np.concatenate(P_all))
Y = np.concatenate(Y_all)
print(f"saved {args.out}: {len(Y)} states x {args.k} actions in {time.time() - t0:.0f} s; score mean {Y.mean():.2f}, "
      f"best-of-K mean {Y.max(1).mean():.2f}, finite {np.isfinite(Y).mean():.3f}")
