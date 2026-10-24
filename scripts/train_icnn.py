#!/usr/bin/env python3
# Train the partially input-convex cost-to-go (ci_mpc_planner/icnn.py PICNN) on icnn_data.py files: cost = -score,
# normalized; MSE; W_z clamped >= 0 after every step (convex in the action). Validation on held-out states: regret of
# choosing, among the K sampled actions, the one with the lowest predicted cost vs the best of K (true scores, mm),
# and the same for a random action (reference).
#   ~/IsaacLab/.venv/bin/python scripts/train_icnn.py "DIR/data_*.npz" OUT.pt [--epochs 30 --hidden 128]
import argparse
import glob
import random
import sys

import numpy as np
import torch

sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0] + "/ci_mpc_planner")
from ci_mpc_planner.icnn import A_MID, A_SCALE, PICNN, S_SCALE  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("data")
p.add_argument("out")
p.add_argument("--epochs", type=int, default=30)
p.add_argument("--hidden", type=int, default=128)
p.add_argument("--layers", type=int, default=3)
p.add_argument("--lr", type=float, default=1e-3)
p.add_argument("--batch", type=int, default=4096)
p.add_argument("--val", type=float, default=0.1)
p.add_argument("--seed", type=int, default=0)
args = p.parse_args()
random.seed(args.seed), np.random.seed(args.seed), torch.manual_seed(args.seed)
dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
D = [np.load(f) for f in sorted(glob.glob(args.data))]
s, a, y = (np.concatenate([d[k] for d in D]) for k in ("s", "a", "y"))
N, K = y.shape
c = -y
c_mu, c_sd = float(c.mean()), float(c.std())
rng = np.random.default_rng(args.seed)
val = rng.random(N) < args.val
to = lambda v: torch.tensor(v, dtype=torch.float32, device=dev)  # noqa: E731
S = to(s / S_SCALE)
A = to((a - A_MID) / A_SCALE)
C = to((c - c_mu) / c_sd)
tr_idx = torch.tensor(np.flatnonzero(~val), device=dev)
va_idx = torch.tensor(np.flatnonzero(val), device=dev)
net = PICNN(hidden=args.hidden, n_layers=args.layers).to(dev)
opt = torch.optim.Adam(net.parameters(), lr=args.lr)
sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, args.epochs)


@torch.no_grad()
def evaluate(idx):
    net.eval()
    pred = net(S[idx].repeat_interleave(K, 0), A[idx].reshape(-1, 5)).view(-1, K)
    mse = ((pred - C[idx]) ** 2).mean().item()
    yv = torch.tensor(y, device=dev)[idx]
    pick = yv.gather(1, pred.argmin(1, keepdim=True))[:, 0]
    regret = (yv.max(1).values - pick).mean().item()
    regret_rand = (yv.max(1).values - yv.mean(1)).mean().item()
    return mse, regret, regret_rand


best = (np.inf, None, -1)
for e in range(args.epochs):
    net.train()
    perm = tr_idx[torch.randperm(len(tr_idx), device=dev)]
    for b in range(0, len(perm), max(1, args.batch // K)):
        idx = perm[b:b + max(1, args.batch // K)]
        pred = net(S[idx].repeat_interleave(K, 0), A[idx].reshape(-1, 5)).view(-1, K)
        loss = ((pred - C[idx]) ** 2).mean()
        opt.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(net.parameters(), 1.0)
        opt.step()
        net.clamp()
    sched.step()
    mse, reg, reg_r = evaluate(va_idx)
    if reg < best[0]:
        best = (reg, {k: v.detach().clone() for k, v in net.state_dict().items()}, e)
    if e % 5 == 0 or e == args.epochs - 1:
        print(f"epoch {e}: val MSE {mse:.3f}, argmin-of-K regret {reg:.3f} mm (random {reg_r:.3f})", flush=True)
net.load_state_dict(best[1])
mse, reg, reg_r = evaluate(va_idx)
print(f"{N} states x {K} actions ({val.sum()} val); best epoch {best[2]}: val MSE {mse:.3f}, argmin-of-K regret "
      f"{reg:.3f} mm (random action {reg_r:.3f} mm)")
torch.save({"model_state_dict": best[1], "arch": {"hidden": args.hidden, "n_layers": args.layers},
            "c_mu": c_mu, "c_sd": c_sd, "epoch": best[2], "val_regret": reg}, args.out)
print(f"saved {args.out}")
