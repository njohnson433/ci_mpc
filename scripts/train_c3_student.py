#!/usr/bin/env python3
# Train the amortized CI-MPC student (step 3): MLP features (10) -> teacher command step (5), from
# c3_teacher_data.py files. Needs PyTorch: run with ~/IsaacLab/.venv/bin/python (the ROS python has none). Saves
# the weights and normalization as .npz for numpy inference (ci_mpc_planner/c3_student.py), so the controller side
# needs no PyTorch. Episodes are split into train / validation (no leakage between steps of one episode).
#   ~/IsaacLab/.venv/bin/python scripts/train_c3_student.py DATA_DIR OUT.npz [--hidden 64] [--epochs 400]
import argparse
import glob
import random

import numpy as np
import torch
from torch import nn

p = argparse.ArgumentParser()
p.add_argument("data")
p.add_argument("out")
p.add_argument("--hidden", type=int, default=64)
p.add_argument("--epochs", type=int, default=400)
p.add_argument("--lr", type=float, default=2e-3)
p.add_argument("--val", type=float, default=0.2)
p.add_argument("--seed", type=int, default=0)
p.add_argument("--device", default="cpu", help="cpu (default: the MLP is tiny and the GPU runs Isaac) or cuda")
args = p.parse_args()


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


class Student(nn.Module):
    """features (B, 10) -> command step (B, 5), both normalized."""

    def __init__(self, n_in: int, n_out: int, hidden: int) -> None:
        super().__init__()
        self.net = nn.Sequential(nn.Linear(n_in, hidden), nn.Tanh(), nn.Linear(hidden, hidden), nn.Tanh(),
                                 nn.Linear(hidden, n_out))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


set_seed(args.seed)
device = torch.device(args.device if args.device == "cpu" or torch.cuda.is_available() else "cpu")

Xs, Ys, eps = [], [], []
for i, f in enumerate(sorted(glob.glob(f"{args.data}/teacher_*.npz"))):
    d = np.load(f)
    if len(d["X"]) == 0:
        continue
    Xs.append(d["X"]), Ys.append(d["Y"]), eps.append(d["meta"][:, 0] + 1000 * i)
X, Y, ep = np.concatenate(Xs), np.concatenate(Ys), np.concatenate(eps)
rng = np.random.default_rng(args.seed)
ids = np.unique(ep)
val_ids = set(rng.choice(ids, max(1, int(args.val * len(ids))), replace=False))
val = np.array([e in val_ids for e in ep])
mu_x, sd_x = X[~val].mean(0), X[~val].std(0) + 1e-6
mu_y, sd_y = Y[~val].mean(0), Y[~val].std(0) + 1e-6
tx = lambda a: torch.tensor((a - mu_x) / sd_x, dtype=torch.float32, device=device)  # noqa: E731
ty = lambda a: torch.tensor((a - mu_y) / sd_y, dtype=torch.float32, device=device)  # noqa: E731
Xtr, Ytr, Xva, Yva = tx(X[~val]), ty(Y[~val]), tx(X[val]), ty(Y[val])
net = Student(X.shape[1], Y.shape[1], args.hidden).to(device)
opt = torch.optim.Adam(net.parameters(), lr=args.lr, weight_decay=1e-5)
criterion = nn.MSELoss()


@torch.no_grad()
def evaluate(xs: torch.Tensor, ys: torch.Tensor) -> float:
    net.eval()
    return criterion(net(xs), ys).item()


best = (np.inf, None, -1)
for e in range(args.epochs):
    net.train()
    perm = torch.randperm(len(Xtr), device=device)
    for b in range(0, len(Xtr), 128):
        idx = perm[b:b + 128]
        loss = criterion(net(Xtr[idx]), Ytr[idx])
        opt.zero_grad(set_to_none=True)
        loss.backward()
        nn.utils.clip_grad_norm_(net.parameters(), max_norm=1.0)
        opt.step()
    lv, lt = evaluate(Xva, Yva), evaluate(Xtr, Ytr)
    if lv < best[0]:
        best = (lv, {k: v.detach().clone() for k, v in net.state_dict().items()}, e)
    if e % 50 == 0:
        print(f"epoch {e}: train {lt:.4f}, val {lv:.4f} (normalized MSE)")
net.load_state_dict(best[1])
net.eval()
with torch.no_grad():
    pred = net(Xva).cpu().numpy() * sd_y + mu_y
W = [prm.detach().cpu().numpy() for prm in net.parameters()]
err = pred - Y[val]
r2 = 1 - (err ** 2).sum(0) / ((Y[val] - Y[val].mean(0)) ** 2).sum(0)
torch.save({"epoch": best[2], "model_state_dict": best[1], "optimizer_state_dict": opt.state_dict(),
            "val_loss": best[0], "norm": {"mu_x": mu_x, "sd_x": sd_x, "mu_y": mu_y, "sd_y": sd_y},
            "hidden": args.hidden}, args.out.replace(".npz", ".pt"))
print(f"samples: {(~val).sum()} train, {val.sum()} val ({len(ids)} episodes); best val {best[0]:.4f} (epoch {best[2]}); "
      f"val RMSE per output {np.round(np.sqrt((err ** 2).mean(0)), 3)} (mm, mm, mm, mrad, mrad), R^2 {np.round(r2, 2)}")
np.savez(args.out, W0=W[0], b0=W[1], W1=W[2], b1=W[3], W2=W[4], b2=W[5], mu_x=mu_x, sd_x=sd_x, mu_y=mu_y, sd_y=sd_y)
print(f"saved {args.out}")
