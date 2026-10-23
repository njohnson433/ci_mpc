#!/usr/bin/env python3
# Train the option selector (amortized sampled C3, step 3): MLP features (10) -> logits over the teacher's options,
# cross-entropy against soft targets softmax(scores / tau) (near-ties stay ties instead of contradicting labels;
# failed options score -inf and get 0). Data: selector_dagger.py files (all DAgger rounds aggregated). Run with
# ~/IsaacLab/.venv/bin/python (PyTorch); saves .npz for numpy inference (C3Selector) and a .pt checkpoint.
#   ~/IsaacLab/.venv/bin/python scripts/train_c3_selector.py "DIR/*.npz" OUT.npz [--tau 0.3]
import argparse
import glob
import random

import numpy as np
import torch
from torch import nn

p = argparse.ArgumentParser()
p.add_argument("data", help="glob of selector_dagger.py files")
p.add_argument("out")
p.add_argument("--tau", type=float, default=0.3, help="softmax temperature on the scores (score units: ~mm)")
p.add_argument("--hidden", type=int, default=64)
p.add_argument("--epochs", type=int, default=300)
p.add_argument("--lr", type=float, default=2e-3)
p.add_argument("--val", type=float, default=0.2)
p.add_argument("--seed", type=int, default=0)
p.add_argument("--device", default="cpu")
args = p.parse_args()


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


class Selector(nn.Module):
    """features (B, 10) -> option logits (B, n_options)."""

    def __init__(self, n_in: int, n_out: int, hidden: int) -> None:
        super().__init__()
        self.net = nn.Sequential(nn.Linear(n_in, hidden), nn.Tanh(), nn.Linear(hidden, hidden), nn.Tanh(),
                                 nn.Linear(hidden, n_out))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


set_seed(args.seed)
device = torch.device(args.device if args.device == "cpu" or torch.cuda.is_available() else "cpu")
Fs, Ss, eps = [], [], []
for i, f in enumerate(sorted(glob.glob(args.data))):
    d = np.load(f)
    if len(d["F"]):
        Fs.append(d["F"]), Ss.append(d["S"]), eps.append(d["meta"][:, 0] + 1000 * i)
X, S, ep = np.concatenate(Fs), np.concatenate(Ss), np.concatenate(eps)
finite = np.isfinite(S)
Sz = np.where(finite, S - np.where(finite, S, -1e9).max(1, keepdims=True), -np.inf)
T = np.exp(Sz / args.tau)
T = T / T.sum(1, keepdims=True)
rng = np.random.default_rng(args.seed)
ids = np.unique(ep)
val = np.isin(ep, rng.choice(ids, max(1, int(args.val * len(ids))), replace=False))
mu_x, sd_x = X[~val].mean(0), X[~val].std(0) + 1e-6
tx = lambda a: torch.tensor((a - mu_x) / sd_x, dtype=torch.float32, device=device)  # noqa: E731
tt = lambda a: torch.tensor(a, dtype=torch.float32, device=device)  # noqa: E731
Xtr, Ttr, Xva, Tva = tx(X[~val]), tt(T[~val]), tx(X[val]), tt(T[val])
net = Selector(X.shape[1], S.shape[1], args.hidden).to(device)
opt = torch.optim.Adam(net.parameters(), lr=args.lr, weight_decay=1e-4)


def soft_ce(logits: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    return -(target * torch.log_softmax(logits, dim=1)).sum(1).mean()


@torch.no_grad()
def evaluate(xs: torch.Tensor, ts: torch.Tensor) -> tuple[float, float, float]:
    """(soft CE, top-1 agreement with the teacher's best, mean score regret of the chosen option)."""
    net.eval()
    lg = net(xs)
    choice = lg.argmax(1).cpu().numpy()
    best = ts.argmax(1).cpu().numpy()
    return soft_ce(lg, ts).item(), float((choice == best).mean()), choice


best = (np.inf, None, -1)
for e in range(args.epochs):
    net.train()
    perm = torch.randperm(len(Xtr), device=device)
    for b in range(0, len(Xtr), 128):
        idx = perm[b:b + 128]
        loss = soft_ce(net(Xtr[idx]), Ttr[idx])
        opt.zero_grad(set_to_none=True)
        loss.backward()
        nn.utils.clip_grad_norm_(net.parameters(), max_norm=1.0)
        opt.step()
    lv, acc, _ = evaluate(Xva, Tva)
    if lv < best[0]:
        best = (lv, {k: v.detach().clone() for k, v in net.state_dict().items()}, e)
    if e % 50 == 0:
        print(f"epoch {e}: val soft-CE {lv:.3f}, top-1 {acc:.2f}")
net.load_state_dict(best[1])
lv, acc, choice = evaluate(Xva, Tva)
Sv = S[val]
regret = np.nanmean(np.where(np.isfinite(Sv).all(1), np.nanmax(np.where(np.isfinite(Sv), Sv, np.nan), 1)
                             - Sv[np.arange(len(Sv)), choice], np.nan))
heur_regret = np.nanmean(np.nanmax(np.where(np.isfinite(Sv), Sv, np.nan), 1) - Sv[:, 0])
print(f"samples {(~val).sum()} train / {val.sum()} val ({len(ids)} episodes); best epoch {best[2]}: val soft-CE {lv:.3f}, "
      f"top-1 {acc:.2f}; score regret of the chosen option {regret:.3f} (always heuristic: {heur_regret:.3f})")
W = [prm.detach().cpu().numpy() for prm in net.parameters()]
torch.save({"epoch": best[2], "model_state_dict": best[1], "val_loss": best[0], "norm": {"mu_x": mu_x, "sd_x": sd_x},
            "tau": args.tau, "hidden": args.hidden}, args.out.replace(".npz", ".pt"))
np.savez(args.out, W0=W[0], b0=W[1], W1=W[2], b1=W[3], W2=W[4], b2=W[5], mu_x=mu_x, sd_x=sd_x)
print(f"saved {args.out}")
