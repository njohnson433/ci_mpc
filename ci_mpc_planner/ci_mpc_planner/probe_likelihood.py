"""Learned probe likelihood for the hole belief (hole_belief.py): log p(measurement | hole) up to a constant, from an
MLP trained contrastively on the Isaac probe logs (scripts/probe_dataset.py, scripts/train_probe_likelihood.py):
true hole positions vs hole hypotheses drawn from the belief grid, with randomized force-estimate and tip-height errors
(the FR3's end-effector force comes from link-side joint torque sensors through the model and Jacobian: gain /
cross-axis errors, bias drift after the tare, noise). NumPy inference, so the controller side needs no PyTorch.

Features per hypothesis (rotation invariant, the hole is axisymmetric): hole-TCP distance r [mm]; the tared lateral
force projected on the TCP->hole direction and across it [N]; Fz [N]; tip height [mm].
"""
import numpy as np


def features(rel_mm, tip_mm, force):
    """rel_mm: (H, 2) hole - TCP [mm]; tip_mm: scalar or (H,); force: (3,) or (H, 3) [N] -> (H, 5)."""
    rel_mm = np.atleast_2d(rel_mm)
    F = np.broadcast_to(np.asarray(force, float), (len(rel_mm), 3))
    r = np.linalg.norm(rel_mm, axis=1)
    u = rel_mm / np.maximum(r, 1e-6)[:, None]
    f_rad = np.sum(F[:, :2] * u, axis=1)
    f_tan = u[:, 0] * F[:, 1] - u[:, 1] * F[:, 0]
    return np.column_stack([r, f_rad, f_tan, F[:, 2], np.broadcast_to(tip_mm, r.shape)])


class ProbeLikelihood:
    def __init__(self, path, scale=1.0):
        d = np.load(path)
        n = sum(1 for k in d.files if k.startswith("W"))
        self.W = [(d[f"W{i}"], d[f"b{i}"]) for i in range(n)]
        self.mu, self.sd = d["mu"], d["sd"]
        # distance-prior correction: the contrastive logit is log p(meas | rel) + log p_pos(r) / p_neg(r); with probe
        # positions chosen by strategies p_pos(r) is not the grid's p_neg(r) (a flat reading then favoured holes ~4.5 mm
        # away: the belief chased its own probes). Subtract the histogram estimate when the file has one.
        self.rc_edges, self.rc = (d["rcorr_edges"], d["rcorr"]) if "rcorr" in d.files else (None, None)
        self.scale = scale  # temper the log-likelihood (< 1: trust each probe less)

    def logit(self, X):
        h = (X - self.mu) / self.sd
        for i, (W, b) in enumerate(self.W):
            h = h @ W.T + b
            if i < len(self.W) - 1:
                h = h / (1.0 + np.exp(-h))  # SiLU
        return h[:, 0]

    def __call__(self, h_xy, tcp_xy, tip_z, force):
        """Log-likelihood of each hole hypothesis h_xy (H, 2) [m] for a probe at tcp_xy [m] with tip height tip_z [m]
        and tared contact force (3,) [N]."""
        X = features((np.asarray(h_xy) - np.asarray(tcp_xy)[:2]) * 1000.0, tip_z * 1000.0, force)
        return self.scale * (self.logit(X) - self.r_correction(X[:, 0]))

    def r_correction(self, r_mm):
        if self.rc is None:
            return 0.0
        mid = 0.5 * (self.rc_edges[:-1] + self.rc_edges[1:])
        return np.interp(r_mm, mid, self.rc)
