"""Success-guided start sampling for the option-selector DAgger (idea from AlphaNIST, arXiv 2610.13354; see
docs/PROGRESS.md 2026-10-24). Starts come from two pools:

  - fresh engaged starts: hole error drawn as in the fair eval (r 0.3-1.8 mm, uniform direction), but the
    (magnitude x direction) bin is chosen with weight (1 - success) + eps, so bins where the current policy fails
    get more episodes;
  - snapshots: mid-insertion states (every `snap_every` steps, and jams: no depth progress over `jam_steps`) saved
    from earlier rollouts, each with its own success estimate, weighted the same way.

Success estimates are Beta(1, 1) posterior means over the outcomes of episodes started there (`update`). Why: the
earlier negative results were mostly about the start distribution (single-C3 teacher 8/80 from random starts,
DAgger regret 0.18 -> 0.45 when the selector's own failure states were rare in the data).
"""
import json

import numpy as np

R_EDGES = (0.3, 0.8, 1.3, 1.8)  # mm, as the fair eval's uniform(0.3, 1.8)
N_DIR = 8


class StartCache:
    def __init__(self, eps=0.1, p_snap=0.3):
        self.eps, self.p_snap = eps, p_snap
        nb = (len(R_EDGES) - 1) * N_DIR
        self.bin_n, self.bin_s = np.zeros(nb), np.zeros(nb)
        self.snaps = []  # dicts: off [m], x, u, step, n, s

    @staticmethod
    def bin_of(off):
        r = np.linalg.norm(off) * 1000
        i = int(np.clip(np.searchsorted(R_EDGES, r) - 1, 0, len(R_EDGES) - 2))
        j = int((np.arctan2(off[1], off[0]) % (2 * np.pi)) / (2 * np.pi) * N_DIR) % N_DIR
        return i * N_DIR + j

    def _w(self, n, s):
        return (1.0 - (s + 1.0) / (n + 2.0)) + self.eps

    def sample(self, rng):
        """-> (start dict: off, x (None: fresh), u, kind, id). Fresh starts still need the engaging press."""
        if self.snaps and rng.random() < self.p_snap:
            w = np.array([self._w(sn["n"], sn["s"]) for sn in self.snaps])
            k = int(rng.choice(len(self.snaps), p=w / w.sum()))
            sn = self.snaps[k]
            return {"off": np.array(sn["off"]), "x": np.array(sn["x"]), "u": np.array(sn["u"]), "kind": "snap", "id": k}
        w = self._w(self.bin_n, self.bin_s)
        b = int(rng.choice(len(w), p=w / w.sum()))
        i, j = divmod(b, N_DIR)
        r = rng.uniform(R_EDGES[i], R_EDGES[i + 1])
        a = rng.uniform(j, j + 1) * 2 * np.pi / N_DIR
        return {"off": np.array([r * np.cos(a), r * np.sin(a)]) / 1000.0, "x": None, "u": None, "kind": "bin", "id": b}

    def update(self, kind, idx, success):
        if kind == "snap":
            self.snaps[idx]["n"] += 1
            self.snaps[idx]["s"] += float(success)
        else:
            self.bin_n[idx] += 1
            self.bin_s[idx] += float(success)

    def add_snapshot(self, off, x, u, step, success):
        """A visited state; its success prior is the outcome of the episode it came from."""
        self.snaps.append({"off": list(map(float, off)), "x": list(map(float, x)), "u": list(map(float, u)),
                           "step": int(step), "n": 1, "s": float(success)})

    def summary(self):
        sr = (self.bin_s + 1) / (self.bin_n + 2)
        by_r = [float(sr[i * N_DIR:(i + 1) * N_DIR].mean()) for i in range(len(R_EDGES) - 1)]
        snap_sr = float(np.mean([(s["s"] + 1) / (s["n"] + 2) for s in self.snaps])) if self.snaps else float("nan")
        return {"episodes": int(self.bin_n.sum()), "success_by_r_bin": by_r, "snapshots": len(self.snaps),
                "snapshot_success": snap_sr}

    def save(self, path):
        json.dump({"eps": self.eps, "p_snap": self.p_snap, "bin_n": self.bin_n.tolist(), "bin_s": self.bin_s.tolist(),
                   "snaps": self.snaps}, open(path, "w"))

    @classmethod
    def load(cls, path):
        d = json.load(open(path))
        c = cls(d["eps"], d["p_snap"])
        c.bin_n, c.bin_s, c.snaps = np.array(d["bin_n"]), np.array(d["bin_s"]), d["snaps"]
        return c
