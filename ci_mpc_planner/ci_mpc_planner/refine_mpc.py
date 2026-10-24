"""Sample, then QP-refine (2026-10-24): the GPU sampling MPC (sampling_mpc.SamplingMPC) handles the multimodality (which
way around, which contact sequence) by brute force; its best sequence then fixes the mode, and one 3D C3 solve
(c3_3d.PegHoleC3_3D, the convex QP / ADMM inside that mode) warm-started from it refines the commands. The refined
plan replaces the sample only if it scores at least as well in the sampler's own rollout (same plant, cost and
belief samples), so the hybrid is never worse than the sampler on its cost.

Why: sampling alone reached 30/30 offline but with a force tail (p90 5.36 N, max 7.55 N); C3 alone gave 2-3x lower
contact forces in Isaac but got stuck without a good warm start (mode). The combination asks whether a convex
solve inside the sampled mode lowers the forces.
"""
import numpy as np
import torch

from ci_mpc_planner.c3_3d import C33Config, PegHoleC3_3D
from ci_mpc_planner.peg_hole_3d import PegHole3D
from ci_mpc_planner.sampling_mpc import SamplingMPC


class SampleRefineMPC:
    def __init__(self, compliance, arm_friction, margin=0.0, refiner="c3", icnn_path="", box=(2e-4, 1e-3, 0.005),
                 **sampler_kw):
        """refiner: "c3" (one C3 solve warm-started from the best sample) or "icnn" (the learned convex cost-to-go,
        minimized in a box of +-box = (lateral step, press, tilt) around the best sample's action parameters)."""
        self.smpc = SamplingMPC(compliance, arm_friction, **sampler_kw)
        cfg = C33Config()
        cfg.rollout_every = 1  # the nominal is fresh every step
        cfg.arm_friction = np.asarray(arm_friction)
        self.cfg, self.comp, self.margin = cfg, np.asarray(compliance), margin
        self.c3, self.c3_hole = None, None
        self.u_prev = None
        self.stats = {"refined": 0, "kept": 0, "c3_failed": 0}
        self.refiner, self.box = refiner, box
        if refiner == "icnn":
            from ci_mpc_planner.icnn import ICNNPlanner
            self.icnn = ICNNPlanner(icnn_path)

    def _c3_for(self, hole_xy):
        hole_xy = np.asarray(hole_xy, float)
        if self.c3 is None or np.linalg.norm(hole_xy - self.c3_hole) > 1e-4:  # rebuild when the estimate moves
            self.c3 = PegHoleC3_3D(PegHole3D(hole_xy=hole_xy, n_ring=8, n_rim=12), self.comp, self.cfg)
            self.c3_hole = hole_xy
        return self.c3

    @torch.no_grad()
    def score_seqs(self, x, seqs, hole_xy, hole_samples=None):
        """Sampler rollout scores of open-loop (k, H, 5) sequences (padded to the sampler's fixed batch)."""
        s = self.smpc
        seqs = s.t(np.asarray(seqs))
        k = seqs.shape[0]
        seqs = torch.cat([seqs, seqs[:1].repeat(s.n_total - k, 1, 1)], 0)
        if hole_samples is not None and len(hole_samples) > 1:
            M = len(hole_samples)
            hs = s.t(hole_samples)
            sc, _, _ = s.rollout(x, seqs.repeat_interleave(M, 0), hs.repeat(seqs.shape[0], 1))
            sc = sc.view(-1, M).mean(1)
        else:
            sc, _, _ = s.rollout(x, seqs, hole_xy)
        return sc[:k].double().cpu().numpy()

    def _icnn_refine(self, x, nominal, hole_xy, hole_samples):
        """The best sample's action parameters (lateral step, press below x, tilt), a box around them, one convex
        minimization of the ICNN cost-to-go in it -> (first command, {"status", "U"})."""
        from ci_mpc_planner.icnn import sequence
        a0 = np.array([*(nominal[0, :2] - x[:2]), x[2] - nominal[:, 2].min(), *nominal[0, 3:]])
        b = np.array([self.box[0], self.box[0], self.box[1], self.box[2], self.box[2]])
        a, _ = self.icnn.plan(x, hole_xy, self.smpc.tb, hole_samples=hole_samples, box=(a0 - b, a0 + b))
        U = sequence(torch.tensor(x[None], dtype=torch.float32), torch.tensor(a[None], dtype=torch.float32))[0]
        U = U.double().numpy()
        return U[0], {"status": "ok", "U": U}

    def act(self, x, hole_xy, wrench=None, hole_samples=None):
        x = np.asarray(x, float)
        u_s, info = self.smpc.act(x, hole_xy, wrench, hole_samples=hole_samples)
        if self.u_prev is None:
            self.u_prev = x.copy()
        H = self.smpc.H
        nominal = self.smpc.best.double().cpu().numpy() if self.smpc.best is not None else np.repeat(u_s[None], H, 0)
        if self.refiner == "icnn":
            u_c, ic = self._icnn_refine(x, nominal, hole_xy, hole_samples)
        else:
            c3 = self._c3_for(hole_xy)
            c3.prev = None
            goal = np.array([*np.asarray(hole_xy, float), self.smpc.tb - c3.model.g.depth, 0.0, 0.0])
            u_c, ic = c3.solve(x, goal, self.u_prev, nominal=nominal)
        u, src = u_s, info["source"]
        if ic.get("status") != "ok":
            self.stats["c3_failed"] += 1
        else:
            s_c3 = float(self.score_seqs(x, np.asarray(ic["U"])[None], hole_xy, hole_samples)[0])
            info["score_c3"] = s_c3
            if s_c3 >= info["score"] + self.margin:
                u, src = np.asarray(u_c, float), self.refiner + "_refined_" + info["source"]
                info["score"] = s_c3
                self.stats["refined"] += 1
                # the sampler keeps the refined plan as its "previous best" (its random candidates shift from it)
                self.smpc.best = self.smpc.t(np.asarray(ic["U"]))
            else:
                self.stats["kept"] += 1
        self.u_prev = u.copy()
        info["source"] = src
        return u, info
