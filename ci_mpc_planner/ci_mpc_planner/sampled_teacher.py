"""Sampled C3 teacher for the amortized CI-MPC (step 3): per step, C3+ is solved from several warm starts (rollouts of
different nominal command sequences through the convex plant), each plan is executed for `eval_steps` in a copy of
the plant, scored, and the best first command is returned. The force-following heuristic competes as a candidate,
so the teacher is never worse than it on the score. Offline only (seconds per step); privileged: the teacher's
model and evaluation use the true hole.

Why: a single C3 solve depends on its warm start (stuck at the opening, or the right low-force mode sequence);
randomized starts entered the bore in only 8 of 80 episodes with one rollout warm start. Sampling warm starts is
the C3+ remedy (sampled initial conditions), done here at teacher time.
"""
import copy

import numpy as np

from ci_mpc_planner.c3_3d import C33Config, PegHoleC3_3D


def heuristic_command(x, wrench):
    """Force-following centering (as in Isaac): (command, feedforward)."""
    lat = wrench[:2]
    if np.linalg.norm(lat) > 0.3:
        d = lat / np.linalg.norm(lat)
        return np.array([x[0] + 4e-4 * d[0], x[1] + 4e-4 * d[1], x[2] - 0.002, 0, 0]), \
            np.array([2.5 * d[0], 2.5 * d[1], -2.0, 0, 0])
    return np.array([x[0], x[1], x[2] - 0.004, 0, 0]), np.array([0, 0, -3.0, 0, 0])


class SampledTeacher:
    def __init__(self, model, compliance, arm_friction, eval_steps=4, f_soft=3.0, w_depth=1.0, w_center=1.0,
                 w_force=0.5):
        cfg = C33Config()
        cfg.rollout_every = 1
        cfg.arm_friction = np.asarray(arm_friction)
        self.cfg = cfg
        self.mpc = PegHoleC3_3D(model, compliance, cfg)
        self.model, self.comp, self.fa = model, np.asarray(compliance), np.asarray(arm_friction)
        self.eval_steps, self.f_soft = eval_steps, f_soft
        self.w = (w_depth, w_center, w_force)

    def nominals(self, x, goal):
        """Candidate nominal command sequences (horizon x 5)."""
        H, du = self.cfg.horizon, self.cfg.du_max / 1000.0
        out = []
        for lat, press, tilt in ((np.zeros(2), 0.001, np.zeros(2)), (np.zeros(2), 0.004, np.zeros(2)),
                                 (None, 0.003, np.zeros(2)),
                                 (np.array([3e-4, 0]), 0.002, np.zeros(2)), (np.array([-3e-4, 0]), 0.002, np.zeros(2)),
                                 (np.array([0, 3e-4]), 0.002, np.zeros(2)), (np.array([0, -3e-4]), 0.002, np.zeros(2)),
                                 (np.zeros(2), 0.002, np.array([0.01, 0.0])), (np.zeros(2), 0.002, np.array([0.0, 0.01]))):
            u, seq = x.copy(), []
            for k in range(H):
                u = u.copy()
                if lat is not None:
                    u[:2] += np.clip(goal[:2] + lat - u[:2], -du[:2], du[:2])
                u[2] = max(u[2] - du[2], x[2] - press)
                u[3:] = tilt
                seq.append(u)
            out.append(np.array(seq))
        return out

    def score(self, plant, x, commands, ffs=None):
        """Execute commands in a copy of the plant; higher is better."""
        p = copy.copy(plant)
        xs, peak = x.copy(), 0.0
        for k, u in enumerate(commands[:self.eval_steps]):
            ff = np.zeros(5) if ffs is None else ffs[k]
            xs, lam, nc = p.step(xs, u + self.comp * ff, self.comp, self.fa, eps=1e-6, max_iter=4000)
            if nc:
                peak = max(peak, np.linalg.norm(p.contact_wrench(lam)[:3]))
        tip = p.tip(xs)
        wd, wc, wf = self.w
        return -wd * tip[2] * 1000 - wc * np.linalg.norm(tip[:2]) * 1000 - wf * max(peak - self.f_soft, 0.0) ** 2, peak

    N_OPTIONS = 10  # 0: heuristic, 1..9: C3 with nominal 0..8

    def option(self, i, x, goal, u_prev, wrench):
        """Command (u, ff, planned sequence, its feedforwards) of option i, without evaluating it."""
        if i == 0:
            u_h, ff_h = heuristic_command(x, wrench)
            return u_h, ff_h, [u_h] * self.eval_steps, [ff_h] * self.eval_steps
        self.mpc.prev = None
        u0, info = self.mpc.solve(x, goal, u_prev, nominal=self.nominals(x, goal)[i - 1])
        if info.get("status") != "ok":
            return None
        return u0, np.zeros(5), info["U"], None

    def act(self, plant, x, goal, u_prev, wrench):
        """Best first command (and its feedforward) over the sampled C3 plans and the heuristic; info["scores"] has
        every option's score (-inf if it failed), for training an option selector."""
        scores = np.full(self.N_OPTIONS, -np.inf)
        cmds, peaks = [None] * self.N_OPTIONS, np.zeros(self.N_OPTIONS)
        for i in range(self.N_OPTIONS):
            o = self.option(i, x, goal, u_prev, wrench)
            if o is None:
                continue
            u, ff, seq, ffs = o
            scores[i], peaks[i] = self.score(plant, x, seq, ffs)
            cmds[i] = (u, ff)
        b = int(np.argmax(scores))
        src = "heuristic" if b == 0 else f"c3[{b - 1}]"
        return cmds[b][0], cmds[b][1], {"source": src, "score": scores[b], "peak": peaks[b], "scores": scores,
                                        "best": b}
