"""The probe strategy of peg_insert_demo.py (--strategy probe) as per-env state machines over N envs (ROADMAP 9, step 3).

Same logic and parameters as the ros2_control demo with the recommended settings (plain probe + learned likelihood v3,
calibration touch, --track-height 5, --centering-bound 0.008, --stuck-dither 0.0008; jam recovery optional), one
planning step (plan_period 0.1 s) per env step:
  touch      hover 0.4 s, lateral move (stiction feedforward 2.5 N, <= 25 steps, until 0.2 mm), settle 0.3 s, tare,
             press (0.5 mm / step, until Fz > 1 N or the tip 3 mm in, <= 40 steps), hold 1 mm in 0.3 s, measure
             (TCP pose, force mean over 0.3 s)
  calibration touch 9.5 mm from the estimate on the diagonal (grasp height offset), then probes
  belief     grid 25 x 25 (0.5 mm) around the estimate, learned log-likelihood (batched MLP), next probe = the most
             likely 2 mm capture disc away from earlier flat probes (1.5 mm), ties nearest to the TCP
  engaged    tip < -0.3 mm or lateral > 1.2 N: centering (follow the lateral force, <= 60 steps, stop at 8 mm)
  push       tip 3 mm in and inside the bound: push to the bottom (6 / 2 mm below the TCP by Fz < 4 N), done within
             1 mm of the depth or after 150 (+30 per jam retry) steps; jam recovery (stall / lateral 3 N): lift, shift
Model frame as in the demo: origin O = the hole estimate (xy) at the opening's height, orientation R = the grasp's.
Force = -(Franka-sign estimate) - tare. Truth for scoring: env.ci_hole_top() / held_pos (success as the demo:
peg bottom within 1 mm of the hole bottom, xy within 1.5 mm, in the hole frame).
"""
import math

import numpy as np
import torch

# phases
HOVER, MOVE, SETTLE, PRESS, HOLD, CENTER, PUSH, JAM, DONE, FAILED = range(10)
NAMES = ("hover", "move", "settle", "press", "hold", "center", "push", "jam", "done", "failed")


class Likelihood:
    """probe_likelihood.ProbeLikelihood in torch, for (N, H) hypotheses at once."""

    def __init__(self, path, device, scale=1.0):
        d = np.load(path)
        n = sum(1 for k in d.files if k.startswith("W"))
        t = lambda a: torch.tensor(a, dtype=torch.float32, device=device)  # noqa: E731
        self.W = [(t(d[f"W{i}"]), t(d[f"b{i}"])) for i in range(n)]
        self.mu, self.sd = t(d["mu"]), t(d["sd"])
        self.rc_mid = t(0.5 * (d["rcorr_edges"][:-1] + d["rcorr_edges"][1:])) if "rcorr" in d.files else None
        self.rc = t(d["rcorr"]) if "rcorr" in d.files else None
        self.scale = scale

    def _interp(self, x):
        xp, fp = self.rc_mid, self.rc
        i = torch.clamp(torch.searchsorted(xp, x.contiguous()), 1, len(xp) - 1)
        x0, x1, f0, f1 = xp[i - 1], xp[i], fp[i - 1], fp[i]
        w = torch.clamp((x - x0) / (x1 - x0), 0.0, 1.0)
        return f0 + w * (f1 - f0)

    def __call__(self, h, tcp_xy, tip_z, force):
        """h (H, 2), tcp_xy (N, 2) [m], tip_z (N,) [m], force (N, 3) [N] -> (N, H)."""
        rel = (h[None] - tcp_xy[:, None]) * 1000.0
        r = rel.norm(dim=-1)
        u = rel / r.clamp(min=1e-6)[..., None]
        F = force[:, None, :]
        f_rad = (F[..., :2] * u).sum(-1)
        f_tan = u[..., 0] * F[..., 1] - u[..., 1] * F[..., 0]
        X = torch.stack((r, f_rad, f_tan, F[..., 2].expand_as(r), (tip_z * 1000.0)[:, None].expand_as(r)), -1)
        x = (X - self.mu) / self.sd
        for i, (W, b) in enumerate(self.W):
            x = x @ W.T + b
            if i < len(self.W) - 1:
                x = x * torch.sigmoid(x)  # SiLU
        out = x[..., 0]
        if self.rc is not None:
            out = out - self._interp(r)
        return self.scale * out


class BatchedProbe:
    def __init__(self, env, likelihood_path, max_probes=25, timeout=150.0, track_height=5, centering_bound=0.008,
                 stuck_dither=0.0008, jam_recovery=0, jam_shift=0.0006, jam_lift=0.003, jam_force=3.0, seed=0):
        self.env, self.dev, N = env, env.device, env.num_envs
        self.N, self.dt = N, env.step_dt
        self.max_probes, self.timeout, self.track_height = max_probes, timeout, track_height
        self.cbound, self.stuck_dither = centering_bound, stuck_dither
        self.jam_recovery, self.jam_shift, self.jam_lift, self.jam_force = jam_recovery, jam_shift, jam_lift, jam_force
        self.tip_below_tcp = env.cfg_task.held_asset_cfg.height - 0.010  # the demo's nominal (grasp depth 10 mm)
        self.depth = env.cfg_task.fixed_asset_cfg.height
        self.engage, self.engage_lateral, self.sff, self.max_force = 0.003, 1.2, 3.0, 4.0
        self.z_hover = self.tip_below_tcp + 0.001
        self.calib_touch = 0.0095
        self.gen = torch.Generator(device=self.dev).manual_seed(seed)
        self.lik = Likelihood(likelihood_path, self.dev)
        ax = torch.arange(-0.006, 0.006 + 1e-9, 0.0005, device=self.dev)
        gx, gy = torch.meshgrid(ax, ax, indexing="xy")
        self.h = torch.stack((gx.reshape(-1), gy.reshape(-1)), 1)  # (H, 2) model frame
        self.cands = self.h[::3]
        self.cap = ((self.cands[:, None] - self.h[None]).norm(dim=-1) < 0.002).float()  # (C, H)
        self.K = torch.tensor((1500.0, 1500.0, 500.0, 260.0, 260.0, 90.0), device=self.dev)
        self.fbuf_len = 10  # force history: 1 s at 0.1 s steps

    # ---- episode
    def reset(self):
        N, d, f = self.N, self.dev, dict(device=self.dev)
        env, c = self.env, self.env.ctrl
        c.K[:] = self.K
        c.integral_enabled[:] = False
        c.twist_d[:] = 0.0
        c.D_cmd[:] = 0.0
        self.O = env.ci_hole_estimate().clone()  # (N, 3): estimate xy, opening height
        self.R = c.rot_d.clone()  # grasp orientation
        self.logw = torch.zeros(N, len(self.h), **f)
        self.phase = torch.full((N,), HOVER, dtype=torch.long, device=d)
        self.k = torch.zeros(N, dtype=torch.long, device=d)  # steps in the current phase
        self.calib = torch.ones(N, dtype=torch.bool, device=d)  # the current touch is the calibration touch
        self.target = (torch.tensor([-1.0, -1.0], **f) / math.sqrt(2.0) * self.calib_touch).repeat(N, 1)
        self.z_bias = torch.zeros(N, **f)
        self.hover_xy, self.settle_xy = torch.zeros(N, 2, **f), torch.zeros(N, 2, **f)
        self.uz, self.hold_z = torch.zeros(N, **f), torch.zeros(N, **f)
        self.hold_depth = torch.full((N,), 0.001, **f)  # touch: hold this far below the contact, for settle_steps
        self.settle_steps = torch.full((N,), 3, dtype=torch.long, device=d)
        self.tare = torch.zeros(N, 3, **f)
        self.n_probes = torch.zeros(N, dtype=torch.long, device=d)
        self.probes = torch.zeros(N, self.max_probes + 2, 2, **f)
        self.n_flat = torch.zeros(N, dtype=torch.long, device=d)
        self.flat_raw = torch.full((N, self.max_probes + 3), float("nan"), **f)
        self.recent = torch.full((N, 3, 2), float("nan"), **f)
        self.fbuf = torch.zeros(N, self.fbuf_len, 3, **f)  # raw force on the peg, newest last
        self.depth_hist = torch.full((N, 11), float("nan"), **f)
        self.n_hist = torch.zeros(N, dtype=torch.long, device=d)
        self.retries = torch.zeros(N, dtype=torch.long, device=d)
        self.anchor = torch.full((N, 2), float("nan"), **f)
        self.k_push = torch.zeros(N, dtype=torch.long, device=d)
        self.jam_xy, self.jam_z = torch.zeros(N, 2, **f), torch.zeros(N, **f)
        self.t = 0.0
        self.t_done = torch.full((N,), float("nan"), **f)
        self.t_engage = torch.full((N,), float("nan"), **f)
        self.cmd_xy, self.cmd_z, self.cmd_ff = torch.zeros(N, 2, **f), torch.zeros(N, **f), torch.zeros(N, 3, **f)
        x = self.measure()
        self.cmd_xy[:], self.cmd_z[:] = x[:, :2], x[:, 2]

    # ---- measurements (model frame, as measure5 / force3)
    def measure(self):
        tcp = self.env.ci_state()[5]
        x = tcp - self.O
        x[:, 2] -= self.z_bias
        return x

    def tip(self, x):
        return x[:, 2] - self.tip_below_tcp

    def force(self, window_s, raw=False):
        n = max(1, int(round(window_s / self.dt)))
        f = self.fbuf[:, -n:].mean(1)
        return f if raw else f - self.tare

    # ---- belief
    def update_belief(self, m, x, F):
        if m.any():
            ll = self.lik(self.h, x[m, :2], self.tip(x)[m], F[m])
            lw = self.logw[m] + ll
            self.logw[m] = lw - lw.max(1, keepdim=True).values

    def next_probe(self, m, x):
        w = torch.softmax(self.logw[m], 1)
        ps = w @ self.cap.T  # (M, C)
        pr, npb = self.probes[m], self.n_flat[m]
        dist = (self.cands[None, :, None] - pr[:, None]).norm(dim=-1)  # (M, C, P)
        valid = torch.arange(pr.shape[1], device=self.dev)[None, None] < npb[:, None, None]
        excl = ((dist < 0.0015) & valid).any(-1)
        ps = torch.where(excl, torch.full_like(ps, -1.0), ps)
        near = ps >= 0.9 * ps.max(1, keepdim=True).values
        dc = (self.cands[None] - x[m, None, :2]).norm(dim=-1)
        dc = torch.where(near, dc, torch.full_like(dc, 1e9))
        tgt = self.cands[dc.argmin(1)]
        if self.stuck_dither > 0:
            rec = torch.cat((self.recent[m, 1:], x[m, None, :2]), 1)
            self.recent[m] = rec
            spread = (rec - rec[:, -1:]).norm(dim=-1).amax(1)
            stuck = ~torch.isnan(spread) & (spread < 4e-4)
            a = 2 * math.pi * torch.rand(int(m.sum()), device=self.dev, generator=self.gen)
            tgt = tgt + stuck[:, None] * self.stuck_dither * torch.stack((torch.cos(a), torch.sin(a)), 1)
        return tgt

    def estimate(self):
        w = torch.softmax(self.logw, 1)
        mean = w @ self.h
        return mean, (w * ((self.h[None] - mean[:, None]) ** 2).sum(-1)).sum(1).sqrt()

    # ---- one planning step
    def _cmd(self, m, xy, z, ff=None):
        self.cmd_xy[m], self.cmd_z[m] = xy, z
        self.cmd_ff[m] = 0.0 if ff is None else ff
        self.pending &= ~m

    def _go(self, m, phase):
        self.phase[m], self.k[m] = phase, 0

    def _after_measure(self, m, x, F):
        """End of a touch: calibration / belief update, then centering, push or the next probe."""
        tip = self.tip(x)
        cal = m & self.calib
        ok = cal & (F[:, 2] > 1.0) & (tip.abs() < 0.0025)
        self.z_bias[ok] = tip[ok]
        if cal.any():
            self.calib[cal] = False
            self.target[cal] = 0.0
            self._go(cal, HOVER)
        pm = m & ~cal
        if not pm.any():
            return
        first = pm & (self.n_probes == 0) & (self.z_bias == 0.0) & (F[:, 2] > 1.0) & (F[:, :2].norm(dim=1) < 1.5) & \
            (tip.abs() < 0.0015)
        self.z_bias[first] = tip[first]
        x = self.measure()
        tip = self.tip(x)
        self.update_belief(pm, x, F)
        self.n_probes[pm] += 1
        engaged = (tip < -3e-4) | (F[:, :2].norm(dim=1) > self.engage_lateral)
        flat = pm & ~engaged
        if flat.any():
            i = torch.nonzero(flat)[:, 0]
            self.probes[i, self.n_flat[i]] = x[i, :2]
            self.n_flat[i] += 1
            if self.track_height > 0:  # raw tip heights (bias added back), seeded with the bias at the first one
                cnt = (~torch.isnan(self.flat_raw[i])).sum(1)
                seed = i[cnt == 0]
                self.flat_raw[seed, 0] = self.z_bias[seed]
                cnt = (~torch.isnan(self.flat_raw[i])).sum(1)
                self.flat_raw[i, cnt] = tip[i] + self.z_bias[i]
                cnt = cnt + 1
                j = torch.arange(self.flat_raw.shape[1], device=self.dev)[None]
                sel = (j >= cnt[:, None] - self.track_height) & (j < cnt[:, None])
                last = torch.where(sel, self.flat_raw[i], torch.full_like(self.flat_raw[i], float("nan")))
                self.z_bias[i] = torch.nanmedian(last, 1).values
        self._go(pm & engaged, CENTER)
        self._probe_or_fail(pm & ~engaged, x)

    def _probe_or_fail(self, m, x):
        if not m.any():
            return
        out = m & ((self.n_probes >= self.max_probes) | (self.t >= self.timeout))
        self._go(out, FAILED)
        go = m & ~out
        if go.any():
            self.target[go] = self.next_probe(go, x)
            self._go(go, HOVER)

    def step(self):
        """Decide the commands for this planning step (after the previous env step's measurements)."""
        env, c = self.env, self.env.ctrl
        f_raw = -env.ci_force_mean()[:, :3]  # force on the peg
        self.fbuf = torch.cat((self.fbuf[:, 1:], f_raw[:, None]), 1)
        self.pending = torch.ones(self.N, dtype=torch.bool, device=self.dev)
        for _ in range(4):  # phases that end hand over to the next one within the same step
            x = self.measure()
            ph, k = self.phase, self.k
            P = self.pending
            # hover
            m = P & (ph == HOVER)
            if m.any():
                self.hover_xy[m & (k == 0)] = x[m & (k == 0), :2]
                go = m & (k >= 4)
                self._go(go, MOVE)
                h = m & ~go
                self._cmd(h, self.hover_xy[h], torch.full_like(self.cmd_z[h], self.z_hover))
            # move
            m = self.pending & (self.phase == MOVE)
            if m.any():
                d = self.target - x[:, :2]
                dn = d.norm(dim=1)
                done = m & ((dn < 2e-4) | (self.k >= 25))
                self.settle_xy[done] = torch.where((dn[done] > 2e-4)[:, None], x[done, :2], self.target[done])
                self._go(done, SETTLE)
                mv = m & ~done
                if mv.any():
                    ff = torch.zeros(int(mv.sum()), 3, device=self.dev)
                    ff[:, :2] = 2.5 * d[mv] / dn[mv, None]
                    self._cmd(mv, self.target[mv], torch.full_like(self.cmd_z[mv], self.z_hover), ff)
            # settle, then tare
            m = self.pending & (self.phase == SETTLE)
            if m.any():
                done = m & (self.k >= 3)
                self.tare[done] = self.force(0.25, raw=True)[done]
                self.uz[done] = x[done, 2]
                self._go(done, PRESS)
                s = m & ~done
                self._cmd(s, self.settle_xy[s], torch.full_like(self.cmd_z[s], self.z_hover))
            # press until contact / in the hole
            m = self.pending & (self.phase == PRESS)
            if m.any():
                F = self.force(0.1)
                done = m & (((self.k > 0) & ((F[:, 2] > 1.0) | (self.tip(x) < -self.engage))) | (self.k >= 40))
                self.hold_z[done] = x[done, 2] - self.hold_depth[done]
                self._go(done, HOLD)
                pr = m & ~done
                self.uz[pr] = torch.maximum(self.uz[pr] - 5e-4, x[pr, 2] - 0.006)
                ff = torch.zeros(int(pr.sum()), 3, device=self.dev)
                ff[:, 2] = -self.sff
                self._cmd(pr, self.target[pr], self.uz[pr], ff)
            # hold, then measure
            m = self.pending & (self.phase == HOLD)
            if m.any():
                done = m & (self.k >= self.settle_steps)
                if done.any():
                    self._after_measure(done, x, self.force(0.3))
                hd = m & ~done
                ff = torch.zeros(int(hd.sum()), 3, device=self.dev)
                ff[:, 2] = -self.sff
                self._cmd(hd, self.target[hd], self.hold_z[hd], ff)
            # centering on the chamfer
            m = self.pending & (self.phase == CENTER)
            if m.any():
                tip = self.tip(x)
                out_b = (x[:, :2].norm(dim=1) > self.cbound) if self.cbound > 0 else torch.zeros_like(m)
                done = m & (((self.k > 0) & ((tip < -self.engage) | out_b)) | (self.k >= 60))
                if done.any():
                    push = done & (tip < -self.engage) & ~out_b
                    self._start_push(push, x)
                    self._probe_or_fail(done & ~push, x)
                cm = m & ~done
                if cm.any():
                    lat = self.force(0.1)[:, :2]
                    ln = lat.norm(dim=1)
                    big = cm & (ln > 0.3)
                    dirn = lat / ln.clamp(min=1e-9)[:, None]
                    ff = torch.zeros(self.N, 3, device=self.dev)
                    ff[:, :2], ff[:, 2] = 2.5 * dirn, -2.0
                    self._cmd(big, x[big, :2] + 4e-4 * dirn[big], x[big, 2] - 0.002, ff[big])
                    sm = cm & ~big
                    ff2 = torch.zeros(int(sm.sum()), 3, device=self.dev)
                    ff2[:, 2] = -self.sff
                    self._cmd(sm, x[sm, :2], x[sm, 2] - 0.004, ff2)
            # push to the bottom (+ jam recovery)
            m = self.pending & (self.phase == PUSH)
            if m.any():
                tip = self.tip(x)
                chk = m & (self.k_push > 0)
                done = chk & (self.depth + tip < 0.001)
                self.t_done[done] = self.t
                self._go(done, DONE)
                cont = chk & ~done
                if cont.any():
                    self.depth_hist[cont] = torch.cat((self.depth_hist[cont, 1:], tip[cont, None]), 1)
                    self.n_hist[cont] += 1
                    full = self.n_hist > 10
                    stalled = full & (self.depth_hist[:, 0] - self.depth_hist[:, -1] < 2e-4)
                    F_lat = self.force(1.0)[:, :2]
                    wedged = full & (F_lat.norm(dim=1) > self.jam_force)
                    jam = cont & (self.retries < self.jam_recovery) & (stalled | wedged)
                    if jam.any():
                        self.retries[jam] += 1
                        self.n_hist[jam], self.depth_hist[jam] = 0, float("nan")
                        u = F_lat / F_lat.norm(dim=1).clamp(min=1e-6)[:, None]
                        self.jam_xy[jam], self.jam_z[jam] = x[jam, :2], x[jam, 2] + self.jam_lift
                        self.anchor[jam] = x[jam, :2] + self.jam_shift * u[jam]
                        self.phase[jam], self.k[jam] = JAM, 0
                limit = 150 + 30 * self.jam_recovery
                over = m & (self.phase == PUSH) & (self.k_push >= limit)
                self.t_done[over] = self.t
                self._go(over, DONE)
                pu = m & (self.phase == PUSH)
                if pu.any():
                    F = self.force(0.1)
                    uz = x[:, 2] - torch.where(F[:, 2] < self.max_force, 0.006, 0.002)
                    xy = torch.where(torch.isnan(self.anchor), x[:, :2], self.anchor)
                    ff = torch.zeros(self.N, 3, device=self.dev)
                    ff[:, 2] = -self.sff
                    self._cmd(pu, xy[pu], uz[pu], ff[pu])
                    self.k_push[pu] += 1
            m = self.pending & (self.phase == JAM)
            if m.any():
                done = m & (self.k >= 5)
                self.phase[done] = PUSH  # k_push keeps counting
                jm = m & ~done
                self._cmd(jm, self.jam_xy[jm], self.jam_z[jm])
            m = self.pending & ((self.phase == DONE) | (self.phase == FAILED))
            if m.any():
                self._cmd(m, self.cmd_xy[m], self.cmd_z[m])
            if not self.pending.any():
                break
        self.k += 1
        # to the controller (cmd() of the demo): pos = O + (xy, z + z_bias), orientation R, feedforward wrench
        c.pos_d[:, :2] = self.O[:, :2] + self.cmd_xy
        c.pos_d[:, 2] = self.O[:, 2] + self.cmd_z + self.z_bias
        c.rot_d[:] = self.R
        c.F_ff[:, :3] = self.cmd_ff
        c.F_ff[:, 3:] = 0.0
        self.t += self.dt

    def _start_push(self, m, x):
        if not m.any():
            return
        self.t_engage[m] = self.t
        self.n_hist[m], self.depth_hist[m] = 0, float("nan")
        self.retries[m], self.k_push[m] = 0, 0
        self.anchor[m] = float("nan")
        self._go(m, PUSH)

    def finished(self):
        return ((self.phase == DONE) | (self.phase == FAILED)).all()

    def score(self):
        """Success as the demo (before release): peg bottom within 1 mm of the hole bottom, xy within 1.5 mm, in the
        hole frame (tilt)."""
        from isaaclab.utils import math as tu
        env = self.env
        rel = tu.quat_apply_inverse(env.fixed_quat, env.held_pos - env.fixed_pos)
        depth_err, xy_err = rel[:, 2], rel[:, :2].norm(dim=1)
        return (depth_err < 0.001) & (xy_err < 0.0015), depth_err, xy_err
