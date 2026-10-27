"""Batched probe strategy benchmark (ROADMAP 9, step 3): N randomized episodes at once in factory_fr3.TASK_CI with
our controller + observer and batched_probe.BatchedProbe; reports success rate, contact time and probes per run
(compare: ros2_control held-out R, plain probe + v3: 30/30, contact 23.9 s mean / 17.5 s median, 3.8 probes).
Per-env results + episode parameters -> --out (.npz).
    taskset -c 0-15 ~/IsaacLab/.venv/bin/python scripts/isaac/run_batched_probe.py --num_envs 256 --out x.npz
"""
import argparse
import contextlib
import json
import os
import sys
import time

import gymnasium as gym
import numpy as np
import torch

from isaaclab.app import add_launcher_args, launch_simulation

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import resolve_task_config

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
p = argparse.ArgumentParser()
p.add_argument("--num_envs", type=int, default=64)
p.add_argument("--episodes", type=int, default=1, help="episode batches (a reset between)")
p.add_argument("--model", default=os.path.join(HERE, "..", "..", "ci_mpc_planner", "models", "probe_likelihood_v3_both.npz"))
p.add_argument("--max-probes", type=int, default=25)
p.add_argument("--timeout", type=float, default=150.0, help="contact-phase sim seconds")
p.add_argument("--jam-recovery", type=int, default=0)
p.add_argument("--no-tilt", action="store_true")
p.add_argument("--seed", type=int, default=0)
p.add_argument("--out", default="")
p.add_argument("--trace-env", type=int, default=-1, help="print this env's phase changes")
add_launcher_args(p)
p.set_defaults(device=None)
args, rest = p.parse_known_args()
sys.argv = [sys.argv[0]] + rest
os.environ["CI_SEED"] = str(args.seed)

import factory_fr3  # noqa: E402
from batched_probe import NAMES, BatchedProbe  # noqa: E402

env_cfg, _ = resolve_task_config(factory_fr3.TASK_CI, "")
env_cfg.scene.num_envs = args.num_envs
env_cfg.sim.dt, env_cfg.decimation = 1.0 / 240.0, 24  # controller 240 Hz, planning step 0.1 s (--plan-period)
env_cfg.episode_length_s = 1e5
env_cfg.seed = args.seed

with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
    env = gym.make(factory_fr3.TASK_CI, cfg=env_cfg)
    cleanup.callback(env.close)
    cleanup.enter_context(torch.inference_mode())
    u = env.unwrapped
    if args.no_tilt:
        u.rand["tilt_frac"] = 0.0
    bp = BatchedProbe(u, args.model, max_probes=args.max_probes, timeout=args.timeout, jam_recovery=args.jam_recovery,
                      seed=args.seed)
    zero = torch.zeros(u.num_envs, u.single_action_space.shape[0], device=u.device)
    rows = []
    for ep in range(args.episodes):
        env.reset()
        bp.reset()
        t0, steps = time.perf_counter(), 0
        last = bp.phase.clone()
        while not bp.finished() and bp.t < args.timeout + 60.0:
            bp.step()
            env.step(zero)
            steps += 1
            if args.trace_env >= 0:
                i = args.trace_env
                if bp.phase[i] != last[i]:
                    x = bp.measure()[i]
                    print(f"  t {bp.t:6.1f} env {i}: {NAMES[last[i]]} -> {NAMES[bp.phase[i]]}, xy "
                          f"{(x[:2] * 1000).tolist()} mm, tip {float(bp.tip(x[None])[0]) * 1000:+.2f} mm, probes "
                          f"{int(bp.n_probes[i])}, z_bias {float(bp.z_bias[i]) * 1000:+.2f} mm", flush=True)
                last = bp.phase.clone()
            if steps % 100 == 0:
                cnt = torch.bincount(bp.phase, minlength=len(NAMES)).tolist()
                print(f"  ep {ep} t {bp.t:6.1f} s: " + ", ".join(f"{n} {k}" for n, k in zip(NAMES, cnt) if k), flush=True)
        wall = time.perf_counter() - t0
        ok, depth_err, xy_err = bp.score()
        est, std = bp.estimate()
        r = {k: v.cpu().numpy() for k, v in u.ep.items()}
        r.update(success=ok.cpu().numpy(), depth_err=depth_err.cpu().numpy(), xy_err=xy_err.cpu().numpy(),
                 phase=bp.phase.cpu().numpy(), t_done=bp.t_done.cpu().numpy(), t_engage=bp.t_engage.cpu().numpy(),
                 n_probes=bp.n_probes.cpu().numpy(), z_bias=bp.z_bias.cpu().numpy(),
                 belief_err=(est + bp.O[:, :2] - u.ci_hole_top()[:, :2]).norm(dim=1).cpu().numpy(),
                 belief_std=std.cpu().numpy())
        rows.append(r)
        s, td = r["success"], r["t_done"]
        tds = td[s]
        print(f"EPISODE {ep}: success {int(s.sum())}/{len(s)} ({100 * s.mean():.1f} %), contact time (successes) mean "
              f"{np.nanmean(tds) if len(tds) else float('nan'):.1f} s / median "
              f"{np.nanmedian(tds) if len(tds) else float('nan'):.1f} s, probes mean {r['n_probes'].mean():.1f}; "
              f"failed (probe budget / timeout) {int((r['phase'] == 9).sum())}, done but not inserted "
              f"{int(((r['phase'] == 8) & ~s).sum())}, unfinished {int((r['phase'] < 8).sum())}; sim {bp.t:.0f} s in "
              f"{wall:.0f} s wall ({u.num_envs * bp.t / wall:.0f} sim s / wall s)", flush=True)
    R = {k: np.concatenate([r[k] for r in rows]) for k in rows[0]}
    s = R["success"]
    summ = {"num_envs": args.num_envs, "episodes": args.episodes, "n": int(len(s)), "success": int(s.sum()),
            "rate": float(s.mean()), "t_mean": float(np.nanmean(R["t_done"][s])) if s.any() else None,
            "t_median": float(np.nanmedian(R["t_done"][s])) if s.any() else None,
            "probes_mean": float(R["n_probes"].mean()), "jam_recovery": args.jam_recovery, "seed": args.seed}
    for name, m in (("err_lt_2mm", np.linalg.norm(R["hole_err"], axis=1) < 0.002),
                    ("err_ge_2mm", np.linalg.norm(R["hole_err"], axis=1) >= 0.002), ("tilted", R["tilt_deg"] > 0)):
        if m.any():
            summ[name] = {"n": int(m.sum()), "rate": float(s[m].mean())}
    print("RESULT " + json.dumps(summ), flush=True)
    if args.out:
        np.savez(args.out, **R, summary=json.dumps(summ))
