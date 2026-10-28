"""Batched random-probe collection for the learned probe likelihood (vectorized step 5; peg only, "batched, not
validated for" other tasks): as peg_insert_demo.py --collect in collect_probes.sh, one episode per env with the RAND
episode parameters (hole error r U(0, 5) mm, contact friction U(0.5, 1), tilt U(0, 1.5) deg in a third, grasp
offset +-1 mm, observer noise / bias / gain), the calibration touch, probe 0 at the estimate (hold 1 mm, settle
0.3 s), then probes at r U(0, 9) or U(2, 6) mm (half each) around the true hole, hold U(0.5, 2) mm, settle
U(0.2, 0.6) s. Force = the batched observer (its input is the hand wrench, i.e. the true force through the hand plus
the sensor model), not the pipeline's over-reading observer (PROGRESS "vectorized step 4c").
Writes probe_dataset.py's format (tcp / hole / tip in mm in the model frame, tared force in N, run, k, calibrated,
runs) + the episode parameters, for train_probe_likelihood.py.
    taskset -c 0-15 ~/IsaacLab/.venv/bin/python scripts/isaac/batched_collect.py --envs 1024 --seed 0 --out x.npz
"""
import argparse
import contextlib
import math
import os
import sys

import gymnasium as gym
import numpy as np
import torch

from isaaclab.app import add_launcher_args, launch_simulation

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import resolve_task_config

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
p = argparse.ArgumentParser()
p.add_argument("--envs", type=int, default=1024)
p.add_argument("--probes", type=int, default=40)
p.add_argument("--seed", type=int, default=0)
p.add_argument("--tag", default="", help="run-name prefix in the dataset (default: batched_s<seed>)")
p.add_argument("--out", required=True)
add_launcher_args(p)
p.set_defaults(device=None)
args, rest = p.parse_known_args()
sys.argv = [sys.argv[0]] + rest
N, P = args.envs, args.probes

import factory_fr3  # noqa: E402
from batched_probe import DONE, TargetedProbe  # noqa: E402

env_cfg, _ = resolve_task_config(factory_fr3.TASK_CI, "")
env_cfg.scene.num_envs = N
env_cfg.seed = args.seed
env_cfg.sim.dt, env_cfg.decimation = 1.0 / 240.0, 24
env_cfg.episode_length_s = 1e5
# pipeline geometry: hole not yawed (tilt about the world x, peg_scene RotateX), hand x = world x (grasp offset axis)
env_cfg.task.fixed_asset_init_orn_range_deg = 0.0
env_cfg.task.hand_init_orn_noise = [0.0, 0.0, 0.0]

with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
    env = gym.make(factory_fr3.TASK_CI, cfg=env_cfg)
    cleanup.callback(env.close)
    cleanup.enter_context(torch.inference_mode())
    u = env.unwrapped
    dev = u.device
    rp = TargetedProbe(u, os.path.join(HERE, "..", "..", "ci_mpc_planner", "models", "probe_likelihood_v3_both.npz"),
                       max_probes=P, timeout=1e9, track_height=0, centering_bound=0.0, stuck_dither=0.0)
    zero = torch.zeros(N, u.single_action_space.shape[0], device=dev)
    env.reset()
    rp.reset()
    # targets around the true hole in the model frame (origin = the estimate): the demo's col_rng sampling
    hole = (u.ci_hole_top() - rp.O)[:, :2]
    g = torch.Generator(device=dev).manual_seed(args.seed)
    U = lambda lo, hi, *s: lo + (hi - lo) * torch.rand(*s, generator=g, device=dev)  # noqa: E731
    r = torch.where(torch.rand(N, P, generator=g, device=dev) < 0.5, U(0.0, 9.0, N, P), U(2.0, 6.0, N, P)) / 1000.0
    a = U(0.0, 2.0 * math.pi, N, P)
    tg = hole[:, None] + r[..., None] * torch.stack((torch.cos(a), torch.sin(a)), -1)
    hold, settle = U(0.0005, 0.002, N, P), U(0.2, 0.6, N, P)
    tg[:, 0], hold[:, 0], settle[:, 0] = 0.0, 0.001, 0.3  # probe 0: the estimate, nominal hold / settle
    rp.setup(tg, torch.full((N,), P, device=dev), hold, settle)
    steps = 0
    while not (rp.phase == DONE).all() and steps < 20000:
        rp.step()
        env.step(zero)
        steps += 1
        if steps % 200 == 0:
            print(f"  t {rp.t:6.1f} s: probes done {int(rp.rp_k.sum())} / {N * P}", flush=True)
    x, F = rp.rec_x.cpu().numpy(), rp.rec_F.cpu().numpy()
    ok = ~np.isnan(x[..., 0])
    tip = x[..., 2] - rp.tip_below_tcp  # as rp.tip(): measure() already removed the grasp-height offset
    run, k = np.nonzero(ok)
    tag = args.tag or f"batched_s{args.seed}"
    ep = {f"ep_{n}": v.cpu().numpy() for n, v in u.ep.items()}
    np.savez(args.out, run=run, k=k, tcp=x[run, k, :2] * 1000, hole=hole.cpu().numpy()[run] * 1000,
             tip=tip[run, k] * 1000, force=F[run, k], calibrated=rp.calib_ok.cpu().numpy()[run],
             runs=np.array([f"{tag}/env{i:04d}" for i in range(N)]), z_bias=rp.rec_zb.cpu().numpy(), **ep)
    print(f"COLLECT {int(ok.sum())} probes from {N} envs ({int(rp.calib_ok.sum())} calibrated), {rp.t:.0f} s sim, "
          f"tip mean {np.nanmean(tip[ok]) * 1000:+.2f} mm, Fz mean {np.nanmean(F[ok][:, 2]):.2f} N", flush=True)
