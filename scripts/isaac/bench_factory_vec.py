"""Throughput of Isaac Lab's vectorized Factory PegInsert (ROADMAP 9, step 0): N envs in one process, random actions,
headless. Prints env-steps/s, simulated seconds per wall second (all envs), GPU memory of this process and RSS.
Random delta-pose actions put the peg in and out of contact, so the timing includes SDF contacts part of the time.
    ~/IsaacLab/.venv/bin/python scripts/isaac/bench_factory_vec.py --num_envs 64 --steps 300   (headless by default; --viz NAME for a viewer)
"""
import argparse
import contextlib
import json
import os
import subprocess
import sys
import time

import gymnasium as gym
import torch

from isaaclab.app import add_launcher_args, launch_simulation

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import resolve_task_config

TASK = "IsaacContrib-Factory-PegInsert-Direct"
p = argparse.ArgumentParser()
p.add_argument("--num_envs", type=int, default=64)
p.add_argument("--steps", type=int, default=300, help="timed env steps (after --warmup)")
p.add_argument("--warmup", type=int, default=30)
p.add_argument("--task", default=TASK)
p.add_argument("--out", default="", help="append the result as a JSON line")
add_launcher_args(p)
p.set_defaults(device=None)
args, rest = p.parse_known_args()
sys.argv = [sys.argv[0]] + rest


def gpu_mib():
    out = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits"],
                         capture_output=True, text=True).stdout
    return sum(int(m) for pid, m in (l.split(", ") for l in out.strip().splitlines()) if int(pid) == os.getpid())


def rss_gib():
    for l in open("/proc/self/status"):
        if l.startswith("VmRSS"):
            return int(l.split()[1]) / 2**20


env_cfg, _ = resolve_task_config(args.task, "")
env_cfg.scene.num_envs = args.num_envs
with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
    t_load = time.perf_counter()
    env = gym.make(args.task, cfg=env_cfg)
    cleanup.callback(env.close)
    env.reset()
    u = env.unwrapped
    n_act = u.cfg.action_space if isinstance(u.cfg.action_space, int) else u.single_action_space.shape[0]
    act = lambda: 2.0 * torch.rand(args.num_envs, n_act, device=u.device) - 1.0  # noqa: E731
    with torch.inference_mode():
        for _ in range(args.warmup):
            env.step(act())
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        for _ in range(args.steps):
            env.step(act())
        torch.cuda.synchronize()
        wall = time.perf_counter() - t0
    dt_env = u.cfg.sim.dt * u.cfg.decimation
    r = {"task": args.task, "num_envs": args.num_envs, "steps": args.steps, "wall_s": round(wall, 2),
         "env_steps_per_s": round(args.num_envs * args.steps / wall, 1),
         "sim_s_per_wall_s_total": round(args.num_envs * args.steps * dt_env / wall, 2),
         "rtf_per_env": round(args.steps * dt_env / wall, 3), "physics_dt": u.cfg.sim.dt, "decimation": u.cfg.decimation,
         "gpu_mib": gpu_mib(), "rss_gib": round(rss_gib(), 2), "startup_s": round(time.perf_counter() - t_load - wall, 1)}
    print("BENCH " + json.dumps(r), flush=True)
    if args.out:
        open(args.out, "a").write(json.dumps(r) + "\n")
