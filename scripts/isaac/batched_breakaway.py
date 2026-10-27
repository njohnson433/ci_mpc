"""Cartesian breakaway of the batched FR3 (vectorized step 4b), as peg_insert_demo.py --breakaway in the ros2_control
pipeline: peg in hand, TCP set-point 10 mm above the true opening, K 1500 / 1500 / 500, integral off; after settling
the feedforward force is ramped along one direction until the TCP has moved 0.1 mm along it; the spring is ~0 then,
so the feedforward is the static friction. One env per (direction, joint friction scale), all in one process.
Pipeline (PROGRESS 2026-10-27 "C3 in Isaac", nominal scene): -x 2.05, +x 2.60, -y 2.01, +y 1.70, -z 2.08 / 2.30 N.
    taskset -c 0-31 ~/IsaacLab/.venv/bin/python scripts/isaac/batched_breakaway.py --scales 1,0.75,0.5,0.25 --out x.json
"""
import argparse
import contextlib
import json
import os
import sys

import gymnasium as gym
import torch

from isaaclab.app import add_launcher_args, launch_simulation

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import resolve_task_config

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
p = argparse.ArgumentParser()
p.add_argument("--dirs", default="+x,-x,+y,-y,+z,-z")
p.add_argument("--scales", default="1.0", help="joint friction scales (plant and the controller's compensation per env: "
               "full compensation on a reduced plant ran away, 100-1000 mm)")
p.add_argument("--rate", type=float, default=1.0, help="force ramp [N/s]")
p.add_argument("--move", type=float, default=0.1, help="motion threshold along the direction [mm]")
p.add_argument("--max-force", type=float, default=10.0)
p.add_argument("--settle", type=float, default=4.0, help="time to reach and settle at the start pose [s]")
p.add_argument("--out", default="")
add_launcher_args(p)
p.set_defaults(device=None)
args, rest = p.parse_known_args()
sys.argv = [sys.argv[0]] + rest

DIRS = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1)}
dirs, scales = args.dirs.split(","), [float(s) for s in args.scales.split(",")]
cases = [(d, s) for s in scales for d in dirs]
N = len(cases)

import factory_fr3  # noqa: E402

env_cfg, _ = resolve_task_config(factory_fr3.TASK_CI, "")
env_cfg.scene.num_envs = N
env_cfg.sim.dt, env_cfg.decimation = 1.0 / 240.0, 2  # commands at 120 Hz: ramp resolution 0.008 N at 1 N/s
env_cfg.episode_length_s = 1e5
env_cfg.task.fixed_asset_init_orn_range_deg = 0.0
env_cfg.task.hand_init_orn_noise = [0.0, 0.0, 0.0]

with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
    env = gym.make(factory_fr3.TASK_CI, cfg=env_cfg)
    cleanup.callback(env.close)
    cleanup.enter_context(torch.inference_mode())
    u = env.unwrapped
    dev = u.device
    z = lambda *s: torch.zeros(N, *s, device=dev)  # noqa: E731
    u.ep_override = {"hole_err": z(2), "friction": torch.full((N,), 0.75, device=dev), "tilt_deg": z(),
                     "grasp": z(2), "noise": torch.full((N,), 0.05, device=dev), "bias": z(7),
                     "gain": torch.full((N,), 30.0, device=dev),
                     "joint_friction_scale": torch.tensor([s for _, s in cases], device=dev)}
    env.reset()
    c = u.ctrl
    # per-env compensation scale: (N, 7) instead of the shared (7,)
    js = u.ep["joint_friction_scale"][:, None]
    c.coulomb, c.beta = js * c.coulomb, js * c.beta
    c.K[:] = torch.tensor((1500.0, 1500.0, 500.0, 260.0, 260.0, 90.0), device=dev)
    c.integral_enabled[:] = False
    c.twist_d[:], c.D_cmd[:], c.F_ff[:] = 0.0, 0.0, 0.0
    tip_below_tcp = u.cfg_task.held_asset_cfg.height - 0.010  # the demo's grasp depth 10 mm
    x0 = u.ci_hole_top().clone()
    x0[:, 2] += tip_below_tcp + 0.010
    c.pos_d[:] = x0
    d = torch.tensor([DIRS[n] for n, _ in cases], dtype=torch.float32, device=dev)
    zero = torch.zeros(N, u.single_action_space.shape[0], device=dev)
    dt = u.step_dt
    for _ in range(int(args.settle / dt)):
        env.step(zero)
    # stiction stops the approach short of x0 (scale 1: 4.5 mm, a ~2 N spring preload that biased + vs -): set-point
    # re-anchored at the settled pose, so the ramp starts with no spring force
    xs = u.ci_state()[5].clone()
    start_err = (xs - x0).norm(dim=1) * 1000
    c.pos_d[:] = xs
    for _ in range(int(1.0 / dt)):
        env.step(zero)
    xs = u.ci_state()[5].clone()
    f_brk = torch.full((N,), float("nan"), device=dev)
    active = torch.ones(N, dtype=torch.bool, device=dev)
    t = 0.0
    while active.any() and t * args.rate <= args.max_force:
        t += dt
        c.F_ff[:, :3] = torch.where(active[:, None], args.rate * t * d, torch.zeros_like(d))
        env.step(zero)
        s = ((u.ci_state()[5] - xs) * d).sum(1)
        hit = active & (s > args.move / 1000)
        f_brk[hit] = args.rate * t
        active &= ~hit
    out = {"rate": args.rate, "move_mm": args.move, "settle_s": args.settle, "cases": []}
    for i, (n, s) in enumerate(cases):
        f = f_brk[i].item()
        out["cases"].append({"dir": n, "scale": s, "f_breakaway": None if f != f else round(f, 3),
                             "start_err_mm": round(start_err[i].item(), 3)})
    for s in scales:
        row = {x["dir"]: x["f_breakaway"] for x in out["cases"] if x["scale"] == s}
        print(f"BREAKAWAY scale {s}: " + ", ".join(f"{k} {v}" for k, v in row.items()), flush=True)
    print("START_ERR_MM max %.3f" % start_err.max().item(), flush=True)
    if args.out:
        json.dump(out, open(args.out, "w"), indent=1)
