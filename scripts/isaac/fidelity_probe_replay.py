"""Fidelity check of the batched sim against the ros2_control pipeline (ROADMAP 9, step 4): replay the logged
randomized probe sweeps (collect_probes.sh: ~/ci_mpc_runs/likelihood/sweeps*, 40 probes per episode at positions
sampled around the true hole, randomized friction / tilt / grasp / sensor model) in factory_fr3.TASK_CI, one env per
episode with that episode's parameters, the same calibration touch and the same probe positions (relative to the
true hole), hold depths and settle times; compare per probe the measured tip height and tared observer force.
    taskset -c 0-15 ~/IsaacLab/.venv/bin/python scripts/isaac/fidelity_probe_replay.py --out x.npz
"""
import argparse
import contextlib
import glob
import json
import os
import re
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
p.add_argument("--sweeps", default="~/ci_mpc_runs/likelihood/sweeps,~/ci_mpc_runs/likelihood/sweeps2")
p.add_argument("--max-episodes", type=int, default=0)
p.add_argument("--joint-friction-scale", type=float, default=1.0)
p.add_argument("--out", default="")
p.add_argument("--trace", default="", help="npz: env 0 per env step (phase, set-point, TCP, feedforward, integral, "
               "controller force; world frame), to compare with a bag of the pipeline episode")
add_launcher_args(p)
p.set_defaults(device=None)
args, rest = p.parse_known_args()
sys.argv = [sys.argv[0]] + rest

PROBE = re.compile(r"probe (\d+): TCP \[\s*([-\d.e]+)\s+([-\d.e]+)\s*\] mm \(true hole .*?\), tip ([-+\d.]+) mm, force "
                   r"\[\s*([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)\s*\] N;.*?hold ([\d.]+) mm settle ([\d.]+) s")
CAL = re.compile(r"grasp height offset ([-+\d.]+) mm \(calibration touch")


def load_episode(d):
    P = json.load(open(os.path.join(d, "params.json")))
    txt = open(os.path.join(d, "run.out")).read()
    rows = [m.groups() for m in PROBE.finditer(txt)]
    if not rows:
        return None
    a = np.array([[float(v) for v in r[1:]] for r in rows])  # tcp x y, tip, fx fy fz, hold, settle (mm, N, s)
    cal = CAL.search(txt)
    return {"dir": d, "params": P, "tcp": a[:, 0:2] / 1000.0, "tip": a[:, 2] / 1000.0, "F": a[:, 3:6],
            "hold": a[:, 6] / 1000.0, "settle": a[:, 7], "z_bias": float(cal.group(1)) / 1000.0 if cal else float("nan")}


eps = []
for root in args.sweeps.split(","):
    for d in sorted(glob.glob(os.path.join(os.path.expanduser(root), "w*_e*"))):
        e = load_episode(d)
        if e is not None:
            eps.append(e)
if args.max_episodes:
    eps = eps[:args.max_episodes]
N, P = len(eps), max(len(e["tip"]) for e in eps)
print(f"REPLAY {N} episodes, up to {P} probes each", flush=True)

import factory_fr3  # noqa: E402
from batched_probe import DONE, TargetedProbe  # noqa: E402

env_cfg, _ = resolve_task_config(factory_fr3.TASK_CI, "")
env_cfg.scene.num_envs = N
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
    # the controller's velocity friction compensation goes with the plant's joint friction (scale 0 with the
    # compensation on: it pushes along q_dot with nothing to cancel it, the arm ran away, xy error 280 mm)
    u.ctrl.coulomb *= args.joint_friction_scale
    u.ctrl.beta *= args.joint_friction_scale
    t = lambda a: torch.tensor(np.asarray(a), dtype=torch.float32, device=dev)  # noqa: E731
    u.ep_override = {
        "hole_err": t([[v / 1000.0 for v in e["params"]["offset"]] for e in eps]),
        "friction": t([e["params"]["friction"] for e in eps]),
        "tilt_deg": t([e["params"]["tilt"] for e in eps]),
        "grasp": t([[v / 1000.0 for v in e["params"]["grasp"]] for e in eps]),
        "noise": t([e["params"]["noise"] for e in eps]),
        "bias": t([e["params"]["bias"] for e in eps]),
        "gain": t([e["params"]["gain"] for e in eps]),
        "joint_friction_scale": torch.full((N,), args.joint_friction_scale, device=dev)}
    rp = TargetedProbe(u, os.path.join(HERE, "..", "..", "ci_mpc_planner", "models", "probe_likelihood_v3_both.npz"),
                max_probes=P, timeout=1e9, track_height=0, centering_bound=0.0, stuck_dither=0.0)
    tg = torch.zeros(N, P, 2, device=dev)
    hold, settle = torch.full((N, P), 0.001, device=dev), torch.full((N, P), 0.3, device=dev)
    n = torch.tensor([len(e["tip"]) for e in eps], device=dev)
    for i, e in enumerate(eps):
        k = len(e["tip"])
        tg[i, :k], hold[i, :k], settle[i, :k] = t(e["tcp"]), t(e["hold"]), t(e["settle"])
    zero = torch.zeros(N, u.single_action_space.shape[0], device=dev)
    env.reset()
    rp.reset()
    rp.setup(tg, n, hold, settle)
    steps = 0
    tr = []
    while not (rp.phase == DONE).all() and steps < 20000:
        rp.step()
        env.step(zero)
        steps += 1
        if args.trace:
            c = u.ctrl
            tr.append(np.concatenate(([rp.t, rp.phase[0].item(), rp.rp_k[0].item()], c.pos_d[0].cpu().numpy(),
                                      u.ci_state()[5][0].cpu().numpy(), c.F_ff[0, :3].cpu().numpy(),
                                      c.I[0, :3].cpu().numpy(), c.F[0, :3].cpu().numpy(),
                                      c.F_ext[0, :3].cpu().numpy())))
        if steps % 200 == 0:
            print(f"  t {rp.t:6.1f} s: probes done {int(rp.rp_k.sum())} / {int(n.sum())}", flush=True)
    sim_x, sim_F, sim_zb = rp.rec_x.cpu().numpy(), rp.rec_F.cpu().numpy(), rp.rec_zb.cpu().numpy()
    tip_off = rp.tip_below_tcp

    # ---- comparison
    log_tip = np.full((N, P), np.nan)
    log_F = np.full((N, P, 3), np.nan)
    log_xy = np.full((N, P, 2), np.nan)
    for i, e in enumerate(eps):
        k = len(e["tip"])
        log_tip[i, :k], log_F[i, :k], log_xy[i, :k] = e["tip"], e["F"], e["tcp"]
    sim_tip = sim_x[..., 2] - tip_off
    hole = -np.array([[v / 1000.0 for v in e["params"]["offset"]] for e in eps])  # true hole in the model frame
    r_log = np.linalg.norm(log_xy - hole[:, None], axis=-1)
    valid = ~np.isnan(log_tip) & ~np.isnan(sim_tip)

    def cls(tip, F):  # flat face / engaged (chamfer push or sunk) / in the hole
        lat = np.linalg.norm(F[..., :2], axis=-1)
        return np.where(tip < -0.003, 2, np.where((tip < -3e-4) | (lat > 1.2), 1, 0))

    cl, cs = cls(log_tip, log_F), cls(sim_tip, sim_F)
    out = {"episodes": N, "probes": int(valid.sum())}
    out["xy_track_mm"] = float(np.nanmedian(np.linalg.norm(sim_x[..., :2] - log_xy, axis=-1)[valid]) * 1000)
    out["class_agree"] = float((cl == cs)[valid].mean())
    conf = np.zeros((3, 3), int)
    for a in range(3):
        for b in range(3):
            conf[a, b] = int(((cl == a) & (cs == b) & valid).sum())
    out["confusion_log_rows_sim_cols"] = conf.tolist()
    flat = valid & (cl == 0) & (cs == 0)
    d_tip = (sim_tip - log_tip)[flat] * 1000
    out["flat_tip_diff_mm"] = {"mean": float(d_tip.mean()), "sd": float(d_tip.std()), "n": int(flat.sum())}
    for name, m in (("flat", flat), ("engaged_both", valid & (cl == 1) & (cs == 1)), ("all", valid)):
        if m.any():
            dF = sim_F[m] - log_F[m]
            out[f"force_{name}"] = {"log_mean": np.round(log_F[m].mean(0), 2).tolist(),
                                    "sim_mean": np.round(sim_F[m].mean(0), 2).tolist(),
                                    "log_sd": np.round(log_F[m].std(0), 2).tolist(),
                                    "sim_sd": np.round(sim_F[m].std(0), 2).tolist(),
                                    "diff_rms": np.round(np.sqrt((dF ** 2).mean(0)), 2).tolist()}
    eng = valid & (cl == 1) & (cs == 1)
    if eng.any():
        a, b = log_F[eng][:, :2], sim_F[eng][:, :2]
        big = (np.linalg.norm(a, axis=1) > 1.2) & (np.linalg.norm(b, axis=1) > 1.2)
        cosang = (a * b).sum(1) / (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1) + 1e-9)
        out["engaged_lateral_dir_cos_median"] = float(np.median(cosang[big])) if big.any() else None
    for lo, hi in ((0, 4), (4, 5.5), (5.5, 9)):
        m = valid & (r_log * 1000 >= lo) & (r_log * 1000 < hi)
        if m.any():
            out[f"r_{lo}_{hi}mm"] = {"n": int(m.sum()), "agree": float((cl == cs)[m].mean()),
                                     "log_engaged": float((cl >= 1)[m].mean()), "sim_engaged": float((cs >= 1)[m].mean())}
    log_zb = np.array([e["z_bias"] for e in eps])
    out["z_bias_mm"] = {"log_mean": float(np.nanmean(log_zb) * 1000), "sim_mean": float(np.nanmean(sim_zb) * 1000),
                        "diff_sd": float(np.nanstd(sim_zb - log_zb) * 1000),
                        "corr": float(np.corrcoef(*[v[~np.isnan(log_zb) & ~np.isnan(sim_zb)] for v in (log_zb, sim_zb)])[0, 1])}
    print("FIDELITY " + json.dumps(out), flush=True)
    if args.trace:
        np.savez(args.trace, trace=np.array(tr), cols=np.array(
            "t phase k pos_d_x pos_d_y pos_d_z x y z ff_x ff_y ff_z I_x I_y I_z F_x F_y F_z Fext_x Fext_y Fext_z".split()),
                 hole_top=u.ci_hole_top()[0].cpu().numpy(), tip_off=tip_off, dir=eps[0]["dir"])
    if args.out:
        np.savez(args.out, sim_x=sim_x, sim_F=sim_F, sim_zb=sim_zb, log_tip=log_tip, log_F=log_F, log_xy=log_xy,
                 hole=hole, tip_off=tip_off, summary=json.dumps(out), dirs=np.array([e["dir"] for e in eps]))
