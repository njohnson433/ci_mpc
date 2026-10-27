"""Checks of the per-env episode randomization in factory_fr3.TASK_CI (ROADMAP 9, step 2): every sampled parameter
is read back from the sim (grasp offset as the peg's pose in the TCP frame, contact friction from the material
properties, hole tilt from the hole pose, joint friction from PhysX, sensor model from the controller, hole estimate
error), then a 2 s press with the search stiffness checks that the randomized envs stay stable, and a second reset
draws new values.
    ~/IsaacLab/.venv/bin/python scripts/isaac/test_ci_rand.py --num_envs 64
"""
import argparse
import contextlib
import json
import math
import os
import sys

import gymnasium as gym
import torch

from isaaclab.app import add_launcher_args, launch_simulation

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import resolve_task_config

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
p = argparse.ArgumentParser()
p.add_argument("--num_envs", type=int, default=64)
p.add_argument("--out", default="")
add_launcher_args(p)
p.set_defaults(device=None)
args, rest = p.parse_known_args()
sys.argv = [sys.argv[0]] + rest

import factory_fr3  # noqa: E402

env_cfg, _ = resolve_task_config(factory_fr3.TASK_CI, "")
env_cfg.scene.num_envs = args.num_envs
env_cfg.sim.dt, env_cfg.decimation = 1.0 / 240.0, 2
env_cfg.episode_length_s = 1e4
res = {"num_envs": args.num_envs}


def corr(a, b):
    a, b = a - a.mean(), b - b.mean()
    return float((a * b).sum() / (a.norm() * b.norm() + 1e-12))


with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
    env = gym.make(factory_fr3.TASK_CI, cfg=env_cfg)
    cleanup.callback(env.close)
    cleanup.enter_context(torch.inference_mode())
    u = env.unwrapped
    u.rand["joint_friction_scale"] = (0.7, 1.3)  # on here, to check it reaches PhysX
    from isaaclab.utils import math as tu
    import warp as wp

    def check(tag):
        ep, out = u.ep, {}
        _, _, _, _, _, tcp, R, _ = u.ci_state()
        peg = u.held_pos
        rel = (R.transpose(1, 2) @ (peg - tcp).unsqueeze(-1)).squeeze(-1)  # peg bottom in the TCP frame (z down)
        dz_exp = factory_fr3.GRASP_DEPTH - u.cfg_task.held_asset_cfg.height + ep["grasp"][:, 1]  # z of peg bottom
        dx = ep["grasp"][:, 0]
        out["grasp_z_err_mm"] = float(((-rel[:, 2]) - (-dz_exp)).abs().max() * 1000) if False else None
        out["grasp_rel_mm_env0"] = [round(v * 1000, 3) for v in rel[0].tolist()]
        out["grasp_z_mm"] = {"expected_bottom_below_tcp": [round(float(-dz_exp.min()) * 1000, 2),
                                                           round(float(-dz_exp.max()) * 1000, 2)],
                             "corr_with_dz": corr(rel[:, 2], ep["grasp"][:, 1]),
                             "resid_mm": float((rel[:, 2] - (-dz_exp)).abs().max() * 1000),
                             "resid_flipped_mm": float((rel[:, 2] - dz_exp).abs().max() * 1000)}
        out["grasp_x"] = {"corr_with_dx": corr(rel[:, 0], dx), "resid_plus_mm": float((rel[:, 0] - dx).abs().max() * 1000),
                          "resid_minus_mm": float((rel[:, 0] + dx).abs().max() * 1000),
                          "y_abs_max_mm": float(rel[:, 1].abs().max() * 1000)}
        mat = wp.to_torch(u._held_asset.root_view.get_material_properties())
        out["friction_err"] = float((mat[:, :, 0].to(u.device) - ep["friction"][:, None]).abs().max())
        z = tu.quat_apply(u.fixed_quat, torch.tensor([0.0, 0.0, 1.0], device=u.device).repeat(u.num_envs, 1))
        tilt = torch.rad2deg(torch.acos(z[:, 2].clamp(-1, 1)))
        out["tilt_err_deg"] = float((tilt - ep["tilt_deg"]).abs().max())
        out["tilted_envs"] = int((ep["tilt_deg"] > 0).sum())
        jf = u._robot.data.joint_friction_coeff.torch[:, :7]
        nom = torch.tensor([p["static"] for p in factory_fr3._joint_params().values()], device=u.device)
        out["joint_friction_err"] = float((jf - ep["joint_friction_scale"][:, None] * nom).abs().max())
        out["sensor_err"] = float(max((u.ctrl.noise[:, 0] - ep["noise"]).abs().max(), (u.ctrl.bias - ep["bias"]).abs().max(),
                                      (u.ctrl.obs_gain[:, 0] - ep["gain"]).abs().max()))
        e = (u.ci_hole_estimate() - u.ci_hole_top())[:, :2].norm(dim=1)
        out["hole_err_mm"] = {"min": float(e.min() * 1000), "max": float(e.max() * 1000),
                              "match": float((e - ep["hole_err"].norm(dim=1)).abs().max() * 1000)}
        bad = ((rel[:, 2] - (-dz_exp)).abs() > 0.003) | ((rel[:, 0] - dx).abs() > 0.003)
        out["grasp_bad_envs"] = torch.nonzero(bad)[:, 0].tolist()
        zres = (rel[:, 2] - (-dz_exp))[~bad]
        out["grasp_z_resid_good_mm"] = {"mean": float(zres.mean() * 1000), "sd": float(zres.std() * 1000)} if len(zres) else None
        if bad.any():
            i = int(torch.nonzero(bad)[0, 0])
            out["bad_env_example"] = {"env": i, "peg": u.held_pos[i].tolist(), "tcp": tcp[i].tolist(),
                                      "hole_top": u.ci_hole_top()[i].tolist()}
        out["ranges"] = {k: [round(float(v.min()), 4), round(float(v.max()), 4)] for k, v in ep.items()}
        print(f"RAND {tag}: " + json.dumps(out), flush=True)
        return out

    env.reset()
    res["reset1"] = check("reset 1")
    first = {k: v.clone() for k, v in u.ep.items()}

    # press 2 s with the search stiffness: randomized envs stable, F_ext ~ K_z * 1 cm
    c = u.ctrl
    c.K[:] = torch.tensor((1500.0, 1500.0, 500.0, 260.0, 260.0, 90.0), device=u.device)
    c.integral_enabled[:] = False
    zero = torch.zeros(u.num_envs, u.single_action_space.shape[0], device=u.device)
    for _ in range(int(2.0 / u.step_dt)):
        pos = u.ci_state()[5]
        c.pos_d[:] = pos
        c.pos_d[:, 2] = pos[:, 2] - 0.01
        env.step(zero)
    fz = c.F_ext[:, 2]
    depth = (u.ci_hole_top()[:, 2] - u.held_pos[:, 2])
    res["press_peg_below_hole_top_mm"] = {"n_in_hole": int((depth > 0.003).sum()), "max": float(depth.max() * 1000)}
    res["press"] = {"F_ext_z_mean": float(fz.mean()), "F_ext_z_min": float(fz.min()), "F_ext_z_max": float(fz.max()),
                    "nan_envs": int(torch.isnan(c.tau_prev).any(1).sum())}
    print("PRESS (free fall onto ... / in air: K_z 500 * 1 cm = 5 N): " + json.dumps(res["press"]), flush=True)

    env.reset()
    res["reset2"] = check("reset 2")
    res["new_values"] = {k: bool((u.ep[k] != first[k]).any()) for k in first}
    print("NEW VALUES after reset 2: " + json.dumps(res["new_values"]), flush=True)
    print("RESULT " + json.dumps(res), flush=True)
    if args.out:
        open(args.out, "a").write(json.dumps(res) + "\n")
