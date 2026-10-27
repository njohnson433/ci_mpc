"""Checks of the batched controller (ci_ctrl_torch) in the vectorized FR3 Factory env (ROADMAP 9, step 1), against
Pinocchio and against the ros2_control results of the same tests:
  model     PhysX M / C dq / J / TCP pose at the TCP vs Pinocchio (fr3.urdf, fr3_hand_tcp), armature in M or not
  hold      2 s hold at the FR3 home pose (ros2_control hold/reach: 0.11 mm)
  circle    r 5 cm, period 5 s, y-z plane, pose + twist, 2 laps, RMS after half a lap (mpc_tracking_test.py: 1.29 mm
            with friction)
  observer  known force on the TCP body at rest, per env random in [-10, 10] N; F_ext vs -F (ros2_control: exact up to
            the sensor bias / noise model)
  --stock-frame: print the stock Panda's panda_fingertip_centered offset from panda_hand and exit.
    ~/IsaacLab/.venv/bin/python scripts/isaac/test_ci_ctrl.py --num_envs 64
"""
import argparse
import contextlib
import json
import math
import os
import sys
import time

import gymnasium as gym
import numpy as np
import torch

from isaaclab.app import add_launcher_args, launch_simulation

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import resolve_task_config

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
p = argparse.ArgumentParser()
p.add_argument("--num_envs", type=int, default=64)
p.add_argument("--tests", default="model,hold,circle,observer")
p.add_argument("--out", default="")
p.add_argument("--stock-frame", action="store_true")
add_launcher_args(p)
p.set_defaults(device=None)
args, rest = p.parse_known_args()
sys.argv = [sys.argv[0]] + rest

HOME = (0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4)

if args.stock_frame:
    task = "IsaacContrib-Factory-PegInsert-Direct"
else:
    import factory_fr3
    task = factory_fr3.TASK_CI
env_cfg, _ = resolve_task_config(task, "")
env_cfg.scene.num_envs = 1 if args.stock_frame else args.num_envs
hz = float(os.environ.get("CI_PHYS_HZ", 240.0))  # test switch
env_cfg.sim.dt, env_cfg.decimation = 1.0 / hz, max(1, round(hz / 120.0))  # physics / controller 240 Hz, command 120 Hz
env_cfg.episode_length_s = 1e4
res = {"num_envs": env_cfg.scene.num_envs}

with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
    env = gym.make(task, cfg=env_cfg)
    cleanup.callback(env.close)
    cleanup.enter_context(torch.inference_mode())  # the controller state becomes inference tensors in env.step
    u = env.unwrapped
    rob = u._robot
    env.reset()
    if args.stock_frame:
        from isaaclab.utils import math as tu
        d = rob.data
        ih, it = rob.body_names.index("panda_hand"), rob.body_names.index("panda_fingertip_centered")
        rel = tu.quat_apply_inverse(d.body_quat_w.torch[:, ih], d.body_pos_w.torch[:, it] - d.body_pos_w.torch[:, ih])
        print(f"STOCK panda_fingertip_centered in panda_hand: {rel[0].tolist()} m", flush=True)
        sys.exit(0)

    N, dev, c = u.num_envs, u.device, u.ctrl
    if os.environ.get("CI_FRICTION_SCALE"):  # test switch: friction compensation off in a frictionless scene
        c.coulomb *= float(os.environ["CI_FRICTION_SCALE"])
        c.beta *= float(os.environ["CI_FRICTION_SCALE"])
    zero_act = torch.zeros(N, u.single_action_space.shape[0], device=dev)

    def go_home():
        u.ci_active = False
        jp = rob.data.joint_pos.torch.clone()
        jp[:, :7] = torch.tensor(HOME, device=dev)
        rob.write_joint_position_to_sim_index(position=jp)
        rob.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(jp))
        rob.set_joint_effort_target_index(target=torch.zeros_like(jp))
        u.ctrl_target_joint_pos[:, :7] = jp[:, :7]
        for _ in range(3):
            u.step_sim_no_action()
        u.ci_reset_ctrl()

    def run(seconds, each=None):
        rows = []
        steps = int(round(seconds / u.step_dt))
        for k in range(steps):
            if each is not None:
                each(k * u.step_dt)
            env.step(zero_act)
            rows.append(tuple(x.clone() for x in u.ci_state()))  # q, dq, M, J are views of PhysX buffers
        return rows

    go_home()
    p_home, R_home = c.pos_d.clone(), c.rot_d.clone()

    if "model" in args.tests or "circle" in args.tests:
        r_c, period = 0.05, 5.0
        w = 2 * math.pi / period
        center = p_home - torch.tensor([0.0, r_c, 0.0], device=dev)

        def circle(t):
            c.pos_d[:] = center + r_c * torch.tensor([0.0, math.cos(w * t), math.sin(w * t)], device=dev)
            c.twist_d[:, :3] = r_c * w * torch.tensor([0.0, -math.sin(w * t), math.cos(w * t)], device=dev)

        if "hold" in args.tests:
            rows = run(2.0)
            drift = (rows[-1][5] - p_home).norm(dim=1) * 1000
            res["hold_mm"] = {"mean": drift.mean().item(), "max": drift.max().item()}
            print(f"HOLD 2 s: drift mean {drift.mean():.3f} mm, max {drift.max():.3f} mm (ros2_control 0.11 mm)", flush=True)
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        errs, model_rows = [], []

        fext = []

        def circle_and_log(t):
            circle(t)
            errs.append((t, c.pos_d.clone()))
            fext.append(c.F_ext[:, :3].norm(dim=1).clone())

        rows = run(2 * period, circle_and_log)
        torch.cuda.synchronize()
        wall = time.perf_counter() - t0
        # TCP after the step k vs the command of step k (the pipeline compares against the last published command)
        e = torch.stack([(r[5] - cmd).norm(dim=1) for r, (t, cmd) in zip(rows, errs) if t >= period / 2]) * 1000
        rms = e.pow(2).mean(0).sqrt()
        res["circle_rms_mm"] = {"mean": rms.mean().item(), "max": rms.max().item(), "max_err": e.max().item()}
        res["circle_sim_s_per_wall_s"] = N * 2 * period / wall
        print(f"CIRCLE r 5 cm / 5 s pose + twist: RMS mean {rms.mean():.2f} mm (envs {rms.min():.2f}-{rms.max():.2f}), "
              f"max {e.max():.2f} mm (ros2_control 1.29 mm); {N * 2 * period / wall:.0f} sim s / wall s", flush=True)
        fe = torch.stack(fext[len(fext) // 4:])
        res["circle_F_ext_N"] = {"rms": fe.pow(2).mean().sqrt().item(), "max": fe.max().item()}
        print(f"CIRCLE free-motion |F_ext| RMS {fe.pow(2).mean().sqrt():.2f} N, max {fe.max():.2f} N (pipeline 0.1-0.4 N "
              "RMS with the sensor model)", flush=True)
        model_rows = [rows[i] for i in (len(rows) // 3, len(rows) // 2)]
        c.twist_d[:] = 0.0
        go_home()

    if "model" in args.tests:
        import pinocchio as pin
        full = pin.buildModelFromUrdf(os.path.expanduser("~/ci_mpc_assets/fr3.urdf"))
        lock = [full.getJointId(n) for n in ("fr3_finger_joint1", "fr3_finger_joint2") if full.existJointName(n)]
        m = pin.buildReducedModel(full, lock, pin.neutral(full))
        m.gravity.linear[:] = 0.0
        dm, fid = m.createData(), m.getFrameId("fr3_hand_tcp")
        out = {}
        for i, (q, dq, M, cor, J, pos, rot, tau) in enumerate(model_rows):
            qn, dqn = q[0].double().cpu().numpy(), dq[0].double().cpu().numpy()
            Mp = pin.crba(m, dm, qn)
            Mp = np.triu(Mp) + np.triu(Mp, 1).T
            nle = pin.nonLinearEffects(m, dm, qn, dqn)
            pin.computeJointJacobians(m, dm, qn)
            pin.updateFramePlacements(m, dm)
            Jp = pin.getFrameJacobian(m, dm, fid, pin.LOCAL_WORLD_ALIGNED)
            Mx = M[0].double().cpu().numpy()  # after subtracting the armature (ci_M_has_armature)
            arm = u.ctrl_arm.double().cpu().numpy()
            out[i] = {"M_rel_err": float(np.abs(Mx - Mp).max() / np.abs(Mp).max()),
                      "M_rel_err_other_armature": float(np.abs(Mx + (1 - 2 * u.ci_M_has_armature) * np.diag(arm) * -1 - Mp).max()
                                                        / np.abs(Mp).max()),
                      "cor_err_Nm": float(np.abs(cor[0].double().cpu().numpy() - nle).max()),
                      "cor_max_Nm": float(np.abs(nle).max()),
                      "J_err": float(np.abs(J[0].double().cpu().numpy() - Jp).max()),
                      "tcp_err_mm": float(np.linalg.norm(pos[0].double().cpu().numpy() - dm.oMf[fid].translation) * 1000),
                      "rot_err": float(np.abs(rot[0].double().cpu().numpy() - dm.oMf[fid].rotation).max()),
                      "M_diag_physx": np.round(np.diag(Mx), 4).tolist(), "M_diag_pin": np.round(np.diag(Mp), 4).tolist()}
        res["model"] = out
        print("MODEL (env 0, two circle samples; M after the armature correction): " + json.dumps(out), flush=True)

    if "observer" in args.tests:
        g = torch.Generator(device=dev).manual_seed(0)
        F = 20.0 * torch.rand(N, 3, device=dev, generator=g) - 10.0
        run(1.0)
        import warp as wp
        inc0 = wp.to_torch(rob.root_view.get_link_incoming_joint_force())[0]
        for i in range(1, 8):
            b = rob.body_names.index(f"fr3_link{i}")
            print(f"OBSERVER no force, incoming fr3_link{i}: {[round(v, 3) for v in inc0[b].tolist()]}, link z "
                  f"{rob.data.body_pos_w.torch[0, b, 2]:.3f} m", flush=True)
        print(f"OBSERVER no force: F_ext env 0 {c.F_ext[0].tolist()}, held peg pos env 0 {u.held_pos[0].tolist()}",
              flush=True)
        fb = rob.body_names.index(os.environ.get("CI_FORCE_BODY", "panda_fingertip_centered"))
        rob.permanent_wrench_composer.set_forces_and_torques_index(
            forces=F[:, None, :], torques=torch.zeros(N, 1, 3, device=dev),
            positions=(rob.data.body_pos_w.torch[:, fb]).clone()[:, None, :], body_ids=[fb], is_global=True)
        rows = run(3.0)
        est = torch.stack([c.F_ext[:, :3].clone() for _ in range(1)])[0]
        err = (est + F).norm(dim=1)
        moved = (rows[-1][5] - p_home).norm(dim=1) * 1000
        res["observer_err_N"] = {"mean": err.mean().item(), "max": err.max().item(), "F_mean_norm": F.norm(dim=1).mean().item()}
        print(f"OBSERVER known force |F| mean {F.norm(dim=1).mean():.1f} N: |F_ext + F| mean {err.mean():.2f} N, "
              f"max {err.max():.2f} N (env 0: F {F[0].tolist()}, est {est[0].tolist()}); TCP moved {moved.mean():.2f} mm",
              flush=True)
        q, dq, M, cor, J, pos, rot, tau = (x.clone() for x in u.ci_state())
        Jf = rob.data.body_link_jacobian_w.torch[:, fb - 1, 0:3, 0:7]
        tau_link = -(Jf.transpose(1, 2) @ F.unsqueeze(-1)).squeeze(-1)  # link-side torque holding F at rest
        res["observer_torques_env0"] = {"tau_meas": tau[0].tolist(), "tau_cmd": c.tau_prev[0].tolist(),
                                        "tau_link_expected": tau_link[0].tolist(), "dq": dq[0].tolist()}
        print("OBSERVER torques env 0: " + json.dumps(res["observer_torques_env0"]), flush=True)
        print(f"OBSERVER |tau_meas - tau_link| mean {(tau - tau_link).abs().mean():.3f} Nm, |tau_meas - tau_cmd| mean "
              f"{(tau - c.tau_prev).abs().mean():.3f} Nm", flush=True)
        import warp as wp
        inc = wp.to_torch(rob.root_view.get_link_incoming_joint_force())[0]  # (links, 6) env 0
        for i in range(1, 8):
            b = rob.body_names.index(f"fr3_link{i}")
            print(f"OBSERVER incoming fr3_link{i}: {[round(v, 3) for v in inc[b].tolist()]} expected tau "
                  f"{tau_link[0, i - 1]:.3f} meas {tau[0, i - 1]:.3f}", flush=True)
        W = u.ci_hand_wrench()
        print(f"OBSERVER hand wrench env 0 (should be -F at rest): force {W[0, :3].tolist()}, |W_f + F| mean "
              f"{(W[:, :3] + F).norm(dim=1).mean():.3f} N", flush=True)
        rob.permanent_wrench_composer.reset()

    print("RESULT " + json.dumps(res), flush=True)
    if args.out:
        open(args.out, "a").write(json.dumps(res) + "\n")
