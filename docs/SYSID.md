# Joint identification (friction, effective armature)

Goal: one set of FR3 joint parameters that is used in three places, so that sim and real robot behave
the same: the Isaac scene (`scripts/isaac/prepare_scene.py --params`), the controller's friction
compensation, and the MPC model. The parameters are identified **on the real robot, through the same
interface the controller uses** (FCI torque commands, gravity compensated by the robot), because what
matters is the friction and inertia the controller sees, not the raw gearbox values.

## Why not use published values

| source | what it gives | problem |
|---|---|---|
| Franka `franka_description` `dynamics.yaml` | `friction 0.2`, `damping 0.003` for every joint; `motor_inertia`, `gear_ratio` | friction is a placeholder (same for all joints, units undocumented). Franka publishes no friction data ("The manufacturer does not provide any information on" it, Gaz et al. 2019). |
| upstream author's identification (controller header) | per-joint static/Coulomb 0.42–1.29 Nm, viscous 0–1.18 Nm s/rad, offsets, LuGre | one specific FR3, method undocumented here |
| MuJoCo Menagerie `franka_fr3` | frictionloss 1.14 (j1–4), 0.76, 0.44, 0.25 Nm; damping 0.21; armature 0.195 / 0.074 | origin not documented; armature 3× below `motor_inertia·gear²` |
| Gaz et al. 2019 (Panda) | sigmoid friction, φ1 0.30–1.28 Nm | Panda, not FR3 |

## Model

Per joint, while moving: `tau_f = coulomb·sign(dq) + viscous·dq + offset`; at rest the joint holds
until `|tau| > static`. Effective inertia = URDF link inertia (Pinocchio) + `armature`. PhysX can
represent static, Coulomb, viscous and armature, not the offset.

## Tools (package `ci_mpc_sysid`)

- `JointIdController` (`ci_mpc_sysid/JointIdController`): holds all joints with a joint PD at the
  activation pose and, per joint, runs
  - **sweep**: constant-velocity passes ±`sweep_amplitude` at each of `velocities`, both directions,
  - **breakaway**: open-loop torque ramp (`breakaway_rate`) until the joint moves, both directions,
  - **sine**: sinusoidal tracking at `sine_freqs` (amplitude limited by `sine_max_velocity`).
  Safety: per-joint torque limits, excursion limit (tested joint `sweep_amplitude + 0.15` rad, others
  `max_excursion`), velocity limit `max_velocity` → abort and hold; torque-rate limit 1 Nm/ms.
  Logs every cycle on `/joint_id_controller/log`.
- `run_joint_id.py --out id.npz`: loads/activates the controller, records the log until done/abort.
- `fit_joint_id.py id.npz [--truth params.yaml] [--out fitted.yaml]`: residual
  `tau_cmd − RNEA(q, dq, ddq)` (gravity off) → sweep: least squares for Coulomb/viscous/offset;
  breakaway: torque at motion onset → static; sine: residual minus friction = armature · ddq.

## Validation in sim (ground truth known)

`ci_mpc_sysid/config/fr3_joint_params_sim.yaml` is what the scene contains; the fit must recover it.

```bash
cd ~/franka_ros2_ws/src/ci_mpc
CHECKER=$PWD/ci_mpc_sysid/scripts/run_joint_id.py \
CONTROLLER_CONFIG=~/franka_ros2_ws/install/ci_mpc_sysid/share/ci_mpc_sysid/config/isaac_sysid.yaml \
  scripts/isaac/run_impedance_test.sh --out /tmp/id.npz
python3 ci_mpc_sysid/scripts/fit_joint_id.py /tmp/id.npz --truth ci_mpc_sysid/config/fr3_joint_params_sim.yaml
```

Result 2026-10-22 (all 7 joints, 508 s sim, `ci_mpc_sysid/results/sim_validation_2026-10-22.yaml`):

| | Coulomb | viscous | armature | static |
|---|---|---|---|---|
| error vs truth | 0.0–0.4 % | 0.1–2.0 % | 2.1–3.7 % (all high) | 1.4–4.7 % (all high) |

Sweep samples are selected by measured acceleration (`--max-acc`): selecting by time inside the reference's
cruise window left the heavy joints still accelerating at 0.8 rad/s and biased viscous by 10–23 %. A joint
regression of friction + armature over sweep and sine data was worse (up to 40 %) and was dropped.
Breakaway torque is taken at motion onset (`--onset-vel`), not at the controller's stop detection (+13 %).

## On the real FR3 (not done yet)

1. Robot bring-up first (ROADMAP task 6): FCI, `franka_hardware`, a controller_manager config that
   registers `ci_mpc_sysid/JointIdController` (generic interfaces only, no franka model needed).
2. Clear workspace around the arm; user enabling device in hand. Start with **one joint**, the slowest
   velocities, small amplitude (`joints: [7]`, `sweep_amplitude: 0.15`), and check the hold gains: the
   real joints are much stiffer/heavier in feel than the sim's; `tau_limit` stays conservative.
3. **Warm up** the robot 10–15 min (friction drops noticeably when warm), then run all joints.
4. Repeat in 2–3 arm configurations (load-dependent friction on j2/j4) and on two days.
5. Fit, compare to the sim/published values, write `fr3_joint_params_<robot>.yaml`.
6. Put the values into the scene (`prepare_scene.py --params ...`), re-run the impedance suite in sim
   and the same suite on the real robot, and compare error **time series**, not only final numbers.

Notes for real data: velocity is noisier → raise `--onset-vel` for the breakaway fit until the onset is
stable; check the log for gaps (`run_joint_id.py` reports them; at 1 kHz the realtime publisher may drop).
