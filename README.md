<div align="center">

# CI-MPC on the Franka FR3

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![Isaac Sim 6.2](https://img.shields.io/badge/Isaac_Sim-6.2-76B900?style=flat-square&logo=nvidia)](https://developer.nvidia.com/isaac/sim)
[![libfranka 0.21.3](https://img.shields.io/badge/libfranka-0.21.3-blue?style=flat-square)](https://github.com/frankaemika/libfranka)
![Real robot: not tested](https://img.shields.io/badge/real_robot-not_tested-lightgrey?style=flat-square)

Contact-implicit model predictive control for a Franka Research 3: developed in Isaac Sim first, then deployed on the real robot.

[Overview](#overview) • [Features](#features) • [Getting started](#getting-started) • [Usage](#usage) • [Documentation](#documentation)

</div>

## Overview

The plan has three stages. Each stage changes only the solver, not the controller or the sim:

1. **Free-space MPC**: 6-DOF tracking with force-aware limits. Finished in sim.
2. **Contact with a known mode**: planar pushing with a sticking contact. Works in sim.
3. **Contact-implicit MPC**: the contact sequence is part of the optimization. In progress, on Factory peg-in-hole: 30/30 on two held-out sets of hole offsets in sim, using a learned probe likelihood.

A fourth track started on 2026-10-26: a **vectorized Isaac Lab sim** (many robots in one process) for data collection and statistics, because the ros2_control pipeline runs only one robot per Isaac process.

> [!NOTE]
> Everything so far runs in **Isaac Sim** only. The real-robot connection test is not done yet.
> See [docs/REAL_ROBOT.md](docs/REAL_ROBOT.md) for the bring-up checklist.

## Features

- **PdZ Cartesian impedance controller in Isaac.** It runs in-process in the physics step (`isaacsim.ros2.control`) with a Pinocchio model. With identified friction: hold 0.00 mm, reach 0.11 mm, steps 0.06–0.38 mm. See [CONTROLLER_FINDINGS.md](docs/CONTROLLER_FINDINGS.md).
- **Realistic sim.** The scene has FR3 joint armature and identified joint friction (`scripts/isaac/prepare_scene.py`). Joint identification (`ci_mpc_sysid`) is validated in sim to within 0–0.4 % on Coulomb friction, 0.1–2 % on viscous and 2–4 % on armature. See [SYSID.md](docs/SYSID.md).
- **Stage-1 MPC** (`ci_mpc_planner`, OSQP, 6-DOF). It passes 9/9 tests (reach 0.25–0.42 mm, circle 1.0 mm RMS, rotation 3–4 mrad) and stays stable at friction ×0.7 / ×1.3. It drives the pick-and-place and stacking demos, including stacking from a simulated Gemini 305 wrist camera.
- **Stage-2 pushing MPC** with a sticking contact. A straight 20 cm push ends 2.2 mm off and a 90° arc 0.9 mm off. The open point is contact sliding on arcs.
- **External wrench estimate.** A momentum observer on the measured joint torques comes within 0.17 / 0.31 / 0.80 N RMS of a simulated wrist F/T sensor in contact.
- **Peg-in-hole with a 3D hole belief and active probing**, in Isaac with SDF collision:
  - With the calibration touch, height tracking, centering bound and a **learned probe likelihood**, the plain probe and the GPU MPC entry both reach **30/30** on the D1 benchmark (3 × 10 hole errors up to 5 mm). They need about half the hand model's contact time (median ~22–29 s vs ~56 s).
  - **Recommended setup: plain probe + learned v3 likelihood** (no GPU server). It inserts 30/30 on both held-out sets: R (23.9 s mean contact vs 45.7 s for the hand model) and R2 (39.4 s; larger offsets, up to 5 mm).
  - **No regression on the other tasks.** The peg-tuned flags were rerun on H1–H3: H1 peg 15/15 (base) vs 14/15 (new), H2 gear 2/5 vs 3/5, H3 AutoMate 6/12 vs 5/12. The changes neither help nor hurt outside the peg. The H3 differences are within run-to-run noise.
  - **Jam recovery** (`--jam-recovery`, off by default) frees the friction-1.0 wedge: 8/8 inserted vs 0/3 without it, and it fired no retries on D1.
  - **Gear (H2) work in progress.** All gear stalls end at the bore clearance (0.23–0.30 mm off the shaft). Yaw search plus push-follow inserts 2/3 at a 3° yaw error. A gripper-release bug (the gripper opened to 30 mm, narrower than the 30.1 mm gear) was fixed.
  - **Negative results:** the confidence gate (`--gpu-std-max`) costs +6 s, the GPU entry ties the plain probe on R2 (+0.2 s), and more data (v4, +40 sweep episodes) improves val loss by only 1 %.
- **CI-MPC research (3D C3+, GPU sampling MPC, C3 refine).** The GPU sampling MPC uses a batched FISTA plant that matches OSQP to 0.012 mm at ~140 ms per decision. Offline it enters 30/30 vs 22/30 for the heuristic. A comparison with dairlib's C3 found an **ADMM dual-rescale bug** in our C3+ (3D and planar). After the fix, the fast C3 enters 16/16 offsets offline (was 5/16) but stalls at 18–21 mm depth. **Negative results:** the learned option selector (16–19/30) and the ICNN cost-to-go.
- **Vectorized sim** (Isaac Lab Factory PegInsert, ROADMAP 9). With our FR3 asset at 240 Hz with contact, N = 1024 envs run 64 sim s per wall s, about 50× the ros2_control pipeline. The stock Panda runs 157 at N = 1024 and 224 at N = 4096. Factory's controller is still in use: porting ours to batched torch is the next step.

> [!TIP]
> The full numbers for every run (D1, R, R2, H1–H4, X, offline studies) and the reason each one won or lost are in the dated log [docs/PROGRESS.md](docs/PROGRESS.md). Open tasks are in [docs/ROADMAP.md](docs/ROADMAP.md).

## Architecture

```
user_input_client ─┐
perception ────────┴─► ci_mpc_planner (MPC, 20–100 Hz)
                              │  setpoint: pose, twist, feedforward wrench, gains
                              ▼
                     Cartesian impedance controller (1 kHz)
                              │  joint torques (gravity excluded)
                    ┌─────────┴─────────┐
                    ▼                   ▼
          franka_hardware → FR3     Isaac Sim (ros2_control in-process)
```

- **Two rates.** The MPC never commands torques. It sends setpoints, and the impedance controller turns them into torques every control step.
- **Same controller code in sim and on hardware.** Only the hardware interface underneath changes.
- **Gravity is handled below the controller.** The real FR3 compensates gravity internally (libfranka torques exclude gravity). In the sim, gravity is off on the robot links to match. Objects in the scene keep normal gravity.
- **Fixed setpoint interface** (`ci_mpc_msgs/MpcCommand`: pose, twist, feedforward wrench, gains). Later MPC stages change only the solver.

## Getting started

Full step-by-step setup: [docs/SETUP.md](docs/SETUP.md). Known problems and fixes: [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md).

### Prerequisites

| | Version |
|---|---|
| Ubuntu | 22.04, kernel 6.8.0-138-generic |
| GPU / driver | RTX 4080 16 GB / NVIDIA 595.99.02 |
| FR3 system version | 5.10 (robot server 10) |
| libfranka | 0.21.3 (`.deb`, Jammy) |
| franka_ros2 | `humble` branch (v2.x), libfranka taken from the system |
| ROS 2 | Humble |
| Isaac Sim / Isaac Lab | 6.2.0 / 3.0 (pip via `uv`, Python 3.12) |

### Workspace

The repo sits in `src/` so that colcon builds its ROS packages. Folders without a `package.xml` (docs, scripts) are ignored.

```
~/franka_ros2_ws/src/
  franka_ros2 packages …          upstream, not in this repo
  cartesian_impedance_control/    PdZ controller (own clone, branch isaac-sim-pinocchio; backup in patches/)
  messages_fr3/
  ci_mpc/                         ← this repo
~/ci_mpc_assets/                  URDF + USD scene (not in git)
~/IsaacLab/                       Isaac Lab + Isaac Sim (uv)
~/ocs2_ws/                        OCS2, separate workspace
```

### Environments: one terminal each

| Purpose | Activate | Python |
|---|---|---|
| ROS 2 / franka_ros2 | `source /opt/ros/humble/setup.bash && source ~/franka_ros2_ws/install/setup.bash` | 3.10 |
| Robot / MPC prototyping | `source ~/venvs/cimpc/bin/activate` | 3.10 |
| Isaac Sim / Isaac Lab | nothing: `cd ~/IsaacLab && uv run ...` or `~/isaac_ros.sh` | 3.12 |

> [!IMPORTANT]
> Never mix these environments in one terminal. ROS sets `PYTHONPATH` to its Python 3.10 packages, and that breaks Isaac's 3.12 environment.

> [!WARNING]
> `install/ci_mpc_planner` is a stale copy from 2026-10-23 (it is not a symlink install), and `peg_insert_demo.py` imports most planner modules from it. Until you rebuild with `colcon build --symlink-install --packages-select ci_mpc_planner`, the Isaac C3 strategies run the code without the ADMM fix.

### Quick start (sim)

```bash
# Terminal 1: Isaac Sim (no ROS sourced, no venv active)
~/isaac_ros.sh
#   File → Open → /home/njohnson/ci_mpc_assets/fr3_scene.usd
#   Press Play, then run scripts/isaac/pd_home_controller.py in the Script Editor

# Terminal 2: ROS
source /opt/ros/humble/setup.bash
source ~/franka_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic echo /joint_states --once
```

Headless controller test (~1 min): `scripts/isaac/run_impedance_test.sh`. The planner unit tests (`ci_mpc_planner/test`, workspace sourced) give 13 passed and 1 expected failure (`test_c3_inserts_at_3mm`: the retired planar C3 line).

## Usage

| Demo | Command |
|---|---|
| Pick and place (blended MPC motion, grasp from the detection) | `scripts/demo_pick_place.sh [box\|cube] [--noise] [--stops] [--headless]` |
| Box stacking, 3-cube tower (1.2–3.2 mm per level) | `scripts/demo_stack.sh [--noise] [--headless] [--camera]` |
| Planar pushing | `scripts/demo_push.sh [line\|arc\|yaw10]` |
| Factory peg-in-hole, hand-written search (0–1 mm hole error ✅, ≥ 2 mm ❌) | `scripts/demo_peg.sh [--offset dx,dy]` |
| Peg-in-hole with probing / GPU entry / learned belief (recommended: plain probe + `models/probe_likelihood_v3_both.npz`) | `scripts/peg_insert_demo.py --strategy probe [--entry gpu] [--belief-model ...]` |
| Jam recovery (recommended values, off by default) | `peg_insert_demo.py ... --jam-recovery 3 --jam-lift 0.003 --jam-shift 0.0006` |
| Gear mesh (H2), AutoMate (H3) | `scripts/demo_gear.sh`, `scripts/demo_automate.sh` |
| Impedance test suite (steps, rotations, circle, pushes, windup) | `scripts/impedance_suite.py` |
| Vectorized Factory throughput (Panda, or `--fr3`) | `~/IsaacLab/.venv/bin/python scripts/isaac/bench_factory_vec.py --num_envs 1024 [--mode contact] [--physics-hz 240] [--fr3]` |

> [!TIP]
> Benchmark queues (`scripts/queue_*.sh`) run in the background. Follow them with the live dashboard `scripts/watch_runs.sh`.

## Repository layout

```
ci_mpc_msgs/          MpcCommand (MPC → controller setpoint)
ci_mpc_planner/       MPCs (stage 1, pushing, C3+ peg-hole, GPU sampling MPC, C3 refine),
                      hole belief + learned probe likelihood (models/), mpc_node
ci_mpc_perception/    simulated Gemini 305 depth noise, box detector
ci_mpc_sim_gripper/   sim gripper controller
ci_mpc_sysid/         joint identification
patches/              backups of the controller / franka_ros2 changes
scripts/              demos (demo_*.sh), benchmark queues, tests, offline experiments
  isaac_ros.sh        start Isaac Sim with the ROS 2 (Humble) bridge
  gen_fr3_urdf.sh     generate the FR3 URDF from franka_description
  isaac/              Isaac scene setup (prepare_scene.py, *_scene.py), in-sim scripts,
                      vectorized Factory env with the FR3 (factory_fr3.py, bench_factory_vec.py)
docs/                 setup, roadmap, progress log, findings (below)
```

## Documentation

| Document | Contents |
|---|---|
| [docs/PROGRESS.md](docs/PROGRESS.md) | Dated log with all results, plus where to resume |
| [docs/ROADMAP.md](docs/ROADMAP.md) | Staged plan, next tasks (incl. 9: vectorized sim), open questions |
| [docs/SETUP.md](docs/SETUP.md) | Full environment setup, step by step |
| [docs/CONTROLLER_FINDINGS.md](docs/CONTROLLER_FINDINGS.md) | What was learned about the controller in sim |
| [docs/SYSID.md](docs/SYSID.md) | Joint identification |
| [docs/REAL_ROBOT.md](docs/REAL_ROBOT.md) | Real-robot bring-up checklist |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Problems we hit and how they were solved |
