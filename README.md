<div align="center">

# CI-MPC on the Franka FR3

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![Isaac Sim 6.2](https://img.shields.io/badge/Isaac_Sim-6.2-76B900?style=flat-square&logo=nvidia)](https://developer.nvidia.com/isaac/sim)
[![libfranka 0.21.3](https://img.shields.io/badge/libfranka-0.21.3-blue?style=flat-square)](https://github.com/frankaemika/libfranka)
![Real robot: not tested](https://img.shields.io/badge/real_robot-not_tested-lightgrey?style=flat-square)

Contact-implicit model predictive control for a Franka Research 3: developed in Isaac Sim first, then deployed on the real robot.

[Overview](#overview) • [Features](#features) • [Getting started](#getting-started) • [Demos](#demos) • [Documentation](#documentation)

</div>

## Overview

The plan has three stages. Each stage changes only the solver, not the controller or the sim:

1. **Free-space MPC**: 6-DOF tracking with force-aware limits. Finished in sim.
2. **Contact with a known mode**: planar pushing with a sticking contact. Works in sim.
3. **Contact-implicit MPC**: the contact sequence is part of the optimization. In progress, on Factory peg-in-hole: 30/30 on held-out hole offsets in sim with a learned probe likelihood.

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
  - With the calibration touch, height tracking, centering bound and **learned probe likelihood**, the plain probe and the GPU MPC entry both reach **30/30** on the D1 benchmark (3 × 10 hole errors up to 5 mm). They take about half the hand model's contact time (median ~22–29 s vs ~56 s).
  - **Held-out random offsets:** with the learned v3 likelihood, the plain probe inserts 30/30 on both held-out sets: 23.9 s mean contact on R (hand model 45.7 s) and 39.4 s on R2, where the offsets are larger (up to 5 mm). On R2 the GPU entry ties the plain probe (+0.2 s, 95 % CI −6.9 to 8.3 s), so it brings no measurable gain. **Recommended default: plain probe + v3**, with no GPU server.
  - **Negative results (2026-10-26):** the confidence gate `--gpu-std-max 1 mm` effectively switches the GPU entry off and costs +6 s, so it is not adopted. More training data (v4, +40 sweep episodes) improves val loss by only 1 %, so the likelihood is close to saturated.
  - A cross-task regression on H1–H3 is queued, because all of this week's fixes were tuned on the peg only. Earlier held-out H1: probe 14/15 vs spiral 7/15. A jam-recovery option (`--jam-recovery`, off by default) is in progress, not yet benchmarked.
- **CI-MPC research (3D C3+, GPU sampling MPC, C3 refine).** The GPU sampling MPC uses a batched FISTA plant that matches OSQP to 0.012 mm at ~140 ms per decision. Offline it enters 30/30 vs 22/30 for the heuristic. **Negative results:** the learned option selector (16–19/30) and the ICNN cost-to-go. See [PROGRESS.md](docs/PROGRESS.md).

> [!TIP]
> The full numbers for every run (D1, R, R2, H1–H4, offline studies) and the reason each one won or lost are in the dated log [docs/PROGRESS.md](docs/PROGRESS.md). Open tasks are in [docs/ROADMAP.md](docs/ROADMAP.md).

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

Headless controller test (~1 min): `scripts/isaac/run_impedance_test.sh`.

## Demos

| Demo | Command |
|---|---|
| Pick and place (blended MPC motion, grasp from the detection) | `scripts/demo_pick_place.sh [box\|cube] [--noise] [--stops] [--headless]` |
| Box stacking, 3-cube tower (1.2–3.2 mm per level) | `scripts/demo_stack.sh [--noise] [--headless] [--camera]` |
| Planar pushing | `scripts/demo_push.sh [line\|arc\|yaw10]` |
| Factory peg-in-hole, hand-written search (0–1 mm hole error ✅, ≥ 2 mm ❌) | `scripts/demo_peg.sh [--offset dx,dy]` |
| Peg-in-hole with probing / GPU entry / learned belief (recommended: plain probe + `models/probe_likelihood_v3_both.npz`) | `scripts/peg_insert_demo.py --strategy probe [--entry gpu] [--belief-model ...]` |
| Gear mesh (H2), AutoMate (H3) | `scripts/demo_gear.sh`, `scripts/demo_automate.sh` |
| Impedance test suite (steps, rotations, circle, pushes, windup) | `scripts/impedance_suite.py` |

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
  isaac/              Isaac scene setup (prepare_scene.py, *_scene.py) and in-sim scripts
docs/                 setup, roadmap, progress log, findings (below)
```

## Documentation

| Document | Contents |
|---|---|
| [docs/PROGRESS.md](docs/PROGRESS.md) | Dated log with all results, plus where to resume |
| [docs/ROADMAP.md](docs/ROADMAP.md) | Staged plan, next tasks, open questions |
| [docs/SETUP.md](docs/SETUP.md) | Full environment setup, step by step |
| [docs/CONTROLLER_FINDINGS.md](docs/CONTROLLER_FINDINGS.md) | What was learned about the controller in sim |
| [docs/SYSID.md](docs/SYSID.md) | Joint identification |
| [docs/REAL_ROBOT.md](docs/REAL_ROBOT.md) | Real-robot bring-up checklist |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Problems we hit and how they were solved |
