# CI-MPC on the Franka FR3

Contact-implicit model predictive control (CI-MPC) for a Franka Research 3, developed in
simulation (Isaac Sim) first and then deployed on the real robot.

The plan is staged: start with a simple MPC in free space, then add contact with a known
mode, and finally make the contact sequence part of the optimization (contact-implicit).

## Status

| Component | Status |
|---|---|
| Ubuntu 22.04, NVIDIA driver 595 | ✅ |
| libfranka 0.21.3 (FR3, system version 5.10) | ✅ installed |
| ROS 2 Humble + franka_ros2 (humble, v2.x) | ✅ tested with fake hardware |
| Isaac Sim 6.2 + Isaac Lab 3.0 | ✅ running, ROS 2 bridge works |
| FR3 in Isaac Sim (`fr3_scene.usd`) | ✅ imported, gravity off on robot links, effort mode |
| In-sim controller (physics callback) | ✅ joint PD holds home pose at 240 Hz |
| PdZ Cartesian impedance controller in Isaac (ros2_control in-process, Pinocchio model) | ✅ hold 0.00 mm, reach 0.11 mm, steps 0.06–0.38 mm with identified friction |
| Sim realism: FR3 joint armature + identified friction | ✅ `scripts/isaac/prepare_scene.py` |
| Joint identification (`ci_mpc_sysid`) | ✅ validated in sim (Coulomb 0–0.4 %, viscous 0.1–2 %, armature 2–4 %); real robot pending |
| Impedance test suite (steps, rotations, circle, pushes, windup) | ✅ `scripts/impedance_suite.py` |
| MPC tracking interface (task 3, `ci_mpc_msgs/MpcCommand`) | ✅ circle 0.65 mm RMS without friction |
| Stage-1 MPC (`ci_mpc_planner`, OSQP, 6-DOF) | ✅ finished in sim: 9/9 tests (reach 0.25–0.42 mm, circle 1.0 mm RMS, rotation 3–4 mrad), stable at friction ×0.7/×1.3 |
| Pick-and-place demo (sim) | ✅ `scripts/demo_pick_place.sh [box\|cube] [--noise] [--stops] [--headless]`; blended MPC motion, grasp from the detection |
| Box stacking demo (sim) | ✅ `scripts/demo_stack.sh [--noise] [--headless]`: 3-cube tower, 1.2–3.2 mm per level; `--camera`: from the simulated Gemini 305 wrist camera (box detector on color + noisy depth), top 1.7 mm off base |
| Stage-2 pushing MPC (sticking contact, sim) | ✅ `scripts/demo_push.sh [line\|arc\|yaw10]`: straight 20 cm 2.2 mm, 90° arc 0.9 mm final (8 mm RMS), 10° start corrected; contact sliding on arcs open → stage 3 |
| Real robot connection test | ⏳ not done yet |

See [docs/REAL_ROBOT.md](docs/REAL_ROBOT.md) for the real-robot bring-up checklist, [docs/CONTROLLER_FINDINGS.md](docs/CONTROLLER_FINDINGS.md) for what was learned about the controller in
sim, [docs/SYSID.md](docs/SYSID.md) for the identification, [docs/ROADMAP.md](docs/ROADMAP.md) for the next
tasks and [docs/PROGRESS.md](docs/PROGRESS.md) for the dated log.

## Architecture

```
user_input_client ─┐
perception (later) ┴─► ci_mpc_planner (MPC, 20–100 Hz)
                              │  setpoint: pose, twist, feedforward wrench, gains
                              ▼
                     Cartesian impedance controller (1 kHz)
                              │  joint torques (gravity excluded)
                    ┌─────────┴─────────┐
                    ▼                   ▼
          franka_hardware → FR3     Isaac Sim (ros2_control in-process)
```

Design rules:

- **Two rates.** The MPC never commands torques. It sends setpoints; the impedance controller
  turns them into torques every control step.
- **Same controller code in sim and on hardware.** Only the hardware interface underneath changes.
- **Gravity is handled below the controller.** The real FR3 compensates gravity internally
  (libfranka torques exclude gravity). In the sim, gravity is disabled on the robot links to
  match. Objects in the scene keep normal gravity.
- **Keep the setpoint interface fixed** (pose, twist, feedforward wrench, gains), so later MPC
  stages only change the solver, not the controller or sim.

## Where this repo lives

```
~/franka_ros2_ws/src/
  franka_ros2 packages …          upstream, not in this repo
  cartesian_impedance_control/    PdZ controller (own clone; later our fork)
  messages_fr3/
  ci_mpc/                         ← this repo
~/ci_mpc_assets/                  URDF + USD scene (not in git)
~/IsaacLab/                       Isaac Lab + Isaac Sim (uv)
~/ocs2_ws/                        OCS2, separate workspace
```

The repo sits in `src/` so that the ROS packages we add later (`ci_mpc_msgs`,
`ci_mpc_planner`, …) are built by `colcon` automatically. Folders without a `package.xml`
(docs, scripts) are ignored by colcon.

## Repository layout

```
README.md                  this file
docs/
  SETUP.md                 full environment setup, step by step
  ROADMAP.md               staged plan and open tasks
  TROUBLESHOOTING.md       problems we hit and how they were solved
scripts/
  isaac_ros.sh             start Isaac Sim with the ROS 2 (Humble) bridge
  gen_fr3_urdf.sh          generate the FR3 URDF from franka_description
  isaac/                   scripts to run in Isaac Sim's Script Editor
    find_articulation_root.py
    disable_robot_gravity.py
    pd_home_controller.py
    stop_controller.py
```

## Quick start (sim)

```bash
# Terminal 1 – Isaac Sim (no ROS sourced, no venv active)
~/isaac_ros.sh
#   File → Open → /home/njohnson/ci_mpc_assets/fr3_scene.usd
#   Press Play, then run scripts/isaac/pd_home_controller.py in the Script Editor

# Terminal 2 – ROS
source /opt/ros/humble/setup.bash
source ~/franka_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic echo /joint_states --once
```

## Environments – one terminal each

| Purpose | Activate | Python |
|---|---|---|
| ROS 2 / franka_ros2 | `source /opt/ros/humble/setup.bash && source ~/franka_ros2_ws/install/setup.bash` | 3.10 |
| Robot / MPC prototyping | `source ~/venvs/cimpc/bin/activate` | 3.10 |
| Isaac Sim / Isaac Lab | nothing — `cd ~/IsaacLab && uv run ...` or `~/isaac_ros.sh` | 3.12 |

Never mix them in one terminal. ROS sets `PYTHONPATH` to its 3.10 packages, which breaks Isaac's 3.12 environment.

## Key versions

| | Version |
|---|---|
| Ubuntu | 22.04, kernel 6.8.0-138-generic |
| GPU / driver | RTX 4080 16 GB / NVIDIA 595.99.02 |
| FR3 system version | 5.10 (robot server 10) |
| libfranka | 0.21.3 (`.deb`, Jammy) |
| franka_ros2 | `humble` branch (v2.x), libfranka taken from the system |
| ROS 2 | Humble |
| Isaac Sim / Isaac Lab | 6.2.0 / 3.0 (pip via `uv`, Python 3.12) |
