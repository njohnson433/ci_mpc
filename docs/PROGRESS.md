# Progress log

Running log of what has been done, what was found, and where to resume. Newest entry on top.
Read this before starting work (and before re-investigating something "new").

## Where to resume

- **Current task:** ROADMAP task 4 (run the impedance controller in Isaac via `isaacsim.ros2.control`).
  Task 2 (Pinocchio model source) is done on the controller branch below.
- **Next action:** get `scripts/isaac/run_impedance_test.sh` to pass (hold pose, then reach
  `reference_pose` (0.4, 0.1, 0.4)). Then ROADMAP task 3 (MPC tracking mode).
- **Code that is not in this repo:**
  - `~/franka_ros2_ws/src/cartesian_impedance_control` (upstream `LucasG2001/cartesian_impedance_control`,
    no push access). Our work is on local branch `isaac-sim-pinocchio` (commit on top of `main` f8d5fe2).
    Backup: `patches/cartesian_impedance_control_isaac-sim-pinocchio.patch`.
    Restore: `git checkout -b isaac-sim-pinocchio f8d5fe2 && git am < .../ci_mpc/patches/cartesian_impedance_control_isaac-sim-pinocchio.patch`
  - `~/franka_ros2_ws/src` (franka_ros2 `humble`): `franka_bringup/config/controllers.yaml` registers
    `cartesian_impedance_controller`. Backup: `patches/franka_ros2_humble_controllers_yaml.patch` (`git apply`).
  - Assets outside git: `/home/njohnson/ci_mpc_assets/fr3_scene.usd`, `fr3.urdf`.
- **Regenerate the controller patch after changes:**
  `cd ~/franka_ros2_ws/src/cartesian_impedance_control && git format-patch main..isaac-sim-pinocchio --stdout > ../ci_mpc/patches/cartesian_impedance_control_isaac-sim-pinocchio.patch`

## Log

### 2026-10-22: Isaac end-to-end test (task 4), in progress
- Added a headless test: `scripts/isaac/run_impedance_test.sh` starts Isaac (`headless_impedance_sim.py`,
  clean `env -i`, `ROS_DOMAIN_ID=42`) and the ROS-side checker `scripts/check_impedance_tracking.py`
  (hold 3 s within 5 mm, then reach target within 1 cm / 0.05 rad in 10 s). The two sides sync through
  files in a temp dir. `scripts/isaac/add_ros2_control_graph.py` builds the Action Graph
  (`On Playback Tick → ROS2ControlManager` + `ROS2 Publish Clock`); also usable from the Script Editor.
- `scripts/isaac_ros.sh` now exposes the workspace plugins: `AMENT_PREFIX_PATH` = workspace + `/opt/ros/humble`,
  `LD_LIBRARY_PATH` = Isaac's bundled Humble libs first, then workspace, then `/opt/ros/humble/lib`.
  `--python script.py` runs a SimulationApp script in the same environment.
- Isaac bundles its own `controller_manager`, `hardware_interface`, `controller_interface`, `rclcpp`, so our
  plugin runs against those, not apt's. ABI mismatch is still the main open risk.
- First run (previous session) was cut off when the session ended, ~12 s into Isaac startup. No result.
- Note: the GPU is shared with the `ski_detection` YOLO training (~8.8 GB + 1.4 GB of 16 GB).

### 2026-10-22: Pinocchio model source (task 2), done
- Branch `isaac-sim-pinocchio` of `cartesian_impedance_control`: `PinocchioModel` (pimpl), params
  `model_source` (franka | pinocchio), `urdf_path`, `ee_frame`, `arm_id`, `enable_input_server`;
  rate-independent timing (integrator `* dt_`, filter `1-(1-a)^(dt/1ms)`, torque-rate limit scaled).
  Configs `config/isaac_sim_controllers.yaml` (240 Hz, pinocchio) and `config/real_robot_controllers.yaml`.
- Sanity check `pinocchio_model_check`: home TCP = (0.3069, 0, 0.4869), matches libfranka.
- **Bug found:** libfranka embeds its own Pinocchio and exports template instances (`ModelTpl`, `DataTpl`)
  with a different layout. If libfranka is linked first, `PinocchioModel` binds to them and crashes in
  `Model::operator=` (`std::length_error: vector::reserve`). Fix: link Pinocchio 4 before libfranka
  (CMakeLists). Same Eigen 3.4.0 on both sides, so that was not the cause.
- libfranka's own Model expects an arm-only URDF (our `fr3.urdf` has finger joints).
- Regression on mock hardware (isolated `ROS_DOMAIN_ID`): update loop runs, effort ≈ 1e-14 Nm at rest,
  torques respond to a reference pose and stay bounded (clamps/anti-windup saturate since mock doesn't move).
  `model_source: franka` path unchanged.
