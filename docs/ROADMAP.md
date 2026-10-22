# Roadmap

## Stages

1. **Simple MPC, no contact.** The end effector reaches or tracks a target in free space,
   using a task-space model and a QP (OSQP). This tests the full pipeline: MPC → impedance
   controller → Isaac Sim → real FR3.
2. **Contact with a known mode.** For example, pushing an object with an assumed sticking
   contact. The contact is in the model, but the mode sequence is fixed. OCS2 (switched
   systems) is a candidate solver here.
3. **Contact-implicit MPC.** The solver chooses the contact modes itself, e.g. a complementarity
   formulation such as C3 / C3+, or our own.

## Next tasks

### 1. Inspect the PdZ controller's Franka dependencies
- [x] `grep` the controller for `franka_semantic_components`, `FrankaRobotModel`,
      `FrankaRobotState`, `getMassMatrix`, `getCoriolis`, `getZeroJacobian`, `getPoseMatrix`,
      and the `*_interface_configuration` functions.
- [x] List which values come from Franka (M, C, J, EE pose, q, dq, τ_ext).

### 2. Port the controller to a Pinocchio model source
- [x] Compute M(q), C(q, q̇)q̇, J(q) and the EE pose with Pinocchio from `fr3.urdf`.
- [x] Use only generic interfaces: `fr3_jointN/position`, `/velocity` (state), `/effort` (command).
      These exist in both `franka_hardware` and Isaac's in-process ros2_control.
- [x] Keep gravity out of the command (the FR3 compensates it internally; the sim has robot gravity off).
- [ ] Keep: torque rate limit (100 Nm/s) ✓ (rate-scaled); nullspace term: only optional damping (`nullspace_damping`, off);
      friction compensation: still commented out upstream (check that it makes sense in sim).
- [x] Optional parameter: real robot uses `FrankaRobotModel`, sim uses Pinocchio.
      Pinocchio everywhere is simpler and fine to start with.

### 3. MPC-ready controller interface
- [ ] New mode 3 = "MPC tracking": subscribe to a setpoint topic via `realtime_tools::RealtimeBuffer`.
- [ ] Message `MpcCommand`: desired pose (quaternion), twist, feedforward wrench, K, D, stamp.
- [ ] Damping on velocity error: `D·(ẋ_des − ẋ)`.
- [ ] Bypass the desired-pose exponential filter in mode 3.
- [ ] Timeout (≈ 50 ms without a new setpoint) → hold current pose.

### 4. Run the controller in Isaac Sim via `isaacsim.ros2.control`
- [x] Add `On Playback Tick → ROS2ControlManager` (targetPrim `/fr3`, articulation root `/fr3/Geometry/base/fr3_link0`) + `ROS2 Publish Clock`.
- [x] Controller YAML with `joint_state_broadcaster` + the ported impedance controller.
- [x] Check that the in-process Controller Manager can load our plugin from `~/franka_ros2_ws/install`.
- [x] Verify with `ros2 control list_controllers` / `list_hardware_interfaces`.
- [ ] Physics rate: 1 kHz (runs at 240 Hz now), headless or standalone if the GUI is too slow.

### 5. Stage-1 MPC
- [ ] `ci_mpc_msgs`, `ci_mpc_planner` packages.
- [ ] Solver behind an interface: `solve(state, goal) → command`.
- [ ] Model: end effector as a mass-spring-damper pulled toward the setpoint (matches the impedance loop).
- [ ] First task: reach a fixed pose, then track a circle.

### 6. Real robot bring-up
- [ ] Network (static IP), FCI, read-only test with pylibfranka.
- [ ] RT kernel (Ubuntu Pro `realtime-kernel`). Check that the NVIDIA driver still loads,
      or keep generic and RT kernels side by side and choose in GRUB.
- [ ] Run the ported controller on `franka_hardware` with conservative gains, hold pose first.

### Later
- [ ] Object(s) in the sim scene and object state estimation (perception package).
- [ ] Stage 2 / stage 3 solvers (OCS2, C3 via dairlib needs Drake + Bazel).
- [ ] Logging (rosbag) and plots for sim-to-real comparison.
- [ ] Re-check which FR3 hardware revision the URDF should use (`fr3` vs `fr3v2*`). Currently `fr3`.

## Open questions

- ~~Does the in-process Controller Manager in Isaac load C++ plugins built against the system
  Humble without problems?~~ Yes (2026-10-22).
- Joint friction/damping in the sim: armature + identified static/Coulomb/viscous friction modelled
  (2026-10-22); offset and LuGre terms are not representable in PhysX.
- Which contact task for stages 2–3 (planar pushing, pivoting, …)?
