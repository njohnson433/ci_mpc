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
- [x] Keep: torque rate limit (100 Nm/s) ✓ (rate-scaled); nullspace term: only optional damping (`nullspace_damping`, off);
      friction compensation: upstream LuGre fixed and evaluated, replaced in sim by velocity-based compensation +
      conditional integral (docs/CONTROLLER_FINDINGS.md).
- [x] Optional parameter: real robot uses `FrankaRobotModel`, sim uses Pinocchio.
      Pinocchio everywhere is simpler and fine to start with.

### 3. MPC-ready controller interface
- [x] MPC tracking: `/cartesian_impedance_controller/mpc_command` via `realtime_tools::RealtimeBuffer`
      (active while commands arrive; no control_mode number, mode 3 is the upstream "lift" set).
- [x] Message `ci_mpc_msgs/MpcCommand`: desired pose (quaternion), twist, feedforward wrench, K, D, stamp,
      integral on/off (free space vs contact).
- [x] Damping on velocity error: `D·(ẋ_des − ẋ)` (now on absolute velocity: 6.8 mm RMS lag on a 5 cm circle).
- [x] Bypass the desired-pose exponential filter (τ = 0.5 s: ~27 mm lag on the same circle).
- [x] Integral: no reset on every setpoint (`integrator_reset_on_target`), gating from the command.
- [x] Timeout (50 ms, counted in control cycles) → hold current pose, drop twist/feedforward, reset integral.
- [x] Test in sim: `scripts/mpc_tracking_test.py` (circle 0.65 mm RMS without friction, 1.29 mm with).
- [ ] On the real robot: start with `friction_scale` < 1 (compensating friction that is not there is
      negative damping: unstable in the frictionless sim).

### 4. Run the controller in Isaac Sim via `isaacsim.ros2.control`
- [x] Add `On Playback Tick → ROS2ControlManager` (targetPrim `/fr3`, articulation root `/fr3/Geometry/base/fr3_link0`) + `ROS2 Publish Clock`.
- [x] Controller YAML with `joint_state_broadcaster` + the ported impedance controller.
- [x] Check that the in-process Controller Manager can load our plugin from `~/franka_ros2_ws/install`.
- [x] Verify with `ros2 control list_controllers` / `list_hardware_interfaces`.
- [ ] Physics rate: 1 kHz (runs at 240 Hz now), headless or standalone if the GUI is too slow.

### 5. Stage-1 MPC
- [x] `ci_mpc_msgs`, `ci_mpc_planner` packages.
- [x] Solver behind an interface: `TranslationalMpc.solve(x0, p_ref, v_ref)` (OSQP, 1 ms).
- [x] Model: EE double integrator; the plan goes to the impedance controller as pose + twist + Λ·a
      feedforward; replan from the previous plan (replanning from the measured state switches the
      impedance feedback off).
- [x] First task: reach a fixed pose (0.3–0.4 mm), then track a circle (1.0 mm RMS) — sim, identified friction.
- [x] Orientation in the MPC (rotation-vector QP, J_l-consistent twist).
- [x] Robustness in sim: friction ×0.7 / ×1.3, goal changes, workspace wall, singularity guard, fast circle,
      figure-8; force-aware acceleration limits (controller clamp 18 N vs Λ 20–30 kg).
- [x] Launch file + sim/real configs (`ros2 launch ci_mpc_planner mpc.launch.py config:=real`).
- [ ] Same tests on the real robot.

### 6. Real robot bring-up
- [ ] Network (static IP), FCI, read-only test with pylibfranka.
- [ ] RT kernel (Ubuntu Pro `realtime-kernel`). Check that the NVIDIA driver still loads,
      or keep generic and RT kernels side by side and choose in GRUB.
- [ ] Run the ported controller on `franka_hardware` with conservative gains, hold pose first.

### 4b. Done along the way (2026-10-22)
- [x] Joint identification package `ci_mpc_sysid`, validated in sim (docs/SYSID.md).
- [x] Sim armature + identified friction; impedance test suite; friction handling (docs/CONTROLLER_FINDINGS.md).
- [ ] Identify our FR3 (after task 6), feed values into the scene, re-run suite in sim and on the robot.
- [ ] Faster headless sim (physics steps without rendering every step).
- [ ] External wrench estimate in sim (`o_f_ext_hat_k` equivalent) for the controller's mode 1.

### 7. Stage-2 MPC: planar pushing, sticking contact (sim, 2026-10-23)
- [x] Pusher-slider model (quasi-static, ellipsoidal limit surface, motion cone) + offline loop with a sliding
      "true" contact (`pusher_slider.py`, `scripts/push_offline_sim.py`, unit tests).
- [x] LTV MPC (OSQP) around the reference path, motion-cone constraints; push node (approach, contact, push,
      retreat) on the MPC interface; detection-latency compensation.
- [x] Isaac push scene (box, rod on the hand); straight, arc, rotated start pass (docs/PROGRESS.md).
- [x] Slider parameter c fitted from Isaac logs (0.045); contact location chosen for the path curvature.
- [ ] Contact sliding on arcs (9 mm along the face): sliding modes in the model → stage 3.
- [ ] External wrench estimate (contact force feedback instead of the quasi-static feedforward only).
- [ ] Real robot: pusher pin in the gripper, vision latency measured, same tests.

### 8. Perception (wrist camera)
- [x] Simulated Gemini 305 on the hand (assumed mount), OrbbecSDK_ROS2 topic names, stereo depth noise model.
- [x] Box detector (color masks + depth → Detection3DArray); stacking from camera detections in sim.
- [ ] Real pipeline (YOLO + depth) on the same topics; occlusion handling (a box under another one).
- [ ] Measure the real mount (hand-eye calibration) → `gemini305_sim.yaml` / real config; check depth noise
      against the real 305.

### Later
- [x] Object(s) in the sim scene and object state estimation (perception package).
- [ ] Stage 2 / stage 3 solvers (OCS2, C3 via dairlib needs Drake + Bazel).
- [ ] Logging (rosbag) and plots for sim-to-real comparison.
- [ ] Re-check which FR3 hardware revision the URDF should use (`fr3` vs `fr3v2*`). Currently `fr3`.

## Open questions

- ~~Does the in-process Controller Manager in Isaac load C++ plugins built against the system
  Humble without problems?~~ Yes (2026-10-22).
- Joint friction/damping in the sim: armature + identified static/Coulomb/viscous friction modelled
  (2026-10-22); offset and LuGre terms are not representable in PhysX.
- Which contact task for stages 2–3 (planar pushing, pivoting, …)?
