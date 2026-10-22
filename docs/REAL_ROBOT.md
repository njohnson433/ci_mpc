# Real FR3: bring-up checklist (not done yet)

Everything below is prepared in sim; none of it has run on the robot. Agreed procedure (2026-10-23).

## 1. Connection (at the robot)
- [ ] Ethernet PC ↔ FR3 control box, static IP on the PC interface (Franka default network 172.16.0.x).
- [ ] Desk: unlock joints, **activate FCI**.
- [ ] Enabling device (user stop) in the hand of the person at the robot for every test below.

## 2. Real-time PC
- [ ] RT kernel (Ubuntu Pro `realtime-kernel` or PREEMPT_RT); check the NVIDIA driver, otherwise keep both
      kernels and choose in GRUB.
- [ ] libfranka `communication_test`: packet loss at 1 kHz.

## 3. Read-only
- [ ] Read q / robot state (libfranka / pylibfranka), no commands.
- [ ] Pinocchio EE pose vs Franka's pose, joint order.

## 4. Controller, step by step
- [ ] Franka's example controller via `franka_hardware` (ROS 2 side works).
- [ ] `cartesian_impedance_controller`, conservative gains, **friction compensation off**, hold pose only.
      Decide `model_source`: start with `franka` (Franka's validated model), compare with `pinocchio`.
- [ ] Small steps (a few mm), then the integral (conditional, as in the Isaac config).
- [ ] **Friction compensation ramp** (`friction_compensation: velocity`, identified values from the
      controller header):
      - `friction_scale` **0.5** → hold, small motions, **nudge test**;
      - **0.75** → nudge test; **1.0** → nudge test.
      - Nudge test: push the arm a little by hand and let go. Comes back and settles = fine. Keeps moving,
        oscillates or speeds up = over-compensated (negative damping): go back to the last good scale.
      - Why: the values come from another FR3, friction drops when the robot is warm, and Franka's torque loop
        already removes part of it. In sim, compensating 1.0 against friction ×0.7 went unstable; compensating
        0.5–0.7 cost only 1–2 mm of tracking (PROGRESS.md 2026-10-23).
      - Better: identify one or two joints first (`ci_mpc_sysid`, SYSID.md) to know whether our friction is
        above or below these values.

## 5. Identification and tests
- [ ] `JointIdController`: one joint, small amplitude, warm robot; then all joints, 2–3 poses (SYSID.md).
- [ ] Fitted values → sim scene (`prepare_scene.py --params`) and controller.
- [ ] Stage-1 MPC: `ros2 launch ci_mpc_planner mpc.launch.py config:=real` (v 0.1 m/s, a 0.5 m/s², small box,
      singularity stop σ 0.1), then the stage-1 tests; compare with sim.
