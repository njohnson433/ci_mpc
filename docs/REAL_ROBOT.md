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

## 4b. Gripper: sim commands → franka_gripper
In sim the fingers are driven by the stand-in `ci_mpc_sim_gripper` (`/gripper/width`); `franka_gripper` cannot run
in sim because it is a wrapper around libfranka's `franka::Gripper`, which connects to the robot's control box.
On the robot, replace the demo's gripper calls (`scripts/pick_place_demo.py`) by:

| step | sim | real FR3 (`franka_gripper`) |
|---|---|---|
| once after start | — | action `/franka_gripper/homing` (calibrates the fingers) |
| open | `/gripper/width` ← 0.08 | action `/franka_gripper/move`: `width 0.08, speed 0.1` |
| grasp the 4 cm cube | `/gripper/width` ← 0.0 (force-limited drive: 20 N per finger = 40 N grasp) | action `/franka_gripper/grasp`: `width 0.04, speed 0.05, force 40.0, epsilon inner/outer 0.005` (sim grasps with 40 N; the 8 mm peg slid at ~8 N grasp) |
| grasp check | opening from `/joint_states` ≈ 40 mm | result `success` (final width within width ± epsilon) |
| finger state | `/joint_states` (`fr3_finger_joint1/2`) | `/franka_gripper/joint_states` |

Real actions block until done: wait for the result instead of the sim's fixed settle times.

## 5b. C3 arm stiction + friction-compensation feedforward (`scripts/real/c3_breakaway_real.py`)
Parameters for the C3 variant that worked best offline (true stiction in the C3 model + compensation feedforward:
16/16, PROGRESS 2026-10-27 "C3 friction"): `--c3-arm-friction x,y,z,rx,ry` and `--c3-stiction-ff lateral,z`.
Needs section 4 up to the hold test (controller active, `friction_compensation` set to what C3 will use).
- No fixture needed: free space, hand mounted, no peg (a few grams; the ± pairs cancel the bias anyway).
  Enabling device in hand.
- [ ] `python3 scripts/real/c3_breakaway_real.py --dry-run --goto 0.5,0.1,0.28` (read only: pose, /joint_states rate,
      controller params, planned move).
- [ ] Two poses, both measured in Isaac: home (`--goto 0.307,0,0.487`) and Isaac's probe pose (`--goto 0.5,0.1,0.28`,
      tip 10 mm above where the hole is in sim). `--goto` moves the set-point in a straight line at 15 mm/s,
      orientation kept, aborts at 15 mm lag, refuses goals outside xy radius 0.25–0.75 m, z 0.15–0.7 m, > 0.5 m away.
  `python3 scripts/real/c3_breakaway_real.py --goto 0.5,0.1,0.28 --dirs +x,-x,+y,-y,+z,-z,+rx,-rx,+ry,-ry
  --out ~/ci_mpc_runs/real/breakaway/<name>` (3 repeats, 0.5 N/s up to 8 N, 0.1 Nm/s up to 2 Nm, aborts at
  5 mm / 0.03 m/s; ~6 min per pose).
- [ ] Repeat with the robot warm and at a second pose; per axis stiction = (f+ + f−)/2, bias = (f+ − f−)/2.
- [ ] Check the printed suggestion offline before C3 runs on the robot: `c3_stall_diag.py` with the measured values
      as the plant and the suggested values in the model, sweep the feedforward ±1 step.
Validated against Isaac's ros2_control pipeline (home pose, PROGRESS 2026-10-27 "real-robot breakaway tool").

## 5. Identification and tests
- [ ] `JointIdController`: one joint, small amplitude, warm robot; then all joints, 2–3 poses (SYSID.md).
- [ ] Fitted values → sim scene (`prepare_scene.py --params`) and controller.
- [ ] Stage-1 MPC: `ros2 launch ci_mpc_planner mpc.launch.py config:=real` (v 0.1 m/s, a 0.5 m/s², small box,
      singularity stop σ 0.1), then the stage-1 tests; compare with sim.
