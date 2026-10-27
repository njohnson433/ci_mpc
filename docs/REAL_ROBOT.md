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
- On the robot always pass `--ee-profile scripts/real/desk_ee_hand_camera_mount_v0.1.endeffector-profile.json`
  (the Desk profile in use: hand + camera mount, 0.88 kg, TCP 111.9 mm from the flange, yaw −45°). The controller
  (`model_source: franka`) tracks that TCP, 8.5 mm beyond the URDF `fr3_hand_tcp`; without it the script aborts at
  start (the arm jumps toward the mismatched set-point). `--goto` stays in `fr3_hand_tcp` coordinates (Isaac's numbers).
- [ ] `python3 scripts/real/c3_breakaway_real.py --ee-profile ... --dry-run --goto 0.5,0.1,0.28` (read only: pose, /joint_states rate,
      controller params, planned move).
- **Session script** (run in your own terminal; asks before anything moves; Ctrl-C deactivates and stops all):
  `scripts/real/c3_breakaway_session.sh hold [--goto x,y,z]` (bring-up hold test) and
  `scripts/real/c3_breakaway_session.sh measure --goto ... --dirs ... --out ...`. It launches franka_bringup
  (`robot_type:=fr3`, robot 192.168.1.200, ROS_DOMAIN_ID 61), checks read-only, activates the controller with
  `cartesian_impedance_control/config/real_robot_controllers.yaml`, passes `--ee-profile` itself.
- [ ] Two poses, both measured in Isaac: home (`--goto 0.307,0,0.487`) and Isaac's probe pose (`--goto 0.5,0.1,0.28`,
      tip 10 mm above where the hole is in sim). `--goto` moves the set-point in a straight line at 15 mm/s,
      orientation kept, aborts at 15 mm lag, refuses goals outside xy radius 0.25–0.75 m, z 0.15–0.7 m, > 0.5 m away.
  `python3 scripts/real/c3_breakaway_real.py --ee-profile ... --goto 0.5,0.1,0.28 --dirs +x,-x,+y,-y,+z,-z,+rx,-rx,+ry,-ry
  --out ~/ci_mpc_runs/real/breakaway/<name>` (3 repeats, 0.5 N/s up to 8 N, 0.1 Nm/s up to 2 Nm, aborts at
  5 mm / 0.03 m/s; ~6 min per pose).
- [ ] Repeat with the robot warm and at a second pose; per axis stiction = (f+ + f−)/2, bias = (f+ − f−)/2.
- [ ] Check the printed suggestion offline before C3 runs on the robot: `c3_stall_diag.py` with the measured values
      as the plant and the suggested values in the model, sweep the feedforward ±1 step.
Validated against Isaac's ros2_control pipeline (home pose, PROGRESS 2026-10-27 "real-robot breakaway tool").

## 5. Identification and tests
- [ ] `JointIdController`: one joint, small amplitude, warm robot; then all joints, 2–3 poses (SYSID.md).
      Session script: `scripts/real/joint_id_session.sh test` (home, joint 7, ~2 min), then `home`, `probe`, `side`
      (all 7 joints each, ~6–10 min) or `all`. Franka's move_to_start puts the arm at the joint configuration, then
      JointIdController with `ci_mpc_sysid/config/real_sysid.yaml` (±0.2 rad, ≤ 0.4 rad/s, ramps ≤ 3 Nm, sines
      ≤ 0.5 rad/s), fit per pose → `~/ci_mpc_runs/real/joint_id/<pose>_<HHMM>/fitted.yaml`. No questions; Ctrl-C stops.
- [ ] Fitted values → sim scene (`prepare_scene.py --params`) and controller.
- [ ] Stage-1 MPC: `ros2 launch ci_mpc_planner mpc.launch.py config:=real` (v 0.1 m/s, a 0.5 m/s², small box,
      singularity stop σ 0.1), then the stage-1 tests; compare with sim.
