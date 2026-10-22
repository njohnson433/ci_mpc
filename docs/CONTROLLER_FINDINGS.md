# Cartesian impedance controller in Isaac Sim: findings (2026-10-22)

Summary of what was learned getting the PdZ Cartesian impedance controller
(`cartesian_impedance_control`, our branch `isaac-sim-pinocchio`) to run in Isaac Sim with realistic FR3
joint friction, and how it performs. Dated details, every run and the full ablation tables are in
[PROGRESS.md](PROGRESS.md); identification in [SYSID.md](SYSID.md).

## 1. Result

With the identified FR3 joint friction in the sim, the controller now settles 5 cm steps to
**0.06–0.38 mm** (8 s) and the hold/reach test to **0.11 mm**, against 5.4 mm / 2.2 mm with the original
controller. Holding a 15 N push and releasing it moves the arm 0.09 mm past home (no windup). This is the
Isaac config (`config/isaac_sim_controllers.yaml`); the real-robot config still uses the original behavior.

| | original | now (Isaac config) | frictionless sim (reference) |
|---|---|---|---|
| 5 cm steps, worst error | 5.4 mm (4 s) | 0.38 mm (8 s) | 0.57 mm (4 s) |
| hold / reach test | 0.00 / 2.2 mm | 0.00 / 0.11 mm | 0.10 / 0.47 mm |
| 10 N push Fx / Fy / Fz [mm] (set: 3 / 3 / 10) | 1.6 / 2.7 / 6.2 | 3.9 / 4.0 / 11.3 | 3.0 / 3.0 / 10.0 |
| circle r 5 cm / 5 s, RMS vs filtered reference | 10.7 mm | 7.3 mm | 6.8 mm |
| jump after releasing a held 15 N push | 0 | 0.09 mm | — |

## 2. Simulation setup (what it took)

| issue | effect | fix |
|---|---|---|
| scene had no `Physics` variant selected | no articulation, no joints, no rigid bodies | `prepare_scene.py` selects `physx` |
| Controller Manager target `fr3_link0` | synthesized URDF had no arm joints → no interfaces | target `/fr3` (joints live in `/fr3/Physics`) |
| ROS2ControlManager graph node latches its first failed setup | CM never started | call `Ros2ControlManager.setup()` directly with retries |
| no joint armature | wrist (joint 7) chattered at Nyquist (velocity sign flip on 95–99 % of steps), EE orientation wandered 0.6 rad | armature = `motor_inertia·gear²` from `franka_description` (0.606 / 0.462 / 0.206 kg m²) |
| no joint friction | sim far too ideal (0.47 mm reach) | identified static / Coulomb / viscous friction via `PhysxJointAxisAPI` |
| `app.close()` hangs headless | runs never ended | teardown + `os._exit` |

The sim's kinematics match Pinocchio FK (0.02–0.04 mm, 0.000 rad) and our plugin runs in Isaac's bundled
`controller_manager` (no ABI problem).

## 3. Controller audit

- **Works in sim:** impedance law, integral term, force clamps, torque-rate limit, `reference_pose`,
  `control_mode` 0/2/3.
- **Does not work in sim:** mode 1's automatic contact switch (reads Franka's `o_f_ext_hat_k` from
  `franka_robot_state_broadcaster`, nothing publishes it), the gripper (not driven).
- **Matches its equations exactly without friction:** Fx/Fy 10 N → 3.00 mm = (10 − 2.5)/2500, Fz → 10.02 mm =
  10/1000, Tz 1 Nm → 22.05 mrad. Note: the rotational stiffness is **K/2 per rad** (error = quaternion vector
  part ≈ θ/2).
- **Design properties that matter for MPC:**
  - force clamps ±18 N / ±9 Nm / ±2 Nm (tool z): a spring only for ~7 mm at K 2500;
  - z has K 1000 and originally **no** integral term; integral clamps ±2.5 N (≈ 1 mm at K 2500);
  - `reference_pose` goes through a first-order filter (τ = 0.5 s): ~27 mm lag on a 5 cm / 5 s circle;
  - damping acts on the **absolute** EE velocity, not the velocity error: moving targets lag by ≈ D·v/K
    (6.8 mm RMS on the circle even without friction) → task 3;
  - every new `reference_pose` zeroed the integral (so streamed references never build one).
- **Upstream bugs found** in the commented-out friction compensation: fixed 1 kHz integration and filter
  constants; `error < 0.8 mm` threshold instead of `|error|` (zeroes every negative error; fixing it: 1.47 →
  0.98 mm, and no over-compensation of Tz); a local `tau_impedance` shadowed the member it reads (always 0);
  it was added after the torque-rate limit.

## 4. Friction handling: what was tried

All in the sim with the identified friction; one change at a time. Full tables in PROGRESS.md.

| approach | accuracy | behavior | verdict |
|---|---|---|---|
| original (integral 350/350/0, ±2.5 N) | 3–5.5 mm | stuck in the stiction deadband | — |
| z integral / larger clamps only | 2.2–3.2 mm | still stuck | not enough |
| Coulomb in the direction of the impedance torque | 1.7 mm | 2.5× stiffer against external pushes | dropped |
| upstream LuGre (dt-scaled, fixed threshold) | 0.8–1.0 mm | hidden joint integrator: compliance depends on friction level and motion history (1 Nm twist: 1 / 25 / 18 mrad at friction ×0.7 / 1.0 / 1.3; 23 vs 33 mrad in two identical runs) | accurate but unpredictable |
| velocity-based (Coulomb·tanh(q̇/ε) + β·q̇) | 3.1 mm | stateless; restores compliance, removes friction lag when moving; nothing at rest | good base |
| + integral 350 | 1.6 mm (stuck) | deadband; slow wind-up then 2.3 mm slip | not enough |
| + integral 3000, unconditional | 0.6–0.9 mm | windup: 7 mm jump after a held push; arm stiff for slow loads | windup |
| **+ integral 3000, conditional** | **0.06–0.38 mm** | integrates only near the target (|e| < 3 mm, EE < 1 cm/s, output not clamped), decays otherwise, no reset on new targets; 0.09 mm windup jump | **chosen (sim)** |

Lessons:
- At rest, accuracy and predictable compliance pull against each other: anything that removes the stiction
  deadband at rest (LuGre, fast integral) adds low-frequency stiffness. Conditional integration keeps it to
  small errors; for contact phases the MPC should still be able to switch the integral off.
- The last fraction of a millimeter arrives by stick-slip (4–5 bursts per step, transients up to ~2 mm in
  PhysX). Its size depends on the stiction model, so it has to be re-checked on the real robot.
- Compensation tuned to the exact sim friction is optimistic. Friction ×0.7 / ×1.3 tests: velocity-based
  compensation over-compensates at ×0.7 (arm softer, stays stable).

## 5. Open points

1. **Real robot:** all gains are tuned against friction identified on someone else's FR3. Identify ours
   (`ci_mpc_sysid`, SYSID.md), put the values into the scene, re-run the suite in sim and on the robot.
2. **Armature:** `franka_description` gives 0.606 / 0.206 kg m², MuJoCo Menagerie 0.195 / 0.074. The
   identification will tell what the controller effectively sees through FCI.
3. **Task 3 (MPC interface): done** (2026-10-23). `ci_mpc_msgs/MpcCommand` on
   `/cartesian_impedance_controller/mpc_command`: pose without filter, damping on the velocity error,
   feedforward wrench, K/D, integral gating, 50 ms timeout → hold. Circle r 5 cm / 5 s: 0.65 mm RMS without
   friction, 1.29 mm with (old reference_pose path: 33 mm). See PROGRESS.md 2026-10-23.
   **Safety:** velocity friction compensation without (or with much less) real friction is negative damping;
   start the real robot with `friction_scale` < 1.
3b. **Stage-1 MPC (task 5): finished in sim** (2026-10-23): 6-DOF, 9/9 tests at nominal friction (reach
   0.25–0.42 mm, circle 1.0 mm RMS, rotation 3–4 mrad), stable at friction ×0.7 / ×1.3. Lessons:
   - replan from the previous plan, not from the measured state (otherwise the setpoint sits where the arm is
     and the impedance feedback is switched off: 10–15 mm short, the circle did not move at all);
   - the controller's force clamp (18 N) limits the achievable acceleration: Λ_x is 20–30 kg, so 2 m/s² was
     never possible; the MPC now derives its acceleration limits from Λ (overshoot past a wall 22 → 1 mm);
   - keep the workspace away from singularities (x 0.7 at tool-down: σ_min 0.04; the arm ran away);
   - over-compensating friction is unstable, under-compensating costs 1–2 mm: start the robot at scale 0.5;
   - rotation precision is a wrist-stiction deadband (~1–8 mrad); higher rotational K helps up to ~×2.
4. **Contact detection in sim:** an `o_f_ext_hat_k` equivalent for mode 1.
5. **Sim speed:** headless runs at ~0.55× real time because every physics step renders.

## 6. How to reproduce

```bash
cd ~/franka_ros2_ws/src/ci_mpc
~/IsaacLab/.venv/bin/python -I scripts/isaac/prepare_scene.py            # physx variant, armature, friction
scripts/isaac/run_impedance_test.sh                                       # hold / reach
CHECKER=$PWD/scripts/impedance_suite.py scripts/isaac/run_impedance_test.sh --step-hold 8   # full suite
# options: GUI=1, SCENE=..., CONTROLLER_CONFIG=..., ROS_DOMAIN_ID=... (parallel runs)
```
