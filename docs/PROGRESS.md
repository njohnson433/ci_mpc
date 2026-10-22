# Progress log

Running log of what has been done, what was found, and where to resume. Newest entry on top.
Read this before starting work (and before re-investigating something "new").

## Where to resume

- **Current task:** ROADMAP task 3 (MPC tracking mode in the controller). In parallel: identification
  setup (`ci_mpc_sysid`, `docs/SYSID.md`) is validated in sim and ready for the real FR3 once task 6 bring-up is done. Task 4 (controller in Isaac)
  **passes**: `scripts/isaac/run_impedance_test.sh` → hold 0.1 mm / 0.0003 rad, reach 0.47 mm / 0.0000 rad,
  <1 cm in 1.47 s (2 of 2 runs). Tasks 1–2 done.
  With identified joint friction (now the default scene): hold 0.00 mm, reach **2.2 mm** / 0.0035 rad,
  <1 cm in 1.58 s (2 of 2 runs, deterministic).
- **Open follow-ups for task 4:** physics/control at 1 kHz (now 240 Hz); RTF is ~0.35 with the GPU shared;
  the ~2 mm friction offset does not converge out (integrator too weak?); friction compensation still off.
- **Code that is not in this repo:**
  - `~/franka_ros2_ws/src/cartesian_impedance_control` (upstream `LucasG2001/cartesian_impedance_control`,
    no push access). Our work is on local branch `isaac-sim-pinocchio` (2 commits on top of `main` f8d5fe2).
    Backup: `patches/cartesian_impedance_control_isaac-sim-pinocchio.patch`.
    Restore: `git checkout -b isaac-sim-pinocchio f8d5fe2 && git am < .../ci_mpc/patches/cartesian_impedance_control_isaac-sim-pinocchio.patch`
  - `~/franka_ros2_ws/src` (franka_ros2 `humble`): `franka_bringup/config/controllers.yaml` registers
    `cartesian_impedance_controller`. Backup: `patches/franka_ros2_humble_controllers_yaml.patch` (`git apply`).
  - Assets outside git: `/home/njohnson/ci_mpc_assets/fr3_scene.usd`, `fr3.urdf`. The scene needs
    `scripts/isaac/prepare_scene.py` (physx variant + joint armature); old copy `fr3_scene.usd.bak-20261022`.
- **Regenerate the controller patch after changes:**
  `cd ~/franka_ros2_ws/src/cartesian_impedance_control && git format-patch main..isaac-sim-pinocchio --stdout > ../ci_mpc/patches/cartesian_impedance_control_isaac-sim-pinocchio.patch`

## Log

### 2026-10-22 (night): friction handling in the controller (option 1)
- Controller params (defaults = previous behavior): `integrator_weights`, `integrator_limits`,
  `friction_compensation` none|lugre|coulomb, `friction_offsets`, `friction_abs_threshold`, `friction_scale`,
  `friction_tau_eps`. Upstream issues found in the commented-out friction code: integrates with a fixed
  0.001 s and α = 0.1 per step (1 kHz only → now dt-scaled), thresholds `error < 0.8 mm` instead of
  `|error|` (zeroes all negative errors), a local `tau_impedance` shadowed the member it reads (always 0),
  and it was added after the torque-rate limit (now before). At rest the "LuGre" state integrates
  0.025·τ_impedance, i.e. it acts as a clamped (±3.5 Nm) joint-space integrator.
- Batch 1, friction scene, steps (worst error after 4 s) and 10 N / 1 Nm pushes (end deflection;
  frictionless ideal 3.0 / 3.0 / 10.0 mm / 22.1 mrad):

  | config | worst step | Fx / Fy / Fz / Tz |
  |---|---|---|
  | B0 baseline | 5.52 mm | 1.50 / 2.61 / 6.21 / 16.6 |
  | B1 z integral 350 | 3.21 | 1.58 / 2.62 / 6.54 / 16.6 |
  | B2 z integral + limits 8 N | 2.18 | 1.62 / 2.63 / 5.85 / 16.6 |
  | B3 coulomb feed-forward | 1.66 | 1.27 / 1.72 / 4.77 / 6.55 |
  | **B4 lugre (dt-scaled, abs threshold, no offsets)** | **0.98** | 2.85 / 1.99 / 5.20 / 22.8 |

  Coulomb feed-forward adds friction torque in the direction of the impedance torque, which under an external
  push is the restoring direction → 2.5× stiffer than without compensation (Tz): bad for contact, dropped.
  LuGre is best on accuracy and closest to frictionless compliance. Integrator changes only help accuracy.
- Batch 2 (+ robustness: scene friction scaled, controller unchanged):

  | config | worst step | Fx / Fy / Fz / Tz |
  |---|---|---|
  | B5 lugre + z integral | 0.79 | 2.88 / 1.99 / 3.33 / 23.1 |
  | B6 lugre, upstream threshold | 1.47 | 2.69 / 2.00 / 5.67 / 35.8 |
  | friction ×0.7: B0 / lugre | 3.89 / 1.07 | lugre Tz ≈ 1 mrad (nearly rigid) |
  | friction ×1.3: B0 / lugre | 7.17 / 1.10 | lugre Tz 17.0 |

  The threshold fix matters. LuGre's hidden joint integrator makes the compliance depend on friction level
  and on motion history: Tz under 1 Nm = 1 / 25 / 18 mrad at friction ×0.7 / 1.0 / 1.3, and the same B5
  config gave 23.1 and 32.7 mrad in two runs with different preceding tests.
- Batch 3 (`velocity` mode = Coulomb·tanh(q̇/0.02) + β·q̇, stateless; circle added):

  | config | worst step | Fx / Fy / Fz / Tz | circle RMS vs filtered ref |
  |---|---|---|---|
  | B0 baseline | 5.52 | 1.62 / 2.68 / 6.24 / 16.0 | 10.7 mm |
  | B5 lugre + z integral | 0.79 | 2.92 / 2.39 / 3.66 / 32.7 | 8.8 |
  | V1 velocity | 3.10 | 3.57 / 3.70 / 9.79 / 14.3 | 7.3 |
  | V1 velocity, friction ×0.7 | 1.63 | 4.30 / 3.06 / 12.5 / 31.0 | 5.6 |
  | V2 velocity + z integral + limits 8 N | 1.56 | 3.24 / 3.47 / 4.61 / 14.3 | 7.3 |

  (frictionless: 0.57 mm, 3.0 / 3.0 / 10.0 / 22.1, circle 6.8.) Velocity compensation restores the
  translational compliance and removes almost all friction lag while moving; it does nothing at rest
  (stiction), which the explicit integral handles (V2). Over-compensation at friction ×0.7 makes the arm
  softer but stays stable.
- **Recommendation: V2** (predictable impedance for the contact MPC; ~0.8 mm less static accuracy than
  LuGre); the MPC can re-plan setpoints for the residual and switch the integral off in contact
  (`integrator_limits`). Pending user decision; Isaac config unchanged. Controller commit e.g. `git log`
  on `isaac-sim-pinocchio`; all new params default to the previous behavior.

### 2026-10-22 (evening): identification package, impedance test suite, controller audit
- **Published friction values compared** (details + sources in `docs/SYSID.md`): Franka publishes none
  (URDF `friction 0.2` for all joints = placeholder); upstream identification 0.42–1.29 Nm; Menagerie FR3
  frictionloss 1.14/0.76/0.44/0.25 Nm, armature 0.195/0.074 (3× below `motor_inertia·gear²` = 0.606/0.206,
  unresolved); Gaz 2019 Panda φ1/2 0.15–0.64 Nm. Plan: identify on the real FR3 through FCI.
- **New package `ci_mpc/ci_mpc_sysid`**: `JointIdController` (generic interfaces, runs in sim and on
  franka_hardware; sweep / breakaway / sine per joint, safety limits, log topic), `run_joint_id.py`,
  `fit_joint_id.py`. Joint parameters now live in `ci_mpc_sysid/config/fr3_joint_params_sim.yaml`, which
  `prepare_scene.py` reads (`--params`, `--no-friction`).
- **Validation, joint 7 in sim** (truth = scene values): static 0.550 (0.534, +3.0 %), Coulomb 0.537 (+0.5 %),
  viscous 0.436 (−3.0 %), armature 0.210 (+2.1 %). Static needed onset back-dating (+13 % with the
  controller's stop-detection torque).
- **Full 7-joint validation in sim:** Coulomb 0.0–0.4 %, viscous 0.1–2.0 %, armature 2.1–3.7 %, static 1.4–4.7 %
  (`ci_mpc_sysid/results/sim_validation_2026-10-22.yaml`). Fix that got there: select sweep samples by measured
  |ddq| < 0.05 rad/s² (time-based selection: viscous 10–23 % off on j1–4; joint friction+armature regression:
  up to 40 % off, dropped).
- First full 7-joint run lost at joint 5: the sim script's 600 s wall timeout ended Isaac, the recorder only
  saved at the end. Fixed (timeout 7200 s, recorder saves on interruption). Also: editing
  `run_impedance_test.sh` while a run executes it corrupts that run (bash reads incrementally).
- **Controller audit (sim usability):** works: impedance law, integrator, clamps, rate limit,
  `reference_pose`, `control_mode` 0/2/3. Does not work in sim: mode-1 automatic contact switch (needs
  Franka's `o_f_ext_hat_k` from `franka_robot_state_broadcaster`, nothing publishes it), friction
  compensation (commented out upstream), gripper (not driven). Force clamps ±18 N / ±9 Nm / ±2 Nm (tool z)
  → the controller is a spring only for ~7 mm at K 2500. z has K 1000 and **no integral term**.
- **Impedance suite (`scripts/impedance_suite.py`), with identified friction (GUI run):**
  steps 5 cm: final err after 4 s 3.0–3.9 mm (x/y), 4.4–5.5 mm (z), all undershoot; rotations 0.3 rad:
  0.006–0.021 rad; circle r 5 cm / 5 s: 36.5 mm RMS vs commanded (input filter alone 26.6 mm),
  **11 mm RMS vs the filtered reference** (damping acts on absolute velocity, not velocity error → lag
  ≈ D·v/K); pushes 10 N: effective stiffness 6180 / 3730 / 1600 N/m vs K 2500 / 2500 / 1000 (stiction
  carries part of the load), Tz 1 Nm: 63 vs 45 Nm/rad; mode 2 3 N push: 0.57 mm vs 3.3 mm predicted.
  The pass criteria assumed a frictionless arm.
- **Friction ablation (same suite, scene without friction):** steps 0.05–0.57 mm, rotations 0.007–0.011 rad,
  Fx/Fy 10 N → 3.00/2.99 mm (predicted (10−2.5)/2500 = 3.00), Fz → 10.02 mm (10/1000), Tz 1 Nm → 22.05 mrad,
  mode 2 → 2.86 mm (3.33 predicted), circle 6.8 mm RMS vs filtered reference. **The controller matches its
  equations exactly; friction causes the 3–5 mm steady errors and the higher apparent stiffness.**
  Rotational stiffness is K/2 per rad (error = quaternion vector part ≈ θ/2): predicted Tz deflection
  (1 − 0.5)/(45/2) = 22.2 mrad vs 22.05 measured. The circle lag without friction (6.8 mm) comes from damping
  on absolute velocity (task 3: damp the velocity error).
- Consequence for the real robot: precision is limited by friction handling (friction compensation, z has no
  integral term, integral clamps ±2.5 N ≈ 1 mm at K 2500), so the real-robot identification matters.
- Isaac GUI works (`GUI=1`); first GUI start 89 s (shader compile). RTF GUI 0.51 vs headless 0.55 (GPU free):
  rendering every physics step, not the window, limits speed.

### 2026-10-22 (later): identified joint friction in the sim
- User decision: use the **identified** friction (controller header `static_friction`, `coulomb_friction`,
  `beta`; upstream author's FR3), not Franka's URDF `<dynamics>` (`friction 0.2` for every joint = placeholder,
  units undocumented). Earlier claim "~1 Nm on joints 5 and 7" was wrong: identified j5 1.29, j7 0.53 Nm.
- `prepare_scene.py` applies `PhysxJointAxisAPI:angular`: staticFrictionEffort = static, dynamicFrictionEffort =
  Coulomb, viscousFrictionCoefficient = beta·π/180 (PhysX unit Nm·s/deg). Not representable, left out:
  `offset_friction` (asymmetric bias, j2 −0.70 Nm) and the LuGre states. `--no-friction` removes it.
  Scene backup before friction: `fr3_scene.usd.bak-armature`.
- Open-loop check, joint 7 at home: 0.30 / 0.45 Nm → no motion, 0.60 Nm → 0.144 rad/s after 0.5 s
  (hand calc with armature 0.206 kg m²: 0.157 minus viscous) → friction active and in the right units.
- Test (no friction compensation in the controller): hold 0.00 mm / 0.0000 rad (stiction), reach final
  **2.2 mm** / 0.0035 rad (was 0.47 mm frictionless), <1 cm in 1.58 s (was 1.47). The error creeps
  1.8 → 2.2 mm from t = 5 to 10 s instead of converging: the integral term does not beat stiction here.

### 2026-10-22 (later): task 4 passes, the fix was FR3 joint armature in the sim
- Ablation (single runs; hold rot drift / reach rot err / reach pos err):

  | scene | nullspace damping | hold rot | reach rot | reach pos | result |
  |---|---|---|---|---|---|
  | no armature | 0 | 0.72 rad | 0.70 rad | 1.1 mm | FAIL |
  | no armature | 0.3 | 0.50 | 0.33 | 1.9 | FAIL |
  | no armature | 1.0 | 0.21 | 0.38 | 2.3 | FAIL |
  | no armature | 3.0 | 1.94 | 0.015 | 1.4 | hold FAIL |
  | **armature** | **0** | **0.0003** | **0.0000** | **0.47** | **PASS** (3 of 3 runs) |
  | armature | 1.0 | 0.0002 | 0.0000 | 0.47 | PASS |

- **Mechanism:** joint 7 velocity reversed sign on 95–99 % of 240 Hz steps (period-2 chatter): the
  controller's rotational damping (`D(5,5) ≥ 2√K`) is explicit, and the bare hand's inertia about joint 7
  is a few 1e-3 kg m², so D·dt/I ≫ 2 → unstable; the ±2 Nm tool-z clamp turned that into a slow 0.2 Hz
  wander. The real FR3 has rotor inertia reflected through the gearbox. Armature = `motor_inertia ·
  gear_ratio²` from the URDF `<dynamics>` tags: 0.606 (j1–2), 0.462 (j3–4), 0.206 kg m² (j5–7). After:
  sign-flip rate 6 %, |q̇7| median 0.002 rad/s.
- Nullspace damping only reduced the elbow self-motion (j1 swing 2.05 → 0.28 rad) and did not fix the
  wrist; with armature it adds nothing measurable. Kept as a parameter (default 0), Isaac config = 0.
- Checked: sim kinematics = Pinocchio FK (0.02–0.04 mm, 0.000 rad at 5 random q), so no axis/frame bug.

### 2026-10-22 (later): controller runs in Isaac; position tracks, orientation drifts
- **Our plugin loads and runs in Isaac's bundled controller_manager** (no ABI problem). joint_state_broadcaster
  + cartesian_impedance_controller active, /joint_states at 4.2 ms (240 Hz), EE at home = (0.3069, 0, 0.4869).
- Three Isaac-side fixes were needed to get there (all in `scripts/isaac/`):
  1. **Controller Manager target must be `/fr3`, not the articulation root `fr3_link0`.** The URDF synthesizer
     (`isaacsim.ros2.control/urdf_synth.py`) collects joints under the target prim; our joints live in
     `/fr3/Physics`, so with `fr3_link0` the URDF had no arm joints → "None of requested interfaces exist".
  2. With `fr3_link0` as target the first `setup_cm` always failed ("did not match any articulations") and a
     retry succeeded; the ROS2ControlManager graph node latches the first failure until Stop→Play. We now call
     `Ros2ControlManager.setup()` directly with retries (`setup_controller_manager()`), graph = /clock only.
     With `/fr3` as target it succeeded on attempt 1.
  3. Play first, set home pose, then start the Controller Manager. `app.close()` can hang on RTX sync in
     headless mode → teardown + `os._exit`.
- **Result (runs 6, 7), nullspace damping 0:** reach position **0.2–0.5 mm** final error, <1 cm in 1.47 s.
  Orientation **fails**: hold rot drift 0.60 rad, reach rot err 0.17–1.12 rad.
- **Mechanism:** joint 7 wanders 0.9 rad (hold) / 2.7 rad (reach) at ~0.2 Hz, rotation error is purely about
  tool z; joints 1–3, 5 swing 1–2 rad during a 15 cm reach = undamped nullspace self-motion. The sim joints
  have zero damping/friction, the controller's nullspace term was commented out, and the tool-z torque is
  clamped to ±2 Nm, so it cannot hold the wrist once the self-motion couples in.
- Added controller param `nullspace_damping` (Nms/rad, default 0 = unchanged real-robot behavior),
  `(I − Jᵀ Jᵀ⁺)(−d·q̇)`; Isaac config sets it. Sweep 0 / 0.3 / 1 / 3 running.
- Also: zero-torque arm drifts before activation (q7 0.785 → 0.675 between home and activation).
- Tools: `check_impedance_tracking.py --dump x.npz`, `CONTROLLER_CONFIG=... run_impedance_test.sh`.

### 2026-10-22 (later): first full Isaac test run, scene had no physics
- Run 1 failed before the controller loaded: `ROS2ControlManager setup failed ... no ArticulationRootAPI
  prim under '/fr3/Geometry/base/fr3_link0'`, then `set_home_pose` crashed (prim path not found).
- **Cause:** `fr3_scene.usd` references `fr3/payloads/base.usda` and carries the `Physics` variant set
  (mujoco / none / physics / physx), but had **no variant selected**. So no physics payload was composed:
  no articulation root, no joints, no rigid bodies (`fr3_link0` was only an `over`).
- **Fix:** selected `Physics = physx` in the scene (backup: `fr3_scene.usd.bak-20261022`). Checked with pxr:
  articulation root at `/fr3/Geometry/base/fr3_link0`, joints 1–7 revolute, drive type `force` with
  stiffness/damping 0 (effort mode), maxForce 87, robot gravity off.
- `run_impedance_test.sh`: the checker now gets stopped if Isaac exits early (run 1 left it waiting forever).
- Isaac headless startup took ~500 s with the GPU shared with the YOLO jobs (94 % util, 5.9 GB free).
  The YOLO jobs stay running (user decision).

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
