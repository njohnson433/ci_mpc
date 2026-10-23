# Progress log

Running log of what has been done, what was found, and where to resume. Newest entry on top.
Read this before starting work (and before re-investigating something "new").

## Where to resume

- **Perception (2026-10-23 night):** simulated Gemini 305 wrist camera + box detector; stacking from camera
  detections works (`scripts/demo_stack.sh --camera`). Next: plug in the real pipeline (YOLO) on the same topics,
  measure the real mount (hand-eye calibration).
- **Stage 2 (2026-10-23 evening):** planar pushing with a sticking contact works in sim
  (`scripts/demo_push.sh [line|arc|yaw10]`):
  - straight 20 cm: 2.2–2.3 mm final
  - 90° arc: 0.9–1.0 mm final, 8 mm RMS
  - box rotated 10° at the start: 4–4.8 mm, yaw corrected to < 1°
  - contact sliding on arcs (9 mm along the face) is the open point and motivates stage 3. Also still open:
    external wrench estimate (roadmap 4b).
- **Earlier:** stage 1 is finished in sim (everything that can be done without the robot): 6-DOF MPC with
  force-aware limits, 9/9 tests at nominal friction, stable at friction ×0.7 / ×1.3. Next: real robot (task 6:
  bring-up, identification, the same tests with `config:=real`, friction_scale 0.5), or stage 2 (contact) in sim.
  Pick-and-place demo works in sim (`PICK_PLACE=1 ... pick_place_demo.py`, placed 0.6 mm from the target). Identification setup (`ci_mpc_sysid`, `docs/SYSID.md`)
  is validated in sim and ready for the real FR3 once task 6 bring-up is done. Task 4 (controller in Isaac)
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

### 2026-10-23 (night): simulated Gemini 305 wrist camera, stacking from camera detections
- **Camera** (`scripts/isaac/wrist_camera.py`, `CAMERA=1`; config `ci_mpc_perception/config/gemini305_sim.yaml`):
  - generic Isaac camera with the 305's color FOV (94° H, f 298.4 px at 640 × 400) and a 42 × 42 × 23 mm, 68 g
    body (collider) on fr3_hand.
  - Assumed mount: 7 cm in front of the hand center, 3 cm below the flange, tilted 15° toward the fingers.
  - Isaac 6.2 has no 305 asset (Orbbec: Gemini 2 / 335 / 335L / Femto Mega); the 335 is a 50 mm baseline camera
    for 0.26–3 m, the 305 an 18 mm wrist camera for 7–50 cm.
  - Topics follow OrbbecSDK_ROS2: `/camera/color/image_raw`, `/camera/color/camera_info`, and depth aligned to
    color (same render pose = D2C).
- **Depth noise** (`ci_mpc_perception depth_noise`): Isaac depth (`/sim/camera/depth`, exact) → `/camera/depth/image_raw`
  (16UC1 mm):
  - stereo error σ_z = z²σ_d/(f b), b 18 mm, correlated in patches
  - holes at depth edges (2 px), 1 % dropout, range 4–100 cm, 60 ms latency.
- **Box detector** (`ci_mpc_perception box_detector`): HSV color masks (stand-in for YOLO: class → box name) →
  depth back-projection with the camera pose from FK + the mount → top face minAreaRect (center, yaw mod 90°,
  extent), height from the visible sides → Detection3DArray. `stack_demo.py --perception camera` looks from a
  viewing pose (TCP 0.5, 0, 0.45) before each pick and uses the median over 1 s.
- **Bugs on the way:**
  - The default renderer (real-time path tracing + DLSS at 640 × 400, below DLSS's 300 px minimum input) ran out
    of RTX resource descriptors and hung → RaytracedLighting without anti-aliasing for camera runs.
  - The scene had no lights (black image) → dome + distant light.
  - The camera published at 7 Hz: frameSkipCount counts app updates (~60 per sim second, 4 physics steps each),
    not physics steps.
  - The detector dropped every frame: `/joint_states` stamps are ~0.24 s behind the image stamps (Isaac's
    in-process controller manager, same offset as in stage 1) → joint states timed on arrival in sim
    (`joint_stamp: arrival`; `header` on the real robot).
- **Results (stacking, camera perception, headless):**

  | box | detection xy | z | yaw | size (true 40 mm) |
  |---|---|---|---|---|
  | box0 (red) | 3.0 mm | +2.4 | −0.2° | 37.7 / 37.5 / 37.2 |
  | box1 (green, base) | 2.6 | +1.7 | +0.1° | 37.1 / 37.8 / 38.7 |
  | box2 (blue) | 0.7 | +0.5 | +0.1° | 35.1 / 34.9 / 41.0 |

  Tower: box0 0.7 mm on box1, box2 1.3 mm on box0, top 1.7 mm off the base (ground-truth run: 2.1 mm). Total
  21.9 s sim (2 viewing moves). Sizes are 2–5 mm small: the edge holes eat the top face's border. Once box0 is on
  box1, box1's estimate is off by 20 mm (only a 4 mm strip visible). It isn't used (the demo places on the top
  box), but a real pipeline needs occlusion handling. Camera runs: RTF ~0.6 (render + physics).

### 2026-10-23 (evening): box stacking demo (stage-1 MPC)
- `scripts/demo_stack.sh [--noise] [--headless]` → `scripts/stack_demo.py`:
  - three 4 cm cubes (box0 yaw 20°, box1 = base, yaw 10°, box2 yaw −25°); box0 and box2 go onto box1.
  - Every round re-reads the detections and places on the detected top box: bottom 2 mm above its top, hand yaw
    turned so the carried box matches the top box's yaw (mod 90°, hand within ±45° of home), safe height above
    the current stack.
- Scene: `pick_place_scene.add_objects(boxes=...)` (`BOXES='x,y,yaw;...'`). object.json has `objects` (all boxes);
  the detector publishes one detection per box (`id` = name, fixed per-box error).
- Results (sim, identified friction):

  | case | box0 on box1 | box2 on box0 | top vs base | yaw diff | time |
  |---|---|---|---|---|---|
  | exact detections | 1.2 mm | 1.3 mm | 2.1 mm | −0.3 / +1.2° | motion 12.9 s, total 20.1 s |
  | σ 3 mm / 2 mm / 3° per box | 1.4 mm | 3.2 mm | 4.5 mm | +2.0 / +0.4° | same |

  Placement error stays at the 1–3 mm level because each box goes onto the *detected* box below, so errors don't
  add up along the tower beyond the estimate error. Pick-and-place regression unchanged (1.1 mm).

### 2026-10-23 (evening): stage 2, planar pushing with a sticking contact (sim)
- **Task:** pusher-slider. An 8 × 8 × 5 cm, 300 g box on the table (μ table 0.5), pushed by a rod on the hand
  (capsule collider r 6 mm, 7–19 cm below the hand frame, μ pusher 0.3 with combine mode "min";
  `scripts/isaac/push_scene.py`, `PUSH=1`, `PUSH_XY`, `PUSH_YAW`). On the real robot the rod would be a pin held
  in the closed gripper.
- **Model** (`ci_mpc_planner/pusher_slider.py`):
  - quasi-static, ellipsoidal limit surface (c = m_max/f_max), sticking point contact (Lynch 1992):
    V = M(p_y) u; motion cone from μ_p
  - `step()` also simulates sliding along the face and separation, so the offline loop has a "true" contact.
- **MPC** (`PushingMpc`):
  - LTV around the reference slider path; OSQP, dt 0.1 s, N 50 (5 s)
  - constraints: v_n ≥ 2 mm/s while the reference moves; motion cone shrunk to 0.7
  - solve 11–17 ms in Isaac runs
- **Node** (`push_node`):
  - approach (min-jerk) → creep into contact → push (MPC at 20 Hz, commands at 200 Hz) → retreat
  - command: TCP = planned slider pose + R·[−a/2 − r + 0.5 mm, p_y], twist R·u, feedforward = quasi-static
    contact force, integral off in contact
  - p_y and the slider pose are re-measured every solve (from `/perception/detections`, sim detector at 30 Hz).
  - Harness: `scripts/push_test.py`; demo `scripts/demo_push.sh [line|arc|yaw10]`; offline loop
    `scripts/push_offline_sim.py`; unit tests `test/test_pusher_slider.py`.
- **What it took (Isaac, one lever at a time):**

  | change | straight 20 cm: final / RMS | arc 90° r 15 cm: final / RMS | box rotated 10°: final, yaw | why |
  |---|---|---|---|---|
  | first run (c 0.03, q_θ 0.1, N 30) | 1.94 / 1.88 mm | — | — | slider 1 mm behind (detection latency), 1.8 mm overshoot at the stop |
  | + latency comp. 30 ms (forward prediction) | 2.70 / 1.23 | 0.57 / **29.4** | 5.2 mm, **9.7°** | arc: contact slid 10 mm; yaw ignored (q_θ 0.1) |
  | c 0.03 → 0.045 (fitted from the logs) | 5.28 / 2.56 | 117 / 59 | 11.8, 11.6° | right ω gain, but steering at p_y 0 is weak and the arc is outside the cone |
  | **+ q_θ 3, N 50, contact p_y chosen for the curvature** | **2.34 / 1.20** | **0.88 / 8.19** | **4.77, 0.24°** | all pass |
  | repeat | 2.19 / 1.32 | 0.97 / 8.34 | 4.00, 0.78° | repeatable |
  | stiffness xy 2500 → 1000 (500) | 3.58 / 3.07 | 1.52 / 8.01 (6.43 / 7.14) | 35 mm, 22° (fail) | lost: contact still slides, rod tracks worse |

- **Findings:**
  - **c from the data:** fitting the rotation response gives c = 0.041–0.052 (median per run), above the 0.03
    "uniform pressure" guess (≈ 0.38 a). PhysX supports the box mostly at its corners (a/√2 = 0.057).
    μ_p 0.3 agrees best with when the contact slid (88 % of samples).
  - **Contact location is part of the plan:** with a sticking contact at the face center, the slider can only turn
    by pushing sideways inside the cone. An arc of r 15 cm needs v_t/v_n −0.43 at p_y 0 (cone ±0.54, ×0.7
    margin = ±0.38): infeasible. `contact_for_curvature` picks the p_y that centers the nominal input in the cone
    (−13.5 mm for r 15 cm, −8 mm for r 25 cm; left turns contact right of center). Offline arc: 50 → 1.8 mm.
  - **Heading weight:** q_θ 0.1 let the MPC correct lateral error with body slip (gain c²/D = 0.56 at c 0.045)
    at the price of heading. The heading then drifted it back after the horizon (Isaac yaw drift 12°).
    q_θ 3 with a 5 s horizon fixes it.
  - **Remaining:** on the arc the contact still slides 9 mm along the face (−15 → −6.5 mm). The cone itself is
    right; with ~1.5 N normal force the contact holds only ~0.45 N tangentially, and the rod's tracking error is
    ~1 mm, so small tracking or model errors leave the cone. The MPC tolerates it because it re-measures p_y, but
    a sticking-only model cannot plan it. The 10° start has a 14 mm transient: the slider has to drift sideways
    before sticking steering turns it. Both are the stage-3 case (sliding modes in the model).
- Offline (c 0.045, 30 ms latency, 0.3 mm noise): straight 0.6 mm, arc 1.6 mm, 10° start 1.4 mm / 0.8°. Isaac is
  worse mainly in the transients (arc RMS 8 mm), which the quasi-static model does not reproduce.

### 2026-10-23: blended waypoints, perception-driven grasp
- **Blending:** `ci_mpc_planner/path.py` `BlendedPath`: from the current plan pose through via points (corners =
  circular arcs, radius `path_blend` 3 cm, ≤ 40 % of the adjacent segments) to a stop point, timed with
  v ≤ 0.25 m/s, a ≤ 0.35 m/s² (tangential, and v ≤ √(a r) in arcs), v = 0 at the ends; rotation vector linear in arc
  length. mpc_node: `~/path` (geometry_msgs/PoseArray) → task "path", the MPC tracks the timed path with its 1 s
  preview. Unit test: corner within 3 cm, vertical final approach kept, faster than stopping.
- **Perception interface:** `vision_msgs/Detection3DArray` on `/perception/detections` (box center, yaw, size).
  Sim stand-in `scripts/sim_object_detector.py` = ground truth + a fixed per-run estimation error (σ position,
  size, yaw). The demo plans the grasp from the detection: center, hand yaw = object yaw (smallest 90°-multiple
  rotation that fits), opening = extent across the fingers + 2 cm, grasp check ± 8 mm. Object size / yaw
  configurable (`OBJECT_SIZE`, `OBJECT_YAW`).
- Results (sim, identified friction):

  | case | estimate error | grasp | placed | motion / total |
  |---|---|---|---|---|
  | 4 cm cube, stop at every waypoint | 0 | OK | 0.5 mm | 13.7 / 16.0 s |
  | 4 cm cube, **blended** | 0 | OK | 0.9 mm | **7.3 / 9.6 s** |
  | 3×5×4 cm box, yaw 30° | 0 | yaw +30°, opening 70 mm, closed 50.2 mm | 0.2 mm | 7.3 s |
  | same, σ 3 mm / 2 mm / 3° | 2.9 mm, −1.6° | OK (50.6 mm) | 2.5 mm | 7.3 s |
  | same, σ 6 mm / 2 mm / 6° | 4.2 mm, −2.0° | OK (50.4 mm) | 0.6 mm | 7.4 s |
  | cube, σ 3 mm (seed 3) | 9.9 mm, −6.1° | OK (40.0 mm) | **9.1 mm** | 7.4 s |

  The grasp is robust (opening margin, fingertip margin, tolerance); the parallel gripper centers the object
  along the finger axis and re-aligns a box's faces, but the estimate error along the other axis (and the hand yaw
  error on a cube) carries over to the placement → precise placing needs a better estimate along that axis,
  an in-hand re-estimate after the grasp, or placing relative to a detected target.

### 2026-10-23: pick-and-place demo in sim (stage-1 MPC)
- `PICK_PLACE=1` (sim): table (top z 0.2, x 0.5), 4 cm / 100 g cube (friction 1.0) at (0.45, −0.12); place at
  (0.45, +0.12); grasp TCP z 0.225, transport z 0.33 (σ_min 0.21, links ≥ 0.33 m above the table).
- Gripper = sim stand-in: new package `ci_mpc_sim_gripper` (`FingerWidthController`, position interfaces of both
  fingers, `/gripper/width`), config `isaac_pick_place.yaml` (Isaac controller config + gripper). Real robot:
  franka_gripper action server instead.
- Gotcha: isaacsim.ros2.control picks each joint's command kind from the USD drive (stiffness > 0 = position,
  else effort). The finger drives had stiffness 0 → position commands ignored (fingers stayed closed, the hand
  landed on the cube). Setting the drives in USD before Play (2000 N/m, 100, 40 N) fixed it; tensor-API gains
  alone do not change the command kind.
- `scripts/pick_place_demo.py`: starts mpc_node with `config/mpc_sim.yaml`, goal poses on
  `/ci_mpc_planner/goal_pose`, open → above → down → close → lift → above place → down → open → up → home.
- Result: every move within 1.5 mm in 1.2–1.9 s; grasp at 40.0 mm opening, cube lifted 106 mm, placed
  **0.6 mm** from the target, total 16 s sim.

### 2026-10-23: finishing stage 1 in sim
- **Orientation in the MPC:** second QP of the same form on the rotation vector φ (world, R = exp(φ) R_start),
  ω_max 0.6 rad/s, α_max 4 rad/s², box ±1.2 rad; commanded ω = J_l(φ) φ̇ (unit-tested vs finite differences);
  feedforward [F; τ] = Λ(6×6) [a; α]. Offline tuning: q_vel 50 → no overshoot, ~1.3 s to < 1 mrad.
- New tests: rotate, reach + rotate, retarget (~/goal_pose mid-motion), wall (goal outside the box), fast circle
  (2.5 s), figure-8; return-home + health check before each test; `--out` saves joints and commands.
- **First full run, three friction levels: two serious problems.**
  1. Friction ×0.7 with compensation scale 1.0 (over-compensation) → the closed loop went unstable with the
     MPC's faster motions (103 mm overshoot, then divergence). Same mechanism as the frictionless instability.
  2. Wall test drove the arm to x 0.7, where J is nearly singular (σ_min 0.04 vs 0.21 at x 0.4); the arm ran
     21 mm past the plan, the MPC became infeasible (measured state outside the box; 74–464 "primal
     infeasible", solves 20–90 ms, controller timeouts) and the later tests started from a wrecked state.
  Fixes: box x ≤ 0.6 (σ_min 0.137), measured state projected into the feasible set before a reset
  (position clipped, velocity towards a wall ≤ √(2 a d)), OSQP time limit 10 ms, singularity guard
  (σ_min < 0.08 → hold), harness health checks.
- **Friction compensation scale vs true friction** (fixed suite):

  | true friction / scale | reach out / back | circle / fast / figure-8 RMS | rotate / reach+rot |
  |---|---|---|---|
  | 1.0 / 1.0 | 0.28 / 0.79 mm | 1.27 / 1.67 / 1.06 mm | 2.2 / 7.5 mrad |
  | 0.7 / 0.7 | 0.26 / 0.24 | 0.79 / 2.22 / 0.87 | 1.1 / 3.0 |
  | 0.7 / 0.5 | 0.13 / 0.19 | 1.90 / 2.58 / 1.81 | 4.0 / 7.9 |
  | 1.0 / 0.7 | 0.53 / 0.28 | 2.53 / 3.78 / 2.22 | 5.3 / 11.4 |

  Over-compensation is unstable, under-compensation is stable and costs ~1–2 mm of tracking →
  **real robot: start with friction_scale 0.5.**
- **Rotational integral faster (50 → 300 / 1000, z clamp 0.5 → 1.5 Nm): lost.** Overshoots then sticks past the
  target (stick-slip): final 5.1 / 6.7 mrad vs 2.2 / 7.5 before. Rotation precision is a stiction deadband:
  joint 7 static 0.53 Nm vs rotational stiffness K/2 = 22.5 Nm/rad about z (and the old z clamp 0.5 Nm < 0.53).
- **Wall overshoot (14–22 mm) = force limit.** The plan braked at 2 m/s²; the arm managed ~1 m/s². Λ_x is
  20–30 kg with armature, so 2 m/s² needs 47–80 N, but the controller clamps at 18 N. The MPC's acceleration
  limits are now computed per solve from Λ: a_lim = force_limit / max row-sum |Λ_tt| (12 N), rotation with
  torque limits 6 / 6 / 1.5 Nm → a_lim 0.48–0.51 m/s², α_lim 4.3–4.9 rad/s². Wall: +21 mm → +1.0 mm past the
  box; reach to < 1 cm 0.75 → 0.91 s.
- **Rotational stiffness via MpcCommand** (the deadband scales with friction/K): rotate / reach+rot / retarget
  rotation error: default 2.2–4.0 / 6.5–7.5 / 4.1–7.1 mrad; ×2 0.8–3.1 / 4.0 / 5.9; ×3 2.5 / 1.2 / 5.0;
  ×4 2.2 / 1.8 / 2.5. Large run-to-run spread (stick-slip history); above ×2 no clear gain and more contact
  stiffness → **×2 in the sim config** (`stiffness: [2500, 2500, 1000, 260, 260, 90]`), real config: unset.
- Launch file `ros2 launch ci_mpc_planner mpc.launch.py config:=sim|real task:=... goal:=[...]`;
  `config/mpc_sim.yaml`, `config/mpc_real.yaml` (real: v 0.1 m/s, a 0.5 m/s², small box, σ stop 0.1, untested).
- Test criteria adjusted to the physics: rotation < 5 mrad (stiction deadband), wall = never > 1.5 mm past
  (stopping up to 2 mm short is stiction).
- **Final stage-1 result (sim, final config):**

  | test | friction 1.0 | 0.7 (comp. scale 0.5) | 1.3 (under-compensated) |
  |---|---|---|---|
  | reach 15 cm out / back | 0.25 / 0.42 mm | 0.16 / 0.14 | 0.10 / 0.31 |
  | overshoot, t < 1 cm | ≤ 0.5 mm, 0.9 s | 0, 0.9 s | ≤ 0.34 mm, 0.93 s |
  | circle 5 cm / 5 s | 1.04 mm RMS | 1.30 | 2.14 |
  | fast circle 2.5 s / figure-8 | 2.17 / 0.89 | 2.79 / 2.12 | 3.74 / 2.72 |
  | 10 N push, back to | 0.22 mm | 0.23 | 0.53 |
  | rotate / reach+rotate | 3.1 / 4.0 mrad | 1.9 / 6.2 | 5.2 / 7.8 |
  | retarget | 0.52 mm | 0.06 | 0.56 |
  | wall, past the box | 1.3 mm | 0.3 | 0 |
  | pass | 9 / 9 | 8 / 9 | 6 / 9 |

  Stable in all three; positions sub-mm; the failures are rotation (wrist stiction deadband) and the fast circle
  at friction ×1.3.

### 2026-10-23: task 5, stage-1 MPC
- New package `ci_mpc/ci_mpc_planner` (ament_python, OSQP 1.1.3 via `pip install --user osqp`):
  `TranslationalMpc` = double integrator on the EE position, input acceleration, dt 0.02 s, horizon 50 (1 s),
  |v| ≤ 0.3 m/s, |a| ≤ 2 m/s², workspace box; constant QP matrices (only x0 / reference / a_prev change),
  solve 1.5 ms mean, 4.3 ms max. `mpc_node`: solves at 50 Hz, streams MpcCommand at 200 Hz (plan
  interpolated: pose, twist, wrench = Λ_t(q)·a), tasks hold / reach / circle, sim time. Offline unit tests
  (`test/test_translational_mpc.py`). Test harness `scripts/mpc_stage1_test.py` (reach out/back 15 cm,
  circle r 5 cm / 5 s, 10 N disturbance while holding).
- **First design lost: replanning from the measured state.** The plan's first point is where the arm already
  is, so the impedance spring never builds up: the controller's position feedback is switched off and only
  the feedforward Λ·a drives the arm. Reach ended 10–15 mm short, 44 mm off the line, peak 0.14 m/s; the
  circle never moved (feedforward ~0.2 N < stiction). Fix: replan from the previous plan's state, reset to the
  measured state only beyond 2 cm.

  | variant | reach out final / t<1 mm | back final | peak speed (v_max 0.3) | off-line / overshoot |
  |---|---|---|---|---|
  | measured, no armature | 10.0 mm / never | 15.2 mm | 0.14 m/s | 44 / 3.8 mm |
  | measured + MPC armature | 9.6 / never | 13.6 | 0.25 | 32 / 4.8 |
  | **plan** | **0.29 / 0.61 s** | **0.50** | 0.38 | 9.5 / 5.8 |
  | plan + MPC armature | 0.57 / 1.46 s | 0.86 | 0.36 | 9.3 / 4.8 |

- "Exceeds the speed limit / overshoot" was mostly **the plan itself**: v_max is per axis, so a diagonal move
  plans 0.367 m/s total (each axis ≤ 0.245), and with q_vel 10 the plan overshoots 3.9 mm. The arm matched it
  (0.36–0.37 m/s, 3.6–4.6 mm). Hypothesis "controller underdamped (no armature in its model)" **lost**: added
  an optional `armature` to the controller's PinocchioModel, no measurable change (kept as param, not set).
- Cost sweep (offline, 15 cm diagonal / 5 cm x): terminal weight irrelevant; q_vel 10 → 3.9 mm overshoot,
  0.68 s to < 1 mm; 30 → 0.7 mm, 0.88 s; **50 → 0, 1.2 s (new default)**; 70 → 0, 1.5 s.
- **Result (plan replanning, MPC armature, q_vel 50; sim with identified friction):**

  | test | result |
  |---|---|
  | reach out / back (15 cm), final error | 0.37 / 0.30 mm |
  | overshoot | 0.94 / 0.91 mm |
  | time to < 1 cm / < 1 mm | 0.73 / 1.40–1.98 s |
  | circle r 5 cm / 5 s, RMS vs timed reference | 1.03 mm (shape 0.60 mm); direct MpcCommand streaming was 1.29 mm |
  | 10 N push 2 s while holding | 3.3 mm, back to 0.21 mm |
  | solve time | 0.9–1.5 ms mean, < 4.3 ms max, all "solved" |

- Measurement artifact: arm vs commanded plan looked like 16 mm RMS on the circle; with the command stamps
  shifted by −250 ms it is 0.99 mm. In sim the MPC's clock (/clock) and the /joint_states stamps differ by a
  constant 0.25 s. No effect on control (MPC uses one clock, the controller no stamps); matters for latency
  measurements later.
- Test bugs fixed: float parameters for the circle, fail fast if mpc_node exits, ROS_DOMAIN_ID must be ≤ 101
  (112 did not discover the Controller Manager: DDS ports collide with the ephemeral range).
- Test bug: `circle_laps:=2` (int) vs a float parameter → node did not start; the circle results above
  (70.7 mm, radial 0.00 = arm never moved) are invalid for the plan variants. Harness now fails fast.

### 2026-10-23: task 3, MPC interface in the controller
- New package `ci_mpc/ci_mpc_msgs` with `MpcCommand` (header, pose, twist, wrench, stiffness[6], damping[6],
  integral_enabled). Controller (`isaac-sim-pinocchio`): `/cartesian_impedance_controller/mpc_command` via a
  realtime buffer; newest command sets pose directly (no 0.5 s filter), desired twist (damping on the velocity
  error), feedforward wrench, optional K/D, integral gating (decays at `mpc_integral_off_leak` 5/s). No command
  for `mpc_timeout` 50 ms (counted in control cycles, because the controller node's clock is wall time while
  the CM runs on sim time) → hold current pose, drop twist/feedforward, reset the integral.
- `scripts/mpc_tracking_test.py` (circle 3 ways, timeout, feedforward, commanded K, integral gating):

  | test | no friction (comp. off) | identified friction (Isaac config) |
  |---|---|---|
  | circle r 5 cm / 5 s: reference_pose / MpcCommand pose / pose + twist (RMS) | 33.0 / 7.6 / **0.65 mm** | 33.3 / 7.9 / **1.29 mm** |
  | timeout drift over 2 s | 0.037 mm | 0.006 mm |
  | feedforward −10 N vs 10 N push | 4.00 → −0.01 mm | 3.16 → 0.23 mm |
  | commanded K 500, 5 N (10 mm expected) | 9.97 mm | 2.83 mm (stiction) |
  | integral on / off, 6 N (spring 2.40 mm) | 0.01 / 2.39 mm | 0.75 / 1.39 mm |

- Bug found by the test: at timeout the integral built for the old setpoint pushed the arm 1.85 mm away from
  the new hold pose → reset on hold (0.006 mm after).
- **Safety finding:** with velocity friction compensation on and no friction in the scene the arm went unstable
  (hundreds of mm): compensating friction that is not there = negative damping. Stable at friction ×0.7.
  → real robot: start with `friction_scale` < 1.

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
- V2 is **not** good enough (user): the trace shows it gets **stuck** at ~1 mm by 2.5 s (no joint moving):
  the stiction deadband, K·e = 2500 N/m × 1 mm = 2.5 N cannot break 0.4–1.3 Nm per joint, and the integral at
  350 builds only 0.35 N/s at 1 mm.
- Batch 4, integral speed (velocity comp., 8 s step holds; worst final error, behavior):

  | integral weight (limit) | worst at 8 s | behavior |
  |---|---|---|
  | 350 (8 N) | 3.30 mm | stuck ~1 mm, then a 2.3 mm slip at 7–8 s (wind-up breaks stiction, overshoots) |
  | 1000 (8 N) | 0.85 | converges 0.2–0.7 mm, up to 9 stick-slip bursts per step |
  | **3000 (8 N)** | **0.64** (most 0.11–0.41) | < 1 mm in ~2–3 s, 1–6 small slips per step |
  | 3000 (15 N) | 0.67 | similar, occasional jumps up to 2 mm |

  Sub-mm is reachable with friction, but the last fraction comes as stick-slip jumps (friction limit cycle,
  size depends on PhysX's stiction model). A fast integral = very high low-frequency stiffness (rigid for
  slow loads up to the clamp), so accuracy at rest and compliance at rest cannot come from one fixed set of
  gains → **gate the integral**: on in free space (velocity comp. + weight ~3000), off in contact
  (`integrator_limits` 0, velocity comp. stays). The MPC knows the phase → part of the task 3 interface.
- **Windup** (user question): yes. Cases: large moves saturate the integral (masked by the reset on every new
  reference_pose, which also means streamed MPC references never build an integral), blocked motion/contact
  winds it to the clamp → jump on release, stick-slip = small-scale windup. Added conditional integration
  params (`integrator_windows`, `integrator_velocity_window`, `integrator_freeze_on_saturation`,
  `integrator_leak`, `integrator_reset_on_target`; defaults = old behavior) and a suite `windup` test
  (hold 15 N 3 s, release). Batch 5 (velocity comp., integral 3000 / 8 N, steps held 8 s):

  | config | worst step | windup jump x / z | pushes Fx/Fy/Fz/Tz | circle |
  |---|---|---|---|---|
  | B0 original | 5.39 mm | 0 / 0 | 1.62 / 2.68 / 6.24 / 15.9 | 10.7 |
  | W0 unconditional | 0.89 | **2.90 / 7.01** | 1.20 / 1.14 / 2.71 / 19.6 | 7.1 |
  | W1 conditional (3 mm, 1 cm/s, freeze, leak 2/s) | 0.55 | 0.33 / 0.00 | 3.92 / 3.22 / 11.5 / 17.2 | 7.3 |
  | **W2 = W1 + no reset on target** | **0.38** | **0.09 / 0.00** | 3.92 / 3.96 / 11.3 / 19.5 | 7.3 |

  W2 costs: t90 1.05 → 1.3 s, step overshoot 1.3–3.1 %, settling still by stick-slip (4–5 bursts per step,
  one 1.9 mm transient); small contacts (< 3 mm deflection) still stiffened → gating in contact still useful.
- **Isaac config now W2** (`isaac_sim_controllers.yaml`); real-robot config unchanged. Regression hold/reach:
  hold 0.00 mm, reach **0.11 mm** (was 2.2 mm), rot err 0.0082 rad (was 0.0035).

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
