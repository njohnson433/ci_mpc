# Progress log

Running log of what has been done, what was found, and where to resume. Newest entry on top.
Read this before starting work (and before re-investigating something "new").

## Where to resume

- **Peg-in-hole (2026-10-23 night):** Factory 8 mm peg in sim (`scripts/demo_peg.sh --offset dx,dy`): succeeds
  at 0–1 mm hole error, fails at ≥ 2 mm (jamming, in-hand slip). Proposed next: contact force estimate → CI-MPC
  (planar peg-hole with a compliant grasp) → Isaac.
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

### 2026-10-24 (late morning): AlphaNIST lever 1, success-guided starts for the selector: no gain
- **Setup:**
  - `ci_mpc_planner/start_cache.py`: 3 hole-error magnitude bins (0.3 / 0.8 / 1.3 / 1.8 mm, the fair eval's range)
    × 8 directions, weight (1 − Beta(1,1) posterior success) + 0.1.
  - Snapshots every 10 steps of earlier rollouts form a second pool (p_snap 0.3, the hardest 400 kept).
  - `selector_dagger.py --cache` / `--no-label`, `scripts/start_cache_update.py`, `scripts/sgs_round.sh`. Data in
    `~/ci_mpc_runs/selector/` (the earlier DAgger data was copied there from /tmp).
- **Only lever changed: the start distribution.** Same budget and settings as the old round 1: 16 × 5 episodes,
  β 0.5, rolled out with `sel_r0`, trained on r0 + this round, tau 0.3.
- **Pre-pass** (`sel_r0` alone, 96 episodes): success 0.75 / 0.69 / 0.48 by magnitude bin; 341 snapshots, mean
  success 0.55. The round drew 51 bin and 29 snapshot starts.
- **Baseline reproduces exactly:** `sel_r1` re-evaluates at 19/30, 2.41 / 4.18 N.
- **Result (fair 30 starts):**

  | training data | entered by training seed 0 / 1 / 2 / 3 | mean |
  |---|---|---|
  | r0 + r1 (uniform starts) | 19 / 19 / 18 / 19 | **18.75** |
  | r0 + SGS round | 16 / 19 / 14 / 20 | 17.25 |

  - Seed 0 also got a worse force tail: p90 6.92 N vs 4.18 N.
  - Validation regret across the 4 seeds: 0.16–0.54 with SGS data vs 0.32–0.37 for r0 + r1.
  - Neither beats the heuristic (22/30).
- **Mechanism (likely):** the starts are not the bottleneck; the labels are. The dense 4-step score is
  near-tied or misleading in hard states (jams, edge contact), so more such states add label noise, not signal.
  Training gets less stable (regret spread), and the mean does not move.
- **Binary-label check on 160 of these states** (each option, then the heuristic or `sel_r0` for 20 steps;
  entered = tip 2 mm in; force gate 6 N):
  - Options disagree in only 29–33 % of states; in 19–27 % no option enters.
  - Where any option enters, the dense-score best already enters 93–95 % of the time (the heuristic option
    65–75 %).
  - So binary labels mainly widen the target to "any of the ~8 entering options". That may be easier to learn
    than the dense argmax (top-1 only 0.3–0.5), but it cannot add much ranking information. Full test running
    (dense vs binary on the same 3249 states, 3 seeds each).

### 2026-10-24: AlphaNIST (NVIDIA, arXiv 2610.13354) checked, not adopted
- Sim-only RL from scratch, binary reward, one policy for all 20 NIST board tasks (98.2 % train success), up to
  256k envs (4k per GPU), point-cloud observations, joint-position targets at 25 Hz. No code or weights, no
  sim-to-real, no training time given.
- Not usable directly: the compute is ~64 GPUs (we have one shared RTX 4080 and 31 GB RAM), it skips our
  MPC → impedance stack, and it replaces CI-MPC instead of helping it.
- Ideas worth taking: success-guided sampling of start states (amortized-C3 / DAgger starts), and the finding
  that dense shaping rewards scored lower than binary ones.
- A cheaper RL baseline is already installed: Isaac Lab `contrib/factory` and `contrib/forge` (peg insert,
  gear mesh, nut thread; FORGE uses force feedback). Not run yet.

### 2026-10-24 (morning): weekend runs lost to an OOM, restarted; gear-mesh scene (H2)
- **OOM, 2026-10-23 18:52.** Every weekend job died at once, and nothing ran overnight.
  - The cause: 8 Isaac instances (Track A and Track B, 4 parallel each), plus the GPU MPC servers, the offline H4 and
    a browser, on 31 GB of RAM. The kernel OOM-killer then took the user session.
  - Kept: Track A repeat 1 (probe 8/10, spiral 3/10), the H4 mu0.5 lines and 2 bench_gpu runs (both inserted).
  - Fix: `scripts/run_weekend.sh` runs the phases one after another (A reps 2–3 together with H4, then B with
    3 repeats × 10 offsets at 3 parallel, then H1), started with `setsid nohup`.
  - Also, `peg_benchmark.sh` no longer starts an Isaac instance while `MemAvailable` < `MIN_MEM_GB` (7), and
    `repeat_benchmark.sh` takes `START=` to resume.
- **Track A done (D1 Isaac benchmark, 3 repeats × 10 offsets, cores 0–15):**
  - **probe 24/30 (80 %, Wilson 95 % CI 63–90 %)**, spiral **7/30 (23 %, CI 12–41 %)**. The intervals do not
    overlap, so probing is the better baseline beyond noise.
  - Per repeat, probe went 8 / 9 / 7 and spiral 3 / 2 / 2.
  - Probe is 3/3 at every offset ≤ 3 mm except (3, 3); its failures are at 3.6–5 mm. Successes there are slow
    (41–109 s median contact phase vs 3–9 s near 0).
  - Spiral fails even at 0,0 once (2/3) and is 0/3 at four offsets.
  - Table: `~/ci_mpc_runs/weekend/A/summary.md`.
- **H2 gear mesh** (`scripts/isaac/gear_scene.py`, `GEAR=1`, `GEAR_DROP_TEST=1`): Factory gear base plus the
  small and large gears (kinematic); the medium gear lies free on the table. SDF collision is already in the assets.
  - Measured geometry: shaft r 4.75 mm, top at z 25 mm (0.5 mm chamfer); bore r 5.00 mm, so 0.25 mm radial clearance;
    20 mm travel to seated. Teeth: medium 40, small 20, large 60, with no tooth chamfer.
  - **The asset frame is not in mesh at yaw 0:** tooth on tooth, a dropped gear stuck 10 mm above the seat. At
    yaw +4.5° (half a pitch) it meshes, so `MESH_YAW` = 4.5.
  - Drop test at `MESH_YAW`: aligned seats 0.12 mm high; 0.2 mm xy offset seats 0.04 mm; **2° yaw error jams at
    10 mm**.
  - So H2 tests a DOF our peg strategies never use (the 5-D command has no yaw). The yaw tolerance is < 2°, and
    either the grasp yaw must be that good or a yaw search is needed.
  - Demo: `scripts/demo_gear.sh`, the peg demo with `--geo` set to the contact model for the bore on the shaft
    (also passed to the GPU MPC server), `--open-width 0.05` and `--grasp-depth 0.004`.
    - Grasp failures before that: a 30 mm opening landed the fingers on the 30 mm hub; grasping 10 mm below the top
      put the pads on the teeth (closed at 42 mm).
    - Smoke tests at 0 offset (setup only, nothing tuned): spiral 0.15 / 0.11 mm in 6.3 s; probe 0.11 / 0.05 mm;
      probe + GPU entry 0.12 / 0.10 mm (456 ms per decision).
  - `scripts/heldout_h2.sh` is queued; it starts after `run_weekend.sh`'s "all done". Conditions:
    - nominal: 5 random errors × spiral / probe / GPU
    - yaw 1° and 3°: 3 errors each × spiral / probe
- **H3 AutoMate** (`scripts/isaac/automate_scene.py`, `AUTOMATE=ID`, `scripts/demo_automate.sh ID`): all 100
  assemblies downloaded (65 MB, without the 15 MB demonstration trajectories) to `~/ci_mpc_assets/automate`.
  - The grasp comes from AutoMate's `plug_grasps.json` (TCP = hand − 103.4 mm); the travel comes from
    `disassembly_dist.json`.
  - Setup-only criteria: top-down grasp ≤ 40 mm wide, TCP ≥ 10 mm above the socket when seated, travel ≤ 40 mm.
    56/100 pass, and seed 31 drew 12: 00016 00021 00028 00211 00301 00319 00388 00499 00638 00652 00700 01041.
  - Setup fixes on dev assembly 00015 (not in the set):
    - The plug does not stand on its own, so it now starts **seated in a second (source) socket**.
    - The assets carry mass 0, so PhysX used ~3 g and the 20 N fingers sank into the SDF (closed to 5.3 mm on a
      10.2 mm head). Now mass 0.05 kg and contact offset 5 mm, as Isaac Lab sets them.
  - 00015 spiral at 0 offset: inserted, depth 0.03 mm, xy 0.28 mm, 6.8 s.
  - `scripts/heldout_h3.sh` is queued after H2: 12 assemblies × 1 random error (r 1–4 mm) × spiral / probe / GPU.
    Grasp failures are reported separately.
- **Memory, measured:** each Isaac instance takes **~6.2 GB** RSS (not 3–4 GB). 4 instances plus H4's 3 workers leave
  ~3 GB free, so dev runs go through a ≥ 9 GB guard.
- **H4 so far** (offline, 30 fair starts; true plant varied, the MPC keeps its nominal model):

  | condition | heuristic entered | GPU MPC entered | GPU peak p90 |
  |---|---|---|---|
  | mu 0.5 | 30/30 | 30/30 | 2.75 N |
  | mu 1.0 | **10/30** | **30/30** | 6.57 N (heuristic 6.51 N) |
  | R 4.4 (0.4 mm clearance) | **7/30** | **30/30** | 5.14 N |
  | R 4.6 | 28/30 | 30/30 | 3.74 N |
  | stiction ×0.5 | 30/30 | 30/30 | 3.79 N (heuristic 3.01 N) |
  | stiction ×1.5 | **0/30** | **0/30** | – (no contact) |

  - **Stiction ×1.5 is degenerate, but it is a real gap.** The eval's setup press (4 mm × 500 N/m + 3 N = 5.0 N) is
    below the true z stiction of 5.25 N, so the arm never moves and the tip stays 0.5 mm above the hole.
  - Neither policy adapts when commanded motion does not happen: no integrator, no stiction estimate. The MPC's
    nominal model predicts that small commands move the arm (it picks the heuristic candidate on all 60 steps).
  - The Isaac demo has `--stiction-ff 3` for this. Online stiction identification (from command vs measured motion)
    would be the fix. Not tuned here: H4 is held out.

  The GPU MPC holds 30/30 under every plant change tested; the heuristic falls apart with high friction and tight
  clearance. The force gate (p90 < 4 N) is missed under mu 1.0 and R 4.4 by both: more friction and tighter clearance
  need more force to slide in.

### 2026-10-23: daily summary
- **Stage 2, planar pushing (sim):** pusher-slider LTV MPC with a sticking contact: straight 20 cm 2.2–2.3 mm
  final, 90° arc 0.9–1.0 mm final (8 mm RMS), 10° start corrected to < 1° yaw. Open: the contact slides 9 mm
  along the face on arcs (sticking-only model) → stage 3.
- **Stacking and perception:** 3-cube tower from stage-1 MPC, 1.2–3.2 mm per level. New `ci_mpc_perception`
  (simulated Gemini 305 wrist camera, depth noise, box detector): tower top 1.7 mm off the base from camera
  detections. A GUI run failed (brighter faces → 19.6 mm detection error); fixed with depth-grown top faces,
  then GUI and headless agree (1.9 / 2.0 mm).
- **Factory 8 mm peg-in-hole benchmark (sim, SDF):** the hand-written search-and-insert succeeds at 0–1 mm hole
  error, fails at ≥ 2 mm (jamming, in-hand slip). This is the CI-MPC target.
- **External wrench estimate (momentum observer on the measured joint torques):** against a simulated wrist F/T
  sensor, error RMS 0.17 / 0.31 / 0.80 N in contact, up to 7 N briefly at impacts (bandwidth ~5 Hz). The PhysX
  contact report was misleading (2× the weight, no friction).
- **CI-MPC, planar model + C3+:** offline, localization alone fixes 2–5 mm, and C3 adds nothing on top. In Isaac the
  planar model is wrong in 3D (the filter converged to the wrong side), so offsets ≥ 3 mm still failed.
- **3D model + active probing:** exact axisymmetric hole SDF, grid belief, information-driven probes. Offline
  12/12 offsets up to 5 mm; in Isaac 7/10 first, then 4/4 on the failed ones after fixes (force-controlled 40 N
  grasp, grasp-height calibration, wrench tare, tolerances). 3D C3+ gives 2–3× lower contact forces
  (1.1–1.7 N vs 3.6–5.7 N), but only with a ~1.5 s rollout warm start per solve; the fast variants get stuck.
- **Amortized C3, negative result:** with randomized starts the single-C3 teacher entered only 8/80 episodes, and
  the MSE student 1/10. The **sampled teacher** (9 warm starts + heuristic, scored in a plant copy) enters 9/10
  with 1.4–2.35 N peak force (heuristic 8/10, 2.2–5.8 N) at 1–6 s/step. The option selector + DAgger reaches 8/10
  at 2.1–2.5 N and 0.2–0.5 s/step. Fair 30-start test (separate RNGs): heuristic 22/30 (peak median 2.80 N, p90
  4.96 N), selector round 1 19/30 (2.41 / 4.18 N), round 2 16/30 (2.33 / 2.75 N): lower forces, fewer entries;
  DAgger made it worse (validation regret 0.18 → 0.45). Learned amortization does not beat the heuristic yet.
- **Clean Isaac benchmark (pinned to cores 0–15):** probing 6/10, spiral (40 N grasp) 5/10, estimator (planar)
  2/10. Run-to-run variance is large (probing 4/4, 7/10, 6/10 in earlier runs), so repeated runs are planned.

### 2026-10-23 (night): sampled C3 teacher, option selector + DAgger (step 3, second attempt)
- **Sampled teacher** (`sampled_teacher.py`): per step, C3 from 9 warm starts (rollouts of different nominal
  command sequences: centering with a light / firm press, press only, ±x / ±y lateral, ±tilt) plus the heuristic,
  each executed 4 steps in a plant copy and scored (depth, centering, force above 3 N). Privileged (true hole).
  Same 10 random engaged starts:

  | policy | entered | steps | peak force | time / step |
  |---|---|---|---|---|
  | force-following heuristic | 8/10 | 14–58 | 2.2–5.8 N | 0.1 s |
  | single C3 (rollout warm start), data runs | 8/80 episodes | | | 1.5 s |
  | **sampled teacher** | **9/10** | 8–53 | **1.4–2.35 N** | 1–6 s |

  With randomized data (30 % perturbed commands) it entered 70/80.
- **Command regression student (MSE): 1/10 entered.** NumPy export verified identical to the torch model. The
  causes are the teacher's multimodal actions (≈40 % heuristic, otherwise C3 options; z spread 3.5 mm, MSE
  averages them, mostly losing contact), 1775 samples, and no DAgger.
- **Option selector** (`C3Selector`, `train_c3_selector.py`): MLP → logits over the 10 options, soft targets
  softmax(score / 0.3); at runtime only the chosen option runs (heuristic, or one C3 with its warm start, on the
  *estimated* hole). DAgger (`selector_dagger.py`): the teacher labels every visited state with all option scores.
  - round 0 (teacher drives): validation top-1 0.52, score regret 0.18 (always heuristic: 2.18). Closed loop
    8/10, peak 2.1–2.5 N, 0.2–0.5 s/step.
  - round 1 (β 0.5): 8/10, peak 2.2–2.4 N; still stuck at one start (picks the same C3 option 60 times, light
    contact).
  - Evaluations so far shared one RNG for starts and noise, so only the first episode per seed matched the
    heuristic's start → separate RNGs, 30 starts.
- **Infra:**
  - CPU split: teacher / DAgger workers `taskset` 16–31, Isaac + ROS 0–15. The first final benchmark ran next to 16
    unpinned workers; its ROS side lagged (contact phases 27 s instead of 3 s at 0,0).
  - The estimator baseline in that run: 2/10 (planar model, x only: expected on 2D offsets).
  - Clean pinned rerun of probe / spiral running.
  - A wait loop after a session restart used `pgrep -f` on a pattern in its own command line (never ends); stopped
    by PID, replaced by a file check.

### 2026-10-23 (later): CI-MPC step 3, amortized C3 (student) — stopped, teacher not good enough
- **Pipeline (kept, works):**
  - `scripts/c3_teacher_data.py`: teacher = 3D C3+ with the rollout warm start every solve; random engaged starts
    0.3–1.8 mm, estimate error σ 0.2 mm, 30 % perturbed commands (DART); 16 workers, 80 episodes, 4622 samples.
  - `scripts/train_c3_student.py`: PyTorch (Isaac venv; pytorch-patterns: seeds, train/eval modes, set_to_none,
    grad clipping, state-dict checkpoint) → .npz.
  - `ci_mpc_planner/c3_student.py`: NumPy inference, so the controller side needs no PyTorch.
- **Result (negative):**
  - **The teacher is not reliable once the starts are randomized:** it entered the bore (tip < −5 mm) in only 8 of
    80 episodes within 60 steps (median 35 steps). The rest stall on the chamfer at −0.4…−1.4 mm. The earlier "2 of
    2" were two hand-picked starts without perturbation or estimate error.
  - Its actions are noisy (the z command changes 0.47 mm step to step, mean |z| 1.1 mm: ADMM variability).
  - Student: best validation at epoch 40, then overfits; validation R² 0.36 / 0.39 / −0.35 / 0.29 / 0.41 (x, y,
    z, rx, ry); RMSE z 3 mm.
  - Imitating a teacher that mostly stalls cannot beat the heuristic, so it was not evaluated further.
- **What this says about the CI-MPC (steps 2–3):**
  - The 3D C3 yields lower contact forces (2–3×) when its ADMM converges to the right mode sequence.
  - It is too local and warm-start sensitive to be the reliable layer.
  - The reliable parts of the stack are the belief / probing (localization) and the force-limited centering.
  - Next options, if pursued:
    - a global mode-sequence search on top of C3 (MIQP over a few modes per contact, or sampling the warm starts),
      as C3+ does with sampled initial conditions;
    - a C++ C3 (the C3 papers' speed);
    - a student trained on a better teacher (e.g. MIQP or sampled C3, offline, any speed).

### 2026-10-23 (later): CI-MPC plan, steps 1-2: 3D model, probing localization, 3D C3
- **User question: model the hole with a NN for a convex contact problem?** My answer:
  - The linearized contact force problem is already convex (Anitescu QP) whatever the geometry representation;
    the hard parts (mode switching, contact location vs pose over the horizon) remain with a learned SDF, and the
    hole's free space is concave (an ICNN cannot represent it).
  - The Factory hole is axisymmetric: an exact analytic SDF is cheaper and exact.
  - A NN is better used as an amortized estimator or as a C3 warm start (step 3).
- **Step 1, 3D model** (`peg_hole_3d.py`):
  - TCP pose (x, y, z, rx, ry)
  - exact axisymmetric hole SDF (2D profile in (r, z))
  - peg ring points vs the hole, hole rim points vs the peg side
  - friction pyramid, arm stiction box
  - convex plant
- **Key finding:** one contact does not localize the hole. Resting flat on the top face only rules out holes under
  the tip (~2 mm); a chamfer push only exists while the tip overlaps it. Static likelihood maps after one contact
  were flat (160 of 169 cells within 2 log-units) → **active probing**:
  - `hole_belief.py`: grid belief over the hole xy, vectorized SDF measurement model, ~6 ms per update
  - next probe = max P(hole within the capture radius)
  - offline: 12/12 offsets up to 5 mm in all directions, 0–18 misses.
- **Isaac** (`--strategy probe`), the issues on the way, each found from the logs:
  - The probes did not reach their targets (arm lateral stiction ~2 N) → lateral feedforward.
  - Probe positions were excluded as targets (crawl) → exclude measured positions of clear misses, 1.5 mm
    spacing.
  - In-hand slip at 5–6 N: the sim gripper was a spring drive (2000 N/m, 4 mm on the 8 mm peg = 8 N grip; the
    4 cm cube got 40 N) → force-controlled grasp (stiff drive saturating at 20 N per finger = 40 N, like the
    Franka Hand, also in the pick-place scene; REAL_ROBOT.md grasp force 40).
  - Grasp height offset +0.55–0.60 mm: every hypothesis showed a gap, so the belief never moved → calibrated on
    the first flat-top contact.
  - The wrench estimate's lateral offset (~1 N, torque sensor bias) read as chamfer pushes → tare before each
    probe. The lateral error while pressing (~1 N) stays the size of the chamfer push → the belief uses geometry
    only.
  - Penetration tolerance 0.3 mm: a 0.07 mm residual made every probe favour holes next to it.
  - Touch tolerance 0.15 mm: ring points over the chamfer counted as supported, so each probe excluded too
    little.
  - Results: first full benchmark 7/10 (±5, 3 mm failed), after the last two fixes 4/4 on those (5.2–81 s).
    The final 10-offset run is in the next entry.
- **Step 2, 3D C3+** (`c3_3d.py`, engaged phase after probing; offline vs the force-following heuristic):

  | variant | solve | result (3 offsets 1–1.6 mm on the chamfer) | peak contact force (heuristic: 3.6–5.7 N) |
  |---|---|---|---|
  | rollout warm start each solve (contact radius 2 mm, N 6) | ~1 s (40 contacts) | centers, enters | 0.5 N |
  | rollout each solve (radius 0.5 mm, N 4) | 1.5 s (rollout = cost) | centers, enters, 2 of 2 | 1.1–1.7 N |
  | zero warm start | 25 ms | enters | 5.8 N |
  | shifted previous targets | 13–42 ms | 1 of 2 inserted fastest (61 steps), 1 stuck at the opening | 3.6 N |
  | rollout every 4th solve | 38–74 ms | 1 of 3 (2 stuck at the opening, violation 1–5.6) | 1.9 N |

  - **Reading:** with a physics-consistent warm start, C3 centers and inserts with 2–3× lower contact forces than
    the heuristic; that is the CI-MPC benefit. But that variant costs ~1.5 s per solve in Python, and the fast
    variants fall into local minima (stuck at the opening) or lose the force advantage.
  - → step 3: learn the warm start / an amortized policy from the slow reliable C3 (teacher), or port to C++.

### 2026-10-23: CI-MPC step 3, Isaac: localization / C3 on the Factory peg (planar model along x)
- `scripts/peg_insert_demo.py --strategy spiral|estimate|c3`: 10 Hz model-based contact phase. The TCP pose comes
  from FK, the contact wrench from the controller's estimate (sign flipped, peg weight removed), plus the particle
  filter. estimate = localization + straight down; c3 = localization + C3+. Hole offset along world x only (planar
  model). `mpc_timeout` 0.2 s in the pick-place controller config (planning blocks the command stream briefly).
- **Bugs / traps found on the way (each was a wrong conclusion until fixed):**
  - The `model` name clash crashed two runs. Their orphaned `sim_object_detector` / `mpc_node` kept publishing stale
    detections on the same ROS domains, so later runs "saw" a peg above the hole and the grasp failed. Killed by
    PID; all demo scripts now kill their children with `atexit` (also on a crash).
  - Flat-belief estimate = the argmax of uniform weights = the first particle (−5.6 mm). It moved the peg there
    before any contact; that happened to make +3…+5 mm "succeed" and 0–2 mm fail → mean until the belief has a
    peak.
  - The straight-down press was limited relative to the hole bottom → up to 15 N while the peg sat at the
    opening; the peg slid up in the fingers at ~6–8 N (TCP 18 mm "deep", peg not inserted) → press at most 6 mm
    below the current TCP. C3 grasp axial limit 10 → 4 N.
  - 3 N of spring did not move the arm in free space (z stiction > 3 N at K_z 500, integral off) → 3 N
    feedforward along the descent, press stopped above 4 N measured contact force; the estimator gets the
    feedforward as an equivalent set-point offset.
- **Results** (Isaac, hole error along x; ✅ = peg bottom within 1 mm of the hole bottom):

  | strategy | −5 | −3 | −2 | 0 | 1 | 2 | 3 | 4 | 5 mm |
  |---|---|---|---|---|---|---|---|---|---|
  | spiral (hand-written) | | | | ✅/❌ | ✅/❌ | ❌ | ❌ | | |
  | localization + down, before the press / stiction fixes | ❌ | ❌ | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ |
  | + stiction feedforward, force-limited press | | ❌ | | ✅ 19 s | | ✅ 7.9 s | ❌ | | ❌ |
  | + following the rim's lateral push (light normal force) | | ✅ 28 s | | ✅ 7.8 s | | ✅ 7.7 s | ❌ | | ❌ |
  | localization + C3 (arm stiction in the model) | | | | | | | ❌ stays in free space | | |

- **Why ≥ 3 mm still fails (mechanisms):**
  - **The planar model is wrong in 3D.** The round tip resting off-center on the round chamfer ring touches the
    cone at two points; their resultant pushes the peg sideways toward the axis. The planar cross-section sees
    a corner on the flat top face (no lateral force), so the particle filter converged to the wrong side
    (+1.8 mm, true −3).
  - **Friction cone:** pressing at ~6.5 N on the rim locks sideways motion (μ·F_n ≈ 4.9 N > spring +
    feedforward). Sliding needs a light normal force, which is exactly the trade-off a CI-MPC should make.
  - **Signal level:** the rim's lateral push is ~0.3–1.5 N, about the observer's contact error (0.2–0.3 N RMS)
    plus bias. Following it is asymmetric (−3 ✅, +3 ❌).
  - **C3 with arm stiction:** "stick, nothing moves" is a valid complementarity solution, and the ADMM settles
    there (local minimum); without stiction in the model it plans forces that never break it.
- **Next (decision):**
  - a 3D contact model (cylinder tip vs conical chamfer and bore) for the estimator, with 2D hole position
    particles; that is what makes the rim's push informative
  - then C3 on it with a mode-aware initialization (warm-start ADMM in "slide", not "stick").

### 2026-10-23: CI-MPC step 2, planar peg-hole model, C3+ and hole localization (offline)
- **Model** (`ci_mpc_planner/peg_hole.py`):
  - planar (x-z, tilt about y) Factory peg and hole: hole r 4.5 mm with a 1 mm chamfer, peg half width 3.993 mm
    with a 0.5 mm chamfer
  - contacts: 4 peg vertices vs the hole segments, 4 hole rim vertices vs the peg sides
  - quasi-static impedance x+ = u + C W λ (C = controller compliance)
  - Stewart-Trinkle complementarity for the MPC.
- **Offline plant:** Lemke on that LCP failed on most steps (ray termination with the friction rows, even after
  scaling to mm), so the plant uses the convex Anitescu relaxation (dual QP with OSQP: always solvable; sliding
  contacts separate by μ·slip). It reproduces the Isaac failure: at 1–2 mm offset a straight push jams on the
  chamfer (K_x 1500, μ 0.75 on a 45° chamfer), at 3 mm the peg rests on the top face.
- **C3+** (`ci_mpc_planner/c3.py`):
  - ADMM: OSQP QP plus a closed-form projection per complementarity pair, N 8, 12 iterations
  - grasp wrench limits and command steps as convex constraints
  - 20–25 ms per solve
  - ρ0 1 → 10: complementarity violation 3.0 → 0.14.
- **Hole localization** (`ci_mpc_planner/hole_estimator.py`): particle filter over the hole x; each particle predicts
  the step with the contact model, weighted by the measured TCP pose and contact wrench.
- **Results** (offline, true hole at +h, estimate 0; success = tip within 1 mm of the bottom):

  | policy | 1 mm | 2 mm | 3 mm | 4 mm | 5 mm | −3 mm |
  |---|---|---|---|---|---|---|
  | straight down, no localization | ✅ | ❌ jam | ❌ on top | | | |
  | C3, model at the wrong hole, no localization | ❌ 3 mm short | ❌ | ❌ | | | |
  | **straight down + localization** | | ✅ 53 steps | ✅ 52 | | ✅ 53 | |
  | **C3 + localization** | ✅ 63 | ✅ 91 | ✅ 92 | ✅ 126 | ✅ 95 | ✅ 68 |

- **Honest reading:** offline, localization from contact is what fixes it; the CI-MPC adds nothing on top (it is
  slower). The offline plant has a rigid grasp, no joint stiction and no tilt jamming, which are the effects the
  CI-MPC's force and grasp limits target and that broke the Isaac runs. Next: Isaac with three contact strategies
  (spiral / localization + straight down / localization + C3) on the same offsets.

### 2026-10-23: CI-MPC step 1, external wrench estimate in sim
- **Controller** (`cartesian_impedance_control` 153c12b):
  - generalized momentum observer r = K_O (M q̇ − ∫(τ + Cᵀq̇ − g + r)dt − p0), K_O 30
  - wrench at the TCP = −pinv(Jᵀ) r in Franka's sign convention (robot on environment positive); published on
    `/cartesian_impedance_controller/external_wrench` (WrenchStamped) and fed into F_ext (mode 1 now works in sim)
  - Pinocchio model gained Cᵀq̇ and g(q).
- **Torque source matters:**
  - On the *commanded* torque, joint friction (stiction at rest) is indistinguishable from an external force: with
    known wrenches at the TCP (sim hook) the estimate was 2–4.7 N off on the other axes.
  - Isaac's joint effort state is the *link-side transmitted torque*, like the FR3 joint torque sensors: from it
    the known wrenches come out exact (≤ 0.01 N / Nm), and in motion (circle) the error is ≤ 0.06 N.
  - Config: `observer_torque_source: measured` (M without rotor inertia; `observer_gravity` true on the FR3,
    whose τ_J includes gravity). Sensor model added in sim (Isaac's sensors are perfect): noise 0.05 Nm, fixed
    offsets ≤ 0.15 Nm (assumed, to be measured on the robot).
- **Ground truth for contact:**
  - The PhysX contact report on the hole block was misleading. With a free peg resting in a hole it reads 2× the
    weight (0.373 vs 0.186 N; factor unexplained), and it only carries normal impulses (no friction), so sideways
    loads carried by friction are missing.
  - Replaced by a simulated wrist F/T sensor (`WRIST_FT=1`, `scripts/isaac/wrist_ft.py`): the link incoming joint
    force of fr3_hand. Calibrated with known wrenches: world force = R_hand · raw = Franka sign, torque about the
    flange.
- **Result** (`scripts/compare_wrench.py`, peg insertion with probes):
  - estimate vs wrist F/T in contact: error RMS 0.17 / 0.31 / 0.80 N (true force RMS 0.5 / 1.6 / 4.1 N), mean
    ≤ 0.24 N; up to 7 N briefly at impacts (observer bandwidth ~5 Hz)
  - free motion 0.1–0.4 N RMS.
- The insertion at 1 mm hole error failed in this run (it succeeded in two earlier ones): the hand-written strategy
  is not reliable even at 1 mm, which sets the baseline for the CI-MPC.

### 2026-10-23 (night): Factory peg-in-hole benchmark in sim
- **Assets:** Isaac Lab Factory / FORGE 8 mm round peg and hole (`~/ci_mpc_assets/factory/`, from the Isaac 6.2 asset
  server), meshes with PhysX SDF collision.
  - Actual mesh geometry: hole r 4.5 mm with a 1 mm chamfer, peg r 3.993 mm with 0.5 mm chamfers, so the radial
    clearance is 0.51 mm. The 8.1 mm in the Factory task config is only a nominal number; I first quoted it wrongly
    as a 0.06 mm clearance.
  - Scene `scripts/isaac/peg_scene.py` (`PEG=1`): kinematic hole block, free peg on the table, Factory contact
    settings (μ 0.75, contact offset 5 mm, rest 0), 32 position iterations (Factory: 192; while the robot holds the
    peg it shares the island: RTF 0.19 → 0.7 at 32).
  - Drop test (`PEG_DROP_TEST=1`): pegs dropped 0 / 0.5 / 1.5 mm off all reach the bottom (32 and 192 iterations).
    First version put three pegs over one hole, 0.5 mm apart → they overlapped and blew apart (my bug).
- **Demo** `scripts/demo_peg.sh [--offset dx,dy]` → `scripts/peg_insert_demo.py`:
  - stage-1 MPC grasps the peg and carries it above the hole estimate
  - the MPC is paused (`/ci_mpc_planner/active`, new) and a contact phase commands the controller directly:
    descend until contact (lag 3 mm, stopped, near the expected opening) → spiral search with a push → insert
    (soft laterally) until the peg stalls at the bottom
  - success = peg bottom within 1 mm of the hole bottom (Factory: 4 % of the depth).
- **Fixes on the way:**
  - Contact detection by lag alone fires 10 mm above the hole: joint stiction holds the arm at the start of the
    descent with the soft z spring. Gated by "stopped" + expected height.
  - The insert phase stopped 1.5 mm early → stop when stalled at the bottom.
  - Negative offsets were parsed as options → `--hole-offset=...`.
- **Results (hole position error = perception error):**

  | variant | 0 mm | 1 mm | 2 mm | ≥ 3 mm |
  |---|---|---|---|---|
  | search K 1500 / push 2 N / 4 mm/s | ✅ 0.05 mm, search 1.6 s | ✅ 0.04 mm, search 9.8 s | ❌ jammed 2.4 mm short | ❌ not found (TCP lags the spiral 2.6 mm, ~stuck) |
  | K 2500 / push 1 N / 3 mm/s | ❌ (stiff laterally: the chamfer can't guide it, then stuck 21 mm short) | | ❌ | ❌ |
  | 1500 / 2 N + soft insert (500 N/m) | ✅ 0.04 mm, no search needed | | ❌ jammed 2.7 mm short | ❌ |
  | + dither 0.4 mm @ 8 Hz | ✅ 0.03 mm | | ❌ peg pushed up in the gripper → false "engaged" | ❌ same |

- **Mechanisms (why ≥ 2 mm fails):**
  - Joint stiction plus contact friction (push × 0.75) mean the TCP follows a slow spiral only in jumps.
  - Sideways forces at the tip (40 mm lever) tilt or slip the peg in the grasp; a tilted peg wedges (two-point
    jamming).
  - Engagement is detected from the TCP (assumes a rigid grasp), which is wrong once the peg slips.
  - None of these is in the controller's model → motivation for contact-implicit MPC: grasp and peg-hole contacts
    in the model (friction cones, complementarity), forces chosen to avoid slip and wedging, tilt-insertion
    strategies. Prerequisites: contact force estimate (sim: from joint torques, check against PhysX), planar
    peg-hole model, fast solver.

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
- **GUI run failed, headless passed (user run):**
  - With the Isaac window open, the sunlit top faces render a bit brighter (saturation 90-110, the threshold was
    90). Green's mask was only its shaded side (a 1.2 mm sliver), so its center was 19.6 mm off. box0 was placed on
    it and fell off; box2's mask (17 mm) gave too small an opening, and the fingers landed on it (TCP stuck 24 mm).
  - Fix: saturation threshold 50 and the top face grown by depth (all pixels at the mask's top height connected
    to it, any color).
  - Also: slivers (< 1 cm top face) are dropped, and covered boxes (center under a higher detected box) are not
    published.
  - After the fix the window and headless runs are identical: base 2.6 mm, tower top 1.9 / 2.0 mm off the base.
  - The ~1.45 mm "TCP error" per segment is the segment end tolerance (1.5 mm), not a tracking error.

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
