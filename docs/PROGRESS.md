# Progress log

Running log of what has been done, what was found, and where to resume. Newest entry on top.
Read this before starting work (and before re-investigating something "new").

## Where to resume

- **2026-10-27 evening (real FR3, continue tomorrow):** robot 192.168.1.200 connected, read-only checks pass, hold +
  Cartesian breakaway measured at the probe pose (cold): **C3 arm_friction 1.43, 0.80, 1.25 N, 1.56, 0.61 Nm**; offline
  check recommends `--c3-arm-friction 1.43,0.80,1.25,1.56,0.61 --c3-stiction-ff 1.4,1.25` (entries below). Open, in order:
  1. **communication_constraints_violation reflexes** stopped both warm probe runs (also with SCHED_FIFO 98, while the
     other sessions ran Isaac + MPC nodes: load 15, GPU 97 %). Before the next robot run: no sim jobs on the machine
     (coordinate with the other sessions), user terminal `ulimit -r` = 99 (realtime group: log out/in once); if it
     still happens: RT kernel (REAL_ROBOT.md 2).
  2. Warm repeat: `scripts/real/c3_breakaway_session.sh probe`.
  3. Joint identification: `scripts/real/joint_id_session.sh test`, then `home`, `probe`, `side` (validated in Isaac,
     never run on the robot); then predict the Cartesian breakaway from the joint values (J^T) and compare with the
     breakaway runs.
  4. The frozen-state abort (measured torques bit-identical / no /joint_states for 0.3 s after a reflex) is untested on
     the robot: check it does not fire falsely in the first run.
  5. Free-space MPC (new, entry "2026-10-28"): `scripts/real/mpc_free_space_session.sh test`, then `all`. Breakaway
     moves faster: `c3_breakaway_session.sh probe --goto-speed 0.04` (default 0.015 m/s).
  Controller commit e56a1c5 (real_robot_controllers.yaml) is not pushed (blocked by a permission check).
- **2026-10-27 (workstation move done):** new machine (omar-OMEN: RTX 4090 24 GB, 62 GB RAM, 32 threads) passes
  all `MIGRATION.md` B.3 checks; resume with the two lists below unchanged. Isaac capacity (old limit: 4 instances)
  not yet re-measured here.
- **2026-10-28 (vectorized step 4c, peg only):** the press-force gap is in the **pipeline's observer**, not the
  batched arm (entry "vectorized step 4c"). Same command in the probe holds (spring + 3 N ff ≈ 3.5–3.8 N) on both sides;
  true force through the hand (wrist F/T): pipeline 2.51 N, batched 2.1 N; the pipeline observer reads 5.35 N
  (+2.5 ± 1.7 N). The pipeline is also not repeatable: same seed, 47–50 % class agreement with itself (batched vs
  pipeline 45–51 %), so the ≥ 80 % class target is unreachable; flat Fz repeats (5.28 / 5.23 N). Decision pending (user /
  src-5e): validate the batched sim against the pipeline's *wrist F/T* (true force) and retrain the likelihood on
  true / batched forces, or emulate the pipeline observer's over-read (`CI_TAU_SOURCE=projected` does not: +0.13 N).
  NIST task board #1 = H5 (entry "NIST task board"): assets built; pipeline smoke test round 8 mm: 0 mm offset
  inserted, 2 mm failed (chamferless 0.07 mm hole: probes read flat everywhere, no information); batched NIST / AutoMate only
  after peg fidelity passes. Label batched results "batched, not validated for <task>".
- **2026-10-27 evening (vectorized step 4b, fidelity levers):** step 4 still fails; the cause is narrowed to the
  arm, not the gripper (entry "vectorized step 4b"). Joint friction off (with its compensation) reproduces the
  pipeline's tip drift over k (−0.03 → −0.22 vs −0.03 → −0.20 mm) and closes a third of the press gap (Fz 3.57 vs
  2.80, pipeline 4.93 N); pipeline grip drive and solver iterations 32 do ~nothing. But the batched static breakaway
  at scale 1 **matches** the pipeline's (x 2.51 / y 1.85 / z 2.30 vs 2.33 / 1.85 / 2.2 N), so scaling friction down
  is not the fix. Difference found: the pipeline arm reaches its set-point (start error 0.004–0.17 mm; the MPC
  approach runs the integrator, `mpc_sim.yaml` integral_enabled true) while the batched arm stalls in the stiction
  band (4.5 mm short, spring = stiction). Next, in order:
  1. Log the pipeline's commanded vs measured TCP z through the probe phases of one sweep episode (an external
     recorder on joint states + the MpcCommand topic; `peg_insert_demo.py` logs nothing in probe mode) and the same
     from `fidelity_probe_replay.py --max-episodes 1`: where does the pipeline arm get past stiction that the batched
     one does not (integrator state carried over from the approach, the press loop, the 1 kHz vs 240 Hz command)?
  2. Put that mechanism into the batched probe / replay, rerun the full replay (~40 min at N 120 on the shared GPU):
     target ≥ 80 % class agreement, flat Fz within 0.5 N, the tip drift over k.
  3. Then step 5: batched random-probe collection (as `peg_insert_demo.py --collect`: r U(0, 9) or U(2, 6) mm,
     hold U(0.5, 2) mm, settle U(0.2, 0.6) s; reuse the `Replay` class with sampled targets) → `probe_dataset.py`
     format → `train_probe_likelihood.py`, validated on held-out *pipeline* sweeps2.
- **2026-10-27 (C3 in Isaac, after the move):** steps 1–2 below done. Translational stiction ~2 N (model z 3.5 too
  high); near-full feedforward (MF2, 2.2 / 2.0 N) is best so far: 8 / 20 C3 phases finish (MF 1.5 / 1.3: 5 / 26);
  rotational friction in the model: negative (entry "C3 in Isaac"). Overnight result (entry "C3 overnight result"):
  MF3 (ff 2.9 / 2.7) best, 60 % of C3 phases deep, the 0,0 stall gone. The limit is now the estimate at handover
  (< 1 mm: 86 % deep, ≥ 2 mm: 0 %). MF4 = MF3, so keep MF3. **MF3 + std gate 3 mm: 40/40, 20/20 phases deep, time = plain (25.1 vs 25.0 s), engaged
  observer peak 7.5 vs 10.2 N.** Gate 3 mm beats 2.5 / 3.5. **But the force gain is an observer artifact (2026-10-28):
  on the true contact force (PhysX contact on the hole block, `hole_contact.csv`) the engaged peak is equal, 14.8 vs
  14.8 N (mean diff 0.1 N, p = 0.92); see entry "true contact force".** So gated C3 = plain on D1 (success, time, true
  force). Real-arm match: gated C3 30/30 but 9 s slower than plain, true peak equal (7.6 vs 7.4 N), so no gain.
  Running 2026-10-28: integrator in the C3 phase (`--c3-integral`, `night/run_integ.sh`), then held-out
  `queue_cross_task_c3.sh` (H1–H3, `~/ci_mpc_runs/cross_c3`), then H3-100 (`queue_h3_100.sh`, all AutoMate
  assemblies, `~/ci_mpc_runs/h3_100`, resumable, stops starting jobs at 05:45). Both log the wrist F/T (`WRIST_FT=1`).
- **2026-10-27 (C3, before the workstation move):** goal = contact-implicit MPC, so C3 gets fixed, not dropped.
  The 3D C3 handover fails in Isaac because C3's model **underestimates the arm's stiction**: in Isaac nothing moves
  while the model predicts motion, C3 limit-cycles (violation ~1). Offline on a stickier plant (3 / 5 N vs the
  model's 2 / 3.5 N) C3 inserts 0/16; **with the true stiction in the model + friction-compensation feedforward
  (2 N lateral along the commanded motion, 3 N down): 16/16, 58 steps, peak 2.18 / 3.46 N** (entry 2026-10-27
  "C3 friction"). Next, in order:
  1. Measure Isaac's effective Cartesian breakaway (force ramp ±x, ±y, −z in free space above the block, at the
     probe pose) → C3 model `arm_friction`.
  2. Port `--stiction-ff` (and the contact re-estimate, optional) from `scripts/c3_stall_diag.py` into
     `peg_insert_demo.py --entry c3` (+ `--c3-arm-friction`), rerun `scripts/queue_c3_isaac.sh` (ARMS=c3) on D1 vs the
     plain arm in `~/ci_mpc_runs/c3_isaac/d1/plain`.
  3. Belief-std gate is useless (std at the sink 2.5–5.7 mm in 12/14 handovers; 0.5 mm gate never opened).
  Replay tool: `scripts/c3_isaac_replay.py RUN_DIR [--grasp-offset]` on `c3_trace.json` (written by `--entry c3`).
- **2026-10-26 evening:** C3 ADMM dual-rescale bug fixed (found by comparing with dairlib); the fast C3 now
  enters 16/16 offline (was 5/16). Next: rerun refine-MPC / sampled teacher / Isaac C3 with the fix; find why
  insertion stalls at 18–21 mm.
- **2026-10-26 morning:** learned probe likelihood works in closed loop (with the distance-prior correction): D1 30/30
  for A7 / B13 / B14 at ~half the contact time. Running: held-out random-offset test (`queue_heldout_r.sh`) and 40 more
  sweep episodes. To do: centering-bound default on; belief collapse fix (stuck dither) evaluated in the held-out run;
  rebuild the stale `ci_mpc_planner` colcon install and recheck D1; real-robot sweep on a fixture.
- **2026-10-25:** calibration touch: plain probe 29/30 (was 24/30); GPU entry 22/30, refine 16/30 (false engagements).
  H3 rerun: spiral 7, probe 4, GPU 4 of 12 (noisy). B6/B7 (sink only): 26/29, 26/30, fail only at ±5 mm. B8/B9 (push → centering): 28/30, 27/30. A3/B10 (height tracking): 29/30 each. A4/B11 (learned v1): 27/30, B11 below B10: negative (off-policy distance prior). A5/B12 (learned + correction): 29/30 each at ~27 / 33 s contact vs ~58 s. Running: A6/B13 (`queue_combined.sh`), then A7/B14 (`queue_v2_belief.sh`, v2 model
  with probe sweeps).
  Watch: `scripts/watch_runs.sh`. When done: compare B4/B5 with A2 (same code), H3 rerun with run 1.
  Then `queue_engage_sink.sh` runs B6/B7 (tip-sink-only engagement); compare with B4/B5.
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
  - `~/franka_ros2_ws/src/cartesian_impedance_control` (`LucasG2001/cartesian_impedance_control`). Our work is
    on branch `isaac-sim-pinocchio` (7 commits on top of `main` f8d5fe2), **pushed to origin** since 2026-10-27
    (we do have push access; only the branch, never merge into `main`). Push after each controller commit.
    Get it: `git clone -b isaac-sim-pinocchio git@github.com:LucasG2001/cartesian_impedance_control.git`.
    Fallback backup: `patches/cartesian_impedance_control_isaac-sim-pinocchio.patch`
    (`git checkout -b isaac-sim-pinocchio f8d5fe2 && git am < .../ci_mpc/patches/cartesian_impedance_control_isaac-sim-pinocchio.patch`).
  - `~/franka_ros2_ws/src` (franka_ros2 `humble`): `franka_bringup/config/controllers.yaml` registers
    `cartesian_impedance_controller`. Backup: `patches/franka_ros2_humble_controllers_yaml.patch` (`git apply`).
  - Assets outside git: `/home/njohnson/ci_mpc_assets/fr3_scene.usd`, `fr3.urdf`. The scene needs
    `scripts/isaac/prepare_scene.py` (physx variant + joint armature); old copy `fr3_scene.usd.bak-20261022`.
- **Regenerate the controller patch after changes:**
  `cd ~/franka_ros2_ws/src/cartesian_impedance_control && git format-patch main..isaac-sim-pinocchio --stdout > ../ci_mpc/patches/cartesian_impedance_control_isaac-sim-pinocchio.patch`

## Log

### 2026-10-28: vectorized step 4c: the press-force gap is the pipeline observer's over-read; the pipeline is not repeatable (peg only, `~/ci_mpc_runs/vec/step4c`)
- Pipeline episode sweeps2 w0_e000 (seed 5000) rerun twice with a bag of /joint_states, mpc_command, external_wrench
  (`scripts/isaac/bag_trace.py`: TCP by FK, sim time from the joint-state stamps; the bag's record times are wall
  clock, RTF 0.3–0.6), the second with `WRIST_FT=1`; batched replay of the same episode with
  `fidelity_probe_replay.py --trace` (env 0 per env step: phase, set-point, TCP, ff, integral, controller F, F_ext).
- No command timeouts (0 "holding the current pose"), integral off in all probe commands, 1 publisher in the probe
  phase (~340 Hz sim): the command side is identical to the batched one.
- **Probe holds (39 per side), median, tared before the press:**

  | | commanded push (K e + ff) | true force through the hand (wrist F/T) | observer |
  |---|---|---|---|
  | pipeline | 3.84 N | **2.51 N** | **5.35 N** |
  | batched (`CI_TAU_SOURCE=wrist`) | 3.5–3.8 N | 2.1 N (its observer input is the hand wrench) | 2.11 N |
  | batched `CI_TAU_SOURCE=projected` | same | – | +0.13 N (proximal-joint error, class agree 0.375) |

  The arm does not move in the hold (z travel ≤ 0.01 mm). Pipeline observer − true = +2.50 ± 1.68 N per hold
  (corr 0.33); "observer = 2·commanded − true" fits the mean (−0.09 N) but not per hold (sd 2.5 N). Mechanism open: the
  observer runs on Isaac's ros2_control effort state, which was exact for known wrenches at rest (entry 2026-10-22),
  but in a stiction-held press it reads ~2.5 N high. The PhysX contact report on the hole (3.56 N) is not truth either
  (normal impulses only, 2× a resting weight; entry "Ground truth for contact").
- So the batched arm presses like the pipeline arm (2.1 vs 2.5 N true); the 2.8 vs 4.9 N "flat Fz" gap of step 4 is
  the pipeline observer's over-read. The learned likelihood (v3/v4) was trained on that over-read force.
- **The pipeline does not repeat itself:** same seed and probe targets, achieved probe xy differs (median 0.23 mm,
  up to 2 mm); probe-class agreement with itself 0.47 (0.50 where the two runs are within 0.5 mm, n 32); flat Fz
  repeats (5.28 / 5.23 N), the tip height not (flat tips +0.25 vs +0.08 mm). Batched vs pipeline class agreement
  (0.45–0.51) is at this floor: the ≥ 80 % target of step 4 is not reachable, use distributions (flat Fz, tip, engaged
  fraction by radius) and true force instead.
- The step-4b hypothesis (the pipeline arm gets past stiction via the integrator and presses harder) is refuted for the
  holds: the pipeline arm is equally stuck and its true force is the batched one's.

### 2026-10-28: NIST task board #1 = held-out H5: assets in the AutoMate layout (`~/ci_mpc_assets/nist`)
- Scoping (web, full report with sources in the session scratchpad, summary here): NVIDIA hosts simulation-ready
  USDs with SDF collision for all ATB #1 parts (Isaac/6.2/Isaac/IsaacLab/Factory/NIST/; Isaac Lab
  `contrib/nist`, gym id IsaacContrib-Factory-Franka); no CAD conversion needed. NIST originals: SolidWorks + an STL zip,
  "public information" (attribution), no license file; NVIDIA asset license unverified: **internal use only until
  checked**. No usable third-party contact sim (MuJoCo gym-taskboard: convex decomposition; RoCo: fixed-joint snaps).
  REASSEMBLE (CC BY 4.0, real FR3 + wrist F/T on ATB #1): no meshes, use only as a per-object insertion-force
  distribution check. NIST protocol: random board pose per trial, ≥ 30 trials, time + pass/fail with a binomial bound.
- `scripts/isaac/nist_assets.py` builds round 4/8/12 mm and rectangular 8/12 mm as `<id>/{plug,socket}.usd` (wrappers
  around the source bodies: metersPerUnit 1, Z-up, body `a_<id>_*`, socket SDF resolution 512) + `.obj` (subdivided to
  2 mm edges for `info()`) + `plug_grasps.json` (TCP 10 mm below the peg top), `disassembly_dist.json`, `nist.json`.
  Plug and socket share the seated frame (peg bottom at z 0 = the table; 9 mm through-holes in a 25 mm block; pegs
  50 mm long, 0.5 mm end chamfers, **holes without chamfer**).
- Clearance (diametral, mid-depth cross-section, area-equivalent diameter for round): round 4 / 8 / 12 mm
  0.095 / 0.144 / 0.261 mm; rect 8 0.185 × 0.153, rect 12 0.155 × 0.225 mm (our Factory 8 mm hole: 1.01 mm).
- The pipeline loads them as AutoMate assemblies: `automate_scene.py` reads `AUTOMATE_ASSETS` (default unchanged),
  `run_impedance_test.sh` passes it through `env -i`:
  `AUTOMATE_ASSETS=~/ci_mpc_assets/nist AUTOMATE=nist_round_8mm scripts/demo_automate.sh --headless ...` (hand-written
  likelihood via `--geo`; its chamfer guess 0.5 mm does not match the chamferless NIST holes).
- Unit metadata: `round_hole_4mm/8mm` (and our `factory_hole_8mm.usd`) declare metersPerUnit 0.01 / upAxis Y with
  metre, Z-up points. Composition ignores a referenced layer's stage metadata, and every peg result inserts into the
  9 mm bore at the right height, so **no existing peg result is affected**; the wrappers author the correct metadata,
  the shared Factory file is left unchanged.
- **Pipeline smoke test** (`~/ci_mpc_runs/h5/smoke`, nist_round_8mm, plain probe, `--geo`, wrist F/T logged, 1 run
  each, H5 = held out, no tuning): offset 0: **inserted** (depth err 0.00, xy 0.06 mm, 12.6 s; a direct hit with
  probe 0). Offset 2 mm (1.41, 1.41): **failed**, 24 probes / 155 s, belief std 5.16 → 5.19 mm (no information),
  probes drifted to +y 3–4 mm (true hole −1.4, −1.4). Mechanism: chamferless hole, 0.07 mm radial clearance: an upright
  peg partly over the hole is still carried by the face, so every probe > ~0.1 mm off reads flat (tip ±0.1 mm) and
  excludes almost nothing; the `--geo` 0.5 mm chamfer guess is wrong for these holes. Probing as built (tip height +
  chamfer contacts, tuned on the chamfered Factory hole) does not transfer; needs an edge cue (lateral force / tilt when
  the peg's 0.5 mm chamfer catches the hole edge) or a spiral / tilt search. Recorded as a peg-overfit result.
- Batched design (after peg fidelity passes): one task per run (Factory, `ci_hole_top`, `batched_probe` and the
  likelihood assume one geometry); `CI_TASK=nist:<id>` / `automate:<id>` filled from the same `info()`; the
  `ep_override` randomization carries over, plus socket yaw. AutoMate-100: one assembly per run, H3-100's offsets
  (seed 1000 + id) for a paired comparison with src-f1's pipeline runs.

### 2026-10-28: true contact force: C3's engaged-peak gain was an observer artifact; the true peak is the same in every arm (`scripts/c3_night_cmp.py`)
- Prompted by src-35: with the wrist F/T in pipeline holds, the observer over-reads by ~2.5 N (stiction: the spring
  stretch counts as contact force). `peak_force_engaged_N` is the observer's |F|, so arms with different feedforward
  could shift its bias without any change in true force. Checked offline on every D1 run against the PhysX contact
  report on the hole block (`hole_contact.csv`, in each run dir; `c3_night_cmp.py` now prints both).
- **Caveat (src-35): the hole contact report is not full truth.** It carries normal impulses only (no friction), and a
  free peg resting in the hole reads 2x its weight (entry "Ground truth for contact"). In src-35's holds it read 3.56 N
  vs wrist F/T 2.51 N. Friction-carried load matters most in centering (lateral stick-slip). Recheck pending: D1 plain vs
  MF3 + gate 3 mm, 2 repeats each, with the wrist F/T (`night/run_ft.sh`, after the integrator queue).
- **Result (hole contact, pending the wrist F/T recheck): the engaged peak is equal across arms; the observer reads
  5–8 N below it, by an amount that depends on the arm.**

  | Arm (nominal unless noted) | n | Observer peak median | True peak median / mean | Observer − true median |
  |---|---|---|---|---|
  | plain | 30 | 10.2 N | 14.8 / 13.7 N | −5.0 N |
  | MF3, never hand over | 40 | 9.1 N | 15.0 / 13.7 N | −5.9 N |
  | MF3 + gate 3 mm | 40 | 7.5 N | 14.8 / 13.8 N | −6.4 N |
  | MF3 + gate 2.5 mm | 40 | 7.5 N | 14.9 / 13.7 N | −7.0 N |
  | MF3 ungated | 40 | 8.3 N | 15.9 / 17.7 N (p90 26.8) | −8.2 N |
  | real-arm match: plain | 30 | 6.4 N | 7.4 / 8.2 N | −3.5 N |
  | real-arm match: C3 + gate 3 mm | 30 | 5.8 N | 7.6 / 8.1 N | −2.5 N |

  True peak plain − gated C3: mean −0.11 N (bootstrap 95 % CI −1.85 to +1.58, Mann-Whitney p = 0.92); real-arm
  match +0.18 N (p = 0.40). The "−1.46 N, p = 0.001" gain (entry above, observer) does not survive.
- Where the true peak sits: in the **centering phase** in 27/30 plain, 34/40 never-hand-over and 32/40 gated runs.
  Centering runs before any C3 handover and is the same code in every arm, so the C3 phase cannot change the run's peak.
  Per-phase true peak medians: centering 11.4–14.5 N, push to the bottom 4.8–5.5 N, C3 phase 10.2 N. So C3's own
  phase presses about twice as hard as plain's push over the same depth. The observer under-reads centering (lateral wedge
  contact during a slow stick-slip move), which is why it ranked the arms wrongly.
- Mechanism of the artifact: the observer bias depends on the motion and the feedforward (stiction holds the spring
  stretch; src-35 step 4c), so it differs by arm. Don't compare arms on `peak_force_engaged_N`; use the true peak
  (hole contact on the peg scene, wrist F/T where logged).
- Consequence: on D1, gated C3 (MF3 + 3 mm) = plain on success (40/40 vs 30/30), time (25.1 vs 25.0 s) and true
  force. MF3 ungated is worse on true force (mean 17.7 N, p90 26.8 N). Gated C3 is safe, not better. The held-out
  check (`queue_cross_task_c3.sh`, H3-100) now logs the wrist F/T (`WRIST_FT=1`); gear / AutoMate scenes have no
  hole contact report.

### 2026-10-28: free-space MPC session for the real FR3 (`scripts/real/mpc_free_space_session.sh`; Isaac check `~/ci_mpc_runs/real_prep/sim_mpc_free*`)
- **Frame fix:** mpc_node planned in URDF `fr3_hand_tcp`, but on the robot the controller (model_source franka) tracks
  the Desk profile's TCP, 8.5 mm further along the tool (same orientation): starting the MPC would have moved the arm
  8.5 mm. New `ee_profile` parameter (EeModel adds the frame fr3_link8 * profile tcp); set in `mpc_real.yaml` (+ the
  controller's default K 2500/2500/1000/130/130/45 commanded explicitly). Isaac (`mpc_sim.yaml`) unchanged.
- `scripts/real/mpc_free_space_real.py`: starts mpc_node, runs hold / home / steps / rotate / circle via `~/goal_pose` and
  `~/path`, safety stops (entry in REAL_ROBOT.md 5), json + raw.npz. The session script moves to home in joint space
  first (move_to_start), so the start pose does not matter.
- **Isaac check** (same driver, `mpc_real.yaml` limits, no profile, sim time): mpc_node solves 1.2 ms mean / 2.8 ms max;
  ±5 cm steps 1.22–1.33 s to 1 cm, 1.9–3.9 s to 1 mm, overshoot ≤ 0.13 mm, line deviation ≤ 2.1 mm, final 0.3–1.1 mm,
  peak 0.14 m/s (v_max 0.1 per axis); rotations reach 10 mrad in 2.1–2.9 s, final 1–11 mrad (wrist stiction, as before);
  circle r 4 cm at 6 cm/s: 1.06 mm RMS / 2.5 mm max vs the command, 0.72 mm radial RMS. The frame check aborts after
  2.0 mm when the profile frame is used against the sim's fr3_hand_tcp controller (as intended). Command gaps up to
  120 ms in sim are wall time vs a slower-than-real-time sim; on the robot the controller logs gaps > 50 ms ("no MPC
  command"), the session counts them.
- Both real scripts (`mpc_free_space_real.py` raw.npz `wrench*`, `c3_breakaway_real.py` raw.npz cols `obs_*`, `o*`) now
  log the controller's external-wrench observer and Franka's O_F_ext_hat_K side by side: src-35 found the observer
  over-reads the hand force in Isaac probe holds (5.35 vs 2.5 N F/T: stiction stretch counted as contact), and the v3
  likelihood was trained on it; the real arm's bias needs measuring (free space first). Sim smoke test OK (Franka
  source absent in sim).

### 2026-10-27 (20:30): C3 overnight result: feedforward MF3 fixes the stiction stall, the handover estimate is now the limit; plain still wins (`~/ci_mpc_runs/c3_isaac/night`)
- 18 D1 queue runs (`night/run_night.sh`, 3 workers, 16:38–20:14, no tracebacks), pooled with the earlier repeats
  (`scripts/c3_night_cmp.py`). "Deep" = a C3 phase reached ≥ 21.9 mm.

  | arm (scene) | reps | inserted | contact mean / median | engaged peak median / p90 | C3 phases deep |
  |---|---|---|---|---|---|
  | M: measured friction, no ff (nominal) | 1 | 10/10 | 44.4 / 43.8 s | 10.4 / 12.7 N | 0/11 |
  | MF: ff 1.5 / 1.3 | 2 | 20/20 | 47.2 / 47.5 s | 9.3 / 11.3 N | 5/26 (19 %) |
  | MF2: ff 2.2 / 2.0 | 4 | 36/40 | 45.5 / 38.1 s | 8.6 / 11.4 N | 17/38 (45 %) |
  | **MF3: ff 2.9 / 2.7** | 4 | 35/40 | 40.5 / 35.3 s | 8.3 / 13.2 N | **25/42 (60 %)** |
  | MF2, lateral ff only above 5 mm | 3 | 27/30 | 45.4 / 42.0 s | 8.7 / 11.1 N | 0/30 |
  | **plain probe (nominal)** | 3 | **30/30** | **31.5 / 25.0 s** | 10.2 / 11.0 N | – |
  | real-arm match: C3 (real friction + ff 1.4 / 1.25) | 3 | 28/30 | 38.6 / 39.0 s | 6.6 / 8.8 N | 13/33 (39 %) |
  | **real-arm match: plain** | 3 | **30/30** | **20.7 / 16.0 s** | 6.4 / 7.7 N | – |

- **Feedforward ladder is monotone so far** (19 → 45 → 60 % deep): at MF3 the applied ff (2.9 / 2.7 N) is above the
  measured breakaway (x 2.3, y 1.85, z 2.2 N), so the commanded force no longer has to beat stiction. The **0,0 stall at
  3.9 mm is gone at MF3**: 4/4 reach 24–25 mm (MF2: 1/4, the rest 5 mm). Lateral ff gated to the first 5 mm: 0/30 deep.
  The lateral ff is needed in the bore too, so the stall is lateral stiction all the way down, not only at the chamfer.
- **The remaining limit is the hole estimate at the handover, not friction** (MF2 + MF3, 80 phases):

  | estimate error at handover | phases | deep | depth median |
  |---|---|---|---|
  | < 1 mm | 36 | 86 % | 24.2 mm |
  | 1–2 mm | 20 | 55 % | 22.4 mm |
  | 2–3 mm | 14 | 0 % | 10.3 mm |
  | ≥ 3 mm | 10 | 0 % | 0.7 mm |

  On the real-arm match it is the same (< 1 mm: 13/14; ≥ 1 mm: 0/19). All 11 failures (MF2 / MF3 / real C3) are at
  −5,0 / −4,2 / 5,0. The handover came with an estimate 1.6–4.9 mm off (belief std 4–6 mm), the peg wedged in the bore at
  7–11 mm, and push-to-bottom timed out. Belief std predicts it: std < 3 mm → 79 % deep (47 phases),
  std ≥ 3 mm → 27 %.
- **C3 vs plain: plain still wins** on success (30/30 vs 35/40) and time (median 25 vs 35 s nominal; 16 vs 39 s
  real-arm match). The force gain is small: engaged median 8.3 vs 10.2 N nominal, none on the real match (6.6 vs 6.4 N).
  Plain's peak is the push to the bottom, so even a working C3 phase removes only ~2 N. Not adopted; `--entry c3` stays off.
- Follow-ups (20:15–21:45, `night/run_mf4.sh`, `run_gate.sh`, 4 repeats each):

  | arm | inserted | contact mean / median | engaged peak median / p90 | C3 phases deep |
  |---|---|---|---|---|
  | MF3 (above) | 35/40 | 40.5 / 35.3 s | 8.3 / 13.2 N | 25/42 (60 %) |
  | MF4: ff 3.6 / 3.4 | 36/40 | 39.8 / 32.9 s | 8.2 / 13.3 N | 24/39 (62 %) |
  | **MF3 + `--c3-std-max 0.003`** | **40/40** | **30.9 / 25.1 s** | **7.5 / 9.9 N** | **20/20 (100 %)** |
  | plain (above) | 30/30 | 31.5 / 25.0 s | 10.2 / 11.0 N | – |

  - MF4 = MF3: the ff is already above breakaway, so more does not help. **Keep MF3** (less ff for the real arm).
  - **The std gate fixes the handover**: C3 starts only on 20 of 40 runs (the rest center until the push), and every
    phase reaches the bottom. Success and time now **equal plain**; the engaged peak is 2.7 N lower (median) and
    1.1 N lower (p90).
  - **Caveat, not yet attributed:** inside the gated arm, the push peak is the same with and without a C3 phase
    (7.2 vs 7.4 N). The c3-arm push peak was also lower than plain on D1 before any C3 phase worked (8.1 vs 10.8 N).
    So part or all of the force gain may come from the c3-arm path (how it reaches the push), not from C3.
- Sweep + control (21:46–00:13, `night/run_sweep.sh`, 4 repeats; real-arm match 3):

  | arm | inserted | contact mean / median | engaged peak median / p90 | C3 phases deep |
  |---|---|---|---|---|
  | plain | 30/30 | 31.5 / 25.0 s | 10.2 / 11.0 N | – |
  | MF3, never hand over (`--c3-std-max 0`) | 40/40 | 36.3 / 28.4 s | 9.1 / 11.0 N | – |
  | MF3 + gate 2.5 mm | 40/40 | 35.8 / 30.6 s | 7.5 / 10.3 N | 18/19 |
  | **MF3 + gate 3 mm** | **40/40** | **30.9 / 25.1 s** | **7.5 / 9.9 N** | **20/20** |
  | MF3 + gate 3.5 mm | 39/40 | 33.8 / 29.7 s | 8.0 / 10.5 N | 17/18 |
  | real-arm match: plain | 30/30 | 20.7 / 16.0 s | 6.4 / 7.7 N | – |
  | real-arm match: C3 + gate 3 mm | 30/30 | 24.1 / 24.9 s | 5.8 / 7.7 N | 9/10 |

  - **Gate: 3 mm is the best of the three** on time (25 vs 30–31 s) at the same force, so it is not on a slope. The
    time differences between gates (~5 s median) are close to repeat noise (MF2 repeats differed by more).
  - **Force attribution (nominal), corrected 2026-10-28:** the never-hand-over control's 1.1 N lower median is noise,
    not a code-path effect. With `--c3-std-max 0` the C3 branch never fires and the run takes plain's exact code (the
    model is only built; centering and push are shared). The peaks are bimodal (~5–7 N or ~10–11 N, a locked-in ramp
    force per src-35 step 4c), so the medians jump: mean difference plain − never 0.58 N, bootstrap 95 % CI −0.33 to
    1.47, Mann-Whitney p = 0.19. C3 phase vs never: mean −0.88 N (CI 0.12–1.69, p = 0.045). Gated C3 vs plain: mean
    −1.46 N (CI 0.57–2.33, p = 0.001); peaks above 9.5 N in 22 % of runs vs 63 % for plain. Across all gated runs, the push after a C3 phase
    peaks at 6.9–7.5 N vs 7.4–9.3 N without one. **So C3 does reduce force, by ~1.5 N (~15 %), at plain's time and success.**
  - **Real-arm match: no gain.** C3 + gate is 30/30 but 9 s slower than plain (median 24.9 vs 16.0 s), and the peak
    is only 0.6 N lower (p90 equal). With the real arm's lower friction, plain's push is already gentle (6.4 N).
    For the real robot, plain stays the default. Gated C3 is safe (no failures) but brings no benefit there yet.

### 2026-10-27: vectorized step 4b: fidelity levers: joint friction reproduces the slip, but the batched breakaway matches; the batched arm stalls in the stiction band (`~/ci_mpc_runs/vec/step4b`)
- 40-episode replays (`fidelity_probe_replay.py --max-episodes 40`), one lever each, sequential (the GPU is shared:
  5 Isaac processes time-sliced made 8-env runs crawl; one process at a time, ~35–50 min each next to 3 C3 Isaac
  instances). Summary by `scripts/isaac/fidelity_slip.py` (tip of probes > 6.5 mm from the hole, not class-filtered:
  a slipped tip reads "engaged"); the f1 baseline on the same 40 episodes:

  | run | switch | tip k 1–10 / 10–20 / 20–30 / 30–40 mm | after in-hole | flat Fz | class agree | xy track |
  |---|---|---|---|---|---|---|
  | pipeline | – | −0.03 / −0.08 / −0.20 / −0.20 | −0.37 (n 7) | 4.93 N | – | – |
  | f1 | – | flat (−0.15 offset) | −0.14 | 2.80 | 0.449 | 0.72 mm |
  | grip | `FR3_GRIP=pipeline` (force drive 20 N / finger) | 0.00 / −0.01 / −0.05 / −0.08 | +0.04 | 2.65 | 0.449 | 0.70 |
  | iters | `FR3_PEG_ITERS=32` | flat | +0.01 | 2.87 | 0.444 | 0.73 |
  | iters32 | + `FR3_ITERS=32,1` (robot) | flat | +0.01 | 2.76 | 0.453 | 0.70 |
  | grip_iters | grip + iters32 | 0.00 / 0.02 / −0.01 / −0.04 | 0.00 | 2.62 | 0.458 | 0.70 |
  | nojf2 | `--joint-friction-scale 0` | **−0.03 / −0.08 / −0.13 / −0.22** | −0.11 | **3.57** | **0.507** | **0.20** |

- Bugs on the way: `--joint-friction-scale 0` left the controller's velocity friction compensation on: it pushes
  along q̇ with nothing to cancel it, the arm ran away (xy 280 mm, run `nojf`); the replay now scales
  `ctrl.coulomb` / `ctrl.beta` with it (also why the pre-move attempt "stopped"). `FR3_PEG_ITERS` alone changed
  nothing: the held peg is in the robot's island, which runs at the max count (robot 192); the pipeline robot runs
  the PhysX default 32 (`fr3_scene.usd` authors none). The replay npz now stores `tip_off`.
- **Batched breakaway** (`scripts/isaac/batched_breakaway.py`, as `peg_insert_demo.py --breakaway`: peg in hand, 10 mm
  above the opening, K 1500 / 1500 / 500, 1 N/s to 0.1 mm, one env per direction × scale, compensation scaled too,
  set-point re-anchored at the settled pose; `breakaway2.json`): scale 1: +x 2.09, −x 2.93, +y 2.03, −y 1.67, +z 2.18,
  −z 2.43 → symmetric **x 2.51, y 1.85, z 2.30 N** vs the pipeline 2.33 / 1.85 / ~2.2 (rep_a): the static friction
  matches. Scale 0.75 / 0.5 / 0.25 / 0: x 2.17 / 1.87 / 1.07 / 0.29, y 1.58 / 0.94 / 0.20 / 0.33, z 1.98 / 1.45 /
  0.26 / 0.28 N.
- **The difference:** approaching the start pose by a set-point jump, the batched arm stops 4.5 mm short (spring =
  stiction); the pipeline's breakaway runs start 0.004–0.17 mm from x0 (its MPC approach runs the integrator).
  Without friction the batched arm reaches its set-points (xy track 0.72 → 0.20 mm), presses harder and the peg
  creeps: that is why friction off "fixes" the slip. So the pipeline gets past stiction somewhere in its command
  path that the batched replay lacks; not measured yet (Where to resume 1).
- First attempt (`breakaway.json`) invalid: full compensation on reduced-friction plants ran away (100–1000 mm), and
  the 4.5 mm stall preload biased + vs −.

### 2026-10-27: C3 in Isaac: arm stiction ~2 N is not above the model; stiction feedforward deepens C3 but does not fix it (`~/ci_mpc_runs/c3_isaac/{breakaway,d1_meas,d1_meas_ff}`)
- **Breakaway at the probe pose** (`peg_insert_demo.py --breakaway=-x,+x,...`, peg in hand, 10 mm above the opening,
  K 1500/1500/500, ramp 1 N/s to 0.1 mm; `breakaway/rep_a`, 2 repeats each, within 0.01 N): −x 2.05, +x 2.60, −y 2.01,
  +y 1.70, −z 2.08 / 2.30 N → symmetric x 2.33, y 1.85, z ~2.2 N (matches the real-tool smoke test below). C33Config
  2 / 2 / 3.5 N is right laterally and **1.3 N too high in z**: "the model underestimates the translational stiction"
  (entry "C3 friction") is wrong; the rotational one (0.05 vs 0.55–1.2 Nm, entry below) is the open candidate.
- **What stalls C3** (d1_trace): commanded lateral spring force median 0.4–2.8 N, mostly ~1 N, below the ~2 N
  breakaway, while the model's chamfer contact predicts motion. MF 0,0 trace: spring + feedforward 1.95 N in +x
  (breakaway 2.6), z total −1.9 / −2.4 N alternating (≈ breakaway + 0.2 N contact), TCP 0.6 mm off the axis (> 0.51 mm
  clearance), tilt 5.3 mrad, contact force 0.2–0.6 N (not jammed), violation 0.40 / 0.53 (2-cycle); 3.84 → 3.89 mm
  in 150 steps.
- **D1 queues** (new `--c3-arm-friction`, `--c3-stiction-ff lateral,z` in `peg_insert_demo.py`; 1 repeat each):

  | arm | inserted | contact mean / med | C3 phases ≥ 21.9 mm | C3 depth med | C3 peak med / p90 |
  |---|---|---|---|---|---|
  | d1_trace (model 2/2/3.5, no ff) | 9/10 | 66.7 / 55.5 s | 1 / 12 | 2.8 mm | 3.84 / 7.08 N |
  | M: model 2.2/1.9/2.2 | 10/10 | 44.4 / 43.8 s | 0 / 11 | 6.3 mm | 4.88 / 12.64 N |
  | MF: M + ff 1.5 / 1.3 N | 10/10 | 44.5 / 47.5 s | 2 / 13 | 8.5 mm | 4.86 / 10.96 N |

  MF with estimate error ≤ 0.4 mm: 22.5 / 21.7 / 20.3 mm, but 0,0 (0.4 mm) stuck at 3.9 mm. The contact-time gain is
  within the one-repeat spread (d1/c3 52.6 s). Reading: compensation at 60–70 % of the stiction leaves the total push
  just under breakaway in the stickiest direction; C3 does not finish alone.
- **Repeats + next levers** (`d1_meas_ff_r2`, `d1_meas_ff2`, `d1_meas_ff2_r2`, `d1_meas_ff_rot`):

  | arm | repeats | C3 phases ≥ 21.9 mm | C3 depth med per repeat | inserted | contact med per repeat |
  |---|---|---|---|---|---|
  | MF (ff 1.5 / 1.3) | 2 | 5 / 26 (19 %) | 8.5 / 15.3 mm | 20/20 | 47.5 / 47.0 s |
  | **MF2 (ff 2.2 / 2.0)** | 2 | **8 / 20 (40 %)** | 16.7 / 12.1 mm | 19/20 | 39.7 / 43.2 s |
  | MFR (MF + rot. friction 1.53 / 0.57 Nm in the model) | 1 | 1 / 10 | 11.9 mm | 9/10 | 49.6 s |

  - Repeat noise is large (MF 8.5 vs 15.3 mm median depth on identical settings): one repeat does not separate arms.
  - MF2 > MF: compensation near the full stiction. MF2's one failure (−5,0): C3 drove the peg to 9.5 mm from a 3 mm
    estimate error, then the push timed out 11.2 mm short (bore jam; lateral ff in the bore is the suspect).
  - **MFR negative:** the rotational friction in the model (correct value, 30× the default) does not help; 0,0 still
    stops at 3.9 mm in every arm (unexplained; same depth since d1_trace). **Offline and Isaac disagree on this
    lever:** offline (c3_stall_diag, real-arm plant) the measured model incl. rx/ry 1.56 / 0.61 inserts 16/16 (entry
    "Offline C3 check with real-arm stiction"). The quasi-static plant misses whatever stops C3 in Isaac (also: offline
    measured model without ff 16/16 vs Isaac M 0/11 phases), so offline C3 results are not predictive for Isaac yet.
- **Real-arm match in Isaac:** `fr3_scene_realfric.usd` = `prepare_scene.py --params
  ci_mpc_sysid/config/fr3_joint_params_sim_realfric.yaml` (joints 1–4 static/Coulomb ×0.5, wrist unchanged) **plus**
  `ci_mpc_sim_gripper/config/isaac_pick_place_nofc.yaml` (`friction_compensation: none`, as `real_robot_controllers.yaml`).
  Without the config change the velocity compensation (tuned to the full friction) drives the arm away (4–110 mm drift
  between ramps). `--breakaway` (now also ±rx, ±ry: torque ramp 0.1 Nm/s, 1 mrad) symmetric / bias:
  x 1.49 / −0.91, y 1.08 / +0.42, z 1.40 / −0.65 N, rx ≥ 1.55 Nm, ry 0.57 Nm; real FR3 (entry below) x 1.43, y 0.80,
  z 1.25 N, rx 1.56, ry 0.61 Nm, bias ≈ −1.1 / −0.85 / −0.9 N. Matches except the y bias sign. Nominal scene rotation:
  ry 0.54 / 0.54, rx +: none to 2 Nm / −: 0.88 Nm (src-54's tool: 1.53 / 0.57 symmetric).
- **Overnight** (`~/ci_mpc_runs/c3_isaac/night/run_night.sh`, 18 D1 queue runs, 3 workers, domains 170–190): MF2 r3–r4,
  MF3 (ff 2.9 / 2.7, one step above MF2) ×4, MF2 + `--c3-ff-lat-depth 5` ×3 (bore jam), plain probe ×3 (reference on
  this machine), real-arm match: plain ×3 and `--c3-arm-friction 1.43,0.80,1.25,1.56,0.61 --c3-stiction-ff 1.4,1.25` ×3.
  Throughput: every run at RTF 0.26 with 2–3 instances + src-35's batched replay vs 0.70–0.83 alone (the machine is
  saturated; ~13 min machine time per D1 queue run).
- Fixed on the way: `segment()` waits for the planner's path/goal subscriptions (a path published before discovery
  is lost and the arm stays home: seen with two Isaac instances starting together). `queue_c3_isaac.sh`:
  `DOMAIN_BASE` env for parallel queues.

### 2026-10-27: real-robot breakaway tool for the C3 stiction / feedforward parameters (robot not connected yet)
- `scripts/real/c3_breakaway_real.py`: standalone ROS node (no sim clock, no MPC node) for the real FR3 via
  `franka_hardware` + `cartesian_impedance_controller`: set-point fixed, integral off, contact K, feedforward ramp per
  direction until 0.1 mm / 1 mrad motion; repeats; stiction = (f+ + f−)/2, bias = (f+ − f−)/2; prints
  `--c3-arm-friction` / `--c3-stiction-ff` (offline ratios 2/3 lateral, 3/5 down). Safety: force / torque cap,
  abort outside a 5 mm ball or above 0.03 m/s, `--dry-run` read only, typed confirmation; killing it is safe
  (controller mpc_timeout holds the pose). Procedure: REAL_ROBOT.md 5b.
- Robot not reachable from omar-OMEN today (enp3s0 is on the shared lab LAN 10.0.0.0/24; 172.16.0.2 and the
  launch default 192.168.1.200 have no route); stock kernel, no RT. The user will say when the robot is connected.
- Smoke test on Isaac's ros2_control pipeline (home pose, no peg, velocity friction compensation 1.0,
  `~/ci_mpc_runs/real_prep/sim_smoke`, `sim_rot`): x 2.26 N (bias +0.22), y 2.65 (+0.09), z 2.15 (+0.06);
  **rx 1.21 Nm, ry 0.55 Nm** (2 repeats within 0.03 Nm). The C3 model's rotational arm friction is 0.05 Nm:
  10–25× below Isaac at this pose. Check at the probe pose before reading much into it.
- `--goto x,y,z` (user: no fixture at the robot, free space only, hand mounted): straight-line set-point move at
  15 mm/s, orientation kept, then measure where it settled. Isaac (`~/ci_mpc_runs/real_prep/sim_goto`, home →
  0.5, 0.1, 0.28, reached 0.8 mm off; 2 repeats): x 2.39 N, y 1.94, z 2.46 (down 2.65), rx 1.53 Nm, ry 0.57 Nm.
  Per-direction values move with the settled state (+x 1.53 / 2.23, −x 3.14 / 2.68) but the pair sums do not
  (4.67 / 4.90): use the symmetric stiction, not single directions. Abort path checked (`--goto-abort-mm 1` →
  exit 2, feedforward 0, controller holds).
- **Frame bug caught before the robot:** the real controller (`model_source: franka`) tracks
  `franka::Frame::kEndEffector` = the Desk profile TCP. The robot's profile (`desk_ee_hand_camera_mount_v0.1`, copied
  to `scripts/real/`: 0.88 kg, COM (−12.5, 4.4, 33.8) mm, TCP z 111.9 mm, yaw −45°) puts it 8.5 mm beyond the URDF
  `fr3_hand_tcp` (103.4 mm), so the script's set-points would have been 8.5 mm off (Isaac never shows it: there
  the controller uses the same URDF). Fix: `--ee-profile` (EE = fr3_link8 · profile TCP), `--goto` in fr3_hand_tcp
  coordinates (same posture as Isaac), start-up guard (abort if the arm moves > 2 mm when the hold starts).
  Isaac: matched frames → normal run (`sim_frameA`); profile vs Isaac's URDF controller → abort at 4.2 mm, before
  any ramp (`sim_frameB`). Side note: the mount makes the payload 0.88 kg vs 0.71 kg (hand + fingers) in our URDF;
  irrelevant on the robot (Franka's model uses the profile), relevant for the Isaac / Pinocchio model later.
- **Robot connected (15:00):** FR3 at 192.168.1.200 (PC 192.168.1.145, enp3s0), ping 0.12 ms, 0 % loss over 200.
  User added to the `realtime` group (rtprio 99). Kernel is not RT: libfranka examples refuse (kEnforce);
  franka_hardware falls back to kIgnore itself. Read-only `readOnce` (scratch tool, kIgnore): server version 10,
  Idle, no errors; Desk profile active (F_T_EE z 0.1119, yaw −45°, m_ee 0.88). **Pinocchio + profile vs Franka
  O_T_EE: 0.001 mm, 1e-5 rad** (REAL_ROBOT.md step 3 passes). `scripts/real/c3_breakaway_session.sh` (bringup →
  read-only check → [asks] activate → hold or measure; cleanup on exit) tested on the robot up to the activation
  prompt (no motion): /joint_states 200 Hz, clean shutdown. franka.launch.py needs `robot_type:=fr3`.
  Controller commit e56a1c5 (real_robot_controllers.yaml) is local only: the push was blocked by a permission check;
  backup in patches/ (base f8d5fe2, no local `main` on this machine).
- **First real measurement (probe pose, cold, `~/ci_mpc_runs/real/breakaway/probe_cold`, friction compensation none,
  K 1500/1500/500/260/260/90, 3 repeats):** x 1.44 N (+1.68 / −1.19), y 0.79 (+0.94 / −0.65), z 1.11 (down 1.02),
  rx 1.52 Nm, ry 0.61 Nm. Translational ~half of Isaac's (2.4 / 1.9 / 2.5 N), rotational about the same.
  The arm settled 0.5–0.8 mm off x0 between ramps (≈ 1 N of spring preload on the next ramp).
- **Home run aborted** (`home_cold`, after the goto, before the first ramp result): user: **fast buzz at the wrist after
  reaching home** (high-frequency wrist instability, as the Isaac wrist chatter without armature); no
  Franka reflex in launch.log, so the script's 5 mm / 0.03 m/s stop fired (message only on the terminal). Changes:
  rotational K 260/260/90 → 130/130/45 (controller default, held fine near home at activation), watchdog on TCP
  speed in every phase (was ramps only), raw 200 Hz log (`raw.npz`, also on abort), abort reason in the JSON,
  session tees the output; each ramp now starts from the settled pose (spring force 0). Isaac with the changes
  (`real_prep/sim_v3`): +x 2.45 / −x 2.48 N, +rx 1.67 / −rx 1.67 Nm: the single-direction "bias" in Isaac was the
  spring preload, now gone.
- **Second real runs (`breakaway/home_1544`, `probe_1539`, rot K 130/130/45, ramps from the settled pose):** no buzz,
  no aborts. But zeroing the spring was wrong for the real arm: the model / gravity bias (~1 N) then drifts the arm
  by itself and one side of a pair reads ~0 N (home −z 0.027, +x 0.045, +ry 0.012 N(m); probe −y 0.05 N). In Isaac
  (no model bias) zeroing removed a spurious bias; on the robot the spring at x0 is what holds the bias inside the
  friction cone. Pair average = stiction only if both sides are > 0 (f± = s ∓ b); a clipped pair gives s ≤ f_big/2.
  Spring zeroing is now opt-in (`--zero-spring`), default back to the set-point at x0.
  Valid probe-pose estimates agree across both methods: x 1.44 / 1.42, z 1.11 / 1.38, rx 1.52 / 1.60, ry 0.61 / 0.61,
  y 0.79 (first run; second clipped, ≤ 0.90). Model bias at the probe pose (second run): ≈ −1.1 N x, ≤ −0.85 N y,
  −0.9 N z (the arm drifts toward −x, −y, down when the spring is zeroed). Home: y 1.68, rx 0.97 N(m) valid; x, z, ry
  clipped (≤ 1.66 N, ≤ 1.8 N, ≤ 0.53 Nm).
- **Real-arm C3 values (probe pose, cold, friction compensation none): arm_friction ≈ 1.43, 0.80, 1.25, 1.56,
  0.61** (x, y, z [N], rx, ry [Nm]) vs C33Config 2.0, 2.0, 3.5, 0.05, 0.05: translational below the guess, rotational
  30× above. Feedforward by the offline ratios: ≈ 0.75 lateral, 0.75 down. Offline check running:
  `~/ci_mpc_runs/c3_stall/real_probe` (set16s offsets, plant = real values; model default / matched / matched + ff
  0.5, 0.75, 1.0).
- **Offline check with the real-arm values** (`c3_stall_diag.py`, set16s offsets / estimate errors / grasp offsets,
  plant arm friction = the real values; new `--plant-bias fx,fy,fz` = unmodelled constant force, real ≈ −1.1, −0.85,
  −0.9 N; `~/ci_mpc_runs/c3_stall/real_probe`):

  | plant | C3 model / compensation | inserted | steps med | peak \|F\| med / p90 / max |
  |---|---|---|---|---|
  | real friction | C33Config default (2, 2, 3.5, 0.05, 0.05) | 10/16 (fail 13.9–22.1 mm) | 52.5 | 2.67 / 3.03 / 3.50 N |
  | real friction | measured, no ff | **16/16** | 49 | **1.88 / 2.72 / 3.32 N** |
  | real friction | measured + ff 0.5 / 0.75 / 1.0 | 16/16 each | 46.5–47 | 1.99–2.32 / 2.61–2.94 / 3.25–3.64 N |
  | real friction | measured + ff 1.1/1.0, 1.4/1.25 | 16/16 each | 46.5–47 | 2.38–2.67 / 3.38–3.99 / 4.84–5.11 N |
  | real friction + bias | default | 13/16 (fail 20–22.6 mm) | 66 | 3.86 / 4.77 / 5.05 N |
  | real friction + bias | measured, no ff | 16/16 | 45.5 | 3.03 / 4.34 / 5.45 N |
  | real friction + bias | measured + ff 0.75/0.75 | 15/16 (one stuck at 0.4 mm) | 45 | 3.10 / 3.57 / 6.51 N |
  | real friction + bias | measured + ff 1.4/1.25 | **16/16** | 44 | 3.60 / 4.19 / **4.32 N** |

  - The lever is the friction model: measured values instead of the guesses +6/16 (bias-free) and +3/16 (with bias).
    The default fails in the bore with no contact (13.9–22.6 mm, contacts 0/0/0): the model's z stiction 3.5 N /
    rotational 0.05 Nm is wrong in both directions.
  - Feedforward is not needed on the real arm's (lower) friction without bias; above ~1 N it only raises the p90 / max
    force. With the bias, near-full compensation (1.4/1.25) is the most robust (lowest max force), consistent with
    src-f1's Isaac D1 (ff 2.2/2.0 5/12 vs ff 1.5/1.3 2–3/13). One-sample differences (15 vs 16 of 16) are noise.
  - **Recommendation for the real arm:** `--c3-arm-friction 1.43,0.80,1.25,1.56,0.61 --c3-stiction-ff 1.4,1.25`; fallback
    without ff. The bias was measured only at the probe pose and the plant is the quasi-static model.
- **Joint identification session prepared** (user: are two Cartesian poses enough / what does SotA do → per-joint
  identification, then predict the Cartesian breakaway via J^T and compare with the breakaway runs):
  `scripts/real/joint_id_session.sh test|home|probe|side|all` (bringup → move_to_start to the joint configuration →
  JointIdController → fit_joint_id.py; probe = IK of fr3_hand_tcp 0.5, 0.1, 0.28; side = 0.6, −0.4, 0, −2.4, 0, 2.0, π/4;
  limit margins ≥ 0.68 rad), `ci_mpc_sysid/config/real_sysid.yaml` (conservative vs Isaac: ±0.2 rad, ≤ 0.4 rad/s,
  accel 1, sines ≤ 0.5 rad/s, Franka's example gains), `run_joint_id.py --param-file` (spawner with the type;
  unloads at the end). Isaac test (`NO_LAUNCH=1 JOINTS=7 ... probe`): pose reached within 0.014 rad; joint 7 fit vs
  scene truth: static 0.550 / 0.534, Coulomb 0.531 / 0.534, viscous 0.459 / 0.450, armature 0.208 / 0.206.
  Not run on the robot yet. Note: the real hand + camera mount is 0.88 kg vs 0.71 kg in the URDF (RNEA residual).
- **Warm probe repeat broke** (`breakaway/probe_1552`): robot reflex `communication_constraints_violation` at the
  first +z ramp (rep 0); the arm stopped but /joint_states kept coming, so the script ramped on and logged "none up to
  8 N" for the other 26 ramps. Valid before the stop: x 1.44 N (+1.93 / −0.96), y 0.82 N (+1.01 / −0.64), same as
  cold. Cause: the user's terminal had no realtime priority (controller_manager "Could not enable FIFO RT
  scheduling", realtime group needs a new login) on top of the non-RT kernel. Fixes: both scripts abort when
  /franka_robot_state_broadcaster/robot_state leaves Move mode; both sessions send
  /action_server/error_recovery at the start and print reflexes from launch.log at the end; louder ulimit -r warning.
- **Warm probe, second try** (`breakaway/probe_1610`, controller_manager now SCHED_FIFO 98): reflex again after ~36 s
  (+z of rep 0), the robot_mode check did not fire (the state apparently stays at its last value). Valid before:
  x 1.45 N, y 0.71 N, +z 1.35 N (same as cold). Machine load at the time: other sessions' Isaac + MPC nodes (load 15,
  GPU 97 % / 10 GB): likely latency source on the non-RT kernel. Added: abort when the measured joint torques are
  bit-identical or /joint_states is silent for 0.3 s (both robot scripts; real robot only).
- Earlier Isaac run at the probe pose (`~/ci_mpc_runs/c3_isaac/breakaway/fric`, peg in hand): +x 3.10, −x 1.56,
  ±y 1.75 / 1.83, −z 2.04 N (x stiction 2.33, bias 0.77). `nofric` is invalid: the lift hit the planner's
  singularity stop (TCP 154 mm off), so the arm was not at the pose and drifted up to 570 mm.

### 2026-10-27: moved to the second workstation (omar-OMEN), all checks pass
- Machine: Ubuntu 22.04.5, RTX 4090 24 GB (driver 595.91.07), 62 GB RAM, 32 threads. Data from the T7 transfer
  (checksums OK); `cartesian_impedance_control` cloned from the bundle (branch `isaac-sim-pinocchio`, 7 commits on
  f8d5fe2), `messages_fr3` 9d5f4ab; Isaac Sim 6.2.0.0 + IsaacLab 46d6cc3d7, extras already at the old versions.
- Checks: `pinocchio_model_check` all OK (home TCP err 2.0e-5); `ci_mpc_planner` tests 13 passed + 1 xfail;
  impedance test hold 0.00 mm, reach 0.11 mm / 0.0083 rad, < 1 cm in 1.54 s (same as the W2 reference; the
  2.2 mm in MIGRATION.md predated W2, fixed there); `demo_peg.sh --offset 0,0 --headless`: success, depth err
  0.02 mm, xy 0.44 mm, insert 2.16 s, peak 4.65 N, RTF 0.70–0.83.
- Bug on the way: numpy 2.2.6 in `/usr/local` (ZED SDK) breaks apt-built ROS Python (scipy 1.8 `_ARRAY_API not
  found` via osqp, `pinocchio` segfault). Fix without touching the ZED stack: user scipy 1.15.3 + a numpy 1.26.4
  shim in `~/.local/ros_numpy1`, used by the `ros_franka` alias and by `run_impedance_test.sh` (MIGRATION.md B.3a).

### 2026-10-27: vectorized step 4: fidelity vs the ros2_control pipeline FAILS (grasp slip, press force) (`~/ci_mpc_runs/vec/step4`)
- `scripts/isaac/fidelity_probe_replay.py`: the 120 logged sweep episodes (sweeps + sweeps2, 4800 probes), one env
  per episode with its params.json (hole error, contact friction, tilt, grasp offset, sensor model), the demo's
  calibration touch, then the logged TCP positions with the logged hold depths / settle times
  (`BatchedProbe.hold_depth` / `settle_steps` now per env; `CiCtrlFactoryEnv.ep_override` fixes the episode
  parameters). Hole yaw / hand yaw noise 0 (pipeline geometry). ~25 min wall for N 120.
- Result (`f1.npz`):

  | metric | pipeline (logged) | batched |
  |---|---|---|
  | class agreement (flat / engaged / in hole) | – | **46 %** (58 % on probes 1–9) |
  | flat Fz (tared) | 4.98 N | 2.79 N |
  | flat lateral \|F\| median, probes > 6.5 mm from the hole | 0.92 N | 0.39 N |
  | "engaged" on probes 9–20 mm from the hole (flat face) | 43 % | 8 % |
  | sunk at r 0–1 / 1–2 / 2–3 mm (probes 1–9) | 0.99 / 0.81 / 0.17 | 0.92 / 0.64 / 0.23 |
  | in hole at r 0–1 mm (probes 1–9) | 0.52 | 0.24 |
  | flat tip difference (both flat) | – | +0.04 ± 0.29 mm |
  | grasp-height calibration | +0.49 mm | −0.05 mm (corr 0.96, constant offset) |
  | engaged lateral direction cos (median) | – | 0.35 |
  | TCP xy reached vs logged (median) | – | 0.74 mm |

- Sim geometry is right: in the batched sim engagement falls off cleanly with distance to the true hole (85 % < 2 mm,
  7 % > 5.5 mm); the hole-offset sign alternatives do not fit.
- **Mechanism 1, in-hand slip in the pipeline:** logged flat tips far from the hole drift with the probe index
  (median −0.03 / −0.16 / −0.20 / −0.28 mm for k 1–10 / 10–20 / 20–30 / 30–40; share below −0.3 mm 1 % → 47 %);
  a flat probe right after an in-hole probe reads −1.15 mm (n 15): the peg is pushed up into the fingers. Batched:
  0.00 mm in every block (stiff PD gripper). Half the "engaged" disagreements are slip-shifted flat probes.
- **Mechanism 2, press force:** flat Fz regressed on hold / settle: pipeline 3.17 + 0.75 N/mm·hold + 2.5 N/s·settle,
  batched 1.93 + 0.56·hold + 0.32·settle. The impedance law predicts 3 N feedforward + 500 N/m · hold ≈ 3.6 N;
  the pipeline sits 1.4 N above, batched 0.85 N below. Not the integrator (the demo sends integral_enabled false;
  the observer trace shows a flat −6.2 N plateau within ~0.2 s, no ramp), not the observer (both validated on known
  forces), **not the PhysX backend** (batched on CPU, 8 episodes: flat Fz 2.74 vs 4.86 N logged). Open; joint
  stiction holding preload or the gripper are the candidates. The larger pipeline lateral readings are likely the
  same effect.
- Verdict: batched probe data is not a substitute for pipeline data yet; the learned likelihood (trained on pipeline
  data) would see cleaner tips and ~half the force. Step 5 waits for the two fixes (Where to resume).

### 2026-10-27: C3 friction: the Isaac failure is underestimated stiction; true stiction + compensation fixes it offline
- **Handover estimate error** (14 handovers of the ungated D1 run, from the probe lines): errors 1.3–3.2 mm → all
  stuck at the entry (≤ 1.1 mm); 0.16–1.1 mm → 3 of 4 deep (≥ 21.9 mm); belief std at the sink 2.5–5.7 mm in 12/14.
- **Gated queue** (`--c3-std-max 0.0005`, `~/ci_mpc_runs/c3_isaac/d1_gate`): 0 handovers in the first 2 runs (the gate
  never opens), stopped by process group; a third run (3,0) was killed mid-run (its Traceback is the kill).
- **Trace queue** (`~/ci_mpc_runs/c3_isaac/d1_trace`, ungated, `--entry c3` now logs the estimate error and
  `c3_trace.json`): 0,0 handover with only 0.25 mm error still stopped at 3.95 mm; 3,0 (1.00 mm) → 12.9 mm; −3,0
  (1.11 / 0.80 / 0.46 mm) → 0.7 / 1.7 / 22.7 mm; 5,0 (4.85 / 3.96 / 1.42 mm) → 0.0 / 0.7 / 12.0 mm; 2,0, −5,0, 0,4, 3,3
  had no handover; −4,2 (0.84 / 1.10 mm) → 1.7 / 7.0 mm, inserted; **−3,−2 (3.98 / 4.24 mm) → 0.8 / 7.2 mm, FAILED
  8.8 mm short** (the first C3-arm failure in Isaac). 9/10 inserted, the push-to-bottom heuristic finishing the rest.
- **Replay** (`scripts/c3_isaac_replay.py`) of the 0,0 handover: from the exact Isaac start state, offline C3 inserts
  (24.4 mm / 57 steps / 3.4 N; with the true grasp offset (0.04, −0.32) mm in the plant: 24.0 mm / 77 steps). In Isaac
  the TCP sat 0.64 mm from the hole axis (> 0.51 mm clearance: the peg is off-centre in the fingers + 4.5 mrad tilt),
  measured z and xy motion 0.000 mm per step for 150 steps, while the model predicts ~0.16 mm lateral motion per step;
  C3 commanded only 1–3.8 mm z deflection (< 7 mm needed) and alternated rollout (~700 ms, violation 1.4) / shifted
  (~40 ms, 0.9) solves with the lateral force toggling 0 ↔ 2 N: a limit cycle on a plant stickier than the model.
- **Offline levers** (`c3_stall_diag.py`, 16 offsets, estimate error 0.3 mm, goal 11 mm below; `~/ci_mpc_runs/c3_stall/set16g`, `set16s`):

  | plant / C3 change | inserted | steps med | peak med / p90 |
  |---|---|---|---|
  | nominal plant | 16/16 | 64 | 2.08 / 2.40 N |
  | + grasp offset 0.3 mm (plant only) | 16/16 | 88 | 2.70 / 3.95 N |
  | + grasp offset + contact re-estimate (`--reest 0.3`, 0.5 N noise) | 16/16 | 62 | 2.62 / 6.16 N |
  | sticky plant (3, 3, 5 N) + grasp offset, model 2 / 3.5 N | **0/16** (0–6 mm) | – | 1.14 / 17.17 N |
  | + friction compensation 1.5 / 3 N (model friction reduced by it) | 11/16 | 111 | 1.04 / 5.92 N |
  | + compensation 1.0 / 3, 2.0 / 3, 1.5 / 4.5 N | 10 / 11 / 10 of 16 | 115 / 99 / 53 | |
  | + compensation 2.0 / 3 + re-estimate | 12/16 | 115 | 1.26 / 3.83 N (max 18.9) |
  | + compensation 2.0 / 3, lateral only above 1 mm depth | 7/16 (9/16 with re-estimate) | | negative: fails at the chamfer–bore transition |
  | model knows the true stiction, no compensation | 9/16 | 80 | 2.23 / 6.52 N |
  | **true stiction in the model + compensation 2.0 / 3** | **16/16** | **58** | **2.18 / 3.46 N** |

  - The contact re-estimate (hole = TCP + (R + c − d − (a − pc)) along the lateral force on the chamfer, (R − a) in
    the bore; it absorbs the grasp offset) needs contact: with a 1.1 mm estimate error and no lateral force it never
    triggers (C3 frozen at 0.5 mm). Marginal on its own (+1 of 16).
  - The fix is the pair: a correctly identified stiction **and** compensating it in the controller. The 2 / 3.5 N in
    `C33Config` / `gpu_mpc_server.py` are guesses; Isaac's effective value is not measured yet (Where to resume).

### 2026-10-27: 3D C3 handover in Isaac (D1): 10/10 but slower and no force gain, negative
- `peg_insert_demo.py --entry c3` (879d625): on a tip sink, 3D C3+ on the belief estimate, goal 11 mm below the bottom
  (`--c3-goal-below`), up to `--c3-steps` 150, one belief update per phase. `res.json` now has `phases` (name, start,
  end, peak |F| of the observer estimate) and `peak_force_engaged_N` for every strategy. Planner install rebuilt with
  `--symlink-install` (the stale 10-23 copy is gone; tests 13 passed + 1 xfail).
- `scripts/queue_c3_isaac.sh`, D1 × 1 repeat, interleaved per offset, one Isaac instance (`~/ci_mpc_runs/c3_isaac/d1`):

  | arm | inserted | contact mean / median | probes | engaged-phase peak median / p90 | push-phase peak median |
  |---|---|---|---|---|---|
  | plain probe + v3 (recommended) | 10/10 | **30.3 / 24.8 s** | 4.8 | centering 6.07 / 7.11 N | 10.81 N |
  | + `--entry c3` | 10/10 | 52.6 / 52.9 s | 4.1 | C3 phases 6.04 / **12.41 N** | 8.08 N |

  - 14 C3 phases (median 18.6 s, max 33.8 s): only 2 reached 24 mm; 7 stopped at 0.1–3.8 mm, 5 at 13–22 mm, and the
    push-to-bottom heuristic finished. Violation 0.3–1.2 (offline 0.1–0.3), 100–600 ms per decision (offline ~80).
  - Smoke test before (3,−2): the first handover came with the estimate 1.7 mm off and lost contact for ~24 s;
    the second (0.4 mm off) inserted.
- **Reading:** the offline gain (16/16 at 1.2–1.4 N) does not transfer. Likely causes, not yet separated: the hole
  estimate at the sink is ~1 mm off (offline 0.6 mm already broke it), and model mismatch (arm stiction / grasp /
  force estimate vs the quasi-static model; the high violation says the QP and the projection disagree on Isaac
  states). The peak forces are dominated by the push to the bottom in both arms (~8–11 N) and centering phases are
  ~6 N, so even a working C3 phase would have to beat ~6 N, not the offline heuristic's numbers.
- Not adopted; `--entry c3` stays off by default. Possible next: gate the handover on belief std < 0.5 mm
  (`--c3-std-max`), log the estimate error at handover, replay Isaac states offline to separate the two causes.

### 2026-10-27: 3D C3 stall at 18–21 mm solved offline: stiction trap + cost, fixed by a goal below the bottom
- Tool: `scripts/c3_stall_diag.py` (setup of `peg_c3_3d_offline.py` + per-step trace: depth, tilt, active contacts by
  type, arm friction, violation; policies c3 / follow / press; `--set` C33Config overrides, `--goal-below`). Set: 16
  offsets, r 0.8 / 1.13 / 1.47 / 1.8 mm × 4 directions (rotated 22.5° per radius), `~/ci_mpc_runs/c3_stall/set16`.
  Success = tip 24 mm deep, 150 steps. Planner imported from the source tree (the install is stale).
- **Not jamming:** at the stall tilt is ~1 mrad, contact force ~0, the arm is held by its own stiction.
  1. Stiction trap: arm z stiction 3.5 N at K_z 500 needs a 7 mm command deflection to move; horizon 4 × du_max
     0.5 mm reaches only 2 mm more, so once the deflection drops below 7 mm C3 never predicts motion again and
     relaxes the command (absorbing; also why zero warm start fails).
  2. Cost: r_force penalizes the whole deflection (contact + arm friction). Near the goal the position gain no
     longer pays for the 7 mm, so C3 stops ~1–6 mm short even when it could move.
  - The offline heuristic also stalls (2/16, 15.6–17 mm): its 3 N press while following is below the 3.5 N stiction.
    Oracle press on the true axis (5 N) stalls on the chamfer at 0.5–1.2 mm (lateral stiction 2 N vs 2.25 N spring).

  | variant (one lever vs base) | inserted | steps med | peak force med / p90 | entry (< 5 mm) / bore p90 |
  |---|---|---|---|---|
  | base (rollout every 4, du_max z 0.5) | 0/16 (17.9–18.5 mm) | – | 2.26 / 3.17 N | 3.06 / 2.60 N |
  | du_max z 2 mm | 3/16 | 50 | 2.97 / 3.89 | |
  | horizon 16 (3 offsets) | 0/3 (20.4, 23.7 mm, 1 at entry) | | | |
  | r_force z ×0.1 + du2 (3 offsets) | 0/3 | | | |
  | goal 5 mm below the bottom | 9/16 | 69 | 1.86 / 3.28 | |
  | **goal 8 mm below** | **16/16** | 61 | **1.37 / 1.93** | 1.93 / 1.63 |
  | **goal 11 mm below** | **16/16** | 60 | **1.18 / 1.96** | 1.79 / 1.80 |
  | goal 15 mm below | 16/16 | 61 | 1.60 / 2.05 | 2.03 / 2.00 |
  | du2 + goal 8 below | 16/16 | 40 | 5.11 / 5.31 | 3.12 / 5.31 |
  | du_max z 1 mm + goal 8 below | 16/16 | 42 | 2.41 / 4.28 | 2.36 / 4.28 |
  | follow heuristic | 2/16 | 124 | 3.82 / 6.59 | 6.59 / 2.25 |

  - The goal below the bottom is the lever (the bottom stops the peg, so it only keeps the press on). Force is
    non-monotone in it, best at 8–11 mm. A larger z step is faster (40 vs 60 steps) but loads a deeper spring that
    hits the bore wall at 7–9 mm (bore p90 5.3 N): not adopted. Solve ~80 ms (Python).
- **Hole-estimate error** (`set16e`, goal 11 below; direction varied per offset): 0.3 mm 16/16 at 2.08 / 2.40 N;
  0.6 mm 14/16 at 4.15 / 5.65 N (max 14 N, both misses at the entry). Bore clearance is 0.51 mm, so with a larger
  error C3 drives toward a wrong axis into the wall. In Isaac C3 needs a belief narrower than ~0.5 mm at handover;
  the heuristic does not depend on the estimate.
- Not changed: C33Config defaults (the goal is the caller's: `refine_mpc.py`, `sampled_teacher.py`, Isaac). Isaac
  `--strategy c3` still runs the planar `c3.py`; a 3D C3 handover needs wiring in `peg_insert_demo.py` (gear/vec
  session's active file) and the colcon rebuild. Planar c3 3 mm failure (arm friction 1.5 N, du_max 0.5) may be the
  same trap: not checked yet.

### 2026-10-27: README rewritten with the stricter create-readme skill
- The skill (`.claude/skills/create-readme`) now requires every number with its metric, set, n and baseline, every
  internal name (D1, R2, v3, GPU entry, ...) defined, a mechanism for each component and each dropped idea, and no
  filler words. The nightly prompt follows it.
- README: status table (result vs baseline per component), "How it works" per component, benchmark/term definitions,
  a recommended peg command. Run IDs stay in this log. Fixed on the way: the model path is
  `ci_mpc_planner/models/...` (the old README said `models/...`).

### 2026-10-27: vectorized step 3: batched probe strategy, first 1024-episode benchmark (`~/ci_mpc_runs/vec/step3`)
- `scripts/isaac/batched_probe.py`: the probe strategy of `peg_insert_demo.py` as per-env state machines (touch:
  hover / move / settle / tare / press / hold / measure; calibration touch; learned v3 belief batched in torch;
  next probe; centering; push; jam recovery), planning step 0.1 s = one env step, recommended settings (v3,
  track-height 5, centering bound 8 mm, stuck dither 0.8 mm, 25 probes, 150 s). `run_batched_probe.py` scores with
  the demo's rule (peg bottom within 1 mm of the hole bottom, xy within 1.5 mm, hole frame) on randomized episodes.
- Seed 1, 1024 episodes (step-2 randomization), one process; jam recovery 3 / lift 3 mm / shift 0.6 mm vs off,
  same episodes (search identical: probes per env equal in all 1024):

  | arm | inserted | contact mean / median (successes) | probes | search failed | wedged |
  |---|---|---|---|---|---|
  | plain probe + v3 | 960 (93.8 %) | 28.6 / 17.1 s | 6.4 | 43 | 21 |
  | + jam recovery 3 | **974 (95.1 %)** | 29.4 / 17.6 s | 6.4 | 43 | 7 |
  | ros2_control held-out R (30 runs) | 30/30 | 23.9 / 17.5 s | 3.8 | | |

  Jam recovery: fixes 16, breaks 2 (McNemar exact p = 0.0013), same contact time on the common successes (28.6 s).
  The ros2_control D1 check (10/10, 0 false triggers) could not separate it; 1024 paired episodes can.
- Failure modes (base): search failures grow with the estimate error (97.5 % at < 1 mm, 89.3 % at 4-5 mm; belief
  flat, std 4.6 mm, after 25 probes); wedges (peg ~3 mm in at 0.5 mm off-axis while the TCP slid ~19 mm down the
  peg, the H1 friction-1.0 mechanism) grow with friction (97.3 % at mu < 0.67, 89.2 % > 0.83). Tilt and grasp offset:
  no effect.
- Throughput: 1024 episodes x 160 s in 29-33 min wall (82-93 sim s / wall s, with a ros2_control Isaac instance of the
  C3 session on the same GPU). The ros2_control pipeline needs ~4 h for 30 episodes.
- Caveat: step 4 (fidelity vs ros2_control) not done; until then these are statements about the batched sim. The
  batched rate (93.8 %) is below the ros2_control R result (30/30) on a harder distribution (|e| U(0, 5 mm) incl.
  tilt / friction / grasp / sensor randomization vs R's fixed offsets); a matched comparison is step 4.

### 2026-10-27: vectorized step 2: per-env episode randomization (`~/ci_mpc_runs/vec/step2`)
- `factory_fr3.TASK_CI` (cfg `FactoryFR3CiCfg`, `RAND`): per env and episode, the ranges of `collect_probes.sh`:
  hole estimate error |e| U(0, 5 mm), contact friction (peg, hole) U(0.5, 1.0), hole tilt U(0, 1.5 deg) about its x
  in 1/3 of the episodes, grasp offset dx / dz U(-1, 1 mm), torque-sensor noise U(0.02, 0.12) Nm, per-joint offsets
  U(-0.3, 0.3) Nm, observer gain U(20, 40); joint friction scale U(lo, hi) (off by default). Our grasp (TCP 10 mm
  below the peg top + offset) replaces Factory's in-hand noise; `ci_hole_top()` / `ci_hole_estimate()` give the true
  opening and what the strategy is told.
- `scripts/isaac/test_ci_rand.py` (64 envs) reads every parameter back from the sim: friction, tilt (20/64 tilted,
  0.0004 deg), joint friction (scale 0.7-1.3 in the test), sensor model, hole error all exact; grasp offset as the
  peg's pose in the TCP frame: depth 0.0003 mm, across 0.11 mm (corr 1.0 / 0.997), new values on every reset; a 2 s
  press (K_z 500) stays stable (no NaN).
- Bug found: Factory starts the hand 47 +- 10 mm above the opening, sized for its grasp (peg 32 mm below the
  fingertip frame); with ours (40 mm) a low start put the peg into the block at the grasp (2/64 dropped it, others
  slipped up to 2.4 mm). Now 55 mm, no start noise (Factory's +-2 cm xy also pushed pegs off the block's side).

### 2026-10-27: vectorized step 1: our controller + observer batched in torch, validated (`~/ci_mpc_runs/vec/step1`)
- `scripts/isaac/ci_ctrl_torch.py` (BatchedImpedance): line-by-line port of `CartesianImpedanceController::update`
  with `isaac_sim_controllers.yaml` (critically damped D from Λ, conditional integral, 18 N / 9 Nm / 2 Nm clamps,
  velocity friction compensation, 1 Nm/ms rate limit, momentum observer K_O 30 with the sensor bias / noise model).
  `factory_fr3.py` env `TASK_CI` runs it every physics step (240 Hz, like ros2_control); Factory's controller only
  does the reset. FR3 joints now get static / Coulomb / viscous friction (was static only).
- Checks (`scripts/isaac/test_ci_ctrl.py`, 16 envs):

  | check | batched | ros2_control |
  |---|---|---|
  | M / C q̇ / J / TCP vs Pinocchio | 0.17 % / 0.03 of 4.8 Nm / 4e-7 / 0.0004 mm | (Pinocchio) |
  | circle r 5 cm pose+twist RMS | **1.25 mm** | 1.29 mm |
  | observer, known 10.6 N at the TCP | **0.26 N** mean error | exact (no sensor model) |
  | observer, free motion (circle) | 0.27 N RMS | 0.1–0.4 N RMS |
  | press with K_z 500, 1 cm (N 1024) | F_ext 5.05 N mean | (5 N commanded) |

- Bugs found on the way:
  - Factory adds 0.01 kg m² to every body's inertia (IsaacGymEnvs armature workaround): M 30 % off → removed in TASK_CI.
  - PhysX's mass matrix **excludes** joint armature (diag = Pinocchio); subtracting it gave negative diagonals,
    garbage Λ / D, circle 2.09 mm.
  - **PhysX joint-force reporting is wrong on the proximal joints** of this imported FR3: with a known force at the TCP
    at rest, joint 2 reads −0.6 instead of −2.6 Nm (joints 5–7 right), and the link incoming forces are not even
    consistent along the chain (9.7 N at links 6–7, 8.8 at 4–5, 4.2 at 2–3). Observer 4.6 N off. Not friction,
    armature, solver iterations (32/192, vel 1–32), GPU vs CPU, link inertia, force body, physics rate (960 Hz).
    Root cause open (our ros2_control scene's effort state was exact; different importer / asset).
    Fix: link-side torque = M q̈ + C q̇ + J_handᵀ W_hand from the hand's incoming wrench (the pipeline's wrist F/T,
    correct to 0.003 N), i.e. exact for everything acting through the hand / fingers / held part; neglects the hand
    subtree's inertial force and forces on links 1–7 (`CI_TAU_SOURCE=projected` keeps the PhysX value).
- Throughput with our controller + observer, FR3, 240 Hz, contact press, N 1024: **124 sim s / wall s**
  (29,800 physics steps/s, GPU 5.1 GB, RSS 6.2 GB): ~100× the ros2_control pipeline. (FR3 with Factory's controller
  measured 64 yesterday; that run had other Isaac jobs on the machine.)
- Fingertip frame: the stock Panda's `panda_fingertip_centered` is 112.1 mm from the hand, ours = `fr3_hand_tcp`
  103.4 mm (the frame ros2_control / the FR3 control). Kept ours; Factory's grasp then holds the peg 8.7 mm higher in
  the fingers than the stock task. Grasp offset parity with our pipeline → step 2.
- Next: step 2 (per-env randomization incl. grasp offset) / 3 (batched probe strategy).

### 2026-10-27: gear round 3 (push dither 0.25 mm + per-step yaw drop check, `dev/yaw/r3`): 1/5, negative
- Round-2 settings (`--yaw-search 9 --push-follow 2e-5`, release fix) + `--push-dither 0.00025` (1 Hz xy circle) and the
  yaw-drop check per sweep step instead of against the sweep start.
- yaw 1° (dwell 1 s): **1/3** (0.86 mm depth err, 21 s); the misses stall 5.2 / 3.6 mm short at xy 0.26–0.27 mm, the
  same clearance-limited off-axis wedge as round 2 (0/3). The sweep found no drop in all 3 (hand yaw −7°, back to 0).
- Nominal 0.77,−3.87 (tooth on tooth): run 1 dropped at −2° cmd, then stuck 9.8 mm short (tip −12.3 mm) for the rest
  of the push, push-follow moving the anchor at 0.6–0.8 N lateral; run 2 never found the shaft (search failed after 22
  probes, belief converged 6.8 mm off): a search failure, not a push one.
- Mechanism: the dither radius (0.25 mm) is the bore clearance, so the circle drags the bore along the shaft wall instead
  of onto the axis; the stall is still ~0.25 mm off-axis in every miss. Dither does not fix the H2 stall; kept off.
- Next candidates (not run): compliance in xy during the push (low lateral K so the shaft centres the bore) instead
  of commanding xy; a dither with 0.05–0.1 mm radius.

### 2026-10-26 (19:00): FR3 in the vectorized Factory env runs: 64 sim s / wall s at 240 Hz + contact (~50×)
- `scripts/isaac/factory_fr3.py` (fr3.urdf via the URDF importer, Panda body names in a generated `fr3_factory.urdf`,
  identified friction / armature, FR3 limits). Bugs on the way, each with its symptom:
  - `robot` lives in the scene cfg (`self.scene.robot`), not the env cfg (AttributeError at cfg build).
  - A link without `<inertial>` is dropped by the importer ("panda_fingertip_centered is not in list"): 10 g frame.
  - The massless `base` link + fixed `fr3_base_joint` and then `fix_base=True` alone both left the articulation
    **floating** (Jacobian (N, 12, 6, 15): 6 base columns; the first reset step threw the joints to 8–11 rad, NaN).
    Fix: drop `base` (fr3_link0 is the root) and set the spawner's `fix_root_link=True`.
  - The "increase totalAggregatePairsCapacity" PhysX error at N 1024 was a symptom of the exploding floating base;
    with the fixed base the default capacity suffices (`FR3_AGG_PAIRS` env override kept).
- Sanity (after 300 contact steps, N 1024): 0 NaN envs; fingertip z 0.026–0.138 m, held peg z −0.003–0.106 m vs
  Panda 0.027–0.150 / −0.003–0.118.
- **Throughput, 240 Hz + contact, N 1024: FR3 64 sim s / wall s (15,400 physics steps/s) vs Panda 157**: the full FR3
  link collision meshes cost 2.4×. vs ~1.2 for 3 ros2_control instances: ~50×. Simplifying the arm's collision
  (only hand / fingers touch the parts) is the obvious speed lever later.
- Still open before trusting data: the fingertip frame is the FR3 TCP (vs the Panda's fingertip frame), and the
  controller / observer are still Factory's (step 1), no fidelity check yet (step 4).

### 2026-10-26 (18:10): vectorized sim step 0b: realistic throughput ~130–190× (Panda, 240 Hz, contact)
N 1024 unless noted, one Isaac instance of mine plus the C3 session's offline jobs running (less contention than
step 0, whose N 1024 stock gave 126 sim s / wall s vs 358 here: the 3 cross-task instances cost ~3×):

| setting | physics steps/s | sim s / wall s |
|---|---|---|
| stock (120 Hz, random actions) | 43,000 | 358 |
| 240 Hz (decimation 16, same control rate) | 43,900 | 183 |
| contact-heavy (press on the hole) | 37,500 | 313 |
| **240 Hz + contact** | 37,600 | **157** |
| 240 Hz + contact, N 4096 | 53,800 | **224** (GPU 7.1 GB, RSS 7.4 GB) |

- Physics steps/s is the invariant: 240 Hz halves sim time per wall second; contact costs ~13 %; N 4096 gives 1.4×
  over 1024. vs ~1.2 sim s / wall s for 3 ros2_control instances: **~130× (N 1024) to ~190× (N 4096)**.
- The FR3 runs did not run (appended to the running script after bash had read it); queued as `vec/run_fr3.sh`
  after the gear round 2.

### 2026-10-26 (17:55): gear with yaw sweep ±9° + push-follow; release bug found
- `dev/yaw/both` (`--yaw-search 9 --push-follow 2e-5`, drop 2 mm): yaw 3° **2/3** (+ test C 1/1; weekend 0/3);
  yaw 1° 0/2 (the sweep turns past the mesh, drop at −8° cmd, then a wedge 4 mm short at 0.27 mm off); H2 nominal
  3/5 + the tooth-on-tooth case 0.77,−3.87 **seated** (0.17 mm, xy 0.06 mm, meshed at +6°) but scored as a failure.
- **Release bug:** after the push the demo opened the gripper to a fixed 30 mm; the gear is grasped at 30.1 mm (and
  AutoMate 00388 at 33.1 mm), so the fingers never let go and the part could ride up with the hand ("stays" check
  failed). Fixed: release to max(30 mm, `--open-width`). Depth is measured before the release, so earlier stall
  results are unchanged; only success flags of seated gear / 00388 runs can change (00388 failed in both X arms).
- Push-follow acts (lateral 0.9 N, xy command moving) but alone did not fix the nominal stall (test A).
- Next: 1 s dwell for the 1° case; rerun H2 nominal + 00388 with the release fix.

### 2026-10-26 (17:45): cross-task regression result (`queue_cross_task.sh`, `~/ci_mpc_runs/weekend/X`)
- base = weekend probe flags (`--max-probes 25`); new = + `--track-height 5 --centering-bound 0.008 --stuck-dither
  0.0008` (+ learned v3 belief on H1 only). Same offsets / seeds as the weekend held-out sets.

  | task | base | new | median contact (successes) base / new | weekend (old code) |
  |---|---|---|---|---|
  | H1 peg, 5 conditions x 3 | 15/15 | 14/15 | 11.3 / 12.8 s | 14/15 |
  | H2 gear nominal, 5 offsets | 2/5 | 3/5 | 44.4 / 28.6 s | 3/5 |
  | H3 AutoMate, 12 assemblies | 6/12 | 5/12 | 43.3 / 47.5 s | 5/12 (rerun 4/12) |

- **No regression and no gain outside D1**: the new flags were tuned on the 8 mm peg (D1) and transfer neither way.
  - H1: the one new failure is friction 1.0 at 0.63,1.57, the jam case that `--jam-recovery` now frees (16:45 entry).
  - H2: new inserts -2.06,-2.22 (base 4.4 mm off); the failures that remain are the 8-12 mm stalls at bore clearance
    (0.27 mm off) and a miss at 3.28,2.06 (8.7 mm off). Gear yaw-search + push-follow (gear session) targets the stall.
  - H3: the two arms differ on one assembly only (00211: base OK, new FAIL, 17.6 mm short; probe also failed there at
    the weekend). 6 assemblies fail in both arms (00028 00301 00319 00388 00700 01041): an AutoMate-specific problem,
    not the probe flags. Within H3's run-to-run noise (7/36 flips at the weekend with identical code).

### 2026-10-26 (17:43): cross-task regression done (`~/ci_mpc_runs/weekend/X`)
Today's code, two arms: base = weekend probe flags (`--max-probes 25`); new = + height tracking / centering bound /
stuck dither, + learned v3 belief on H1 only (no jam recovery, yaw search or push-follow in either arm).

| task | base | new | weekend (old code, probe) |
|---|---|---|---|
| H1 peg, 5 conditions × 3 | **15/15**, median 11.3 s, mean 29.0 s | 14/15, median 12.8 s, mean 22.4 s | 14/15 |
| H2 gear, nominal 5 | 2/5 | **3/5** (found the shaft 4/5) | 3/5 |
| H3 AutoMate, 12 | **6/12** | 5/12 | 5/12, rerun 4/12 |

- **H1:** the fixes since the weekend generalize to untuned conditions (base 15/15 vs 14/15 with the old code).
  The new arm is faster on the hard runs (mean 22 vs 29 s); its one miss is the friction-1.0 wedge at 0.63,1.57
  (an insertion problem, fixed by `--jam-recovery`, see 16:45), not the belief.
- **H2:** the new flags find the shaft more often (4/5 vs 2/5); the remaining failures are a 3.3 mm miss and the
  tooth-on-tooth stall (centering / yaw, see the 17:00 entry).
- **H3:** 10 of 12 assemblies give the same result in both arms; the only flip is 00211 (base OK, new 17.6 mm short;
  it also failed with probe at the weekend). With 7/36 same-code flips measured on H3, 6 vs 5 is noise. The same 6
  assemblies fail in both arms (00028, 00301, 00319, 00388, 00700 at 3 mm short, 01041 fallen over): assembly-specific,
  not strategy.
- **Conclusion:** no cross-task regression from the peg-tuned changes; gains carry over to H1 and the gear search.
  The learned belief stays peg-only (trained on the 8 mm peg).

### 2026-10-26 (17:30): codebase check of the eval path; planar C3 had the dual-rescale bug too
- **Stale planner install (not yet fixed):** `install/ci_mpc_planner` is a copy from 2026-10-23 (no symlink install):
  9 modules missing, `hole_belief.py` / `peg_hole_3d.py` differ. `peg_insert_demo.py` imports `HoleBelief`, `PegHole`,
  `C3`, `HoleEstimator` from the install and only `probe_likelihood.py` from src. No effect on results so far (the
  diffs are an unused `likelihood=` arg and solver tolerance args), but **an "Isaac C3 rerun with the fix" would run
  the old code**, and `--strategy c3` uses the *planar* `c3.py`, not `c3_3d.py` (where 6cedabc fixed the ADMM).
  Plan: `colcon build --symlink-install --packages-select ci_mpc_planner` once no benchmark is running.
- **Planar `c3.py` had the same ADMM bug** (scaled dual not divided by rho_growth): fixed (705cacd, `dual_rescale`).
  Offline nominal (`peg_c3_offline.py`): FAIL / violation 1.05 → inserted after 278 steps, violation 0.079. With a
  3 mm offset + estimator it still does not insert in 400 steps (with or without the fix): broken since b3d3574
  (arm friction, grasp_axial 10 → 4 N). `test_c3_inserts_at_3mm` is now a strict xfail with that reason.
- `nightly_update.sh` stages only README.md / docs / patches (the blanket `git add -A` pushed WIP `--jam-recovery`
  code in 1606d5f). OSQP `polish=` → `polishing=`; pyflakes 12 → 3 (deliberate side-effect imports).
- Baseline: planner tests 13 passed + 1 xfail (needs the workspace sourced: `test_translational_mpc` imports
  ci_mpc_msgs). `peg_insert_demo.py` pruned (2026-10-27): the planar `estimate` strategy, `--follow-force`,
  `--lateral-stiction-ff` and `--force-debias` (negative: 2.5 mm worse) are gone (-51 lines). `--strategy c3`
  (planar) stays until the planar line is decided. Isaac smoke test (D1, offset 2,1, probe + v3 belief):
  inserted, 4 probes, 15.4 s contact. Still open: `command()` is a 549-line closure.

### 2026-10-26 (17:20): vectorized sim step 0 (Isaac Lab Factory PegInsert, one process) + gear fixes, first runs
- **Step 0** (`scripts/isaac/bench_factory_vec.py`, random actions, 300 timed steps; 3 ros2_control Isaac
  instances and the C3 session's GPU jobs ran at the same time, so conservative):

  | N | env-steps/s | sim s per wall s (all envs) | GPU | RSS |
  |---|---|---|---|---|
  | 16 | 40 | 2.7 | 4.3 GB | 5.5 GB |
  | 64 | 93 | 6.2 | 4.4 GB | 5.5 GB |
  | 256 | 352 | 23.5 | 4.5 GB | 5.6 GB |
  | 1024 | 1889 | **126** | 5.0 GB | 6.0 GB |

  vs ~0.4 sim s / wall s per ros2_control instance (~1.2 with 3): **~100× at N = 1024**, memory flat in N (the
  4-instance RAM cap does not apply). Fixed overhead dominates below 256; 256 → 1024 gave 5.4×, so 4096 likely helps.
  Caveat: Panda at 120 Hz with random actions; FR3 at 240 Hz with contact-heavy probing will be slower.
  No `--headless` in this Isaac Lab (headless is the default).
- **Gear fixes, one run each:** A nominal + push-follow: fail (still 0.27 mm off). B yaw3 + sweep ±9°: fail (false
  "drop" at +7°, 1.2 mm of TCP creep, stopped the sweep). **C yaw3 + both: inserted** (real 2.4 mm drop at −5°,
  xy 0.12 mm, 19 s contact). Drop threshold now `--yaw-drop` 2 mm; push-follow logs its moves.
- Queued `dev/yaw/run4.sh` (both fixes): yaw3 ×3, yaw1 ×2, H2 nominal 5 offsets.

### 2026-10-26 (17:00): gear (H2) yaw search: two failure modes, yaw only matters for one
- `--yaw-search DEG` (+ `--yaw-step`, `--yaw-dwell`; off by default): when the push to the bottom stalls or presses
  > `--max-contact-force` for 1 s, sweep the hand yaw 0, +s, −s, … ±DEG under a light press; keep the first yaw with a
  > 1 mm drop. Commands carry `c["yaw"]` (rotation about world z on top of the grasp orientation). The code first
  landed in the C3 session's commit 209f713 (swept in while uncommitted, untested then); logging per step added here.
- Tests (one Isaac, `~/ci_mpc_runs/dev/yaw`), ±5° command: 3° yaw error 0/1, 1° 0/1, nominal 0.77,−3.87 0/1.
  The hand reaches only ~50 % of the command under contact (−5° → −2.5°).
- **Nominal tooth-on-tooth (0.77,−3.87): the gear is already at the meshing yaw** (truth +3.8…+5.2° vs mesh +4.5°)
  and barely turns (±0.7° while the hand turns ±2.3°): not a yaw problem. The "drop at +5°" was the TCP creeping 1 mm.
- **3° yaw error: the gear turns with the hand ~1:1** (truth +5.2…+9.6°); it needed hand −3° (gear +4.5°) and got
  −2.5° (gear +5.2°): the sweep was too small for the hand lag.
- **All 16 gear stalls 8–12 mm short (weekend + today) end 0.23–0.30 mm off the shaft axis** (one 0.09), i.e. at the
  bore clearance (bore r 5.0, shaft r 4.75), vs 0.02–0.16 mm for all 11 successes. Likely: the hand holds xy where
  centering ended, the bore leans on the shaft, the center distance to the fixed gears is off and the teeth interfere.
  (Could also be an effect of the jam; the next test separates it.)
- New `--push-follow GAIN` (off by default): during the push, move the xy command along the lateral contact force
  (max 0.1 mm per step, deadband `--push-follow-deadband` 0.7 N).
- Queued (`dev/yaw/run3.sh`, after step 0): A nominal + push-follow 2e-5 m/N; B yaw3 + sweep ±9°; C yaw3 + both.

### 2026-10-26 (16:45): jam recovery (`--jam-recovery`): wedge fixed, no false triggers
- Trigger case from the cross-task run: H1 friction 1.0 at 0.63,1.57 (same offset failed at the weekend). The peg
  reaches the hole on probe 0, then wedges: from 19 s it presses 5.5 N sideways on the hole wall and 0 N on the
  bottom (hole_contact.csv); the robot estimate shows +5 N lateral, +3.5 N up. The TCP keeps descending while the peg
  slips in the fingers (μ 1.0 wall > grip), so a stall check on the TCP-derived tip height never fires.
- Recovery: trigger on stall or 1 s mean lateral force > `--jam-force` (3 N); lift `--jam-lift`, press again from a
  fixed anchor shifted `--jam-shift` along the lateral force (the force on the peg points away from the wall).

  | variant (friction 1.0, 0.63,1.57) | inserted | wedged | freed |
  |---|---|---|---|
  | none (stall trigger only, never fired) | 0/3 | 3 | 0 |
  | lift 1 mm, shift 0.3 mm | 1/3 | 2 | 0 (anchor moved only 0.03 mm: tilt wedge) |
  | **lift 3 mm, shift 0.6 mm** | **8/8** | 4 | **4** (one needed 3 retries, 0.83 mm short) |

- False-trigger check on D1 (A7 config + jam recovery, 1 repeat): 10/10, **0 retries fired**, contact mean / median
  40.7 / 27.5 s vs A7 39.9 / 25.8 s. Recommended with `--jam-recovery 3 --jam-lift 0.003 --jam-shift 0.0006`
  (still off by default). Not used in the running cross-task queue (one lever at a time).

### 2026-10-26 (16:40): note: Isaac refine results predate the C3 dual-rescale fix (6cedabc)
- B5 (GPU entry + refine −1, 16/30), B7 and B9 (refine variants) ran with the buggy ADMM (scaled dual not rescaled
  when rho grew; see the "2026-10-26 (evening)" entry by the C3 session). Treat their refine conclusions as pre-fix.
  Unaffected: everything without `--gpu-refine-margin` (plain probe, GPU sampler alone: A*, B4, B6, B8, B10–B14,
  R*, R2*, X).

### 2026-10-26 (evening): our C3+ vs dairlib's C3 (DAIRLab/c3): ADMM dual-rescale bug found and fixed
Source comparison (DAIRLab/c3 `core/c3.cc`, `c3_plus.cc`, `c3_options.h`, read on GitHub; not built: needs Drake +
Bazel, and cloning was not permitted in this session):

| | ours (`c3_3d.py`) | dairlib C3+ |
|---|---|---|
| language / QP | Python, OSQP (direct) | C++, OSQP via Drake `MathematicalProgram` |
| complementarity slack eta | explicit variables + equality rows | same (`C3Plus`: eta = E x + F lam + H u + c) |
| projection | closed form, unweighted `a > b` | closed form, weighted `sqrt(U_eta) eta > sqrt(U_lam) lam`, clip at 0 |
| consensus on | complementarity pairs only | all of z = (x, lam, u, eta); x, u pass through (a proximal term) |
| rho schedule | rho 10, x1.8, 10 QPs | G x `rho_scale` (default 10), `admm_iter` 3 + final QP |
| **dual when rho grows** | **unchanged (bug)** | **w /= rho_scale** (scaled-dual ADMM) |
| delta init | physics rollout of a nominal / shifted previous | zeros, x part = x0 (`delta_option` 1) |
| model | quasi-static impedance, LCS built per solve | general LCS, optional `scale_lcs`, Lemke for lam0 if H = 0 |

- **The bug:** omega is the scaled dual (y / rho). Growing rho without dividing omega inflates the effective dual by
  1.8x per iteration (~200x after 10), so later QPs chase stale residuals of the first projection and lock in its mode.
- **A/B, offline engaged phase** (`peg_c3_3d_offline.py`, rollout warm start every 4th solve = the fast variant,
  ~70 ms/solve; "entered" = tip > 5 mm deep):

  | variant | entered (16 offsets, 0.8–1.8 mm) | depth reached | violation | peak force |
  |---|---|---|---|---|
  | old (no rescale) | 5/16 | others stuck at 0.3–1 mm | 0.4–5.9 | – |
  | **+ dual rescale** | **16/16** | 18–21 mm | 0.15–0.36 | 1.1–2.8 N |
  | dairlib schedule (x10, 3+1 QPs) + rescale | 2/4 deep, 2/4 partial | | 0.17–0.46 | |
  | dairlib schedule, no rescale | 2/4 deep, 1 partial | | 0.3–2.9 | |
  - Zero warm start: all variants stay at the surface (0/4 each), so the rollout warm start is still needed.
  - No run reaches the 24 mm success depth in 150 steps (all stall at 18–21 mm); not investigated yet.
- **Reading:** the "fast C3 variants get stuck at the opening" result (2026-10-23) was largely this bug. With it fixed,
  the cheap rollout-every-4 variant enters every offset at ~70 ms per solve (vs ~1.5 s for rollout every solve).
  Default changed to `dual_rescale=True`; this also affects `refine_mpc.py` and `sampled_teacher.py`, so their
  earlier numbers were with the buggy ADMM. Not yet rerun: refine / teacher / Isaac C3 benchmarks.

### 2026-10-26 (15:17): R2 (fresh held-out, seed 78) done: the confidence gate does not help, dropped
| job | inserted | contact mean / median / p90 [s] | probes per run | GPU phases per run |
|---|---|---|---|---|
| R2A_v3 plain probe + v3 | 30/30 | 39.4 / 40.8 / 70.7 | 6.4 | – |
| R2B_v3 GPU entry + centering + v3 | 30/30 | 39.6 / 32.9 / 72.8 | 5.6 | 1.23 |
| R2B_v3g same + `--gpu-std-max 1 mm` | 30/30 | 45.6 / 37.6 / 85.0 | 6.9 | 0.10 |
- Paired per offset (mean over 3 repeats, bootstrap 95 % CI): GPU − plain +0.2 s [−6.9, 8.3]; gated − plain +6.1 s
  [−5.0, 17.7]; gated − ungated +6.0 s [−3.7, 16.7]. Repeat-to-repeat means swing 32–56 s with the same code.
- **Gate: negative.** It opens rarely (0.10 GPU phases per run vs 1.23 ungated): the belief rarely gets below 1 mm std before
  the peg is already sliding in, so it effectively switches the GPU entry off, and the runs that wait for it take
  longer (more probes, 6.9). Not adopted.
- GPU entry vs plain probe: a tie on R2 (R had plain 24 vs GPU 41 s). With the learned belief, the GPU entry gives no
  measurable gain over plain probe on either held-out set. **Recommended default: plain probe + v3** (no GPU server).
- R2 offsets are larger than R (up to 5.0 mm), hence ~40 s vs R's 24 s.

### 2026-10-26 (14:40): v4 likelihood (+ 40 sweep episodes): marginal gain, the data lever is nearly used up
- sweeps2 done (40 episodes, seeds 5000+). Dataset `probes_v4.npz` = all logs + sweeps + sweeps2, **minus the
  held-out benchmark runs** (R*, R2*, X: 170 runs), 10741 probes / 748 runs. Train with `~/IsaacLab/.venv/bin/python`
  (system python has no torch).
- Ablation, same 16 held-out sweeps2 episodes (`--val-filter sweeps2 --val 0.4`, seed 0), one lever:

  | model | val loss | replay all: log p(true) x1 / x2 errors | replay search: mean err x1 / x2 [mm] |
  |---|---|---|---|
  | hand | – | −17.3 / −18.4 | 2.45 / 3.09 |
  | v3 recipe (logs + sweeps) | 1.019 | −5.19 / −5.26 | 2.09 / 2.52 |
  | **v4 (+ sweeps2)** | **1.008** | −5.15 / −5.19 | **1.99 / 2.18** |

- 24 % more sweep data buys 1 % val loss and a small search gain at 2× sensor errors. **Mechanism:** with a
  per-probe likelihood on 5 features the model is close to saturated; the remaining error is the information in a
  single probe (flat readings rule out ~1 mm), not data. Next levers: on-policy rounds (plan step B) only if the
  closed-loop probe distribution matters; better probe placement (information gain) for the slow flat-probe runs.
- Saved as `models/probe_likelihood_v4.npz`; not benchmarked yet (R2 and the cross-task queue keep v3, one lever at a time).

- Probe placement checked before building an information-gain selector: `HoleBelief.next_probe` maximizes
  p = P(hole within the 2 mm capture radius). For a sink-or-flat outcome the expected information is the binary
  entropy H(p), which rises with p for p < 0.5, so the current rule already maximizes the information for that
  outcome. A new selector would only help through the force direction on near misses, which the learned likelihood
  already uses. The slow R2 run is coverage: 12 flat probes 2.5–5 mm apart on a 5 mm-std prior (each rules out a
  ~12 mm² disc). Not built.
### 2026-10-26 (14:10): lateral force on flat probes is noise, not a bias (checked, no fix)
- Trigger: R2B_v3g rep1 at 1.79,−1.94 (106 s): belief std stayed 5.1–5.9 mm for 12 flat probes, all with Fy ≈ −1.4 N.
- All 8912 logged probes (benchmarks + sweeps), 4345 flat (true hole > 3.5 mm away, |tip| < 0.3 mm): mean Fx −0.10,
  Fy −0.08 N, so no global bias. Fy scatters more than Fx (sd 0.94 vs 0.54 N). Not drag from the previous move
  (regression on the move direction R² ≤ 0.03). Per run (414 runs with ≥ 4 flat probes): between-run sd Fx 0.25 /
  Fy 0.47 N, within-run 0.48 / 0.80 N.
- So a per-run debias would remove only ~25 % of the Fy variance; the rest is probe-to-probe noise. Not pursued.
  The slow run is a flat-probe information problem (a flat reading rules out only the ~1 mm disc under the tip),
  not a sensor bias.

### 2026-10-26 (12:49): held-out random offsets R (seed 77, 10 offsets × 3 repeats) done
| job | inserted | contact mean / median [s] | probes per run |
|---|---|---|---|
| RA_hand (plain probe, hand belief) | 30/30 | 45.7 / 40.2 | 8.0 |
| **RA_v3 (plain probe, learned v3)** | **30/30** | **23.9 / 17.5** | **3.8** |
| RB_hand (GPU entry + centering, hand) | 29/30 | 47.4 / 44.5 | 7.5 |
| RB_v3 (GPU entry + centering, v3) | 30/30 | 41.4 / 36.7 | 6.2 |
- v3 carries over to unseen offsets: half the contact time and half the probes of the hand model, no failures.
- The GPU entry gives back most of v3's gain (41 vs 24 s): the MPC takes over while the belief is still wide and
  pulls the peg out of the chamfer. The confidence gate (`--gpu-std-max 1 mm`) is now on the fresh set R2 (running).
- The one RB_hand miss (−2.72,−2.08 rep3) ended 0.96 mm off in xy and not seated.

### 2026-10-26: daily summary
- **Height tracking (A3/B10):** 29/30 each. Plain probe did not change (A2 29/30), and GPU entry went from 28 to 29/30.
  Contact time did not improve (61.6 / 59.8 s mean).
- **Learned likelihood v1 was negative in closed loop:** A4 27/30, B11 26/30, below the hand model. The 2.83 → 1.96 mm
  offline gain did not hold in closed loop. Mechanism: the strategy-chosen training probes carry a distance prior, so the
  belief chases its own probes, and the replay shares that bias. Fix: a distance-prior correction (`rprior_correction.py`).
  With it, **A5 29/30 at 27.0 s and B12 29/30 at 33.1 s mean contact** (hand ~58 s).
- **Combined candidate (learned + height tracking + centering bound):** A6 29/30. Its one miss is a new mode: the belief
  collapsed on 13 repeated same-spot readings, which were counted as independent evidence. New option `--stuck-dither`
  (off by default). **B13, A7 and B14 are 30/30**, the first setups with 30/30, at a median contact of ~22–29 s.
- **v2 / v3 likelihood with randomized probe sweeps (80 episodes, 3200 probes):** on held-out sweeps, logs + sweeps
  gives log p(true) −4.5 vs hand −18.9 (P within 2 mm 0.93–0.96). The logs-only model overfits the strategy probes:
  its sweep loss rises 2.02 → 2.35 during training.
- **Held-out R (random offsets, not D1), reps 1–2, 80/80 runs succeeded:** for plain probe, v3 halves contact time (44.0 → 20.6 s,
  8.0 → 2.9 probes per run). For GPU entry, v3 gives **no gain** (46.7 → 43.8 s, 6.7 vs 6.8 probes). Mechanism: on a
  wide belief the MPC drives toward a wrong estimate and pulls the peg out of the chamfer. New option `--gpu-std-max`
  (off by default); it is being tested on a fresh set R2 (`queue_heldout_r2.sh`).
- **Queued:** a cross-task regression (`queue_cross_task.sh`, 64 runs, ~3 h): H1/H2/H3 base vs new flags. All of
  this week's fixes were tuned on the peg only. Infra: the nightly update script now allows the Skill tool.

### 2026-10-26 (afternoon): cross-task regression queued
- Gap: every fix since the weekend (calibration touch, height tracking, centering bound, stuck dither, learned belief)
  was developed and benchmarked on the peg only (D1, R, R2). The held-out tasks H1 / H2 / H3 last ran with the old code.
- `scripts/queue_cross_task.sh` (starts after "r2 queue done", `~/ci_mpc_runs/weekend/X`): H1 (5 conditions × 3,
  same seeds), H2 nominal (5, seed 21), H3 (12 assemblies, same offsets), arms base (`--max-probes 25`) vs new
  (+ height / centering bound / dither, + v3 belief on H1 only: v3 is peg-trained, and `--geo` tasks keep the hand
  belief and skip the calibration touch). 64 runs, about 3 h.

### 2026-10-26 (night → morning): combined candidate, v2 / v3 learned likelihood, held-out test
- **D1 benchmark, all with the calibration touch** (3 × 10 offsets; contact time = successes, mean / median):

  | setup | success | contact | misses |
  |---|---|---|---|
  | A2 plain probe, hand model | 29/30 | 58.1 / 55.7 s | −5,0 |
  | A3 + height tracking | 29/30 | 61.6 / 58.2 s | −5,0 |
  | A5 + learned v1 + distance correction | 29/30 | 27.0 / 22.0 s | 3,3 (centering left the block) |
  | A6 + learned v1-rcorr + tracking + centering bound | 29/30 | 31.4 / 30.9 s | −5,0 (belief collapse, below) |
  | **A7 as A6 with v2 (logs + 43 sweep episodes)** | **30/30** | 40.0 / 21.8 s | – |
  | B8 GPU entry + lateral centering, hand | 28/30 | 58.1 / 48.5 s | ±5,0 |
  | B10 + height tracking | 29/30 | 59.8 / 55.4 s | 5,0 |
  | B12 + learned v1-rcorr | 29/30 | 33.1 / 28.4 s | −5,0 (drift) |
  | **B13 + learned v1-rcorr + tracking + bound** | **30/30** | 34.6 / 27.2 s | – |
  | **B14 as B13 with v2** | **30/30** | 36.8 / 28.9 s | – |

  - The learned likelihood (with the distance correction) halves the median contact time at equal or better success;
    the combined setups are the first with 30/30. A7's mean is pulled up by a few long runs (max 121 s; median 21.8 s).
  - Caveat: D1 is the set everything was tuned on → held-out test below.
- **v3 likelihood (logs + all 80 sweep episodes, 3200 probes)**, evaluated on 16 held-out sweep episodes (640 random
  probes): log p(true) hand −18.9 / logs-only −9.2 (its sweep loss rises 2.02 → 2.35 while training: overfits the
  strategy probes) / sweeps-only −4.2 / **logs + sweeps −4.5 with the best P(within 2 mm) 0.93–0.96**; at 2× sensor errors
  hand −20.8 vs −4.6. Final localization error ties (0.6–0.9 mm: 40 random probes include sinks); calibration is the
  discriminating metric. Model `models/probe_likelihood_v3_both.npz`.
- **New option `--stuck-dither`** (off by default) for the A6 rep3 collapse: after 3 probes within 0.4 mm, the next target
  is offset in a random direction.
- **Held-out R, reps 1–2 (all 80 runs succeeded):** plain probe hand 44.0 s vs **v3 20.6 s** contact (probes / run 8.0 →
  2.9); GPU entry hand 46.7 s vs v3 43.8 s (probes / run 6.7 vs 6.8: no gain). GPU phases end in the hole 42 % (hand) /
  71 % (v3), so they are not the bottleneck. **Mechanism:** when the first probe sinks into the chamfer, plain probe
  centers (follows the physical push) and slides in (8 / 20 runs in 1 probe), while the GPU entry hands over to the MPC,
  which drives toward the *belief estimate*; with a wide belief (std ~3 mm) that estimate is wrong and the MPC pulls the
  peg out of the chamfer (RB_v3 rep1 0.74,1.19: from (0.2, −0.2) to (0.77, −0.75), hole at (−0.7, −1.2)).
  - Option `--gpu-std-max` (off by default): GPU phase only when the belief std is below the threshold, else center.
    Motivated by an R run, so tested on a fresh set R2 (seed 78): `scripts/queue_heldout_r2.sh` (plain probe + v3,
    GPU + v3, GPU + v3 + gate 1 mm), after the R queue.
- **Running:** `scripts/queue_heldout_r.sh` — 10 random hole errors (seed 77, r 1–5.4 mm, not D1): RA / RB = plain probe /
  GPU entry + lateral centering, hand vs v3 + stuck dither (0.8 mm), all with tracking + bound; 3 repeats. Plus 40 more
  sweep episodes (`~/ci_mpc_runs/likelihood/sweeps2`, seeds 5000+, 1 worker).

### 2026-10-25: daily summary
- **Power-cut restart:** the B2 → B3 → H3_fix chain was restarted at 08:46. The drivers did not create `$OUT`, so with
  the directory missing they reported "done" at once and released the next job; fixed with `mkdir -p`. New live
  dashboard `scripts/watch_runs.sh`. Today's Isaac results are logged under "2026-10-24 (midday): sample, then
  QP-refine" below.
- **Grasp-height calibration bug (affected every Factory-peg probe run):** probe 0 skipped calibration on > 1.5 N
  lateral force, and then the belief froze. B2 (GPU entry) 22/30: 15/16 calibrated vs 4/11 not. Fix: a dedicated
  calibration touch on the top face (v2 at 9.5 mm; 14.5 mm went off the 12.5 mm half-wide face). Success now also needs
  xy < 1.5 mm, because one "success" ended 20 mm off. **Plain probe A2 29/30 (was 24/30).**
- **GPU entries vs plain probe (Isaac, 3 × 10 offsets):** B4 GPU entry 22/30, B5 refine 16/30 (negative). Cause: flat-face
  pushes > 1.2 N counted as engagement and started GPU phases, which collapsed the belief near the start. With sink-only
  engagement (B6/B7): 26/29 and 26/30, 24/24 inside ±5 mm, but only 2/5–2/6 at ±5 mm, because the push cue was lost.
  With the push used for centering (B8/B9): **28/30 and 27/30** (±5 mm 4/6 each). This ties plain probe within noise but
  does not beat it.
- **H3 rerun (AutoMate, entry-plane fix):** spiral 7/12, probe 4/12, GPU 4/12 (run 1: 6/5/5). 00700 is fixed, but 7 of
  36 cells flipped with the same code, so the strategy differences on H3 are noise.
- **Found in B8/B9:** the grasp height drifts ~0.005–0.011 mm per probe (peg creeping in the fingers), which can make the
  flat face read as sunk → `--track-height N` (A3/B10, running). Centering also has no travel bound and pushed the
  peg off the block once (B9 0,4); a bound is a to-do.
- **Learned probe likelihood (sensor-error randomized MLP, 3332 probes):** offline median belief error 2.83 → 1.96 mm
  (P within 2 mm 0.33 → 0.49), and the best calibration at 2× sensor errors. The evaluation is off-policy; closed loop
  (A4/B11) is queued. Randomized probe sweeps (`--collect`, `collect_probes.sh`) are running.
- **Infra:** 3 Isaac instances + 3 GPU servers + 2 collection workers filled RAM and swap (Isaac ~6 GB RSS each). One
  collection worker was stopped; limit is at most 4 Isaac instances. The colcon install of `ci_mpc_planner` is
  older than the source (rebuild after the queues).

### 2026-10-25 (evening): learned probe likelihood for the hole belief (idea #1, user request)
- **Why:** the remaining failures are at ±5 mm, search budget exhausted with a wide belief; the push that walks plain
  probe to the hole is ignored by the belief (`f_lateral=1e9`), and its hard thresholds sit at the force estimate's
  error level. The FR3 estimates the EE force from link-side joint torque sensors (model + Jacobian), which the sim
  already reproduces (momentum observer on Isaac's link-side torques, noise 0.05 Nm, offsets ≤ 0.15 Nm).
- **Data:** `scripts/probe_dataset.py` → 3873 probes from 321 Factory-peg runs (true hole known); training uses the 3332
  from calibrated runs. `~/ci_mpc_runs/likelihood/probes.npz`.
- **Model** (`ci_mpc_planner/probe_likelihood.py`, `scripts/train_probe_likelihood.py`): MLP on rotation-invariant
  features (hole–TCP distance, lateral force toward / across the hypothesis, Fz, tip height), trained contrastively
  (true hole vs 16 grid hypotheses; logit = log-likelihood ratio). **Sensor-error randomization** per sample: force
  gain σ 10 %, cross-axis σ 5 %, bias drift after the tare σ 0.3 N, noise σ 0.2 N; tip bias σ 0.15 mm, noise 0.03 mm
  (assumed until measured on the FR3). NumPy inference; models in `ci_mpc_planner/models/`.
- **Offline replay, search probes only (51 held-out runs):** median belief error / P(within 2 mm) / log p(true cell)

  | sensor errors | hand model (deployed) | learned, no rand | learned, randomized |
  |---|---|---|---|
  | 0× | 2.83 mm / 0.33 / −5.6 | 1.71 mm / 0.51 / −5.4 | 1.96 mm / 0.49 / −5.2 |
  | 1× | 3.09 mm / 0.32 / −5.6 | 1.53 mm / 0.48 / −5.6 | 1.99 mm / 0.48 / −5.3 |
  | 2× | 3.83 mm / 0.26 / −10.9 | 2.69 mm / 0.42 / −6.8 | 2.51 mm / 0.41 / −6.1 |

  - Learned beats the hand model everywhere (~1 mm better, mass near the truth 0.33 → ~0.5); the hand model becomes
    confidently wrong at 2× errors. Randomization trades ~0.3–0.5 mm at low error for the best calibration at 2×
    (one seed, 51 runs: small differences). The randomized model is the deployment candidate.
  - Caveat: off-policy (logged probe positions). Closed loop is the real test.
- **Isaac:** `peg_insert_demo.py --belief-model PATH [--belief-scale s]` (loads from the source tree; replaces the
  installed belief's update). Queued `scripts/queue_learned_belief.sh` after A3/B10: A4 (plain probe, vs A2) and B11
  (B8 + learned belief, vs B8).
- **A5 final (plain probe, learned v1 + distance correction): 29/30 at less than half the contact time.**

  | plain probe | success | contact time, successes: mean / median |
  |---|---|---|
  | A2 hand model | 29/30 | 58.1 / 55.7 s |
  | A3 hand + height tracking | 29/30 | 61.6 / 58.2 s |
  | A4 learned v1 | 27/30 | 35.6 / 28.5 s |
  | **A5 learned v1 + correction** | **29/30** | **27.0 / 22.0 s** |

  - First learned component that wins in closed loop: same success, ~2.5× faster localization. A5's one miss is the
    centering run-off (no bound in A5), not the belief.
- **B12 final (GPU entry + lateral centering, learned v1 + correction): 29/30 at 33.1 s mean contact** (successes;
  B8 28/30 at 58.1 s, B10 29/30 at 59.8 s, B11 26/30 at 29.8 s). One miss, −5,0 (grasp drift → mirror-spot lock;
  B12 has no height tracking).
- **A6 final (combined: learned v1 + correction + height tracking + centering bound):** see the totals in the next
  entry; repeats 1–2 10/10 each. The rep3 miss (−5,0) is a new mode: the belief locked on at std 0.00 mm, 0.7 mm off
  the true hole, with the peg resting 1.2 mm down the chamfer edge; every probe / centering step returned to that spot
  (next probe = belief peak; the 1.2 N centering push could not beat stiction) until time ran out. Mechanism: 13
  near-identical readings counted as independent evidence (the correlated-measurement problem again, now through the
  sharper learned likelihood). To do: skip / temper repeated probes at the same spot, a belief floor (~0.3–0.5 mm
  diffusion), or a "stuck at the same depth" detector → small local spiral.
- **v2 likelihood on the first probe sweeps (43 episodes, 1720 probes; worker 0 done, worker 1 resumed):** trained on
  logs / sweeps / both, all evaluated on the same 8 held-out sweep episodes (320 random probes, varied physics and
  sensors; `train_probe_likelihood.py --train-filter / --val-filter`, distance correction now added automatically):

  | trained on | val loss on sweeps | P(within 2 mm) | log p(true) | log p at 2× errors |
  |---|---|---|---|---|
  | hand model | – | 0.83 | −13.6 | −16.2 |
  | logs only | rises 1.37 → 1.61 | 0.75 | −10.2 | −9.5 |
  | sweeps only | 1.06 | 0.55 | −5.9 | −5.0 |
  | **logs + sweeps** | 1.07 | **0.94** | **−5.5** | **−4.5** |

  - The log-trained model overfits the strategies' probe distribution (its sweep loss grows with training): the
    off-policy problem measured directly. The hand model is often confidently wrong on random probes.
  - Small test set (8 episodes); retrain when the sweeps are complete. Model `models/probe_likelihood_v2_both_partial.npz`;
    queued `scripts/queue_v2_belief.sh` after A6/B13: A7 / B14 = A6 / B13 with the v2 model.
- **A5 / B12 so far (learned + distance-prior correction):** A5 10/10, 9/10 (miss: centering left the block, 3rd time);
  B12 9/10, 10/10 (miss −5,0: belief locked on the mirror spot (−5.2, −0.2) after grasp drift made flat probes read
  −0.5 mm; B12 has no height tracking and the learned model leans on tip height).
  - **Contact time ~2–3× shorter:** A5 32.5 / ~20 s per repeat vs A2 51.7 / 66.2 s, A3 63.4 s; B12 ~34 s (successes) vs
    B8 58.1 s, B10 59.8 s. The correction turns the learned belief from a loss (A4 27/30, B11 26/30) into a speed gain.
  - Centering bound now required (cost one run each in B9, A4, A5). Calibration-touch gate widened 1.5 → 2.5 mm (sweep
    episode w0_e035 read +1.56 mm with a deep grasp + tilt and fell back to probe 0).
  - Queued `scripts/queue_combined.sh` after A5/B12: A6 / B13 = learned (rcorr) + `--track-height 5` +
    `--centering-bound 0.008` (combined deployment candidate).
- **A4 / B11 final (learned likelihood v1, no distance correction):** A4 plain probe **27/30** (9 / 9 / 9; vs A2 / A3
  29/30), B11 GPU entry + lateral centering **26/30** (8 / 9 / 9; vs B8 28/30, B10 29/30).
  - **Negative in closed loop despite the offline gain** (belief error 2.83 → 1.96 mm in replay). B11's misses are at
    moderate offsets every centering setup had passed (−3,0, −4,2, −3,−2) and a −5,0 that ended 11 mm off: the belief
    walks away from the hole and becomes confidently wrong (B11 rep1 −3,0: std 0.69 mm at (−3.9, 5.4), hole at (3, 0)).
  - Mechanism: the distance prior of the strategy-chosen training probes (above); the replay shares it. A5 / B12 test
    the corrected model.
- **A4 rep1 (learned v1): 9/10, ±5 mm both OK; the miss shows the off-policy bias (user's point) in closed loop.** On 0,4
  the belief walked to +1.5 mm y while the hole was at −4, then a centering push left the block (TCP 13 mm, dropped 3.6
  mm, counted as "in the hole"; caught by the xy check).
  - Not the force bias: a constant per-run lateral bias up to 1 N barely moves the offline replay (2.00 vs 1.96 mm);
    `--force-debias` (median of recent flat lateral readings) even worsens it (2.5 mm). Kept as an option, off.
  - **Mechanism:** the contrastive logit is log p(meas | rel) + log p_pos(r) / p_neg(r). Training positives sit where the
    strategies probed (r histogram peaks at 4–5 mm), so a flat reading favoured holes ~4.5 mm away and fell off beyond
    (−1.1 at 8 mm, −2.6 at 12 mm): every probe pulls mass onto a ring around itself, the belief chases its own probes.
    The replay shares the bias, so it could not show this.
  - **Fix v1-rcorr** (`scripts/rprior_correction.py`, table in the model file, applied by `ProbeLikelihood`): subtract the
    smoothed histogram log ratio. Flat reading now flat beyond ~3 mm (−0.1 to −0.6). Side finding: a 1 N push toward the
    hole barely changes the likelihood: the lateral signal (0.17 N mean near the hole vs 0.6–0.9 N noise) is too weak in
    this data; the offline gain is the tip-height model. The sweeps sample chamfer contacts on purpose.
  - `--centering-bound` (stop centering / reject an "in the hole" drop beyond N m from the estimate) added, off.
  - Queued `scripts/queue_rcorr_belief.sh` after A4/B11: A5 / B12 = A4 / B11 + the correction only.
- **Better data than replayed strategy probes (user):** logged probes sit where the benchmark strategies went (75 %
  at 2–6 mm, 6 % within 1 mm), so training and the replay evaluation share that bias, and a closed-loop learned
  belief will probe elsewhere. Plan: (A) randomized probe sweeps, (B) on-policy rounds with the learned belief
  (only the likelihood is learned, labels exact: unlike the failed selector DAgger), (C) evaluation on held-out sweeps +
  closed loop at random offsets, later a small real-robot sweep on a fixture.
  - (A) `peg_insert_demo.py --collect N` (one grasp, calibration touch, N probes sampled around the true hole: r 0–9 mm,
    half in the 2–6 mm chamfer band; random hold depth 0.5–2 mm and settle 0.2–0.6 s; no insertion) and
    `scripts/collect_probes.sh` (per episode: friction U(0.5, 1), hole tilt 0–1.5° in 1/3, grasp dx/dz ±1 mm, and the
    controller's torque-sensor model: noise U(0.02, 0.12) Nm, joint offsets U(±0.3) Nm, observer gain U(20, 40), via a
    per-episode controller YAML). `demo_peg.sh` now honours `CONTROLLER_CONFIG`. No clearance knob (fixed USD asset).
  - Running: 2 workers × 40 episodes × 40 probes on cores 0–15 (~3200 probes, ~6–8 min per episode),
    `~/ci_mpc_runs/likelihood/sweeps`.
  - **Memory incident 21:53:** with B10's 3 Isaac instances + 3 GPU servers, the 2 collection workers pushed RAM to
    2 GB available and swap full (each Isaac instance ~6 GB RSS, not 3–4). Collection worker 1 stopped by PID (its
    episode reruns on restart: the script skips episodes with a res.json); worker 0 continues. Restart worker 1 when
    the benchmark queues are done (`WORKERS=2 scripts/collect_probes.sh ~/ci_mpc_runs/likelihood/sweeps 40`).
- **Centering can push the peg off the block (B9 rep3 0,4):** a misread push drove centering 9.6 mm to y = 14.2 mm, past the
  face edge (12.5 mm); the peg dropped 4.3 mm, counted as "in the hole" and was pushed down beside the block (caught by
  the new xy success check). Centering has no travel bound (60 × 0.4 mm). 41 of 825 logged centering moves went > 4 mm,
  some of them successful (B8 rep2 −5,0: 9.9 mm), so not a hard cap. To do: stop centering (and reject an "in the hole"
  drop) when the TCP leaves ~8 mm around the estimate (the belief grid is ±6 mm).
- **Found:** the colcon install of `ci_mpc_planner` is from 2026-10-23 and older than the source (`peg_hole_3d.py`
  differs, newer modules missing). The demo imports the install; not rebuilt mid-benchmark. To do: rebuild after the
  queues and check the D1 numbers do not move.

### 2026-10-25 (morning): power cut, B2 -> B3 -> H3_fix chain restarted
- The machine lost power at ~15:18 on 2026-10-24, during B2 repeat 1 (6 of 10 offsets started, 1 finished). B3 and
  the H3 rerun had not started. The partial B2 is kept in `~/ci_mpc_runs/weekend/B2_interrupted_20261024`.
- Restarted 08:46 with the same scripts: `rerun_track_b.sh` (B2, 3 repeats), then `track_b3_refine.sh` (B3), then
  `run_h3_fix.sh` (H3 with the entry-plane fix).
- **Restart bug fixed:** the drivers did not create `$OUT`. With the directory missing, every repeat failed at once
  and the driver still wrote "B2 done" / "B3 done", which released the next job in the chain. Both drivers now run
  `mkdir -p "$OUT"`. The early H3 waiter was killed before it started.
- **B2 result (probe + GPU entry, belief fix, no calibration fix): 22/30 (73 %)**, per repeat 6 / 7 / 9 of 10.
  Before the belief fix: 4/10 (B repeat 1). Plain probe (same calibration bug): 24/30.

  | B2, 0,0 excluded | success |
  |---|---|
  | grasp height calibrated on probe 0 | **15/16 (94 %)** |
  | not calibrated | **4/11 (36 %)** |

  - 7 of B2's 8 failures are uncalibrated runs; all failures but one end 1.9–9.9 mm off with the belief frozen.
  - So the belief fix works, and the GPU entry is about level with plain probe on equal footing (both ~93 % when
    calibrated). The calibration bug dominates both; A2/B4/B5 measure them with it fixed.
- **Grasp-height calibration bug (found 2026-10-25 in B2), affects every probe run on the Factory peg.**
  - Probe 0 calibrates the grasp height only if its lateral force is < 1.5 N. On many runs the press reads 1.5–2.6 N
    laterally on the flat face, the calibration is skipped, every hypothesis shows a gap, and the belief stops moving
    (B2 rep2 3,0: belief frozen at [2.08, −1.08] mm, std 5.55, for probes 5–19; ended 7.4 mm off).
  - Success vs calibration (0,0 excluded): plain probe A **14/15 calibrated vs 7/12 not** (5 of its 6 failures);
    B2 so far 5/6 vs 1/5.
  - **Fix (staged as `peg_insert_demo.next.py`, swapped in after B2):** a dedicated calibration touch on the top face
    14.5 mm from the estimate along a diagonal (hole R + chamfer + peg half width + 5 mm error; block 20 mm half
    wide), before probing. `--calib-touch` (default 0.0145; off with `--geo`, so gear / AutoMate are unchanged).
  - **Calibration touch, first try (14.5 mm) failed the gate (2/3):** the Factory hole's top face is a 25 mm square
  (12.5 mm half width) on a 40 mm base plate; the 20 mm I assumed was the plate. On −3,−2 the touch went past the
  face edge (tip −8.3 mm, no force) and the peg later slid down the block's side. That run also scored "success"
  20 mm off: success only checked depth.
  - **v2:** touch at 9.5 mm on the diagonal. The peg is held upright, so its tip sits at face height whenever part of
    its footprint is on the flat face (centre 2–16 mm from the hole); 9.5 mm keeps that for errors up to 5.4 mm.
    No lateral-force gate on the touch; probe 0 calibration stays as the fallback if the touch fails.
  - Success now also needs xy error < 1.5 mm. Of 119 earlier successes, none is affected (only this one, 20 mm).
- New queue `scripts/queue_calib_fix.sh` replaces B3 and the H3 waiter: gate on 3 offsets (abort unless 3/3
    calibrate), then A2 (plain probe), B4 (GPU entry), B5 (refine −1) interleaved per repeat, 3 repeats, then H3_fix.
  - The 24/30 baseline and B2 both carry this bug; compare B4/B5 against A2, not against 24/30.

### 2026-10-24 (midday): sample, then QP-refine (user request)
- **A2 rep1 (plain probe, calibration touch): 10/10**, every run calibrated (old A: 8/10 best repeat).
- **B4 rep1 early: ±5 mm both fail although calibrated (A2: both OK).** Mechanism: a lateral force > 1.2 N on the flat
  face counts as "engaged" (~22 % of flat probes in A2 and B4). Plain probe then only centers; the GPU entry runs a
  full MPC phase and its one belief update narrows the belief (std 1.75 mm) around the start, so the search never
  reaches the hole 5 mm away (A2's belief stays ~5.3 mm wide and walks out to it).
  - Option `--engage-lateral` (default 1.2 N, unchanged). Queued `scripts/queue_engage_sink.sh`: B6 (GPU) and B7
    (refine) with `--engage-lateral inf` (tip sink only), after H3_fix; compare with B4 / B5.
- **A3 / B10 final (grasp-height tracking, `--track-height 5`):** A3 plain probe **29/30** (9 / 10 / 10; miss −5,0 out of
  probes, like A2), B10 GPU entry + lateral centering **29/30** (10 / 9 / 10; miss 5,0, belief flat, time out).
  - Plain probe: no change (A2 29/30). GPU entry: B8 28/30 → B10 29/30, ±5 mm 4/6 → 5/6: within noise, but the one
    drift-caused B8 failure (−5,0, flat face read as sunk) did not recur. Tracking is cheap and does not hurt: keep it on.
  - Every remaining miss at ±5 mm is search budget (25 probes / 150 s) with a wide belief: the target for the learned
    likelihood (A4/B11) and the probe-sweep model.
- **B8 / B9 final (`--lateral-centering`: a push centers, only a sink starts the GPU phase):** B8 GPU entry **28/30**
  (8 / 10 / 10; ±5 mm 4/6), B9 refine **27/30** (9 / 9 / 9; ±5 mm 4/6, plus 0,4 pushed off the block by centering).

  | method (calibration touch) | success | inside ±5 mm | ±5 mm |
  |---|---|---|---|
  | A2 plain probe | 29/30 | 24/24 | 5/6 |
  | B4 / B5 push → GPU | 22 / 16 of 30 | 22 / 15 of 24 | 0 / 1 of 6 |
  | B6 / B7 push ignored | 26/29 / 26/30 | 24 / 24 | 2/5 / 2/6 |
  | **B8 / B9 push → centering** | **28/30 / 27/30** | 24 / 23 | **4/6 / 4/6** |

  - Using the push for centering closes most of the gap to plain probe; the GPU entries now tie it within noise
    (Wilson intervals overlap widely), but do not beat it. The open ±5 mm misses are search budget and grasp drift.
- **Grasp-height drift (found in B8 rep1 −5,0):** the flat-face tip reading creeps down ~0.005–0.011 mm per probe
  (median per method), likely the peg slipping in the fingers under the ~5 N presses. On long searches it reaches the
  −0.3 mm sink threshold (last probes ≤ −0.3 mm: A2 1/15, B6 4/17, B7 5/18 long runs); in B8 −5,0 the flat face then
  read as sunk and started GPU phases at probes 23–24, far from the hole.
  - Option `--track-height N` (off by default): the grasp offset becomes the median raw tip height of the last N
    flat-face probes (seeded with the calibration), so sink checks and belief updates follow the creep.
  - Queued `scripts/queue_track_height.sh` after B8/B9: A3 (plain probe, vs A2) and B10 (B8 + tracking), N = 5.
- **B6 / B7 final (sink-only engagement, `--engage-lateral inf`):** B6 GPU entry **26/29** (one setup failure
  excluded: B6 rep2 5,0, the arm never moved, TCP 310 mm off after 12 s; the only such run today), B7 refine **26/30**.

  | method (calibration touch) | success | inside ±5 mm (24 runs) | ±5 mm |
  |---|---|---|---|
  | A2 plain probe | **29/30** | 24/24 | 5/6 |
  | B4 GPU entry, push → GPU | 22/30 | 22/24 | 0/6 |
  | B5 refine, push → GPU | 16/30 | 15/24 | 1/6 |
  | B6 GPU entry, sink only | 26/29 | 24/24 | 2/5 |
  | B7 refine, sink only | 26/30 | 24/24 | 2/6 |

  - **Removing push → GPU fixes everything inside ±5 mm** (both entries 24/24, refine from 15/24). Refine's B5 deficit was
    entirely this trigger.
  - **±5 mm stays weak (2/5–2/6 vs plain probe 5/6):** ignoring the push loses plain probe's centering cue; the runs end
    out of probes (B6) or out of time (B7 rep2) with the belief wide. B8/B9 (`--lateral-centering`) test using it.
- **B6 rep1 (sink-only engagement): ±5 mm still fail, a different way.** No false GPU phases (0–1 per run) and only
  92–96 s used, but all 25 probes spent with the belief wide (std ~5.9 mm); −5,0 found the hole on probe 24.
  - Mechanism: the lateral push is the cue. In plain probe it triggers centering, which follows the chamfer push toward
    the hole; B4 turned it into a GPU phase (harmful), B6 ignores it (lost).
  - Option `--lateral-centering` (off by default): with `--entry gpu`, a push without tip sink centers as plain probe
    does; only a sink starts the GPU phase. Queued `scripts/queue_lateral_centering.sh`: B8 (GPU) and B9 (refine),
    after B6/B7.
- **H3 rerun (entry-plane fix, same 12 assemblies and offsets): spiral 7/12, probe 4/12, probe + GPU 4/12**
  (run 1: 6 / 5 / 5). `~/ci_mpc_runs/weekend/H3_fix.log`.
  - 00700, the setup artifact, now works: spiral and GPU succeed (probe still fails 3.1 mm short).
  - **Outcomes are not reproducible run to run:** 9 of 36 cells flipped, only 2 explained by the fix (00700). The 7
    others (e.g. 00021 probe, 00028 / 00499 / 00652 GPU, 00301 / 00319 spiral) flipped with the same assembly, offset and
    code. With 12 assemblies × 1 run, the H3 strategy differences (±1–3 of 12) are within this noise.
  - The calibration touch is off here (`--geo`); probe 0 calibrated in 11/12 probe runs (+0.56 to +0.80 mm). No run was
    rejected by the new xy < 1.5 mm check.
  - Conclusion stays: on non-round AutoMate parts the assembly decides, not the strategy; spiral is at least as good.
- **A2 / B4 / B5 final (calibration touch, 3 × 10 offsets, interleaved):**

  | method | success | Wilson 95 % | ±5 mm | 3,3 / −4,2 / 0,4 |
  |---|---|---|---|---|
  | A2 plain probe | **29/30** | 83–99 % | 5/6 | 9/9 |
  | B4 GPU entry | 22/30 | 56–86 % | 0/6 | 7/9 |
  | B5 refine −1 | 16/30 | 36–70 % | 1/6 | 1/9 |
  | (old A, uncalibrated) | 24/30 | 63–90 % | – | – |

  - **The calibration touch is the win: plain probe 24/30 → 29/30.** Its one miss (−5,0) ran out of probes/time with a
    wide belief (search speed, not belief error).
  - **GPU entries are worse than plain probe in Isaac**, B5 worse than B4. Failures are the far offsets; mechanism (see
    above): flat-face lateral pushes start GPU phases, the per-phase belief update collapses the belief near the start,
    the 150 s budget runs out. Refine spends more time per phase. B6/B7 (sink-only engagement) test this next.
- **Repeat 1 with the calibration touch (all runs calibrated):** A2 plain probe **10/10**, B4 GPU entry **7/10**
  (fails ±5,0 and 3,3), B5 refine −1 **5/10** (fails ±5,0, 0,4, 3,3, −4,2).
  - Every GPU-entry failure checked shows the false-engagement pattern: 3–9 GPU phases per run, mostly started on the
    flat face by > 1.2 N lateral force; the belief collapses (std 0.87–1.75 mm) near the start; the run hits the
    150 s budget. Refine is worse than the sampler because each phase is slower and it runs more of them.
  - The offline refine gain (30/30, p90 3.7 N) does not carry over to Isaac while flat-face pushes count as
    engagement. B6/B7 test sink-only engagement.
- **Idea:** the GPU sampler picks the mode (best sampled sequence); one 3D C3 solve warm-started from it refines
  the commands inside that mode.
  - Code: `ci_mpc_planner/refine_mpc.py` (`SampleRefineMPC`), `sampled_teacher_eval.py --policy refine
    --refine-margin m`.
  - The refined plan is used only if its score in the sampler's own rollout is ≥ the sample's + m.
- **Fair 30 starts, margin 0:** 30/30, steps median 14, peak median 2.56 N, **p90 5.36 N**, max 6.29 N; C3 used on
  8 % of steps. The sampler alone: 30/30, 14, 2.38 / 5.36 / 7.55 N.
  - **No change in the tail.** The gate makes C3 beat the sample on the cost the sampler already maximizes, so it
    rarely wins. That cost's force term did not move the tail before either (w_force 2.0: p90 5.42 N).
  - Lower margins let C3's own (force-regularized) objective decide more often.
- **Margin sweep** (fair 30 starts, sampler seed = start seed; `~/ci_mpc_runs/refine/fair_m*`):

  | margin | entered | steps median | peak median | **p90** | max | C3 used |
  |---|---|---|---|---|---|---|
  | sampler only | 30/30 | 14 | 2.38 N | 5.36 N | 7.55 N | 0 |
  | 0 | 30/30 | 14 | 2.56 N | 5.36 N | 6.29 N | 0.09 |
  | −0.25 | 30/30 | 13 | 2.48 N | 3.97 N | 6.44 N | 0.27 |
  | −0.5 | 30/30 | 15 | 2.04 N | 3.81 N | 5.77 N | 0.51 |
  | **−1** | **30/30** | 14 | **2.01 N** | **3.71 N** | 6.42 N | 0.78 |
  | −2 | 23/30 | 26 | 1.36 N | 3.46 N | 5.36 N | 0.97 |
  | always C3 | 4/30 | 34 | 2.10 N | 6.37 N | 11.93 N | 1.00 |

  - **First policy with 30/30 and p90 under the 4 N gate.** The non-monotone optimum is at −0.5 to −1.
  - **Mechanism:** a division of labour. The sampler picks the mode, which prevents C3's mode failures (C3 alone
    enters 4/30). C3's force-regularized QP smooths the commands inside that mode, cutting the force tail.
  - Below −1, C3 overrides even good samples and gets stuck again (−2: 23/30).
  - **The max is the start transient (verified):** the run reproduces exactly (p90 3.71 N), and the two largest
    peaks (6.42 / 5.65 N) both come at step 1 (10/30 episodes peak there).
    - Likely mechanism: the eval's engaging press preloads ~5 N (4 mm × 500 N/m + 3 N ff) before the policy
      acts, so step 1 reads the eval setup, not policy behaviour.
    - The largest mid-episode peak is 4.31 N (step 5). The eval now logs the peak step.
- **Sampler-seed check** (same 30 starts; sampler seed offsets 0 / 1 / 2; all 30/30):

  | policy | p90 per seed | mean p90 | mean peak median | steps median |
  |---|---|---|---|---|
  | sampler only | 5.36 / 5.38 / 4.71 N | 5.15 N | 2.30 N | 13–14 |
  | refine −0.5 | 3.81 / 4.21 / 3.87 N | 3.96 N | 2.09 N | 14–15 |
  | **refine −1** | **3.71 / 3.32 / 3.60 N** | **3.54 N** | **1.89 N** | 14–15 |

  Margin −1 is chosen: p90 is 31 % lower on every seed and always under 4 N, for about one extra step.
  Next: H4 (held out) with margin −1, then Isaac (the GPU server needs the refiner).
- **H4 (held out) for refine −1** (one sampler seed; `~/ci_mpc_runs/weekend/H4_refine/summary.txt`):

  | condition | heuristic | sampler | refine | p90 heur / sampler / refine | peak median sampler → refine |
  |---|---|---|---|---|---|
  | mu 0.5 | 30/30 | 30/30 | 30/30 | 2.80 / 2.75 / **2.28** N | 2.47 → **1.46** N |
  | mu 1.0 | 10/30 | 30/30 | 30/30 | 6.51 / 6.57 / **5.03** N | 3.90 → **2.13** N |
  | R 4.4 | 7/30 | 30/30 | 30/30 | 4.94 / 5.14 / 5.01 N | 2.29 → **1.89** N |
  | R 4.6 | 28/30 | 30/30 | 30/30 | 5.47 / **3.74** / 5.07 N | 2.14 → **1.86** N |
  | stiction ×0.5 | 30/30 | 30/30 | 30/30 | 3.01 / 3.79 / 3.95 N | 2.98 → 3.31 N |
  | stiction ×1.5 | 0/30 | 0/30 | 0/30 | no contact (degenerate) | – |

  - **Typical force transfers, the tail does not.** Success is unchanged and the median peak is lower in 4/5
    conditions (up to −45 %).
  - The held-out p90 averages 4.27 N for refine vs 4.40 N for the sampler (dev set: −31 %). Refine is better in 2,
    equal in 1, worse in 2.
  - Likely reasons: the margin was chosen on the dev starts (selection), and one sampler seed moves p90 by about
    ±0.6 N, so held-out p90 differences are mostly within noise.
  - The honest claim: refine lowers typical contact force robustly; the force tail is unproven.
- **ICNN cost-to-go (user's idea 2): negative** (`ci_mpc_planner/icnn.py`, `scripts/icnn_data.py`,
  `scripts/train_icnn.py`).
  - Setup: a PICNN convex in a continuous version of the sampler's candidate family (lateral step, press depth,
    tilt). Trained on 20000 engaged GPU-plant states × 64 actions, scored exactly like a sampler candidate.
  - Fit: validation MSE 0.010 (normalized); picking the lowest predicted cost among the 64 gives 0.14 mm regret
    (random action 0.45 mm).
  - **Alone:** 0/30, stuck 0.5–1 mm down the chamfer. Diagnosis at a stuck state:
    - the lateral direction is right (toward the hole);
    - the press is the box minimum (−1 mm, a lift), while the true best of 2048 actions is a ~4 mm press
      (true score −0.72 vs −0.28);
    - the true cost is non-convex in press depth (a light press jams and pays force; a deep press gains depth),
      so the convex fit puts its minimum on a box edge. Within-state correlation predicted vs true: 0.66–0.71.
  - **As the refiner behind the sampler** (box ±0.2 mm / ±1 mm press / ±5 mrad, margin −1): 28/30, p90 5.35 N,
    worse than the sampler alone.
    - Mechanism: it approximates the very score the sampler maximizes exactly, so a negative margin only lets
      it replace good samples with worse ones.
    - C3 refine works for a different reason: its own force-regularized, physics-constrained objective.
  - Conclusion: a learned convex surrogate of the same cost cannot beat exact sampling of it. Its only possible
    gain is speed, at the sampler's quality, which it does not reach.
- **H3 (held out, Isaac AutoMate; 12 assemblies × 1 random 1–4 mm error): spiral 6/12, probe 5/12, probe + GPU
  5/12; 0 grasp failures.**
  - All three succeed on 00016, 00021, 00499, 00652 and all fail on 00319, 00388, 00700, 01041.
  - Mixed: 00028 (GPU only), 00211 and 00301 (spiral only), 00638 (probe only).
  - **Setup bug found (00700 and, milder, all 12): run 1 does not count as a clean result.**
    - The scene's socket opening was the plug's seated bottom + the AutoMate disassembly distance. That lies 1.1–2.0
      mm above the socket's real top face around the hole, and 5.1 mm on 00700.
    - On 00700 the plug therefore "engaged" on the top face (−4.5 mm vs the opening), and no strategy searched.
      Its real entry depth is only 2.9 mm, below the demo's fixed 3 mm "in the hole" threshold.
    - Fix: the entry plane is now the socket's top face around the hole (`info()["entry"]`).
      `peg_insert_demo.py` uses `--engage` everywhere: still 3 mm by default; `demo_automate.sh` passes
      min(3 mm, entry / 2).
    - Dev check on 00015 (2,1 mm): contact on the top face (+0.6 mm), spiral search 22.8 s, seated 0.11 mm.
    - **H3 rerun queued after B3** (`~/ci_mpc_runs/weekend/H3_fix`); the table below is run 1.
  - Most other failures never entered (depth error ≈ the full travel). Their 4–9 mm xy errors are probably the
    plug falling over after release, so they say little about the search.
  - **Mechanism:** on non-round shapes the outcome depends mainly on the assembly, not the strategy. The
    round-peg contact model behind probe / GPU gives no edge, and the search has no yaw (as in H2).
  - Table: `~/ci_mpc_runs/weekend/H3/driver.log`.
- **H2 (held out, Isaac gear mesh) done:**

  | condition | spiral | probe | probe + GPU entry |
  |---|---|---|---|
  | nominal (5 random 1–4 mm errors) | 1/5 | 3/5 | 2/5 |
  | gear 1° off the meshing yaw | 0/3 | 0/3 | – |
  | gear 3° off | 0/3 | 0/3 | – |

  - **Main failure: tooth-on-tooth.** The gear is centred on the shaft axis (xy error ~0.25 mm) but stops 9–12 mm
    above the seat, the height where the 2° drop test jammed.
  - Even 1° fails, although it is inside the drop test's tolerance; the search motion probably adds rotation
    (gear yaw at the stall is not logged: unverified).
  - The rest are misses of the shaft at 3–4 mm errors.
  - **Mechanism:** our strategies find the shaft but never command rotation about it (the 5-D command has no
    yaw). A gear task needs a yaw search or compliance, e.g. a small yaw dither while pressing, or low rotational
    stiffness about z.
  - The GPU entry does not help here (2/5 vs 3/5). Its contact model is a round peg, so it has no notion of the
    teeth.
- **H1 (held out, Isaac; 5 conditions × 3 random 1–4 mm errors): probe 14/15, spiral 7/15.**
  - Probe per condition: friction 0.5 3/3, friction 1.0 2/3, grasp offset 3/3, hole elsewhere 3/3, hole tilt
    2° 3/3.
  - Spiral per condition: 2/3, 0/3, 1/3, 3/3, 1/3.
  - Probing generalizes to conditions it was never tuned on. Its only miss is at high friction, where the
    offline H4 heuristic also broke down.
- **Infra mistakes:**
  - 9 parallel PyTorch eval processes (~1.3 GB each) next to H1's 4 Isaac instances brought free memory down to
    1.0 GB. No OOM, but from now on at most 4 PyTorch processes run while Isaac is up.
  - A 7 s ICNN smoke test ran pinned to cores 0–1 (H1's half) by mistake.
  - argparse reads `--refine-margin -1e9` as a flag: use `--refine-margin=-1e9`.

### 2026-10-24 (midday): Track B (GPU MPC entry in Isaac) 4/10, a belief bug; AlphaNIST lever 2 negative
- **Track B repeat 1:** probe `--entry gpu` 4/10, vs plain probe 24/30. All 6 failures end 2.7–8.7 mm off.
  - **Mechanism (bug):** the GPU phase called `belief.update` on every MPC step (up to 80 per engagement).
    Correlated measurements taken while the peg moves counted as independent, so the belief collapsed to std
    0.02 mm on a wrong spot after the first engagement, and the MPC chased each false peak.
  - Plain probe updates once per probe, and its belief stays wide (std ~5.2 mm) until the peg drops.
  - Contributing: lateral force > 1.2 N counts as "engaged", so the GPU phase can start on the flat top face.
  - **Fix:** one update per engagement. Prepared and swapped in atomically when phase 3 starts, so repeats 2–3
    still run the code they started with. Clean rerun queued: `scripts/rerun_track_b.sh` (B2, 3 repeats) after
    H3. The offline 30/30 result is unaffected (no belief offline).
- **Lever 2, binary labels** (`scripts/relabel_binary.py`): same 3249 SGS states, labels "option, then the
  heuristic for up to 20 steps, enters 2 mm without exceeding 6 N"; 2722 kept.

  | labels | entered, seed 0 / 1 / 2 (of 30) | peak median |
  |---|---|---|
  | dense scores | 17 / 19 / 16 | 2.32–2.39 N |
  | binary, heuristic continuation | **3 / 4 / 4** | 0.69–2.17 N |

  - **Mechanism:** a label/deployment mismatch. ~8.5 of 10 options "enter" because the heuristic continuation does
    the work, and the force tie-break then prefers the lightest-contact option.
  - In closed loop no heuristic follows: the selector repeats that light option and never engages (median peak as
    low as 0.69 N).
- **Lever 2 follow-ups (same states, 3 seeds each):**

  | labels | entered (of 30) | peak median |
  |---|---|---|
  | binary, on-policy continuation (`sel_r0`) | 2 / 3 / 5 | 0.29–1.17 N |
  | the same, no force tie-break (pure success) | 2 / 7 / 8 | 0.29–2.24 N |

  - **Lever 2 is negative in all variants.** The tie-break is not the main cause; the label structure is.
  - A binary label says "this option once, then a fixed continuation, enters". About 8 of 10 options pass in most
    states, so the target is nearly uniform: it does not say which option to commit to step after step, while
    dense scores rank options by progress.
  - AlphaNIST's binary reward works because RL keeps re-estimating the closed-loop value under the current
    policy; one-step supervised labels are no analogue of that.
  - The selector line stays below the heuristic (best 19/30 vs 22/30). Sample-then-refine (above) is the stronger
    use of the same idea (choose the mode, then a convex solve).

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
