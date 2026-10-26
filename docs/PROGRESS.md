# Progress log

Running log of what has been done, what was found, and where to resume. Newest entry on top.
Read this before starting work (and before re-investigating something "new").

## Where to resume

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

### 2026-10-26 (15:17): R2 (fresh held-out, seed 78) done: the confidence gate does not help, dropped
| job | inserted | contact mean / median / p90 [s] | probes per run | GPU phases per run |
|---|---|---|---|---|
| R2A_v3 plain probe + v3 | 30/30 | 39.4 / 40.8 / 70.7 | 6.4 | – |
| R2B_v3 GPU entry + centering + v3 | 30/30 | 39.6 / 32.9 / 72.8 | 5.6 | 1.23 |
| R2B_v3g same + `--gpu-std-max 1 mm` | 30/30 | 45.6 / 37.6 / 85.0 | 6.9 | 0.10 |
- Paired per offset (mean over 3 repeats, bootstrap 95 % CI): GPU − plain +0.2 s [−6.9, 8.3]; gated − plain +6.1 s
  [−5.0, 17.7]; gated − ungated +6.0 s [−3.7, 16.7]. Repeat-to-repeat means swing 32–56 s with the same code.
- **Gate: negative.** It opens in only 0.10 runs per run on average: the belief rarely gets below 1 mm std before
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
