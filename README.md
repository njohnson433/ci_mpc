<div align="center">

# CI-MPC on the Franka FR3

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![Isaac Sim 6.2](https://img.shields.io/badge/Isaac_Sim-6.2-76B900?style=flat-square&logo=nvidia)](https://developer.nvidia.com/isaac/sim)
[![libfranka 0.21.3](https://img.shields.io/badge/libfranka-0.21.3-blue?style=flat-square)](https://github.com/frankaemika/libfranka)
![Real robot: not tested](https://img.shields.io/badge/real_robot-not_tested-lightgrey?style=flat-square)

Contact-implicit model predictive control for a Franka Research 3: developed in Isaac Sim first, then deployed on the real robot.

[Status](#status) • [How it works](#how-it-works) • [Benchmarks](#benchmarks-and-terms) • [Getting started](#getting-started) • [Usage](#usage) • [Documentation](#documentation)

</div>

An MPC sends pose, twist and feedforward-wrench setpoints to a Cartesian impedance controller, which turns them into
joint torques. Everything below was measured in **Isaac Sim 6.2 only**: the real robot has not been connected yet
([bring-up checklist](docs/REAL_ROBOT.md)).

The current focus is peg-in-hole insertion when the hole position from perception is off by up to 5 mm. The robot
finds the hole by touching the part (probing), keeps a probability map of where the hole is, and inserts.

> [!NOTE]
> Run IDs (A7, B14, RA_v3, ...), every sweep and every superseded result are in the dated log
> [docs/PROGRESS.md](docs/PROGRESS.md). This README holds only the current best configuration per task and its baseline.

## Status

All results are in simulation, with the FR3's identified joint friction in the scene. Benchmark sets (D1, R, R2,
H1–H4) are defined in [Benchmarks and terms](#benchmarks-and-terms).

| Component | Result (best configuration) | Baseline | Status |
|---|---|---|---|
| Cartesian impedance controller in Isaac | Final error 0.06–0.38 mm after 5 cm Cartesian steps (8 s); 0.11 mm on the hold/reach test | Upstream controller: 5.4 mm / 2.2 mm | Works (sim) |
| Joint identification (`ci_mpc_sysid`) | Recovers the sim's known friction: Coulomb 0.0–0.4 %, viscous 0.1–2.0 %, armature 2.1–3.7 % error (all 7 joints) | – | Works in sim; our FR3 not identified yet |
| Stage-1 MPC (free space, 6-DOF) | 9/9 tests pass at nominal friction: 15 cm reach final error 0.25 / 0.42 mm (out / back), 5 cm circle 1.04 mm RMS | – | Works (sim). 8/9 at friction ×0.7, 6/9 at ×1.3 (wrist rotation stiction) |
| Stacking from a simulated wrist camera | 3-cube tower, top cube 1.7 mm off the base cube | Ground-truth detections: 2.1 mm | Works (sim, colour-mask detector) |
| Stage-2 MPC (planar pushing, sticking contact) | 20 cm straight push: 2.2–2.3 mm final error; 90° arc (r 15 cm): 0.9–1.0 mm final, 8 mm RMS | – | Partial: on arcs the contact slides 9 mm along the box face, which a sticking-only model cannot plan |
| External wrench estimate (momentum observer) | RMS error 0.17 / 0.31 / 0.80 N per axis during peg insertion (true force RMS 0.5 / 1.6 / 4.1 N) vs a simulated wrist F/T sensor | – | Works (sim); ~5 Hz bandwidth, up to 7 N error at impacts |
| **Peg-in-hole, 8 mm Factory peg** (probing + learned likelihood) | **30/30** on R, mean contact time **23.9 s**; **30/30** on R2, 39.4 s | Same probing with the hand-written likelihood: 30/30 on R at 45.7 s. Hand-written spiral search: 7/30 on D1 | Works (sim, one peg geometry) |
| Same strategy on other parts | H1 peg variations 14–15/15; H2 gear 2–3/5; H3 AutoMate 5–6/12 (range over two flag sets) | – | Partial: gear and AutoMate failures are part-specific (below) |
| Jam recovery (`--jam-recovery`) | Friction-1.0 wedge case: 8/8 inserted, 0 false retries on 10 D1 runs | 0/3 without it | Works; off by default |
| GPU sampling MPC for the insertion phase | 30/30 on R2 at 39.6 s mean contact | Plain probe: 30/30 at 39.4 s | Ties plain probe; not recommended |
| C3+ (ADMM complementarity MPC) | After an ADMM bug fix: enters 16/16 offline starts (was 5/16), but stalls at 18–21 mm of 25 mm depth | – | In progress; not rerun in Isaac |
| Vectorized sim (Isaac Lab, many robots in one process) | FR3 Factory env, 1024 envs: 64 sim s per wall s at 240 Hz with contact | 3 ros2_control Isaac instances: ~1.2 sim s per wall s | Runs, but still with Factory's controller (porting ours is next) |
| Real FR3 | – | – | Not started |

## How it works

### Architecture

```
user_input_client ─┐
perception ────────┴─► ci_mpc_planner (MPC, 20–100 Hz)
                              │  ci_mpc_msgs/MpcCommand: pose, twist, feedforward wrench, K, D
                              ▼
                     Cartesian impedance controller (1 kHz on the robot, 240 Hz in Isaac)
                              │  joint torques without gravity
                    ┌─────────┴─────────┐
                    ▼                   ▼
          franka_hardware → FR3     Isaac Sim (ros2_control inside the physics step)
```

- **The MPC never commands torques.** It sends setpoints at 20–100 Hz. The impedance controller closes the loop at the
  control rate, so the MPC can be slow (Python, a GPU server) without touching stability.
- **Same controller code in sim and on hardware.** Only the hardware interface underneath changes. In Isaac the
  controller runs inside the physics step (`isaacsim.ros2.control`), not over ROS topics, so there is no
  communication delay in the control loop.
- **Gravity stays below the controller.** The FR3 compensates gravity itself (libfranka torques exclude it), so robot
  gravity is off in the sim. Objects keep normal gravity.
- **Fixed setpoint interface.** Each later MPC stage only replaces the solver behind `MpcCommand`.

### Controller in Isaac

The controller is our branch `isaac-sim-pinocchio` of
[LucasG2001/cartesian_impedance_control](https://github.com/LucasG2001/cartesian_impedance_control) (backup in
`patches/`). The robot model comes from Pinocchio instead of libfranka, so it runs without the robot.

Two scene changes were needed (`scripts/isaac/prepare_scene.py`):
- **Joint armature** (`motor_inertia · gear²` from `franka_description`). Without it the wrist joint chattered at the
  240 Hz Nyquist rate and the end-effector orientation wandered 0.6 rad.
- **Identified joint friction** (static, Coulomb, viscous). Without friction the sim is too ideal: 0.47 mm reach error
  vs 2.2 mm with friction.

The controller then needed MPC-ready changes: damping on the velocity *error* (the original damped absolute velocity:
6.8 mm RMS lag on a circle), a bypass of the 0.5 s setpoint filter (27 mm lag), and an integral term that is not
reset by every new setpoint. Details: [docs/CONTROLLER_FINDINGS.md](docs/CONTROLLER_FINDINGS.md).

### Stage-1 MPC (free space)

A QP (OSQP) on an end-effector double integrator, dt 0.02 s, 1 s horizon, solved at 50 Hz in ~1.5 ms. Orientation is
a second QP of the same form on the rotation vector. Key decisions:
- **Replan from the previous plan, not the measured state.** Replanning from the measured state makes the plan start
  where the arm already is, so the impedance spring never stretches and only the feedforward drives the arm: reach
  ended 10–15 mm short. The plan is reset to the measured state only beyond 2 cm.
- **Acceleration limits from the controller's force limit.** The controller clamps the force at 18 N, and the arm's
  effective mass Λ is 20–30 kg, so the limits are computed from Λ on every solve. This cut the overshoot at a workspace
  wall from 21 mm to 1 mm.
- **Under-compensate friction on the real robot.** Over-compensating friction (scale 1.0 at true friction ×0.7) made
  the loop unstable; under-compensating costs 1–2 mm of tracking. The real config starts at `friction_scale` 0.5.

### Stage-2 MPC (planar pushing)

A box pushed by a rod on the hand: quasi-static pusher-slider model with an ellipsoidal limit surface and a sticking
point contact (Lynch et al., 1992), as an LTV MPC (OSQP, 5 s horizon, solved in 11–17 ms). The contact point on the
face is chosen for the path curvature, because a centred contact cannot turn a 15 cm-radius arc inside the friction
cone. Open: with ~1.5 N normal force the contact holds only ~0.45 N tangentially, so it slides on arcs. Planning that
needs sliding modes in the model (stage 3).

### Peg-in-hole: finding the hole by probing

**Task:** the Isaac Lab Factory 8 mm round peg (Narang et al., 2022) with PhysX SDF collision. Hole radius 4.5 mm with a
1 mm chamfer, peg radius 3.99 mm, 25 mm deep. The stage-1 MPC grasps the peg and carries it above the *detected* hole,
with a perception error added (`--offset dx,dy` in mm). Then a contact phase at 10 Hz commands the impedance
controller directly (`scripts/peg_insert_demo.py`).

**Why probing:** a hand-written spiral search succeeds at 0–1 mm hole error and fails at ≥ 2 mm. Joint stiction makes
the hand follow the spiral in jumps, and sideways forces at the tip tilt or slip the peg in the gripper until it
wedges. One contact also does not tell where the hole is: a peg resting on the flat top face only rules out holes
directly under it (~1 mm). So the robot has to choose where to touch next.

**The probe strategy** (`--strategy probe`):
1. **Calibration touch.** Touch the flat top face 9.5 mm from the estimate to measure the peg's height in the
   gripper. Without it, a 0.55–0.60 mm grasp-height error made every hole hypothesis look inconsistent and the belief
   stopped moving (D1: 24/30 → 29/30 with the touch).
2. **Belief.** A grid over the hole position (±6 mm around the estimate), updated by Bayes after each probe:
   belief(h) ← belief(h) · p(reading | h). A reading is the tip height and the contact force from the momentum
   observer, tared before each probe.
3. **Next probe** goes where the probability that the hole lies within the 2 mm capture radius is highest.
4. **Centering.** If a probe gets a sideways push from the chamfer, the hand follows the push toward the hole. If the tip
   sinks, the peg is inserted with soft lateral stiffness so the hole aligns it.
5. **Guards found in failed runs:**
   - `--track-height 5`: the peg creeps 0.005–0.011 mm per probe in the fingers, so the grasp height is re-estimated
     from the last 5 flat probes.
   - `--centering-bound 0.008`: a misread push once drove centering past the edge of the block.
   - `--stuck-dither 0.0008`: 13 readings at the same spot counted as independent evidence and the belief collapsed
     0.7 mm off the hole.

**Learned likelihood** (`ci_mpc_planner/probe_likelihood.py`). This replaces the hand-written p(reading | h), which
used geometric constraints with hard force thresholds.
- **Model:** an MLP on 5 features per hypothesis: distance from the TCP to h, lateral force toward and across that
  direction, Fz, and tip height. Training is contrastive: the true hole vs 16 grid hypotheses, so the logit is the
  log-likelihood ratio.
- **Data:** probes from Isaac benchmark logs plus 80 randomized probe-sweep episodes (3200 probes; friction, hole tilt,
  grasp offset and torque-sensor noise varied per episode).
- **Sensor errors are randomized during training, not learned:** force gain σ 10 %, cross-axis σ 5 %, bias drift
  0.3 N, noise 0.2 N; tip-height bias 0.15 mm. These are assumptions until measured on the FR3. They make the
  likelihood less confident than a perfect sensor would allow.
- **Distance-prior correction (needed):** the probes in the logs sit where the strategies chose to probe, 2–6 mm from
  the hole. So the raw logit also contains log p_probe(r) / p_grid(r). Without the correction, a flat reading favoured
  holes ~4.5 mm away and the belief chased its own probes. The first model scored 27/30 on D1, below the
  hand-written model's 29/30. `scripts/rprior_correction.py` subtracts that term, which gave 29/30 at 27.0 s vs 58.1 s.
- **Model in use:** `ci_mpc_planner/models/probe_likelihood_v3_both.npz` (logs + 80 sweep episodes). The later v4 adds 40
  more sweep episodes but improves validation loss only 1 % (1.019 → 1.008) and is not benchmarked. Mechanism: one
  probe carries little information (a flat reading rules out ~1 mm), so data is no longer the limit.

**Tried and dropped:**

| Idea | Result | Why it lost |
|---|---|---|
| GPU sampling MPC for the insertion phase (`--entry gpu`) | R2: 39.6 s vs 39.4 s for plain probe (paired difference +0.2 s, 95 % CI −6.9 to 8.3) | It drives toward the belief's estimate. While the belief is still wide (~3 mm std) that estimate is wrong, and the MPC pulls the peg out of a chamfer that centering would have followed in |
| Start the GPU phase only when the belief std is < 1 mm | R2: 45.6 s | The belief rarely gets that narrow before the peg slides in, so the gate mostly switches the MPC off and runs wait longer |
| GPU sampler + one C3 solve to refine its plan | D1: 16/30 (plain probe 29/30) | Flat-face pushes started MPC phases that collapsed the belief near the start. Ran with the ADMM bug, not rerun |
| Subtract each run's median lateral force | Offline belief error 2.5 mm vs 1.96 mm | Lateral force on flat probes is probe-to-probe noise (sd 0.5–0.9 N), not a per-run bias; a per-run debias removes ~25 % of it |

### Other parts (held-out tasks)

The peg flags were tuned on D1 only, so they were checked on other parts. The table compares the base flags with the
new ones (height tracking, centering bound, stuck dither, and the learned likelihood on H1 only, because v3 was trained
on the 8 mm peg).

| Task | Base | New | Reading |
|---|---|---|---|
| H1: 8 mm peg, 5 conditions × 3 | 15/15 | 14/15 | The miss is a friction-1.0 wedge, which `--jam-recovery` now frees |
| H2: gear onto a shaft, 5 offsets | 2/5 | 3/5 | Gear stalls (16 so far) end 0.23–0.30 mm off the shaft axis, i.e. at the bore clearance (successes: 0.02–0.16 mm) |
| H3: 12 AutoMate plug/socket pairs | 6/12 | 5/12 | Within noise: 7 of 36 results flipped between identical runs. The same 6 assemblies fail in both arms |

The gear also needs rotation, which the probe strategy never commands (the gear jams 10 mm high at 2° of yaw error).
`--yaw-search 9 --push-follow 2e-5` inserts 2/3 at a 3° yaw error. A 0.25 mm push dither did not help (1/5): the dither
radius equals the bore clearance, so it drags the bore along the shaft wall instead of centring it.

### CI-MPC solvers

- **C3+** (`ci_mpc_planner/c3_3d.py`): an ADMM complementarity solver after Consensus Complementarity Control
  (Aydinoglu et al., [DAIRLab/c3](https://github.com/DAIRLab/c3)) on a 3D quasi-static peg-hole model with an exact
  hole SDF. With a physics rollout as warm start on every solve it inserts with 2–3× lower peak force than the
  force-following heuristic (1.1–1.7 N vs 3.6–5.7 N), but takes ~1.5 s per solve in Python.
  - A comparison with dairlib's code found that our scaled dual was not divided when ρ grew, which inflated it ~200×
    over 10 iterations. With the fix, the fast variant (rollout every 4th solve, ~70 ms) enters 16/16 offline starts
    instead of 5/16. It still stalls at 18–21 mm, not investigated yet.
- **GPU sampling MPC** (`ci_mpc_planner/sampling_mpc.py`, `scripts/gpu_mpc_server.py`): candidate command sequences
  are rolled out in a batched GPU version of the same model. The contact QP is solved with FISTA, matching OSQP to
  0.012 mm; one decision takes ~140 ms.
  - Offline, from 30 random starts with the peg on the chamfer and a 0.3–1.8 mm hole-estimate error, it enters 30/30
    within 60 steps vs 22/30 for the force-following heuristic.
  - Under held-out plant changes (H4) it keeps 30/30 where the heuristic drops to 10/30 (friction 1.0) and 7/30
    (0.4 mm clearance).
  - In Isaac it brings no gain over plain probe (table above).
- **Learned amortizations (negative):** a C3 student by regression entered 1/10 because MSE averages multimodal
  actions. An option selector with DAgger reached 16–19/30 vs the heuristic's 22/30. An ICNN cost-to-go entered 0/30:
  the true cost is non-convex in press depth, so the convex fit puts its minimum on a box edge.

### Vectorized simulation

The ros2_control pipeline runs one robot per Isaac process at ~0.4 sim s per wall s, and RAM allows 4 processes. For
data collection, `scripts/isaac/factory_fr3.py` puts the FR3 into Isaac Lab's Factory PegInsert env: 1024 envs run 64 sim
s per wall s at 240 Hz with contact. The FR3's full link collision meshes cost 2.4× vs the stock Panda (157). Next:
port our impedance controller and the probe strategy to batched torch ([ROADMAP task 9](docs/ROADMAP.md)).

## Benchmarks and terms

All peg benchmarks run in Isaac with a contact-phase budget of 150 s of sim time and 25 probes.

| Term | Meaning |
|---|---|
| Success | Peg bottom within 1 mm of the hole bottom (4 % of the 25 mm depth) **and** final xy error < 1.5 mm, from sim ground truth |
| Contact time | Sim time from the start of the descent above the hole to success, averaged over successful runs |
| D1 | Development set (all tuning): 10 fixed hole-position errors (0,0 2,0 3,0 −3,0 5,0 −5,0 0,4 3,3 −4,2 −3,−2 mm) × 3 repeats |
| R | Held-out: 10 random errors (seed 77, radius 1–5.4 mm) × 3 repeats |
| R2 | Second held-out set: 10 random errors (seed 78, up to 5.0 mm) × 3 repeats |
| H1 | Held-out 8 mm peg conditions: friction 0.5, friction 1.0, grasp offset, hole elsewhere, hole tilted 2°; 3 random 1–4 mm errors each |
| H2 | Held-out Factory gear onto its shaft (0.25 mm radial clearance, must mesh with two fixed gears), 5 random 1–4 mm errors |
| H3 | Held-out AutoMate assemblies (Tang et al., 2024): 12 plug/socket pairs, one random 1–4 mm error each |
| H4 | Offline held-out plant changes (friction, hole radius, stiction) on 30 random starts; the MPC keeps its nominal model |
| Plain probe | The probe strategy above with force-following centering (`--entry follow`, the default) |
| GPU entry | Same search, but a sink or push hands control to the GPU sampling MPC (`--entry gpu`) |
| Hand model / v3 | The hand-written measurement model vs the learned likelihood `probe_likelihood_v3_both.npz` |

## Getting started

Full setup: [docs/SETUP.md](docs/SETUP.md). Known problems: [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md).

### Prerequisites

| | Version |
|---|---|
| Ubuntu | 22.04, kernel 6.8.0-138-generic |
| GPU / driver | RTX 4080 16 GB / NVIDIA 595.99.02 |
| FR3 system version | 5.10 (robot server 10) |
| libfranka | 0.21.3 (`.deb`, Jammy) |
| franka_ros2 | `humble` branch (v2.x), libfranka taken from the system |
| ROS 2 | Humble |
| Isaac Sim / Isaac Lab | 6.2.0 / 3.0 (pip via `uv`, Python 3.12) |

### Workspace

This repo sits in `src/` so that colcon builds its ROS packages. Folders without a `package.xml` (docs, scripts) are
ignored.

```
~/franka_ros2_ws/src/
  franka_ros2 packages …          upstream, not in this repo
  cartesian_impedance_control/    impedance controller (own clone, branch isaac-sim-pinocchio; backup in patches/)
  messages_fr3/
  ci_mpc/                         ← this repo
~/ci_mpc_assets/                  URDF + USD scene (not in git)
~/IsaacLab/                       Isaac Lab + Isaac Sim (uv)
~/ocs2_ws/                        OCS2, separate workspace
```

### Environments: one terminal each

| Purpose | Activate | Python |
|---|---|---|
| ROS 2 / franka_ros2 | `source /opt/ros/humble/setup.bash && source ~/franka_ros2_ws/install/setup.bash` | 3.10 |
| Robot / MPC prototyping | `source ~/venvs/cimpc/bin/activate` | 3.10 |
| Isaac Sim / Isaac Lab, training | nothing: `cd ~/IsaacLab && uv run ...`, `~/IsaacLab/.venv/bin/python ...` or `scripts/isaac_ros.sh` | 3.12 |

> [!IMPORTANT]
> Never mix these environments in one terminal. ROS puts its Python 3.10 packages on `PYTHONPATH`, which breaks
> Isaac's 3.12 environment. In C++, link Pinocchio before libfranka: libfranka embeds a Pinocchio with a different
> memory layout.

> [!WARNING]
> `install/ci_mpc_planner` is a stale copy from 2026-10-23 (not a symlink install), and `peg_insert_demo.py` imports
> most planner modules from it. Until you rebuild with `colcon build --symlink-install --packages-select ci_mpc_planner`,
> the Isaac C3 strategies run without the ADMM fix. The learned likelihood is loaded from the source tree and is not affected.

> [!CAUTION]
> Each headless Isaac instance takes ~6 GB of RAM. Run at most 4 at once on the 31 GB machine, and use a separate
> `ROS_DOMAIN_ID` per instance (≤ 101).

### Quick start (sim)

```bash
# Controller test, headless, ~1 min: hold + reach, prints the errors
scripts/isaac/run_impedance_test.sh

# Peg-in-hole with the recommended configuration and a 3,-2 mm hole-position error (run from the repo root)
scripts/demo_peg.sh --headless --offset 3,-2 --strategy probe --timeout-contact 150 --max-probes 25 \
  --track-height 5 --centering-bound 0.008 --stuck-dither 0.0008 \
  --belief-model ci_mpc_planner/models/probe_likelihood_v3_both.npz
```

Planner unit tests (`ci_mpc_planner/test`, workspace sourced): 13 pass and 1 is an expected failure
(`test_c3_inserts_at_3mm`, the retired planar C3 line).

## Usage

| What it shows | Command | Expected result |
|---|---|---|
| Pick and place with the stage-1 MPC | `scripts/demo_pick_place.sh [box\|cube] [--noise] [--stops] [--headless]` | Placed 0.6–1.1 mm from the target |
| 3-cube tower, optionally from the simulated wrist camera | `scripts/demo_stack.sh [--noise] [--headless] [--camera]` | 1.2–3.2 mm per level |
| Planar pushing (stage 2) | `scripts/demo_push.sh [line\|arc\|yaw10]` | 2.2 mm (line), 0.9 mm (arc) final error |
| Peg-in-hole, hand-written spiral | `scripts/demo_peg.sh [--offset dx,dy]` | Works at 0–1 mm error, fails at ≥ 2 mm |
| Peg-in-hole, probing | `scripts/demo_peg.sh --offset dx,dy --strategy probe ...` (flags as in the quick start) | 30/30 on R and R2 |
| Jam recovery for high-friction wedges | add `--jam-recovery 3 --jam-lift 0.003 --jam-shift 0.0006` | Frees the H1 friction-1.0 wedge 8/8 |
| Gear (H2), AutoMate (H3) | `scripts/demo_gear.sh [--offset dx,dy]`, `scripts/demo_automate.sh ID [--offset dx,dy]` | See the held-out table |
| A benchmark set (N runs in parallel, summary table) | `OFFSETS="..." scripts/peg_benchmark.sh OUT_DIR probe [demo options]` | `res.json` per run |
| Impedance test suite (steps, rotations, circle, pushes, windup) | `scripts/impedance_suite.py` | See CONTROLLER_FINDINGS |
| Vectorized Factory throughput (Panda, or `--fr3`) | `~/IsaacLab/.venv/bin/python scripts/isaac/bench_factory_vec.py --num_envs 1024 [--mode contact] [--physics-hz 240] [--fr3]` | sim s per wall s |
| Retrain the likelihood | `~/IsaacLab/.venv/bin/python scripts/train_probe_likelihood.py DATA.npz OUT.npz` (data from `scripts/probe_dataset.py`) | Model `.npz` with the distance correction |

Benchmark queues (`scripts/queue_*.sh`) run in the background; follow them with `scripts/watch_runs.sh`.

## Repository layout

```
ci_mpc_msgs/          MpcCommand (MPC → controller setpoint)
ci_mpc_planner/       MPCs (stage 1, pushing, C3+, GPU sampling MPC, sample + C3 refine), hole belief,
                      learned probe likelihood (models/), mpc_node, push_node
ci_mpc_perception/    simulated Gemini 305 depth noise, box detector
ci_mpc_sim_gripper/   sim gripper controller
ci_mpc_sysid/         joint friction / armature identification
patches/              backups of the controller and franka_ros2 changes
scripts/              demos (demo_*.sh), benchmark drivers and queues, offline experiments, training
  isaac/              scene setup (prepare_scene.py, *_scene.py), in-sim scripts, vectorized Factory env
docs/                 setup, roadmap, progress log, findings
```

## Documentation

| Document | Contents |
|---|---|
| [docs/PROGRESS.md](docs/PROGRESS.md) | Dated log of every experiment with numbers and the reason it won or lost; "Where to resume" |
| [docs/ROADMAP.md](docs/ROADMAP.md) | Staged plan, open tasks (incl. task 9: vectorized sim), open questions |
| [docs/SETUP.md](docs/SETUP.md) | Environment setup, step by step |
| [docs/CONTROLLER_FINDINGS.md](docs/CONTROLLER_FINDINGS.md) | Controller in Isaac: setup issues, audit, friction handling |
| [docs/SYSID.md](docs/SYSID.md) | Joint identification method and sim validation |
| [docs/REAL_ROBOT.md](docs/REAL_ROBOT.md) | Real-robot bring-up checklist |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Problems hit and how they were solved |
