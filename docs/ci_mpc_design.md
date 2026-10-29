# CI-MPC: why C3 brought no gain, and a design for the new controller (2026-10-29)

Status: **proposal for review, nothing implemented.** Stage A as decided by the user (update 10-29): known in-hand
pose (no grasp-height calibration), camera prior N(detection, 2 mm), then the likelihood probe search (v5-clamp / v3,
src-35), then the push. **The CI-MPC replaces the plain push after the search handover; it does not replace the
search.** No CAD models in the controller. The CI-MPC has to beat the stage-A plain push (src-35's baseline). A later
option, not now: a CI-MPC that probes on purpose, merging search and insertion.

## 1. What C3 was, and why it did not help

Numbers are from PROGRESS (10-27 to 10-29): D1 = the Factory 8 mm peg, 10 offsets; H1–H3 = the held-out tasks.

| result | plain probe | C3 (best variant: MF3 stiction feedforward + 3 mm std gate) |
|---|---|---|
| D1 success / contact median | 30/30, 25.0 s | 35/40, 35.3 s |
| D1 true force (wrist, paired, pre-seat) | 9.3 N | 9.0 N (p 0.57) |
| H1 / H2 / H3 (cross-task, paired) | 15/15, 2/5, 5/10 | 14/15, 2/5, 5/10, slower on all |
| real-arm match (realfric) | 30/30, 16 s | 28/30, 39 s |

C3 was never better on success, time or force, on any set. The reasons, by component:

1. **Where it took over: mid-search, on a belief estimate.** C3 started when the tip sank ≥ 0.3 mm. Its goal was the
   belief's hole estimate, off by 0.16–4.2 mm at handover (12/14 handovers with belief std 2.5–5.7 mm). The radial
   clearance is 0.51 mm. Errors > 1.1 mm all stalled at the entry; the 3 mm std gate mostly kept C3 from starting.
   C3 is a goal-tracking controller, so a wrong goal drives the peg into the wall. The plain heuristic has no goal and
   follows the contact force, which is why it was more robust.
2. **Contact model: hand-built, peg-specific, linearized, fixed contact set.**
   - Points: rings on a round peg vs the exact axisymmetric Factory hole profile (chamfer, bore), i.e. CAD-specific.
     For H2 / H3 the same round model was a guess.
   - Linearized at the current state, with the contact set frozen over the horizon (`contact_radius` 0.5 mm, up to
     ~40 contacts). C3 cannot anticipate a contact that opens or closes within the horizon: the chamfer → bore
     transition is exactly that.
3. **Plant model: quasi-static impedance with a box stiction model, far from Isaac.**
   - The z stiction (2.2–3.5 N) at K_z 500 N/m needs a 4–7 mm command deflection before anything moves. Horizon 4 ×
     du_max 0.5 mm = 2 mm, so once the deflection dropped below that, C3 predicted no motion and relaxed: an absorbing
     stall. MF3 feedforward was the patch.
   - Rotational stiction: 0.05 Nm in the model vs 0.55–1.56 Nm on the real arm and in Isaac.
   - Replay of an Isaac handover through the offline plant inserted (24 mm, 57 steps). In Isaac the TCP moved
     0.000 mm per step where the model predicted 0.16 mm. **The model–plant gap, not the optimizer, decided the
     outcome.**
4. **Solver and rate: too slow and not converged.**
   - C3+ ADMM, 10 iterations, one OSQP QP each: 100–700 ms per decision (~80 ms offline), for a 10 Hz loop. The
     controller holds the last setpoint meanwhile.
   - Violation (complementarity residual) 0.3–1.4 in Isaac vs 0.1–0.3 offline: the returned plans were not
     contact-consistent.
5. **Warm start: rollout vs shifted, a 2-cycle.** Every 4th solve took fresh targets from a straight-to-goal physics
   rollout (~700 ms); the others shifted the previous solution. The trace alternated between the two (violation
   1.4 / 0.4); always-shift went stale at the opening.
6. **What it optimized: position tracking, not insertion quality.**
   - Cost = TCP pose to the goal (placed 11 mm below the bottom to overcome the stiction cost trap) + spring deflection
     + command rate.
   - There was no force target, no alignment term beyond a small rx / ry weight, and no use of the measured contact
     direction.
   - So it behaved like the plain push, slower and with a wrong goal. The "lower force" seen early was an observer
     artifact: on the wrist F/T it is equal (paired p 0.2–0.8).

**Lesson for the redesign:** it has to take over once the peg is captured and keep updating the hole axis from the
contacts (not a fixed goal from the search). It needs a contact model that is smooth through the chamfer / bore / face
transitions and is built from dimensions, not CAD, and a plant model matched to the real arm's stiction and
rotation. It has to solve in ~10 ms, and its cost has to be about the insertion (force, alignment, progress), not a
waypoint.

## 2. Proposed CI-MPC (stage A)

**Handover from the search.**
- *Why the old C3 handover did not help:*
  - It started on a 0.3 mm tip sink, i.e. on the first chamfer touch, before the peg was captured. The belief then
    still had std 2.5–5.7 mm (12/14 handovers) and mean errors up to 4 mm.
  - C3 took the mean as a fixed goal (one belief update per phase) and drove to it. Errors > 1.1 mm (2× the clearance)
    all stalled at the entry.
  - The 3 mm std gate mostly kept it from starting at all, so it ran too early or not at all.
- *When to take over:* where the plain push takes over today, i.e. engaged with the tip in the bore (`--engage`
  3 mm on chamfered holes, `--tilt-engage` 0.5 mm on sharp ones), plus a confidence gate: belief std below ~2× the
  radial clearance. From there the bore has captured the peg. The job is alignment, force regulation and a smooth
  seat, where the plain push is weakest: the wedging, the 15 s stalls, the seating spike. An earlier handover (first
  sink, std < 0.5 mm) is a second experiment, after the late one works.
- *How it uses the belief:*
  - The mean is the initial hole position h₀ of the SDF.
  - The covariance Σ replaces the camera σ as the prior of the moving-horizon hole estimate, and weights the
    alignment residual.
  - The contacts then update h at every tick (the bore wall contacts pin the axis to within the clearance), so a
    wrong mean is corrected instead of driven into.
  - If the estimate leaves the 2σ ellipse or the contacts become inconsistent (φ residual high), the CI-MPC hands back
    to the search, an explicit fallback.

**Scope.** It runs from the handover to seated. Inputs: TCP pose, the observer wrench (and the wrist F/T in sim for
evaluation only), the known in-hand peg pose, and the camera hole estimate (σ ≈ 2 mm) as a prior. Output: the
impedance setpoint (6D pose) at 20–50 Hz, plus an optional force feedforward.

**State and plant.**
- x = TCP pose (6D incl. yaw, for rectangles); the peg pose follows from the known grasp.
- Hole pose h = (x, y, yaw) as an estimated parameter.
- Quasi-static impedance: K (u − x) + f_arm(ẋ) + J_cᵀ λ = 0, solved for x+ per step.
- Arm friction: a smooth Coulomb + viscous model per Cartesian axis including rotation, identified from the real
  breakaway (x/y/z 1.4/0.9/1.2 N, rx/ry 1.2/0.7 Nm) and switchable to the Isaac values (no box constraint, so no
  absorbing stall).

**Contact model: analytic SDF of primitives from dimensions.**
- Peg: a cylinder (radius, length, end chamfer) or a box (wx, wy, length), sampled as surface points on the bottom
  rim, the chamfer edge and the side near the bottom (fixed, ~30–60 points).
- Hole: a block with a cylindrical or box bore, from (hole width / diameter, depth, top face). The rounded-box SDF
  with edge radius ρ (0.1–0.3 mm) smooths the rim, so φ is C¹ everywhere: face, rim, bore wall and bottom with one
  formula.
- Gap φ_i(x, h) and normal n_i = ∇φ_i analytic; tangents from n_i.
- Same interface for round and rectangular parts. No per-task code, no CAD.

**Complementarity: smoothed.**
- Normal: λ_n = k_c · softplus(−φ / ε) · ε, a stiff smooth penalty (ε ~ 0.05 mm), equivalent to relaxed
  complementarity φ λ ≤ δ with δ ~ k_c ε²; ε is annealed in the solve if needed.
- Friction: regularized Coulomb λ_t = −μ λ_n · v_t / √(|v_t|² + v_ε²) (stick within v_ε).
- The rollout is differentiable through contact switches, so no fixed contact set and no mode enumeration.

**Optimizer: Gauss–Newton / Levenberg–Marquardt, short horizon, warm-started.**
- Decision variables: the setpoint sequence u_0..u_{H−1}, H = 5–10 steps at 20–50 Hz (≤ 0.5 s look-ahead).
- Single shooting through the smooth plant; residual Jacobians by forward-mode autodiff (JAX or CasADi; numpy +
  finite differences as a first prototype).
- 2–5 LM iterations per tick, warm-started by shifting the last solution (no rollout / shift alternation).
- Target ≤ 10 ms per solve for ~50 contact points (the 6-dim command, H = 8: ~48 unknowns).

**What it optimizes (residuals).**
- *Contact force:* the total normal force at F_ref (3–5 N) while engaged; a soft cap on lateral force and on wrist
  torque (no wedging).
- *Alignment:* peg axis parallel to the hole axis and the peg centre on the hole axis estimate, weighted by that
  estimate's confidence.
- *Progress:* depth increase per step towards seated, with a smooth slow-down near the bottom (no seating spike;
  today's max is a 1-sample impact of up to 25 N).
- *Smoothness:* command rate and acceleration, and the gap between setpoint and pose (bounded deflection).

**Online hole estimate from contacts (moving horizon, same Gauss–Newton machinery).** Over the last N samples, fit
h so that:
1. the measured pose has φ ≈ 0 at the touching points;
2. the measured wrench direction matches −Σ λ_i n_i(h).

The camera prior (σ 2 mm) regularizes it. This replaces the parked belief search in stage A, and it is what C3 lacked:
the goal moves with what the contacts say.

**Validation plan (one step at a time, D1 / dev first, H1–H3 once with a frozen design).**
- A1: the model alone, offline against Isaac logs (wrist F/T + pose; we have hundreds of runs). The predicted
  contact force at the logged poses must match the wrist within an error target, set before the run, for peg and
  rect.
- A2: the MPC in the offline smooth plant with the 2 mm camera error, against the stage-A plain push in the same
  plant.
- A3: Isaac, D1 + dev holes, against src-35's stage-A plain push: success, contact time, true force (pre-seat,
  50 ms, max), and slip.
- A4: H1–H3 (+ H4 offline) once, paired against the plain push. Nothing becomes a default before that.

**Risks.**
- Smooth contact stiffness vs solver conditioning (anneal ε, LM damping).
- The plant gap at stiction (identify both the real and the Isaac arms, and test the MPC under each).
- Observer bias in contact (−0.06 ± 0.20 N per src-35, fine).
- The 10 ms budget in Python (JAX jit or a C++ port if needed).
- Thin pegs stay grasp-limited (in-hand slip of the order of the clearance), so the stage A assumption of a known
  in-hand pose matters most for them.
