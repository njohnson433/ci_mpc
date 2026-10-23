#!/usr/bin/env python3
# Offline 3D peg-in-hole with an information-driven probing search (no Isaac): plant = peg_hole_3d.PegHole3D with
# the true hole at --hole (mm, relative to the estimate) and arm stiction; belief = hole_belief.HoleBelief.
# Each probe: hover 1 mm above the opening at the probe xy, press down (stiction feedforward), measure, update the
# belief; in the hole (tip 3 mm below the opening) -> push to the bottom; else lift and probe where the belief puts
# the most probability within the capture radius.
#   peg_probe_offline.py --hole -3,0 [--max-probes 12]
import argparse
import time

import numpy as np

from ci_mpc_planner.hole_belief import HoleBelief
from ci_mpc_planner.peg_hole_3d import PegHole3D

p = argparse.ArgumentParser()
p.add_argument("--hole", default="-3,0")
p.add_argument("--max-probes", type=int, default=12)
p.add_argument("--noise", type=float, default=0.3, help="force measurement noise [N]")
p.add_argument("--verbose", action="store_true")
args = p.parse_args()

hole = np.array([float(v) for v in args.hole.split(",")]) / 1000.0
plant = PegHole3D(hole_xy=hole, n_ring=8, n_rim=12)
comp = np.array([1 / 1500, 1 / 1500, 1 / 500, 1 / 130, 1 / 130])
fa = np.array([2.0, 2.0, 3.5, 0.05, 0.05])
ff_z = -3.0  # stiction feedforward along the descent
tb = plant.g.tip_below_tcp
belief = HoleBelief(plant.g)
rng = np.random.default_rng(0)
x = np.array([0.0, 0.0, tb + 0.002, 0.0, 0.0])
probes, steps, t_belief = [], 0, []


def run(u, n, ff=(0, 0, 0)):
    global x, steps
    lam = np.zeros(0)
    for _ in range(n):
        x, lam, nc = plant.step(x, u + comp * np.array([ff[0], ff[1], ff[2], 0, 0]), comp, fa)
        steps += 1
    return plant.contact_wrench(lam)[:3] if len(lam) else np.zeros(3)


def move_xy(target, z, n=12):
    """Lateral move with a stiction feedforward along the remaining offset (the arm sticks below ~2 N)."""
    for _ in range(n):
        d = target - x[:2]
        if np.linalg.norm(d) < 2e-4:
            break
        ff = 2.5 * d / max(np.linalg.norm(d), 1e-9)
        run(np.array([target[0], target[1], z, 0, 0]), 1, (ff[0], ff[1], 0.0))


target = np.zeros(2)
result = "not found"
for k in range(args.max_probes):
    # hover above the probe point, then press
    run(np.array([x[0], x[1], tb + 0.001, 0, 0]), 4)
    move_xy(target, tb + 0.001)
    u = np.array([x[0], x[1], x[2], 0, 0])
    F = np.zeros(3)
    for _ in range(30):
        u[2] = max(u[2] - 5e-4, x[2] - 0.006)
        F = run(u, 1, (0, 0, ff_z))
        tip_z = plant.tip(x)[2]
        if F[2] > 1.0 or tip_z < -0.003:
            break
    tip_z = plant.tip(x)[2]
    Fm = F + rng.normal(0, args.noise, 3)
    t0 = time.perf_counter()
    belief.update(x, Fm)
    t_belief.append((time.perf_counter() - t0) * 1000)
    est, std = belief.estimate()
    on_top = tip_z > -1e-4 and np.linalg.norm(Fm[:2]) < belief.f_lateral
    if on_top:
        probes.append(x[:2].copy())  # a clear miss at the measured position
    if args.verbose:
        print(f"probe {k}: at {np.round(x[:2] * 1000, 2)} mm (target {np.round(target * 1000, 2)}), tip z {tip_z * 1000:+.2f} mm, force {np.round(Fm, 2)}; "
              f"belief {np.round(est * 1000, 2)} (std {std * 1000:.2f}) mm")
    if -0.003 <= tip_z < -1e-4 or (tip_z < 0.0005 and np.linalg.norm(Fm[:2]) > 0.5):
        # engaged on the chamfer: follow its lateral push (towards the axis) with a light normal force
        for _ in range(40):
            lat = F[:2]
            if np.linalg.norm(lat) > 0.2:
                dirn = lat / np.linalg.norm(lat)
                u = np.array([x[0] + 4e-4 * dirn[0], x[1] + 4e-4 * dirn[1], x[2] - 0.002, 0, 0])
                F = run(u, 1, (2.5 * dirn[0], 2.5 * dirn[1], -2.0))
            else:
                u = np.array([x[0], x[1], x[2] - 0.004, 0, 0])
                F = run(u, 1, (0, 0, ff_z))
            if plant.tip(x)[2] < -0.003:
                break
        tip_z = plant.tip(x)[2]
        if args.verbose:
            print(f"   centering: tip z {tip_z * 1000:+.2f} mm at {np.round(x[:2] * 1000, 2)}")
    if tip_z < -0.003:  # entered the hole: push to the bottom
        u = np.array([x[0], x[1], x[2], 0, 0])
        for _ in range(60):
            u[2] = max(u[2] - 5e-4, x[2] - 0.006)
            run(u, 1, (0, 0, ff_z))
            if plant.tip(x)[2] < -plant.g.depth + 0.001:
                break
        result = "inserted" if plant.tip(x)[2] < -plant.g.depth + 0.001 else "entered, stuck"
        break
    if not on_top and std < 0.0008:  # engaged on the chamfer and localized: center on the estimate
        target = est
    else:
        target, pmax = belief.next_probe(exclude=probes, current=x[:2])
print(f"hole {np.round(hole * 1000, 1)} mm: {result} after {len(probes)} probes, {steps} plant steps; final tip "
      f"{np.round(plant.tip(x) * 1000, 2)} mm (hole frame); belief update {np.mean(t_belief):.1f} ms")
