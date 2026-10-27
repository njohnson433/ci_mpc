#!/usr/bin/env python3
# Why does the engaged phase stall short of the 24 mm success depth offline? Same setup as peg_c3_3d_offline.py (peg
# pressed onto the chamfer at --offset, 3D convex plant with arm stiction), with a per-step trace: tip depth, tilt,
# active contacts by type (tip ring / chamfer edge ring / hole rim vs peg side) and height, contact force, the arm
# friction force the plant used (f = C^-1 (x+ - u) - W lam), and C3's violation.
# Policies: c3 (C33Config, options below), follow (the offline heuristic), press (oracle: xy command on the true hole
# axis, z pressed with --press-force N through the spring; tells whether the plant can reach the bottom at all).
#   c3_stall_diag.py --offset 1.5,0 --policy press [--press-force 5] [--steps 150] [--every 5] [--csv out.csv]
import argparse
import csv
import time

import numpy as np

from ci_mpc_planner.c3_3d import C33Config, PegHoleC3_3D
from ci_mpc_planner.peg_hole_3d import PegHole3D

p = argparse.ArgumentParser()
p.add_argument("--offset", default="1.5,0")
p.add_argument("--est-error", default="0,0")
p.add_argument("--policy", default="c3", choices=["c3", "follow", "press"])
p.add_argument("--steps", type=int, default=150)
p.add_argument("--press-force", type=float, default=5.0, help="press policy: downward force through the z spring [N]")
p.add_argument("--success-depth", type=float, default=0.024)
p.add_argument("--every", type=int, default=5, help="print every n steps (0: summary only)")
p.add_argument("--csv", default="")
p.add_argument("--goal-below", type=float, default=0.0, help="C3 goal this far below the hole bottom [mm] (the bottom stops the peg)")
p.add_argument("--set", action="append", default=[], help="C33Config override name=value (value: float or a,b,c)")
p.add_argument("--reest", type=float, default=0.0,
               help="C3: re-estimate the hole from the contact each step, blend factor (0 = off): on the chamfer the axis "
                    "is R + c - d - (a - pc) from the tip centre along the lateral force; in the bore R - a")
p.add_argument("--reest-min-force", type=float, default=1.0, help="re-estimate only above this lateral force [N]")
p.add_argument("--force-noise", type=float, default=0.0, help="sigma of the force noise seen by the re-estimate [N]")
p.add_argument("--grasp-offset", default="0,0", help="plant only: peg tip offset from the TCP in xy [mm] (Isaac: the "
               "peg sits off-centre in the fingers, 0.3 mm; the model assumes the tip under the TCP)")
p.add_argument("--stiction-ff", default="0,0", help="C3: friction compensation lateral,z [N]: feedforward along the "
               "commanded lateral motion and down; the C3 model sees the arm friction reduced by it")
p.add_argument("--ff-lat-depth", type=float, default=float("inf"),
               help="lateral friction compensation only while the tip is less than this deep [mm] (the chamfer; in the "
                    "bore it pushes the peg into the wall)")
p.add_argument("--plant-bias", default="0,0,0", help="plant only: constant unmodelled force on the TCP fx,fy,fz [N] "
               "(the real FR3's model / gravity error at the probe pose: about -1.1,-0.85,-0.9)")
p.add_argument("--arm-friction", default="2.0,2.0,3.5,0.05,0.05", help="plant arm stiction (the model keeps cfg)")
args = p.parse_args()

off = np.array([float(v) for v in args.offset.split(",")]) / 1000.0
err = np.array([float(v) for v in args.est_error.split(",")]) / 1000.0
comp = np.array([1 / 1500, 1 / 1500, 1 / 500, 1 / 130, 1 / 130])
fa = np.array([float(v) for v in args.arm_friction.split(",")])
pb = np.array([*(float(v) for v in args.plant_bias.split(",")), 0.0, 0.0])
plant = PegHole3D(hole_xy=-off, n_ring=8, n_rim=12)
model = PegHole3D(hole_xy=-off + err, n_ring=8, n_rim=12)
g, tb = plant.g, plant.g.tip_below_tcp
go = np.array([float(v) for v in args.grasp_offset.split(",")]) / 1000.0
plant.tcp_offset = np.array([-go[0], -go[1], tb])  # world = tcp + R (p_local - tcp_offset): the tip at tcp + go

x = np.array([0.0, 0.0, tb + 0.0005, 0.0, 0.0])
u = x.copy()
for _ in range(10):
    u = np.array([0.0, 0.0, x[2] - 0.004, 0.0, 0.0])
    x, lam, nc = plant.step(x, u + comp * (np.array([0, 0, -3.0, 0, 0]) + pb), comp, fa)
goal = np.array([*(-off + err), tb - g.depth - args.goal_below / 1000.0, 0.0, 0.0])
cfg = C33Config()
for s in args.set:
    k, v = s.split("=")
    cur = getattr(cfg, k)
    if isinstance(cur, np.ndarray):
        setattr(cfg, k, np.array([float(t) for t in v.split(",")]))
    elif isinstance(cur, bool):
        setattr(cfg, k, v.lower() in ("1", "true", "yes"))
    elif isinstance(cur, int):
        setattr(cfg, k, int(v))
    elif isinstance(cur, str):
        setattr(cfg, k, v)
    else:
        setattr(cfg, k, float(v))
sff = np.array([float(v) for v in args.stiction_ff.split(",")])
if sff.any():  # the controller cancels part of the stiction: the model sees the rest
    cfg.arm_friction = np.maximum(cfg.arm_friction - np.array([sff[0], sff[0], sff[1], 0.0, 0.0]), 0.3 * cfg.arm_friction)
mpc = PegHoleC3_3D(model, comp, cfg)


def contact_types(x, lam, nc):
    """Active contacts (normal force > 0.05 N) as (kind, height of the point below the opening [mm], ln [N])."""
    out = []
    if not nc:
        return out
    cs = plant.contacts(x)
    for i, (_, n, _, _, pl) in enumerate(cs):
        ln = lam[5 * i]
        if ln < 0.05:
            continue
        pw = plant.world(x, pl)
        if not any(np.allclose(pl, q) for q in plant.peg_points):
            kind = "rim"  # a hole rim point against the peg's side
        elif np.hypot(pl[0], pl[1]) < 1e-9:
            kind = "center"  # tip center (bottom)
        else:
            kind = "tip" if pl[2] < g.pc / 2 else "edge"
        out.append((kind, -pw[2] * 1000, ln, n))
    return out


rows, F, ms, viol = [], np.zeros(3), [], []
est = -off + err
rng = np.random.default_rng(int(abs(off[0] * 1e6 + off[1] * 1e3)))
t_start = time.perf_counter()
for k in range(args.steps):
    info = {}
    ff = np.zeros(5)
    if args.policy == "c3":
        t0 = time.perf_counter()
        u, info = mpc.solve(x, goal, u)
        ms.append((time.perf_counter() - t0) * 1000)
        viol.append(info.get("viol", np.nan))
        if sff.any():
            dl = u[:2] - x[:2]
            on = np.linalg.norm(dl) > 5e-5 and -plant.tip(x)[2] * 1000 < args.ff_lat_depth
            lat = sff[0] * dl / np.linalg.norm(dl) if on else np.zeros(2)
            ff = np.array([lat[0], lat[1], -sff[1], 0.0, 0.0])
    elif args.policy == "follow":
        lat = F[:2]
        if np.linalg.norm(lat) > 0.3:
            d = lat / np.linalg.norm(lat)
            u = np.array([x[0] + 4e-4 * d[0], x[1] + 4e-4 * d[1], x[2] - 0.002, 0, 0])
            ff = np.array([2.5 * d[0], 2.5 * d[1], -2.0, 0, 0])
        else:
            u = np.array([x[0], x[1], x[2] - 0.004, 0, 0])
            ff = np.array([0, 0, -3.0, 0, 0])
    else:  # oracle press on the true axis
        u = np.array([-off[0], -off[1], x[2] - args.press_force * comp[2], 0, 0])
    cmd = u + comp * ff
    xn, lam, nc = plant.step(x, cmd + comp * pb, comp, fa)
    w = plant.contact_wrench(lam) if nc else np.zeros(5)
    f_arm = (xn - cmd - comp * pb) / comp - w  # arm friction the plant applied (x+ = u + C (W lam + f))
    act = contact_types(x, lam, nc)
    x = xn
    F = w[:3]
    tip = plant.tip(x)
    if args.policy == "c3" and args.reest > 0:
        Fm = F + rng.normal(0.0, args.force_noise, 3)
        f_lat, d = np.linalg.norm(Fm[:2]), -tip[2]
        if d > 2e-4 and f_lat > args.reest_min_force:
            tip_m = model.world(x, [0.0, 0.0, 0.0], hole_xy=np.zeros(2))[:2]  # the model's tip (no grasp offset known)
            r_ax = (g.R + g.c - d - (g.a - g.pc)) if d < g.c else (g.R - g.a)
            est = (1 - args.reest) * est + args.reest * (tip_m + max(r_ax, 0.0) * Fm[:2] / f_lat)
            model.hole_xy = est.copy()
            goal[:2] = est
    kinds = {}
    for kind, h, ln, _ in act:
        kinds.setdefault(kind, []).append((h, ln))
    row = dict(k=k, depth=-tip[2] * 1000, tip_x=tip[0] * 1000, tip_y=tip[1] * 1000, rx=x[3] * 1000, ry=x[4] * 1000,
               Fx=w[0], Fy=w[1], Fz=w[2], f_arm_z=f_arm[2], f_arm_x=f_arm[0], f_arm_y=f_arm[1],
               dz_cmd=(cmd[2] - x[2]) * 1000, n_tip=len(kinds.get("tip", [])), n_edge=len(kinds.get("edge", [])),
               n_rim=len(kinds.get("rim", [])), ln_sum=sum(a[2] for a in act), viol=info.get("viol", np.nan),
               est_err=np.linalg.norm(est + off) * 1000)
    rows.append(row)
    if args.every and k % args.every == 0:
        desc = " ".join(f"{kd}:{len(v)}@{np.mean([h for h, _ in v]):.1f}mm/{sum(l for _, l in v):.2f}N"
                        for kd, v in kinds.items())
        print(f"k {k:3d} depth {row['depth']:6.2f} tip xy {row['tip_x']:5.2f},{row['tip_y']:5.2f} tilt "
              f"{row['rx']:5.1f},{row['ry']:5.1f} mrad F {w[0]:5.2f},{w[1]:5.2f},{w[2]:5.2f} N arm f_z {f_arm[2]:5.2f} "
              f"cmd-x z {row['dz_cmd']:5.2f} mm | {desc}" + (f" | viol {row['viol']:.2f}" if args.policy == "c3" else ""))
    if tip[2] < -args.success_depth:
        break
tip = plant.tip(x)
ok = tip[2] < -args.success_depth
last = rows[-10:]
print(f"{args.policy} offset {args.offset}: {'OK' if ok else 'FAIL'} after {k + 1} steps, depth {-tip[2] * 1000:.2f} mm, "
      f"tilt {x[3] * 1000:.1f},{x[4] * 1000:.1f} mrad; last 10: Fz {np.mean([r['Fz'] for r in last]):.2f} N, "
      f"arm f_z {np.mean([r['f_arm_z'] for r in last]):.2f} N, contacts tip/edge/rim "
      f"{last[-1]['n_tip']}/{last[-1]['n_edge']}/{last[-1]['n_rim']}"
      + (f"; solve {np.mean(ms):.0f} ms, violation {np.nanmean(viol):.3f}" if ms else "")
      + f"; wall {time.perf_counter() - t_start:.0f} s")
if args.csv:
    with open(args.csv, "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0]))
        wr.writeheader()
        wr.writerows(rows)
