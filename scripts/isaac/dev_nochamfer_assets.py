"""Chamferless tight-clearance dev set for the hole search (2026-10-28; H5 NIST stays held out, so design and tune here).
Procedural plug / socket pairs in the AutoMate asset layout (as nist_assets.py), loaded unchanged by automate_scene.py /
demo_automate.sh with AUTOMATE_ASSETS=~/ci_mpc_assets/dev_nochamfer AUTOMATE=<id>:
  dev_round_8mm    Factory-size peg 8.00 mm, hole 8.10 mm (0.10 mm diametral), no hole chamfer
  dev_round_10mm   peg 10.00 mm, hole 10.15 mm
  dev_square_10mm  square peg 10.00 mm, square hole 10.15 mm (yaw matters)
  dev_round_4mm    thin peg 4.00 mm, hole 4.09 mm (a tilted thin peg drops only ~0.1-0.2 mm: H5 round 4 failure)
  dev_rect_12x8mm  rectangular peg 12 x 8 mm (gripped across the 8 mm side), hole 12.15 x 8.15 mm
Pegs 40 mm long with a 0.5 mm end chamfer; sockets 30 x 30 x 15 mm blocks with a through-hole (NIST: 25 x 25 x 9, 50 mm
pegs: deliberately different). Plug and socket share the seated frame (peg bottom at z 0 = block bottom = the table).
Per <id>: plug.usd / socket.usd (body a_<id>_{plug,socket}, SDF collision: socket 512, plug 256) + .obj (vertices for
info()); plug_grasps.json (TCP 10 mm below the peg top), disassembly_dist.json, dev.json in the root.
    ~/IsaacLab/.venv/bin/python scripts/isaac/dev_nochamfer_assets.py
"""
import json
import os

import numpy as np
import trimesh
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, Vt

ROOT = os.path.expanduser("~/ci_mpc_assets/dev_nochamfer")
HAND_TO_TCP = 0.1034
GRASP_BELOW_TOP = 0.010
L, CH, BLOCK, DEPTH, SEG = 0.040, 0.0005, 0.030, 0.015, 256
TASKS = {"dev_round_8mm": ("round", 0.0080, 0.0081), "dev_round_10mm": ("round", 0.0100, 0.01015),
         "dev_square_10mm": ("square", 0.0100, 0.01015), "dev_round_4mm": ("round", 0.0040, 0.00409),
         "dev_rect_12x8mm": ("rect", (0.0120, 0.0080), (0.01215, 0.00815))}  # id: (shape, peg width, hole width)


def plug_mesh(shape, w):
    if shape == "round":
        r = w / 2
        prof = [(0, 0), (r - CH, 0), (r, CH), (r, L), (0, L)]
        return trimesh.creation.revolve(prof, sections=SEG)
    wx, wy = (w, w) if shape == "square" else w  # rect: (x, y) widths
    pts = [(sx * (wx / 2 - d), sy * (wy / 2 - d), z) for z, d in ((0, CH), (CH, 0.0), (L, 0.0)) for sx in (-1, 1) for sy in (-1, 1)]
    return trimesh.convex.convex_hull(np.array(pts))


def _ray_square(h, th):  # where a ray from the centre at angle th meets the square of half width h (or rect (hx, hy))
    c, s = np.cos(th), np.sin(th)
    hx, hy = (h, h) if np.isscalar(h) else h
    return (1.0 / np.maximum(abs(c) / hx, abs(s) / hy))[:, None] * np.stack([c, s], -1)


def socket_mesh(shape, w):
    """Block with a through-hole, built from rays at SEG equal angles (no triangulation engine needed): inner ring =
    the hole (circle, or square; SEG divisible by 8 puts rays exactly on the square's corners), outer ring = the block
    square; quads between the rings on top / bottom, plus the inner and outer walls."""
    th = np.arange(SEG) * 2 * np.pi / SEG
    inner = (np.stack([np.cos(th), np.sin(th)], -1) * w / 2 if shape == "round"
             else _ray_square(w / 2 if shape == "square" else (w[0] / 2, w[1] / 2), th))
    outer = _ray_square(BLOCK / 2, th)
    v = np.concatenate([np.c_[r, np.full(SEG, z)] for r, z in ((inner, 0), (inner, DEPTH), (outer, 0), (outer, DEPTH))])
    ib, it, ob, ot = (np.arange(SEG) + k * SEG for k in range(4))
    n = np.roll(np.arange(SEG), -1)
    f = []
    for a, b in ((it, ot), (ob, ib), (ib, it), (ot, ob)):  # top, bottom, hole wall, outer wall
        f += [np.c_[a, b, b[n]], np.c_[a, b[n], a[n]]]
    m = trimesh.Trimesh(v, np.concatenate(f), process=True)
    trimesh.repair.fix_normals(m)
    return m


def write_obj(path, m):
    v, f = trimesh.remesh.subdivide_to_size(m.vertices, m.faces, max_edge=0.002)  # info() needs vertices along the sides
    with open(path, "w") as out:
        out.writelines(f"v {x:.7f} {y:.7f} {z:.7f}\n" for x, y, z in v)
        out.writelines(f"f {a + 1} {b + 1} {c + 1}\n" for a, b, c in f)


def write_usd(path, name, m, sdf_res):
    layer = Sdf.Layer.CreateNew(path)
    st = Usd.Stage.Open(layer)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    top = st.DefinePrim(f"/{name}", "Xform")
    st.SetDefaultPrim(top)
    body = UsdGeom.Xform.Define(st, f"/{name}/{name}").GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(body)
    mesh = UsdGeom.Mesh.Define(st, f"/{name}/{name}/collision")
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(m.vertices, np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(m.faces)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(np.asarray(m.faces).reshape(-1).tolist()))
    mesh.CreateSubdivisionSchemeAttr("none")
    p = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(p)
    UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("sdf")
    # PhysX schema as list-op metadata (plain pxr does not register PhysxSchema), as nist_assets.py
    p.SetMetadata("apiSchemas", Sdf.TokenListOp.Create(
        prependedItems=["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxSDFMeshCollisionAPI"]))
    p.CreateAttribute("physxSDFMeshCollision:sdfResolution", Sdf.ValueTypeNames.UInt).Set(sdf_res)
    layer.Save()


grasps, travel, meta = {}, {}, {}
for tid, (shape, wp, wh) in TASKS.items():
    d = os.path.join(ROOT, tid)
    os.makedirs(d, exist_ok=True)
    for kind, m, res in (("plug", plug_mesh(shape, wp), 256), ("socket", socket_mesh(shape, wh), 512)):
        assert m.is_watertight, (tid, kind)
        write_obj(os.path.join(d, f"{kind}.obj"), m)
        write_usd(os.path.join(d, f"{kind}.usd"), f"a_{tid}_{kind}", m, res)
    grasps[f"asset_{tid}"] = [0.0, 0.0, L - GRASP_BELOW_TOP + HAND_TO_TCP, 0.0, 1.0, 0.0, 0.0]
    travel[f"asset_{tid}"] = DEPTH
    meta[tid] = {"shape": shape, "peg_mm": np.round(np.array(wp) * 1000, 3).tolist(), "hole_mm": np.round(np.array(wh) * 1000, 3).tolist(),
                 "clearance_diametral_mm": np.round((np.array(wh) - np.array(wp)) * 1000, 3).tolist(),
                 "depth_mm": DEPTH * 1000, "peg_length_mm": L * 1000, "peg_end_chamfer_mm": CH * 1000, "hole_chamfer_mm": 0}
    print(tid, meta[tid])
for fn, obj in (("plug_grasps.json", grasps), ("disassembly_dist.json", travel), ("dev.json", meta)):
    json.dump(obj, open(os.path.join(ROOT, fn), "w"), indent=1)
