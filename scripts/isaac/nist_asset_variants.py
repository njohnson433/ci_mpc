"""NIST H5 asset-fidelity check (item 4): collision variants of nist_assets.py's wrappers, one lever each, in separate
asset roots (AUTOMATE_ASSETS=~/ci_mpc_assets/nist_var/<variant>), for an at-0-offset insertion check of the tight
parts (src-f1: rect 12 fails at 0 with plain; round 4 thin-peg failures). Not a benchmark: H5 results are re-measured
once with the frozen assets afterwards.
  base       as nist_assets.py (plug SDF 256 ~0.2 mm voxels on the 50 mm peg, socket SDF 512)
  plug1024   plug SDF 1024 (~0.05 mm voxels)
  plughull   plug collision = convex hull (the pegs are convex: exact), socket unchanged
  densesock  socket collision mesh subdivided to <= 1.5 mm edges (NIST sockets have 16-48 vertices; the dev sockets
             that insert have 1024), SDF 512 as before
    ~/IsaacLab/.venv/bin/python scripts/isaac/nist_asset_variants.py
"""
import os
import shutil

import numpy as np
import trimesh
from pxr import Sdf, Usd, UsdGeom, Vt

SRC_ROOT = os.path.expanduser("~/ci_mpc_assets/nist")
OUT = os.path.expanduser("~/ci_mpc_assets/nist_var")
TASKS = {"nist_rect_12mm": ("rectangular_peg_12mm", "rectangular_hole_12mm"),
         "nist_round_4mm": ("round_peg_4mm", "round_hole_4mm")}
VARIANTS = {"base": {}, "plug1024": {"plug_res": 1024}, "plughull": {"plug_hull": True},
            "densesock": {"dense_socket": 0.0015}}


def source_parts(path):
    st = Usd.Stage.Open(path)
    body = next(p for p in Usd.PrimRange(st.GetDefaultPrim()) if p.HasAPI("PhysicsRigidBodyAPI"))
    col = next(p for p in Usd.PrimRange(body) if p.HasAPI("PhysicsCollisionAPI"))
    m = UsdGeom.Mesh(col)
    return (body.GetPath(), col.GetPath().MakeRelativePath(body.GetPath()), np.array(m.GetPointsAttr().Get()),
            list(m.GetFaceVertexCountsAttr().Get()), list(m.GetFaceVertexIndicesAttr().Get()))


def wrapper(path, name, src, kind, res=None, hull=False, dense=None):
    body, col_rel, pts, counts, idx = source_parts(src)
    layer = Sdf.Layer.CreateNew(path)
    st = Usd.Stage.Open(layer)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    top = st.DefinePrim(f"/{name}", "Xform")
    st.SetDefaultPrim(top)
    b = st.DefinePrim(f"/{name}/{name}")
    b.GetReferences().AddReference(os.path.relpath(src, os.path.dirname(path)), body)
    col = st.OverridePrim(b.GetPath().AppendPath(col_rel))
    if hull:
        col.CreateAttribute("physics:approximation", Sdf.ValueTypeNames.Token).Set("convexHull")
    else:
        col.SetMetadata("apiSchemas", Sdf.TokenListOp.Create(prependedItems=["PhysxSDFMeshCollisionAPI"]))
        col.CreateAttribute("physxSDFMeshCollision:sdfResolution", Sdf.ValueTypeNames.UInt).Set(res)
    if dense:
        faces, k = [], 0
        for c in counts:
            faces += [[idx[k], idx[k + j], idx[k + j + 1]] for j in range(1, c - 1)]
            k += c
        v, f = trimesh.remesh.subdivide_to_size(pts, np.array(faces), max_edge=dense)
        m = UsdGeom.Mesh(col)
        m.CreatePointsAttr().Set(Vt.Vec3fArray.FromNumpy(v.astype(np.float32)))
        m.CreateFaceVertexCountsAttr().Set(Vt.IntArray([3] * len(f)))
        m.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(f.reshape(-1).tolist()))
        print(f"  {name}: collision mesh {len(pts)} -> {len(v)} vertices")
    layer.Save()


for var, opt in VARIANTS.items():
    root = os.path.join(OUT, var)
    os.makedirs(root, exist_ok=True)
    for f in ("plug_grasps.json", "disassembly_dist.json", "nist.json"):
        shutil.copy(os.path.join(SRC_ROOT, f), root)
    for tid, (plug, sock) in TASKS.items():
        d = os.path.join(root, tid)
        os.makedirs(d, exist_ok=True)
        for kind in ("plug", "socket"):
            shutil.copy(os.path.join(SRC_ROOT, tid, f"{kind}.obj"), d)
            p = os.path.join(d, f"{kind}.usd")
            if os.path.exists(p):
                os.remove(p)
        wrapper(os.path.join(d, "plug.usd"), f"a_{tid}_plug", os.path.join(SRC_ROOT, "src", f"{plug}.usd"), "plug",
                res=opt.get("plug_res", 256), hull=opt.get("plug_hull", False))
        wrapper(os.path.join(d, "socket.usd"), f"a_{tid}_socket", os.path.join(SRC_ROOT, "src", f"{sock}.usd"),
                "socket", res=512, dense=opt.get("dense_socket"))
    print(var, "->", root)
