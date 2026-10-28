"""NIST Assembly Task Board #1 parts (held-out benchmark H5) in the AutoMate asset layout, so the ros2_control
pipeline loads them through automate_scene.py / demo_automate.sh unchanged (AUTOMATE_ASSETS=~/ci_mpc_assets/nist,
AUTOMATE=nist_round_8mm) and the batched env can read the same files later.
Source: NVIDIA's simulation-ready NIST USDs (SDF collision), Isaac/6.2/Isaac/IsaacLab/Factory/NIST/ on the Isaac asset
server, cached in ~/ci_mpc_assets/nist/src (license: NVIDIA Isaac asset terms, unverified: internal use only until
checked). Built per task <id> in ~/ci_mpc_assets/nist/<id>/:
  plug.usda, socket.usda  thin wrappers: body prim a_<id>_{plug,socket} referencing the source body (this also drops
                          the mis-declared stage metadata of round_hole_4mm / 8mm: metersPerUnit 0.01, upAxis Y, while
                          the points are in metres, Z-up), metersPerUnit 1, upAxis Z, SDF resolution (holes 512: the
                          8 mm hole has 0.054 mm radial clearance, ~0.1 mm voxels at 256)
  plug.obj, socket.obj    collision vertices (automate_scene.info() reads them)
and plug_grasps.json / disassembly_dist.json / nist.json (geometry, clearance) in ~/ci_mpc_assets/nist.
Frames as AutoMate: plug and socket share the seated frame (peg bottom at the hole bottom = the table, z 0; the
holes are 9 mm through-holes in a 25 x 25 x 9 mm block). Grasp: TCP 10 mm below the peg top (the Factory peg's).
    ~/IsaacLab/.venv/bin/python scripts/isaac/nist_assets.py
"""
import json
import os

import numpy as np
import trimesh
from pxr import Sdf, Usd, UsdGeom

ROOT = os.path.expanduser("~/ci_mpc_assets/nist")
HAND_TO_TCP = 0.1034
GRASP_BELOW_TOP = 0.010
TASKS = {  # id: (plug source, socket source, symmetry)
    "nist_round_4mm": ("round_peg_4mm", "round_hole_4mm", "round"),
    "nist_round_8mm": ("round_peg_8mm", "round_hole_8mm", "round"),
    "nist_round_12mm": ("round_peg_12mm", "round_hole_12mm", "round"),
    "nist_rect_8mm": ("rectangular_peg_8mm", "rectangular_hole_8mm", "2-fold"),
    "nist_rect_12mm": ("rectangular_peg_12mm", "rectangular_hole_12mm", "2-fold"),
}


def body_and_collision(path):
    st = Usd.Stage.Open(path)
    root = st.GetDefaultPrim()
    body = next(p for p in Usd.PrimRange(root) if p.HasAPI("PhysicsRigidBodyAPI"))
    col = next(p for p in Usd.PrimRange(body) if p.HasAPI("PhysicsCollisionAPI"))
    m = UsdGeom.Mesh(col)
    xf = UsdGeom.Xformable(col).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pts = np.array([xf.Transform(tuple(map(float, v))) for v in m.GetPointsAttr().Get()])
    counts, idx = m.GetFaceVertexCountsAttr().Get(), m.GetFaceVertexIndicesAttr().Get()
    return body.GetPath(), col.GetPath().MakeRelativePath(body.GetPath()), pts, counts, idx


def write_obj(path, pts, counts, idx):
    """Triangulated and subdivided to <= 2 mm edges (same surface): automate_scene.info() finds the grasp band from
    the vertices within 5 mm of the TCP height, and a cylinder mesh has vertices only at its ends."""
    faces, k = [], 0
    for c in counts:
        faces += [[idx[k], idx[k + j], idx[k + j + 1]] for j in range(1, c - 1)]
        k += c
    v, f = trimesh.remesh.subdivide_to_size(pts, np.array(faces), max_edge=0.002)
    with open(path, "w") as out:
        out.writelines(f"v {x:.7f} {y:.7f} {z:.7f}\n" for x, y, z in v)
        out.writelines(f"f {a + 1} {b + 1} {c + 1}\n" for a, b, c in f)


def wrapper(path, name, src, body, col_rel, sdf_res):
    layer = Sdf.Layer.CreateNew(path)
    st = Usd.Stage.Open(layer)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    top = st.DefinePrim(f"/{name}", "Xform")
    st.SetDefaultPrim(top)
    b = st.DefinePrim(f"/{name}/{name}")
    b.GetReferences().AddReference(os.path.relpath(src, os.path.dirname(path)), body)
    col = st.OverridePrim(b.GetPath().AppendPath(col_rel))
    # schema applied as list-op metadata: plain pxr (outside a running Isaac app) does not register the PhysX schemas
    col.SetMetadata("apiSchemas", Sdf.TokenListOp.Create(prependedItems=["PhysxSDFMeshCollisionAPI"]))
    col.CreateAttribute("physxSDFMeshCollision:sdfResolution", Sdf.ValueTypeNames.UInt).Set(sdf_res)
    layer.Save()


grasps, travel, meta = {}, {}, {}
for tid, (plug, sock, sym) in TASKS.items():
    d = os.path.join(ROOT, tid)
    os.makedirs(d, exist_ok=True)
    geo, mesh = {}, {}
    for kind, srcname, res in (("plug", plug, 256), ("socket", sock, 512)):
        src = os.path.join(ROOT, "src", f"{srcname}.usd")
        body, col_rel, pts, counts, idx = body_and_collision(src)
        write_obj(os.path.join(d, f"{kind}.obj"), pts, counts, idx)
        wrapper(os.path.join(d, f"{kind}.usda"), f"a_{tid}_{kind}", src, body, col_rel, res)
        geo[kind], mesh[kind] = pts, (counts, idx)
    pv, sv = geo["plug"], geo["socket"]
    top = pv[:, 2].max()
    grasps[f"asset_{tid}"] = [0.0, 0.0, top - GRASP_BELOW_TOP + HAND_TO_TCP, 0.0, 1.0, 0.0, 0.0]
    travel[f"asset_{tid}"] = float(sv[:, 2].max() - sv[:, 2].min())  # seated = peg bottom at the block bottom
    # clearance from cross-sections at mid-depth (hole: the section's inner ring; peg: its outline). Extent per axis
    # (x, y) for rectangles, equivalent-area diameter for round parts (the meshes are polygons: vertex radii overstate)
    def section(v, counts, idx, z, ring):
        faces, k = [], 0
        for c in counts:
            faces += [[idx[k], idx[k + j], idx[k + j + 1]] for j in range(1, c - 1)]
            k += c
        m = trimesh.Trimesh(v, np.array(faces), process=True)
        poly = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1]).to_2D()[0].polygons_full[0]
        c = np.array(poly.interiors[0].coords if ring else poly.exterior.coords)
        area = abs(trimesh.path.polygons.Polygon(c).area)
        return np.ptp(c, 0), 2 * np.sqrt(area / np.pi)
    zm = sv[:, 2].max() / 2
    (h_ext, h_d), (p_ext, p_d) = section(sv, *mesh["socket"], zm, True), section(pv, *mesh["plug"], zm, False)
    hole, peg = (np.full(2, h_d), np.full(2, p_d)) if sym == "round" else (h_ext, p_ext)
    meta[tid] = {"plug_src": plug, "socket_src": sock, "symmetry": sym, "hole_mm": np.round(hole * 1000, 3).tolist(),
                 "peg_mm": np.round(peg * 1000, 3).tolist(), "clearance_diametral_mm": np.round((hole - peg) * 1000, 3).tolist(),
                 "depth_mm": round(travel[f"asset_{tid}"] * 1000, 2), "peg_length_mm": round(float(top) * 1000, 2)}
    print(tid, meta[tid])
for fn, obj in (("plug_grasps.json", grasps), ("disassembly_dist.json", travel), ("nist.json", meta)):
    json.dump(obj, open(os.path.join(ROOT, fn), "w"), indent=1)
