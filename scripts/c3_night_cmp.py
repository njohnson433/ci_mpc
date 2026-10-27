# Pool C3 / plain D1 queue runs per arm (2026-10-27 overnight, PROGRESS "C3 overnight result"). -v: per offset.
import json, glob, os, sys, numpy as np
R = os.path.expanduser("~/ci_mpc_runs/c3_isaac"); N = R + "/night"
groups = {
 "M (meas, no ff)": ["d1_meas/c3"],
 "MF (ff 1.5/1.3)": ["d1_meas_ff/c3", "d1_meas_ff_r2/c3"],
 "MF2 (ff 2.2/2.0)": ["d1_meas_ff2/c3", "d1_meas_ff2_r2/c3", "night/mf2_r3/c3", "night/mf2_r4/c3"],
 "MF3 (ff 2.9/2.7)": [f"night/mf3_r{i}/c3" for i in range(1, 5)],
 "MF2 lat<=5mm": [f"night/mf2lat5_r{i}/c3" for i in range(1, 4)],
 "plain (nominal)": [f"night/plain_r{i}/plain" for i in range(1, 4)],
 "real C3": [f"night/real_c3_r{i}/c3" for i in range(1, 4)],
 "real plain": [f"night/real_plain_r{i}/plain" for i in range(1, 4)],
}
per = {}
for name, ds in groups.items():
    rs = []
    for d in ds:
        for f in sorted(glob.glob(f"{R}/{d}/rep1/probe_*/res.json")):
            r = json.load(open(f)); r["off"] = os.path.basename(os.path.dirname(f))[6:]; r["src"] = d; rs.append(r)
    if not rs: continue
    ho = [h for r in rs for h in r.get("c3_handovers", [])]
    cs = np.array([r["contact_phase_s"] for r in rs]); pk = np.array([r.get("peak_force_engaged_N", np.nan) for r in rs])
    tot = np.array([r["total_s"] for r in rs])
    ins = sum(r["success"] for r in rs)
    deep = sum(h[2] >= 21.9 for h in ho)
    print(f"{name:18s} reps {len(ds)} n {len(rs):2d} ins {ins:2d} | contact mean/med {cs.mean():5.1f}/{np.median(cs):5.1f} s | engaged peak med/p90 {np.nanmedian(pk):5.2f}/{np.nanpercentile(pk,90):5.2f} N | "
          + (f"C3 phases {len(ho):2d} >=21.9mm {deep:2d} ({deep/len(ho):.0%}) depth med {np.median([h[2] for h in ho]):5.1f}" if ho else ""))
    per[name] = rs
if "-v" in sys.argv:
    for name, rs in per.items():
        print("==", name)
        offs = sorted(set(r["off"] for r in rs))
        for o in offs:
            sel = [r for r in rs if r["off"] == o]
            print(f"  {o:6s}", " ".join(f"{'I' if r['success'] else 'x'}{r['contact_phase_s']:4.1f}" + ("/" + ",".join(f"{h[2]:.0f}" for h in r.get('c3_handovers', [])) if r.get('c3_handovers') else "") for r in sel))
