#!/usr/bin/env python3
# Build / update the success-guided start cache (ci_mpc_planner/start_cache.py) from selector_dagger.py episode
# files (<out>_eps.json): each episode's outcome updates the success estimate of its start (bin or snapshot), and
# its visited-state snapshots join the cache with the episode's outcome as their prior.
#   start_cache_update.py CACHE.json "DIR/*_eps.json" [--new] [--max-snaps 400]
import argparse
import glob
import json

import numpy as np

from ci_mpc_planner.start_cache import StartCache

p = argparse.ArgumentParser()
p.add_argument("cache")
p.add_argument("episodes", help="glob of *_eps.json")
p.add_argument("--new", action="store_true", help="start an empty cache")
p.add_argument("--max-snaps", type=int, default=400, help="keep the hardest (lowest success) snapshots")
args = p.parse_args()
c = StartCache() if args.new else StartCache.load(args.cache)
n_ep = 0
for f in sorted(glob.glob(args.episodes)):
    for e in json.load(open(f)):
        n_ep += 1
        off = np.array(e["off"])
        if e["kind"] in ("bin", "uniform"):
            c.update("bin", StartCache.bin_of(off), e["entered"])
        elif e["kind"] == "snap":
            c.update("snap", e["id"], e["entered"])
        for sn in e["snaps"]:
            c.add_snapshot(off, sn["x"], sn["u"], sn["step"], e["entered"])
if len(c.snaps) > args.max_snaps:  # keep the hardest; the snapshot ids of this round's episodes refer to the old list
    c.snaps = sorted(c.snaps, key=lambda s: (s["s"] + 1) / (s["n"] + 2))[:args.max_snaps]
c.save(args.cache)
print(f"{n_ep} episodes -> {json.dumps(c.summary())}")
