"""Reader for the simulated wrist F/T log (<sync>/wrist_ft.csv, scripts/isaac/wrist_ft.py; WRIST_FT=1).

The CSV's time stamps (Kit timeline time, written at render frames) run WRIST_LAG behind the joint-state header
stamps that every other log uses (state["t"], bags, external_wrench.npy): at each probe press the TCP stops and the
observer rises at the same joint-state instant, the wrist force 0.246 s later; the observer-vs-wrist Fz
cross-correlation peaks at 0.230-0.245 s (median 0.235) in all 52 checked runs (peg, H1-H3, H3-100, H5, nochamfer,
real-arm held-out). Constant, not per episode (PROGRESS 2026-10-28 "RETRACTION"). Known since 2026-10-23
(compare_wrench.py: Isaac stamps /joint_states ~0.24 s behind /clock, i.e. behind the CSV's timeline time), but
later readers (benchmark peaks, hold_forces.py) dropped it.
"""
import numpy as np

WRIST_LAG = 0.235  # [s] subtract from the CSV stamps to get joint-state time


def hand_rotation(q):
    """Rotation matrices (n x 3 x 3) from hand quaternions (n x 4, w x y z)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], 1)


def load(path):
    """-> t (joint-state time), force, torque (n x 3 each) in the world frame, Franka sign (robot on environment)."""
    ft = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    R = hand_rotation(ft[:, 7:11])
    return (ft[:, 0] - WRIST_LAG, np.einsum("nij,nj->ni", R, ft[:, 1:4]), np.einsum("nij,nj->ni", R, ft[:, 4:7]))


def phase_peaks(path, t_hover0, phases):
    """Peak |F - tare| [N] per (name, a, b) phase (joint-state times), tared over the 0.25 s before t_hover0 (first
    free-air hover): the benchmark's true contact force (peak_force_wrist_engaged_N)."""
    t, f, _ = load(path)
    tw = (t >= t_hover0 - 0.25) & (t <= t_hover0)
    if not tw.any():
        return None
    fm = np.linalg.norm(f - np.median(f[tw], axis=0), axis=1)
    return [float(fm[(t >= a) & (t <= b)].max()) if ((t >= a) & (t <= b)).any() else 0.0 for _, a, b in phases]
