"""Time-parameterized end-effector path through waypoints, for the MPC to track.

A path starts at the current pose, passes the intermediate waypoints (via points, corners rounded with circular
arcs of radius `blend`, limited to 40 % of the adjacent segments) and stops at the last one. The translation is
timed with a speed profile: |v| <= v_max along the path, |a| <= a_max (tangential), v <= sqrt(a_max * r) in the
arcs (centripetal), v = 0 at both ends. The rotation vector (relative to the MPC's start orientation) is
interpolated linearly in arc length between the waypoints.
"""
import numpy as np


class BlendedPath:
    def __init__(self, points, rotvecs, v_max=0.25, a_max=0.4, blend=0.03, ds=5e-4):
        P = np.asarray(points, dtype=float)
        RV = np.asarray(rotvecs, dtype=float)
        keep = [0] + [i for i in range(1, len(P)) if np.linalg.norm(P[i] - P[i - 1]) > 1e-6]
        P, RV = P[keep], RV[keep]
        self.duration = 0.0
        if len(P) < 2:
            self.s = np.zeros(1)
            self.p = P[:1].copy()
            self.rv = RV[:1].copy()
            self.t = np.zeros(1)
            self.v = np.zeros(1)
            self.tangent = np.zeros((1, 3))
            return
        # geometric path: straight segments with circular arcs at the inner waypoints
        samples, rv_samples, radii = [P[0]], [RV[0]], [np.inf]
        start = P[0]
        for i in range(1, len(P) - 1):
            a, b, c = P[i - 1], P[i], P[i + 1]
            u1, u2 = (b - a) / np.linalg.norm(b - a), (c - b) / np.linalg.norm(c - b)
            cos_t = np.clip(u1 @ u2, -1.0, 1.0)
            theta = np.arccos(cos_t)  # turning angle
            if theta < 1e-3:
                continue
            # distance from the corner to the tangent points: d = r tan(theta / 2), limited to 40 % of the segments
            d = min(blend * np.tan(theta / 2), 0.4 * np.linalg.norm(b - a), 0.4 * np.linalg.norm(c - b))
            r = d / np.tan(theta / 2)
            t1, t2 = b - u1 * d, b + u2 * d
            self._line(samples, rv_samples, radii, start, t1, ds)
            # arc from t1 to t2 around the center, in the plane of u1, u2
            n = (u2 - u1) / np.linalg.norm(u2 - u1)  # points to the center from the corner side
            center = t1 + n * r
            e1, e2 = t1 - center, t2 - center
            phi = np.arccos(np.clip(e1 @ e2 / (r * r), -1.0, 1.0))
            k = max(2, int(np.ceil(r * phi / ds)))
            w = np.cross(e1, e2)
            w = w / np.linalg.norm(w)
            for j in range(1, k + 1):
                ang = phi * j / k
                pt = center + e1 * np.cos(ang) + np.cross(w, e1) * np.sin(ang)
                samples.append(pt)
                radii.append(r)
            start = t2
        self._line(samples, rv_samples, radii, start, P[-1], ds)
        self.p = np.array(samples)
        seg = np.linalg.norm(np.diff(self.p, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        # rotation: linear in arc length between the waypoints (waypoint i at the path point closest to it)
        s_way = [0.0]
        for i in range(1, len(P) - 1):
            s_way.append(self.s[np.argmin(np.linalg.norm(self.p - P[i], axis=1))])
        s_way.append(self.s[-1])
        self.rv = np.stack([np.interp(self.s, s_way, RV[:, j]) for j in range(3)], axis=1)
        # speed profile: curvature limit, then forward / backward passes with a_max
        radii = np.array(radii[:len(self.s)] + [np.inf] * (len(self.s) - len(radii)))
        v_lim = np.minimum(v_max, np.sqrt(a_max * radii))
        v = v_lim.copy()
        v[0] = v[-1] = 0.0
        for i in range(1, len(v)):
            v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * a_max * seg[i - 1]))
        for i in range(len(v) - 2, -1, -1):
            v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * a_max * seg[i]))
        self.v = v
        v_mid = np.maximum((v[1:] + v[:-1]) / 2, 1e-4)
        self.t = np.concatenate([[0.0], np.cumsum(seg / v_mid)])
        self.duration = self.t[-1]
        tan = np.gradient(self.p, axis=0)
        self.tangent = tan / np.maximum(np.linalg.norm(tan, axis=1, keepdims=True), 1e-12)

    @staticmethod
    def _line(samples, rv_samples, radii, a, b, ds):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / ds)))
        for j in range(1, n + 1):
            samples.append(a + (b - a) * j / n)
            radii.append(np.inf)

    def sample(self, times):
        """Position, velocity, rotation vector at the given times (clamped to [0, duration])."""
        times = np.clip(np.asarray(times, dtype=float), 0.0, self.duration)
        p = np.stack([np.interp(times, self.t, self.p[:, j]) for j in range(3)], axis=1)
        speed = np.interp(times, self.t, self.v)
        tan = np.stack([np.interp(times, self.t, self.tangent[:, j]) for j in range(3)], axis=1)
        tan /= np.maximum(np.linalg.norm(tan, axis=1, keepdims=True), 1e-12)
        rv = np.stack([np.interp(times, self.t, self.rv[:, j]) for j in range(3)], axis=1)
        return p, speed[:, None] * tan, rv
