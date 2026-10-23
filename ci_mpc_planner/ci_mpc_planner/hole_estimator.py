"""Hole localization from contact (particle filter over the hole position along the plane's x axis).

Each particle is a hole position h. After every control step, each particle predicts with the quasi-static contact
model (PegHole.step) where the TCP ends up for the command that was applied and which contact wrench it feels;
the weights follow the mismatch to the measured TCP pose and the measured external wrench (the controller's
estimate). No contact: all particles predict the same, nothing is learned; the peg resting on the top face rules
out holes under the tip; a one-sided support shows up as a torque about the TCP and as tilt.
"""
import numpy as np

from ci_mpc_planner.peg_hole import PegHole


class HoleEstimator:
    def __init__(self, geometry, compliance, center, spread=0.006, n=60, sigma_x=(2e-4, 2e-4, 2e-3),
                 sigma_w=(1.0, 1.0, 0.02), jitter=1e-4, seed=0, arm_friction=None):
        self.rng = np.random.default_rng(seed)
        self.geo, self.compliance = geometry, np.asarray(compliance, float)
        self.h = center + np.linspace(-spread, spread, n)
        self.logw = np.zeros(n)
        self.sigma_x, self.sigma_w = np.asarray(sigma_x), np.asarray(sigma_w)
        self.jitter = jitter
        self.arm_friction = arm_friction  # the predictions need the arm's stiction too (else "stuck" reads as contact)
        self.models = [PegHole(geometry, h) for h in self.h]

    def update(self, x_prev, u, x_meas, wrench_meas):
        """x_prev: TCP pose before the step, u: command applied, x_meas: TCP pose after, wrench_meas: contact wrench
        on the peg (Fx, Fz, torque about y at the TCP)."""
        ll = np.zeros(len(self.h))
        for j, mdl in enumerate(self.models):
            mdl.hole_x = self.h[j]
            xp, lam, nc = mdl.step(x_prev, u, self.compliance, self.arm_friction)
            wp = mdl.contact_wrench(x_prev, lam, self.compliance) if nc else np.zeros(3)
            ll[j] = -0.5 * np.sum(((xp - x_meas) / self.sigma_x) ** 2) - 0.5 * np.sum(((wp - wrench_meas) / self.sigma_w) ** 2)
        self.logw += ll
        self.logw -= self.logw.max()
        w = self.weights()
        ess = 1.0 / np.sum(w ** 2)
        if ess < 0.3 * len(self.h):
            idx = self.rng.choice(len(self.h), len(self.h), p=w)
            self.h = self.h[idx] + self.rng.normal(0, self.jitter, len(self.h))
            self.logw = np.zeros(len(self.h))

    def weights(self):
        w = np.exp(self.logw - self.logw.max())
        return w / w.sum()

    def estimate(self):
        """(estimate, std): the weighted mean of the particles near the most likely one once the belief has a peak;
        before that (no informative contact yet) the plain weighted mean (a flat belief's argmax is just the first
        particle: Isaac runs moved 5.6 mm towards it before touching anything and missed holes at 0-2 mm)."""
        w = self.weights()
        if w.max() < 2.0 / len(w):
            mean = float(np.sum(w * self.h))
            return mean, float(np.sqrt(np.sum(w * (self.h - mean) ** 2)))
        best = self.h[np.argmax(w)]
        near = np.abs(self.h - best) < 0.001
        mean = np.sum(w[near] * self.h[near]) / np.sum(w[near])
        std = np.sqrt(np.sum(w * (self.h - np.sum(w * self.h)) ** 2))
        return mean, std
