"""Amortized 3D CI-MPC (step 3): an MLP trained to imitate the slow, reliable C3+ teacher (rollout warm start on
every solve, ~1.5 s per solve in Python) on the engaged phase of the peg insertion. NumPy inference (~0.05 ms);
the weights come from scripts/train_c3_student.py.

Features (as in scripts/c3_teacher_data.py): TCP relative to the estimated hole (x, y [mm], tip depth below the
opening [mm], rx, ry [mrad]) and the measured contact wrench (Fx, Fy, Fz [N], Mx, My [N m]).
Output: command step u - x (mm, mm, mm, mrad, mrad; includes the spring deflection the teacher commands), clipped
to +-10 as a safety bound.
"""
import numpy as np


class C3Student:
    def __init__(self, path, du_max=(0.5, 0.5, 0.5, 10.0, 10.0)):
        d = np.load(path)
        self.W = [(d["W0"], d["b0"]), (d["W1"], d["b1"]), (d["W2"], d["b2"])]
        self.mu_x, self.sd_x, self.mu_y, self.sd_y = d["mu_x"], d["sd_x"], d["mu_y"], d["sd_y"]
        self.du_max = np.asarray(du_max)

    def features(self, x, est_xy, wrench, tip_below_tcp):
        return np.concatenate([(x[:2] - est_xy) * 1000, [(x[2] - tip_below_tcp) * 1000], x[3:] * 1000, wrench])

    def command(self, x, est_xy, wrench, tip_below_tcp):
        h = (self.features(x, est_xy, wrench, tip_below_tcp) - self.mu_x) / self.sd_x
        for i, (W, b) in enumerate(self.W):
            h = W @ h + b
            if i < len(self.W) - 1:
                h = np.tanh(h)
        step = np.clip(h * self.sd_y + self.mu_y, -10.0, 10.0)
        return x + step / 1000.0
