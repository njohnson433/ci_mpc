import numpy as np

from ci_mpc_planner.c3 import PegHoleC3
from ci_mpc_planner.hole_estimator import HoleEstimator
from ci_mpc_planner.peg_hole import PegHole

COMP = np.array([1 / 1500, 1 / 500, 1 / 130.0])


def run(policy, hole_x, steps=200):
    plant, model = PegHole(hole_x=hole_x), PegHole(hole_x=0.0)
    tb = plant.g.tip_below_tcp
    x = np.array([0.0, tb + 0.002, 0.0])
    u, goal = x.copy(), np.array([0.0, tb - plant.g.depth, 0.0])
    est = HoleEstimator(plant.g, COMP, 0.0, n=30)
    mpc = PegHoleC3(model, COMP)
    for _ in range(steps):
        if policy == "c3":
            u, _ = mpc.solve(x, goal, u)
        else:
            u = u.copy()
            u[0] += np.clip(goal[0] - u[0], -5e-4, 5e-4)
            u[1] = max(u[1] - 5e-4, goal[1] - 0.006)
        x_prev = x
        x, lam, nc = plant.step(x, u, COMP)
        est.update(x_prev, u, x, plant.contact_wrench(x_prev, lam, COMP) if nc else np.zeros(3))
        h, _ = est.estimate()
        model.hole_x = goal[0] = h
        if plant.tip(x)[1] < -plant.g.depth + 0.001:
            return True
    return False


def test_plant_jams_on_chamfer_without_localization():
    plant = PegHole(hole_x=0.002)
    x = np.array([0.0, 0.042, 0.0])
    u = x.copy()
    for _ in range(200):
        u = u.copy()
        u[1] = max(u[1] - 2e-4, 0.032)
        x, _, _ = plant.step(x, u, COMP)
    assert plant.tip(x)[1] > -0.002  # stuck at the opening


def test_estimator_plus_straight_down_inserts_at_3mm():
    assert run("down", 0.003)


def test_c3_inserts_at_3mm():
    assert run("c3", 0.003, steps=150)
