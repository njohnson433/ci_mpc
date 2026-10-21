# Isaac Sim Script Editor (simulation playing): joint-space PD to the Franka home pose,
# applied every physics step via a PhysX step callback. Test bed for in-sim torque control.
# Re-run after every Stop -> Play. Stop with stop_controller.py.
import builtins
import numpy as np
import omni.physx
from isaacsim.core.experimental.prims import Articulation

if getattr(builtins, "_fr3_sub", None) is not None:
    builtins._fr3_sub.unsubscribe()
    builtins._fr3_sub = None

ROOT = "/fr3/Geometry/base/fr3_link0"
art = Articulation(ROOT)
names = list(art.dof_names)
ARM = [names.index(f"fr3_joint{i}") for i in range(1, 8)]
art.switch_dof_control_mode("effort")

Q_HOME = np.array([0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, np.pi/4])
KP = np.array([40, 40, 40, 40, 10, 10, 5], dtype=float)
KD = np.array([6, 6, 6, 6, 1, 1, 0.5], dtype=float)
TAU_MAX = np.array([87, 87, 87, 87, 12, 12, 12], dtype=float)   # FR3 joint torque limits

def _np(x):
    x = x.numpy() if hasattr(x, "numpy") else x
    return np.asarray(x, dtype=float).reshape(-1)

state = {"n": 0, "t": 0.0}

def on_step(dt):
    try:
        q, qd = _np(art.get_dof_positions()), _np(art.get_dof_velocities())
        tau = np.zeros(len(names))
        tau[ARM] = np.clip(KP * (Q_HOME - q[ARM]) - KD * qd[ARM], -TAU_MAX, TAU_MAX)
        art.set_dof_efforts(tau)
        state["n"] += 1; state["t"] += dt
        if state["t"] >= 2.0:
            print(f"steps/s(sim): {state['n']/state['t']:.0f}  max joint err: {np.abs(Q_HOME - q[ARM]).max():.4f} rad")
            state["n"], state["t"] = 0, 0.0
    except Exception as e:
        print("ctrl error:", e)
        builtins._fr3_sub.unsubscribe(); builtins._fr3_sub = None

builtins._fr3_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_step)
print("controller running, DOFs:", names)
