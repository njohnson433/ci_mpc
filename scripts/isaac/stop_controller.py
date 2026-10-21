# Isaac Sim Script Editor: stop the physics-step controller started by pd_home_controller.py
import builtins
if getattr(builtins, "_fr3_sub", None) is not None:
    builtins._fr3_sub.unsubscribe()
    builtins._fr3_sub = None
    print("controller stopped")
else:
    print("no controller running")
