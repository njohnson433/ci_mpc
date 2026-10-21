# Troubleshooting / lessons learned

## NVIDIA driver

**`nvidia-smi`: "couldn't communicate with the NVIDIA driver" after installing 595**
The driver package was installed (`iU`) but no kernel module existed for the running kernel.
Install the pre-built, signed module for that kernel:
```bash
sudo apt install linux-modules-nvidia-595-generic-hwe-22.04 linux-modules-nvidia-595-$(uname -r)
sudo dpkg --configure -a && sudo apt -f install && sudo reboot
```

**`Autoinstall on ... failed for module(s) nvidia` / dpkg errors on every `apt install`**
`nvidia-dkms-595` was installed alongside the pre-built modules, and its DKMS build fails.
It's not needed:
```bash
sudo apt remove --dry-run nvidia-dkms-595   # check only nvidia-dkms-595 is removed
sudo apt remove nvidia-dkms-595
sudo dpkg --configure -a && sudo apt -f install
```
Don't reboot until `linux-image-$(uname -r)` shows `ii` again.

## Python environments

**uv warning `VIRTUAL_ENV=... does not match the project environment`**
The `cimpc` venv was active in the Isaac terminal. Run `deactivate`.

**`uv pip show isaacsim` → not found**
It checked the active venv. Use `uv pip show --python .venv isaacsim` inside `~/IsaacLab`.

## Isaac Sim

**File dialog: `Item at '~/...' not found`** – type full paths (`/home/njohnson/...`).

**Menu bar missing** – Isaac is still loading (watch the % at the bottom right). Maximize the window if needed.

**`JointStateSensor: no DOFs found under articulation root '/fr3'`**
The articulation root is `/fr3/Geometry/base/fr3_link0`. Point `ReadJointState.inputs:prim`
and `ArticulationController.targetPrim` there. Do **not** change
`/fr3/Physics/fr3_base_joint.physics:body0`, which fixes the base to the world.

**`Ctrl+C` in the Isaac terminal kills Isaac Sim.** Keep one terminal only for Isaac.

**GUI very slow** – physics at 1000 Hz. Use 240 Hz for interactive work.

**Ground turns white on Play** – rendering exposure, not physics. Ignore.

**Hand tilts at Play** – the start pose is all-zero, and FR3 joint 4 can't reach 0 (limit ≈ −0.07 rad). Harmless.

## Torque control through ROS topics doesn't work

Tried: Isaac publishes `/joint_states` → external Python node (Pinocchio gravity compensation)
→ `/joint_command` (effort) → Isaac.

What happened: the arm wobbled with a PD to home, and with gravity compensation alone it slowly
sank and drifted, at 30 and 60 Hz physics alike.

Why: the graph runs once per rendered frame, and the sim ran at about 0.25× real time
(`/joint_states` ~8 Hz wall-clock, but every message 1/30 s sim time apart). Commanded torques
only act in frames where a new message happened to arrive. An external loop tied to the frame
rate can't do torque control.

Fix:
1. **Disable gravity on the robot links** (`scripts/isaac/disable_robot_gravity.py`). This
   matches the real FR3, where libfranka torques exclude gravity.
2. **Run the controller every physics step inside Isaac**, either as a physics callback
   (`scripts/isaac/pd_home_controller.py`) or via `isaacsim.ros2.control` (in-process
   Controller Manager). Use ROS only for the slow MPC setpoints.

Masses were checked and match between the URDF (Pinocchio) and Isaac: 18.604 kg total.

## franka_ros2

**Two libfrankas** – the humble `dependency.repos` pins libfranka 0.20.4. Delete `src/libfranka`
after `vcs import` and use the system 0.21.3.

**`controllers.yaml`** – check for tabs:
```bash
grep -nP '\t' src/franka_bringup/config/controllers.yaml
python3 -c "import yaml; yaml.safe_load(open('src/franka_bringup/config/controllers.yaml')); print('ok')"
```
