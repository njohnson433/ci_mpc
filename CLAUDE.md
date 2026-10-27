# ci_mpc: notes for Claude sessions

Contact-implicit MPC on a Franka FR3, Isaac Sim first, then the real robot. See `README.md`.

## Start of every session
1. Read `docs/PROGRESS.md` ("Where to resume" + log) and `docs/ROADMAP.md`.
2. Check for leftovers: `ps` for Isaac / `ros2` / `controller_manager` processes from a crashed session.

## Keep the backup current (this repo is the backup)
- After each meaningful step, add a dated entry to `docs/PROGRESS.md` (what was done, result + why,
  bugs found) and update "Where to resume". Tick boxes in `docs/ROADMAP.md`.
- Controller code lives in `../cartesian_impedance_control` (branch `isaac-sim-pinocchio`, pushed to
  `LucasG2001/cartesian_impedance_control`; never merge into its `main`). After changing it: commit there, push the
  branch (`git -C ../cartesian_impedance_control push origin isaac-sim-pinocchio`; allowed by a rule in
  `~/.claude/settings.json`), regenerate `patches/` (command in PROGRESS.md).
- Whenever writing or updating a README (incl. the nightly `scripts/nightly_update.sh` run), invoke the
  `create-readme` skill (`.claude/skills/create-readme`) and follow it.
- Commit and push this repo when the user asks (they asked for regular backup commits on 2026-10-22).

## Facts that bite
- Isaac: articulation root `/fr3/Geometry/base/fr3_link0`; robot gravity off; effort mode; the controller
  must run inside Isaac's physics step (`isaacsim.ros2.control`), not via ROS topics.
- Don't source `/opt/ros` in the Isaac environment (Python 3.10 vs Isaac's 3.12); use `scripts/isaac_ros.sh`.
- Link Pinocchio before libfranka (libfranka embeds a Pinocchio with a different layout).
- Since 2026-10-27 on omar-OMEN: RTX 4090 24 GB, 62 GB RAM (old box: RTX 4080 16 GB, 31 GB). The GPU may be
  shared with other jobs; check `nvidia-smi` before starting Isaac.
- Each headless Isaac instance takes ~6 GB RSS, each GPU MPC server ~1.4 GB. Old box limit: 4 Isaac instances
  (5 plus 3 GPU servers filled 31 GB RAM and swap, 2026-10-25); re-measure the limit on the new box.
- numpy 2 lives in `/usr/local` here (ZED SDK): ROS Python needs the `~/.local/ros_numpy1` shim
  (`ros_franka` alias / `run_impedance_test.sh` add it; MIGRATION.md B.3a).
- No Isaac / MPC runs while the real FR3 is in use: with the machine loaded (Isaac + MPC, load 15, GPU 97 %) the robot
  stopped with `communication_constraints_violation` reflexes (non-RT kernel, even SCHED_FIFO 98; 2026-10-27).
  Overnight queues must end before the robot session (the robot-side session messages when it starts / ends).
- Use an isolated `ROS_DOMAIN_ID` (e.g. 42) for tests. Kill processes by exact PID (never `pgrep -f`/`pkill -f`
  from a one-line shell: it matches the shell itself).
- Isaac ros2_control: Controller Manager targetPrim is `/fr3` (joints live in `/fr3/Physics`); the scene needs
  `scripts/isaac/prepare_scene.py` (physx variant + FR3 joint armature, else the wrist chatters at 240 Hz,
  + identified joint friction from the controller header; `--no-friction` to remove).
- Test: `scripts/isaac/run_impedance_test.sh` (headless, ~1 min); `--dump x.npz`, `CONTROLLER_CONFIG=...`.
