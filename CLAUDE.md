# ci_mpc: notes for Claude sessions

Contact-implicit MPC on a Franka FR3, Isaac Sim first, then the real robot. See `README.md`.

## Start of every session
1. Read `docs/PROGRESS.md` ("Where to resume" + log) and `docs/ROADMAP.md`.
2. Check for leftovers: `ps` for Isaac / `ros2` / `controller_manager` processes from a crashed session.

## Keep the backup current (this repo is the backup)
- After each meaningful step, add a dated entry to `docs/PROGRESS.md` (what was done, result + why,
  bugs found) and update "Where to resume". Tick boxes in `docs/ROADMAP.md`.
- Controller code lives in `../cartesian_impedance_control` (branch `isaac-sim-pinocchio`, upstream we
  can't push to). After changing it: commit there, regenerate `patches/` (command in PROGRESS.md).
- Commit and push this repo when the user asks (they asked for regular backup commits on 2026-10-22).

## Facts that bite
- Isaac: articulation root `/fr3/Geometry/base/fr3_link0`; robot gravity off; effort mode; the controller
  must run inside Isaac's physics step (`isaacsim.ros2.control`), not via ROS topics.
- Don't source `/opt/ros` in the Isaac environment (Python 3.10 vs Isaac's 3.12); use `scripts/isaac_ros.sh`.
- Link Pinocchio before libfranka (libfranka embeds a Pinocchio with a different layout).
- The GPU (RTX 4080, 16 GB) may be shared with other jobs; check `nvidia-smi` before starting Isaac.
- Use an isolated `ROS_DOMAIN_ID` (e.g. 42) for tests. Kill processes by exact PID.
