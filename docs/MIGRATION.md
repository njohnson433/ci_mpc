# Moving ci_mpc to another workstation

Checklist for continuing the project on a new machine. Written 2026-10-27 from the state of the lab
workstation (Ubuntu 22.04.5, RTX 4080, driver 595.99.02). Install steps are in `SETUP.md`; this file
lists **what** to bring over and in which order. Paths are hard-coded to `/home/njohnson/...` in ~40
places (scripts, configs, `isaac_ros.sh`), so use the **same username and layout** on the new machine;
otherwise you have to fix those paths first (`grep -rn /home/njohnson`).

---

## A. On the old machine, before leaving

1. **Commit and push `ci_mpc`.** All 9 newest commits on `main` are local only (`ahead 9`). Uncommitted at the time of writing:
   `scripts/{c3_stall_diag,peg_insert_demo}.py`, `scripts/isaac/{batched_probe,factory_fr3}.py`,
   `scripts/queue_c3_isaac.sh`, new `scripts/c3_isaac_replay.py`, `scripts/isaac/fidelity_probe_replay.py`.
   ```bash
   cd ~/franka_ros2_ws/src/ci_mpc
   git add -A && git commit -m "WIP before moving workstation" && git push origin main
   ```
2. **Controller patches are up to date** (checked 2026-10-27). If you change the controller again:
   ```bash
   cd ~/franka_ros2_ws/src/cartesian_impedance_control
   git format-patch main..isaac-sim-pinocchio --stdout > ../ci_mpc/patches/cartesian_impedance_control_isaac-sim-pinocchio.patch
   cd .. && git diff franka_bringup/config/controllers.yaml > ci_mpc/patches/franka_ros2_humble_controllers_yaml.patch
   ```
   (`isaac-sim-pinocchio` = 7 commits on top of `main` f8d5fe2; upstream LucasG2001, no push access, so
   the patch is the only copy.)
3. **Disable the nightly cron job** (`55 23 * * * .../scripts/nightly_update.sh`) on whichever machine you stop
   using, so the two don't push to `origin/main` against each other: `crontab -e`.
4. **Copy the data that is not in git** (≈ 1.2 GB) to a drive or straight to the new machine:
   ```bash
   rsync -a --info=progress2 ~/ci_mpc_assets ~/ci_mpc_runs ~/isaac_ros.sh NEWHOST:~/
   rsync -a ~/.claude/CLAUDE.md NEWHOST:~/.claude/
   rsync -a ~/.claude/projects/-home-njohnson-franka-ros2-ws-src/memory NEWHOST:~/.claude/projects/-home-njohnson-franka-ros2-ws-src/
   ```

| What | Size | Why it's needed |
|---|---|---|
| `~/ci_mpc_assets/` | 84 MB | `fr3.urdf`, `fr3_factory.urdf`, `fr3_scene*.usd` (USD is git-ignored), `fr3/fr3.usda`, `factory/` (peg/hole/gear USDs), `automate/` (AutoMate assemblies). Referenced by the Isaac scripts. |
| `~/ci_mpc_runs/` | 1.1 GB | All experiment results + trained models: `selector/**/sel.pt`, `selector/prior/c3data/student.pt`, `icnn/icnn_s0.pt`, `likelihood/` (probe data, 157 MB), `weekend/` (776 MB, can be skipped if short on space). PROGRESS.md comparisons point here. |
| `~/isaac_ros.sh` | 4 KB | Same as `scripts/isaac_ros.sh`; can also be re-copied from the repo. |
| `~/.claude/CLAUDE.md` + project `memory/` | small | Claude working preferences and the "read PROGRESS.md first" memory. The memory folder name encodes the path `~/franka_ros2_ws/src`, so it only applies if the workspace is at the same path. |

The probe-likelihood models used by default (`ci_mpc_planner/models/*.npz`) are already in git.

Not needed: `~/venvs/cimpc` (824 MB, recreate), `~/IsaacLab` (28 GB, recreate), `~/ocs2_ws` (cloned, never
built), `build/ install/ log/` (rebuild).

---

## B. On the new machine

Follow `SETUP.md` in order; the notes below are what to watch for.

| Step | `SETUP.md` | Version on the old machine | Note |
|---|---|---|---|
| OS | §0 | Ubuntu 22.04.5 | ROS 2 Humble needs 22.04. ≥ 32 GB RAM (each headless Isaac ≈ 6 GB). |
| NVIDIA driver | §1 | 595.99.02 | ≥ 595.58 for torch cu130. Pre-built signed modules, no `nvidia-dkms-595`. |
| libfranka | §2 | 0.21.3 `.deb` | Robot system 5.10. |
| Python venv | §3 | `~/venvs/cimpc`: pylibfranka 0.21.3, pin 4.1.0, mujoco 3.16.0, numpy 2.2.6, scipy 1.15.3 | Only for robot prototyping. |
| Realtime group | §4 | — | Log out/in after. |
| ROS 2 Humble + franka_ros2 | §5 | `ros-humble-desktop ros-dev-tools ros-humble-pinocchio` | franka_ros2 branch `humble` (at 3b50164). |
| System Python extras | — | `osqp 1.1.3` in `~/.local` (py3.10) | ROS-side planner imports `osqp`: `pip3 install --user osqp==1.1.3`. |
| Isaac Sim 6.2 + Isaac Lab | §6 | IsaacLab `main` at **46d6cc3d7** | Pin that commit: `git -C ~/IsaacLab checkout 46d6cc3d7` before `uv sync --extra isaacsim`. First start 10+ min. |
| Isaac venv extras | — | torch 2.11.0+cu130, osqp 1.0.5, qpsolvers 4.13.0, pin 3.9.0, warp 1.17.0, opencv-headless | Check with `uv pip list --python ~/IsaacLab/.venv`; install missing ones with `uv pip install --python ~/IsaacLab/.venv <pkg>`. |
| uv, claude CLI | — | in `~/.local/bin` | `curl -LsSf https://astral.sh/uv/install.sh \| sh`; install Claude Code. |

### B.1 Workspace

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/franka_ros2_ws && cd ~/franka_ros2_ws
git clone -b humble https://github.com/frankarobotics/franka_ros2.git src
vcs import src < src/dependency.repos --recursive --skip-existing
rm -rf src/libfranka
cd src
git clone git@github.com:njohnson433/ci_mpc.git                      # needs your GitHub SSH key
git clone https://github.com/LucasG2001/cartesian_impedance_control.git
git clone https://github.com/LucasG2001/messages_fr3.git             # main, 9d5f4ab, no local changes

# our controller branch + controllers.yaml entry
cd cartesian_impedance_control
git checkout -b isaac-sim-pinocchio f8d5fe2
git am < ../ci_mpc/patches/cartesian_impedance_control_isaac-sim-pinocchio.patch
cd .. && git apply ci_mpc/patches/franka_ros2_humble_controllers_yaml.patch

cd ~/franka_ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys="zed_wrapper libfranka"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
```

Build `messages_fr3` and `ci_mpc_msgs` first if the full build complains about missing messages. Use
`--symlink-install` (the planner relies on it since 2026-10-27).

### B.2 Assets and Isaac

1. Put `~/ci_mpc_assets` and `~/ci_mpc_runs` back from step A.4 (same paths).
2. `cp ~/franka_ros2_ws/src/ci_mpc/scripts/isaac_ros.sh ~/isaac_ros.sh && chmod +x ~/isaac_ros.sh`
3. Only if the assets were lost: `bash scripts/gen_fr3_urdf.sh` and redo the scene (`SETUP.md` §7), then
   `scripts/isaac/prepare_scene.py`. The Factory/AutoMate USDs come from the Isaac Lab Nucleus assets.
4. Add the `ros_franka` alias from `SETUP.md` §5 to `~/.bashrc` (not on this machine's `.bashrc` either; do not
   source ROS globally).
5. Optional: re-add the nightly cron job (`crontab -e`, line from A.3) — only on one machine.

### B.3 Verify

```bash
ros_franka
ros2 run cartesian_impedance_control pinocchio_model_check        # home TCP ≈ 0.3069, 0, 0.4869
colcon test --packages-select ci_mpc_planner && colcon test-result --verbose   # 13 passed + 1 xfail
# clean terminal (no ROS, no venv):
ROS_DOMAIN_ID=42 ~/franka_ros2_ws/src/ci_mpc/scripts/isaac/run_impedance_test.sh
#   expect hold 0.00 mm, reach ≈ 2.2 mm / 0.0035 rad, < 1 cm in ≈ 1.6 s (with identified friction)
scripts/demo_peg.sh --offset 0,0                                  # peg inserts
```

If these match, continue from "Where to resume" in `docs/PROGRESS.md`. Note on capacity: the old box had
16 GB VRAM / 31 GB RAM and ran at most 4 Isaac instances in parallel; re-measure on the new machine before
sizing the queue scripts.

### B.4 Real robot (not done yet)

Network/FCI steps are in `SETUP.md` §8 and `REAL_ROBOT.md`. The PREEMPT_RT kernel was never installed;
check it against the NVIDIA driver first.
