# Setup

Full environment setup on the lab workstation (Ubuntu 22.04, RTX 4080). Steps are in the
order they were done. Everything here was verified on this machine.

---

## 0. Hardware and OS check

```bash
nvidia-smi        # driver + VRAM
free -h           # RAM (≥ 32 GB recommended for Isaac Sim)
ldd --version     # glibc ≥ 2.35 (22.04 has 2.35)
```

## 1. NVIDIA driver 595

Isaac Lab 3.0 installs PyTorch with CUDA 13.0, which needs driver **≥ 595.58.03**.

```bash
ubuntu-drivers list
sudo apt install nvidia-driver-595
# pre-built, signed kernel module for the running kernel (needed with Secure Boot):
sudo apt install linux-modules-nvidia-595-generic-hwe-22.04 linux-modules-nvidia-595-$(uname -r)
sudo reboot
nvidia-smi        # expect 595.99.02
```

Use the plain desktop driver: not `-server`, not `-open`, not 615. Do **not** keep
`nvidia-dkms-595` installed alongside the pre-built modules. Its build fails and leaves dpkg
in an error state (see TROUBLESHOOTING.md).

## 2. libfranka 0.21.3

The FR3 runs system version **5.10**, which uses robot server version 10, so the matching
libfranka line is 0.21.x.

```bash
sudo apt install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev python3-venv python3-pip
cd ~/Downloads
wget https://github.com/frankarobotics/libfranka/releases/download/0.21.3/libfranka_0.21.3_jammy_amd64.deb
echo "00097c24f4c48f2d81c964d811462c2dfaf961ccf61d000e4b3ec6cc8d604ff4  libfranka_0.21.3_jammy_amd64.deb" | sha256sum -c
sudo dpkg -i libfranka_0.21.3_jammy_amd64.deb
sudo apt-get install -f
dpkg -l | grep libfranka          # ii libfranka 0.21.3
```

## 3. Python venv for robot / MPC prototyping

```bash
python3 -m venv ~/venvs/cimpc
source ~/venvs/cimpc/bin/activate
pip install --upgrade pip
pip install pylibfranka==0.21.3 mujoco pin numpy scipy matplotlib osqp casadi
python -c "import pylibfranka, mujoco, pinocchio; print('ok')"
deactivate
```

## 4. Real-time permissions

```bash
sudo addgroup realtime
sudo usermod -a -G realtime $USER
sudo tee -a /etc/security/limits.conf > /dev/null <<'EOF'
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
EOF
# log out and back in, then:
groups            # should list 'realtime'
```

The PREEMPT_RT kernel itself is **not installed yet**. It's only needed for the real robot,
and it has to be checked against the NVIDIA driver first (see ROADMAP.md).

## 5. ROS 2 Humble + franka_ros2

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools ros-humble-pinocchio
sudo rosdep init && rosdep update
```

Workspace. Use the **`humble`** branch, since the GitHub default is `jazzy`:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/franka_ros2_ws/src && cd ~/franka_ros2_ws
git clone -b humble https://github.com/frankarobotics/franka_ros2.git src
vcs import src < src/dependency.repos --recursive --skip-existing
rm -rf src/libfranka            # use system libfranka 0.21.3 instead of the pinned 0.20.4
rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys="zed_wrapper libfranka"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
```

Test without the robot:

```bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
# drag the marker in RViz → Plan & Execute → the arm moves
```

Recommended alias in `~/.bashrc`. Don't source ROS globally; see the README for why.

```bash
alias ros_franka='source /opt/ros/humble/setup.bash && source ~/franka_ros2_ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp'
```

### PdZ Cartesian impedance controller

```bash
cd ~/franka_ros2_ws/src
git clone https://github.com/CurdinDeplazes/cartesian_impedance_control.git
git clone https://github.com/CurdinDeplazes/messages_fr3.git
```

Add this to `src/franka_bringup/config/controllers.yaml` (spaces only, no tabs):

```yaml
    cartesian_impedance_controller:
      type: cartesian_impedance_control/CartesianImpedanceController
```

```bash
cd ~/franka_ros2_ws
colcon build --packages-select messages_fr3 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon build --packages-select cartesian_impedance_control --cmake-args -DCMAKE_BUILD_TYPE=Release
```

It builds against franka_ros2 v2.x. Branch `isaac-sim-pinocchio` (from `main`) adds a
Pinocchio model source so the same controller runs in Isaac Sim (needs `sudo apt install ros-humble-pinocchio`):

| Parameter | Default | Meaning |
|---|---|---|
| `model_source` | `franka` | `franka`: M, C, J, pose from `franka_robot_model` (real robot). `pinocchio`: computed from `urdf_path` (sim). |
| `urdf_path` | `""` | URDF for `pinocchio` (`/home/njohnson/ci_mpc_assets/fr3.urdf`) |
| `ee_frame` | `fr3_hand_tcp` | EE frame for `pinocchio` (= libfranka `kEndEffector` with the Franka Hand) |
| `arm_id` | `fr3` | joint prefix |
| `enable_input_server` | `true` | `user_input_service` thread; must be `false` inside Isaac (it calls `rclcpp::shutdown()`) |

Configs: `config/isaac_sim_controllers.yaml`, `config/real_robot_controllers.yaml` (installed to
`share/cartesian_impedance_control/config`). Timing is rate-independent (integrator, filters and
torque-rate limit scale with the control period; identical to before at 1 kHz).

Sanity check of the Pinocchio model against libfranka values (home TCP = 0.3069, 0, 0.4869):
```bash
ros2 run cartesian_impedance_control pinocchio_model_check
```

**libfranka embeds its own Pinocchio** and exports its template instances (`ModelTpl()`, `DataTpl()`, …)
with a different memory layout. The controller library therefore links Pinocchio 4 *before* libfranka
(see `CMakeLists.txt`); otherwise `PinocchioModel` binds to libfranka's copies and crashes
(`std::length_error: vector::reserve`).

## 6. Isaac Sim 6.2 + Isaac Lab 3.0

Use a terminal **without** ROS sourced and **without** a venv active.

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc
git clone https://github.com/isaac-sim/IsaacLab.git --branch main ~/IsaacLab
cd ~/IsaacLab
uv sync --extra isaacsim
uv pip show --python .venv isaacsim        # 6.2.0.0
uv run --extra isaacsim python scripts/tutorials/00_sim/create_empty.py --viz kit
```

The first start takes 10+ minutes (extensions and shader cache). Accept the EULA when asked.
Always pass `--extra isaacsim`, otherwise uv removes Isaac Sim from the environment again.

### ROS 2 bridge (Humble)

Isaac Sim uses its own bundled Humble libraries (Python 3.12). Install the launcher script:

```bash
cp scripts/isaac_ros.sh ~/isaac_ros.sh && chmod +x ~/isaac_ros.sh
~/isaac_ros.sh
```

Check: **Tools → Robotics → ROS 2 OmniGraphs → Clock**, press Play, then in a ROS terminal
`ros2 topic echo /clock`.

## 7. FR3 scene in Isaac Sim

### 7.1 URDF

```bash
bash scripts/gen_fr3_urdf.sh      # writes ~/ci_mpc_assets/fr3.urdf (FR3 + Franka Hand)
```

### 7.2 Import and scene

In Isaac Sim:

1. File → New, then **Create → Physics → Ground Plane**.
2. **File → Import** → `/home/njohnson/ci_mpc_assets/fr3.urdf`. Type the full path; `~` does
   not work in Isaac's file dialog.
   - Fix base link: **on**
   - Merge fixed joints: **off**
3. Joint drives: the URDF has no gains, so the importer creates the drives with zero
   stiffness and damping. That's the effort (torque) mode we want.
4. **File → Save As** → `/home/njohnson/ci_mpc_assets/fr3_scene.usd`.

### 7.3 ROS joint-state graph (optional, for monitoring)

**Tools → Robotics → ROS 2 OmniGraphs → Joint States**, with Publisher and Subscriber both on.
The articulation root is **not** `/fr3` but `/fr3/Geometry/base/fr3_link0`. Run
`scripts/isaac/find_articulation_root.py`, then set `ReadJointState.inputs:prim` and
`ArticulationController.targetPrim` to that path.

> The topic loop (Isaac → ROS → external node → Isaac) runs at frame rate and is **not**
> suitable for torque control. Use it only to monitor `/joint_states`.

### 7.4 Gravity off on the robot links

Press Stop, run `scripts/isaac/disable_robot_gravity.py` in the Script Editor (≈ 11 links),
press Play. The arm should stay in its start pose with no controller running. Save with `Ctrl+S`.

### 7.5 Physics rate

`PhysicsScene` → **Time Steps Per Second = 240** for interactive work. 1000 Hz makes the GUI
very slow; use it later in a headless/standalone run.

### 7.6 Controller test

Press Play, then run `scripts/isaac/pd_home_controller.py` in the Script Editor. The arm moves
to the Franka home pose and holds it. Output shows `steps/s(sim): 240`. Re-run the script after
every Stop → Play.

## 8. Real robot (not done yet)

1. Connect the PC to the **Control box** LAN port. Static IP: `172.16.0.1/24`; the robot is
   `172.16.0.2` unless the lab changed it.
   ```bash
   nmcli con add type ethernet ifname <IFACE> con-name franka ipv4.method manual ipv4.addresses 172.16.0.1/24
   nmcli con up franka
   ping -c 3 172.16.0.2
   ```
2. Desk (`https://172.16.0.2`) → activate **FCI**.
3. Read-only test:
   ```bash
   source ~/venvs/cimpc/bin/activate
   python -c "import pylibfranka; r = pylibfranka.Robot('172.16.0.2'); print(r.read_once().q)"
   ```

## Other

- **OCS2** is cloned in its own workspace `~/ocs2_ws/src` (`ros2` branch + `ocs2_robotic_assets`),
  not built. The `ros2` branch targets Jazzy / Ubuntu 24.04, so building it on Humble will need patching.
