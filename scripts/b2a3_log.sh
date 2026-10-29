#!/bin/bash
# (copied from the session scratchpad 2026-10-29; run from the ci_mpc repo root)
# CI-MPC B2a logging (pre-registered PROGRESS 2026-10-29 16:10): insertions with bags (setpoint / K / ff) + truth pose + wrist.
# src-35's peg_truth recorder, and a bag -> trace.npz (hold selection). Waits for the push-stop selection (domains 160/180).
cd /home/njohnson/franka_ros2_ws/src/ci_mpc
true
export ROS_DOMAIN_ID=160
OUT=$HOME/ci_mpc_runs/b2a3
for spec in "factory:1:1.2,0.6" "factory:2:-0.6,-2.2" "factory:3:2.8,-1.0" "factory:4:-2.3,1.9" "dev:5:0.8,1.8" "dev:6:-2.2,0.6"; do
  IFS=: read -r kind seed off <<< "$spec"
  D=$OUT/${kind}_$seed; [ -f $D/res.json ] && [ -f $D/trace6.npz ] && continue; mkdir -p $D
  ( source /opt/ros/humble/setup.bash; source ~/franka_ros2_ws/install/setup.bash
    exec ros2 bag record -o $D/bag /joint_states /cartesian_impedance_controller/mpc_command \
      /cartesian_impedance_controller/external_wrench ) > $D/bag.out 2>&1 &
  BAG=$!
  if [ $kind = factory ]; then
    WRIST_FT=1 LOG=$D/isaac.log taskset -c 0-15 timeout 3600 scripts/demo_peg.sh --headless --offset "$off" --strategy probe \
      --timeout-contact 150 --max-probes 25 --track-height 5 --centering-bound 0.008 --stuck-dither 0.0008 --belief-model $PWD/ci_mpc_planner/models/probe_likelihood_v3_both.npz --log-dir $D --out $D/res.json > $D/run.out 2>&1
  else
    AUTOMATE_ASSETS=$HOME/ci_mpc_assets/dev_nochamfer AUTOMATE=dev_round_8mm WRIST_FT=1 LOG=$D/isaac.log taskset -c 0-15 timeout 3600 \
      scripts/demo_automate.sh --headless --offset "$off" --strategy probe --timeout-contact 150 --max-probes 25 --track-height 5 --centering-bound 0.008 --stuck-dither 0.0008 --tilt-probe 10 --tilt-axis x --push-wiggle 0.5 --tilt-engage 0.0005 --tilt-first-upright --tilt-thin-auto 0.003 \
      --log-dir $D --out $D/res.json > $D/run.out 2>&1
  fi
  echo "$(date +%T) $kind $seed EXIT $?"
  kill -INT $BAG; wait $BAG
  sync=$(grep -o "sync dir [^,]*" $D/run.out | cut -d' ' -f3); cp $sync/wrist_ft.csv $D/ 2>/dev/null
  ( source /opt/ros/humble/setup.bash; source ~/franka_ros2_ws/install/setup.bash
    PYTHONPATH=$HOME/.local/ros_numpy1:$PYTHONPATH python3 scripts/b2_trace.py $D/bag $D/trace6.npz ) > $D/trace.out 2>&1
  echo "$(date +%T) $kind $seed trace $( [ -f $D/trace6.npz ] && echo ok || echo MISSING ) peg_truth $( [ -f $D/peg_truth.npz ] && echo ok || echo MISSING )"
done
echo "b2a3 logging done"
