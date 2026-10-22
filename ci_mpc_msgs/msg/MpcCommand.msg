# Setpoint from the MPC to the Cartesian impedance controller (topic <controller>/mpc_command).
# All vectors in the robot base frame. The controller tracks the newest command; without a new one for
# mpc_timeout (default 50 ms) it holds the pose it is at.

std_msgs/Header header          # stamp: when the command was computed
geometry_msgs/Pose pose         # desired end-effector pose
geometry_msgs/Twist twist       # desired end-effector twist; the damping acts on (twist - actual twist)
geometry_msgs/Wrench wrench     # feedforward wrench, added to the impedance force
float64[6] stiffness            # diagonal K [N/m x3, Nm/rad x3]; all zero = keep the current stiffness
float64[6] damping              # diagonal D [Ns/m x3, Nms/rad x3]; all zero = critically damped from K
bool integral_enabled           # true in free space, false in contact (the integral then decays)
