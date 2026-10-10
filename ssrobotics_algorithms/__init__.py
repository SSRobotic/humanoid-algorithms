"""Small educational robotics algorithms by SSRobotic. No hardware control."""
from .kinematics import forward_kinematics, solve_ik
from .control import double_integrator, mpc_action
from .planning import plan_rrt, segment_free
