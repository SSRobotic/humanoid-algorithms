# 🧮 Robotics algorithms: choose by engineering task

[🏠 Main Hub](https://github.com/SSRobotic/humanoid-robot-hub) · [↩️ Directory](README.md)

## 🎯 Algorithm map

| Task | Approach | Source implementation | Inputs you need | What to inspect |
| --- | --- | --- | --- | --- |
| Compute poses, Jacobians and rigid-body dynamics | Forward kinematics / dynamics | [Pinocchio](https://github.com/stack-of-tasks/pinocchio) | Robot model, configuration and velocity | Frames, quaternion conventions and units |
| Track an end-effector pose | Differential IK with QP | [Pink](https://github.com/pink-kinematics/pink) | Tasks, Jacobians, joint limits and QP solver | Task error, velocity limits and feasibility |
| Coordinate motion and contact forces | Task-space inverse dynamics | [TSID](https://github.com/stack-of-tasks/tsid) | Dynamics, contacts, task references and actuator limits | Contact constraints, torque bounds and solver status |
| Optimize motion over a contact sequence | DDP-family optimal control | [Crocoddyl](https://github.com/loco-3d/crocoddyl) | State / control model, cost and contact sequence | Convergence, dynamics consistency and initialization |
| Replan with changing dynamics | MPC / switched-system optimal control | [OCS2](https://github.com/leggedrobotics/ocs2) | Model, horizon, costs, constraints and mode schedule | Solve time, constraint residuals and tracking |
| Find a collision-free configuration path | Sampling-based planning | [OMPL](https://github.com/ompl/ompl) | State space, bounds and validity checker | Collision validity and path feasibility |
| Generate robot motion on GPU | Motion generation / trajectory optimization | [cuRobo](https://github.com/NVlabs/curobo) | Supported robot, collision world and target | API version, collision clearance and timing |
| Solve convex quadratic programs | QP solver | [OSQP](https://github.com/osqp/osqp) | Quadratic / linear terms and constraints | Solver status, primal / dual residuals and tolerances |
| Formulate nonlinear optimization | Symbolic differentiation / NLP | [CasADi](https://github.com/casadi/casadi) | Dynamics, objective, constraints and selected solver | Derivatives, scaling and convergence |
| Learn a control policy | Reinforcement learning | [RSL RL](https://github.com/leggedrobotics/rsl_rl), [Stable-Baselines3](https://github.com/DLR-RM/stable-baselines3) | Compatible environment, rewards and action / observation spaces | Separate evaluation episodes and task performance |
| Connect planning to a ROS robot | Planning integration | [MoveIt 2](https://github.com/moveit/moveit2) | Robot description, planning groups and controllers | Frames, collision scene and execution interface |
| Explore systems, optimization and control together | Modeling / simulation / optimization | [Drake](https://github.com/RobotLocomotion/drake) | Plant, state, constraints and chosen workflow | Model assumptions and documented examples |

Not every source provides a ready-made controller for every humanoid. Match the implementation to your embodiment and task before integrating it.

## 🦾 1. Begin with the model and frames

Write down configuration `q`, velocity `v`, actuated joints, floating-base representation, world frame and task frames. Verify joint order and units against the model. For a floating-base robot, configuration and velocity vectors can have different dimensions; do not assume they are interchangeable.

Use a documented kinematics example to inspect a pose and its Jacobian. A successful IK solve tracks a task; collision avoidance, contact balance and torque feasibility require the constraints or a controller that explicitly handles them.

## ⚙️ 2. Separate IK from inverse dynamics

Differential IK selects a velocity increment to reduce task errors subject to the constraints in its formulation. Inverse dynamics also reasons about accelerations, actuation and contacts. A schematic rigid-body equation is:

$$ M(q)\dot v + h(q,v) = S^T\tau + J_c(q)^T\lambda $$

`M` is the mass matrix, `h` the bias term, `S` the actuation selection, `tau` the actuator effort, and `lambda` the contact wrench under a consistent convention. Contact kinematics, friction and actuator limits must be formulated explicitly when required by the task.

## 🧭 3. Separate path planning from timed control

A geometric path specifies where the robot should move in configuration space. A trajectory adds timing. A controller tracks a reference using measured or simulated state. Validate the stages separately: collision / state validity, joint and velocity bounds, then tracking and dynamics. Use documented OMPL, MoveIt or cuRobo examples rather than treating a returned path as an executable humanoid motion.

## 📈 4. Compare MPC / DDP on the same task

Record dynamics, contact schedule, horizon, objective, constraints, initialization and update rate. MPC repeatedly solves a finite-horizon problem and applies a portion of the resulting control. Compare constraint residuals and solve time with the control deadline as well as the task outcome. Keep solver versions and task conditions in the experiment record.

## 🧠 5. Evaluate learning policies separately

Describe the environment, reward, action scaling, observations and reset conditions. Use the algorithm actually supported by your selected framework. Stable-Baselines3 lists PPO / SAC / TD3 implementations; it is not a humanoid environment itself. Evaluate the trained policy on declared conditions, including held-out settings when relevant.

## 📝 Experiment record

| Record | Why it matters |
| --- | --- |
| Robot model and source commit | Reproduces embodiment and frame definitions |
| Algorithm / solver version | Reproduces API and numerical behavior |
| Objective and constraints | Explains what the result is intended to satisfy |
| Time step, horizon and limits | Defines the control and planning problem |
| Initial conditions and seeds | Makes comparisons meaningful |
| Metrics and solver status | Distinguishes task outcome from numerical convergence |

Next: [🟢 Isaac Sim / Isaac Lab](https://github.com/SSRobotic/humanoid-isaac-sim), [⚙️ Whole-body Control](https://github.com/SSRobotic/humanoid-whole-body-control), [🤏 Manipulation](https://github.com/SSRobotic/humanoid-manipulation).

These are original selection notes by SSRobotics. Implementation code, technical documentation and citations remain with the linked original authors. This repository curates sources and guides; it does not claim benchmark results or a tested controller implementation.
