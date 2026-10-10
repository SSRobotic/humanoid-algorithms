<p align="center"><img src="assets/ssrobotics-logo.jpg" alt="SSRobotics — original logo by SSRobotic" width="240"></p>

<h1 align="center">🧮 Robotics Algorithms</h1>
<p align="center"><b>SSRobotics · Source Code Collection</b></p>

<p align="center">
  <a href="https://ssrobotic.github.io/humanoid-robot-hub/"><img src="https://img.shields.io/badge/Resources-13-f28c28?style=flat-square" alt="13 curated resources"></a>
  <a href="https://ssrobotic.github.io/humanoid-robot-hub/"><img src="https://img.shields.io/badge/Language-TH_%2F_EN-555555?style=flat-square" alt="Thai and English"></a>
  <a href="https://github.com/SSRobotic/humanoid-robot-hub/blob/main/CREDITS.md"><img src="https://img.shields.io/badge/Original_sources-Credited-555555?style=flat-square" alt="Original authors credited"></a>
</p>


**Part of [Humanoid Robot Hub](https://github.com/SSRobotic/humanoid-robot-hub)** · [🔎 Search all resources](https://ssrobotic.github.io/humanoid-robot-hub/)

IK, inverse dynamics, MPC / DDP, motion planning, optimization and reinforcement learning.

IK, Inverse Dynamics, MPC / DDP, Motion Planning, Optimization และ Reinforcement Learning

Curated by **SSRobotic — Robotics Engineer & Open-source Curator**. This repository contains actual source files and models in `projects/`, alongside a resource catalog and engineering guide. The imported implementations are unchanged snapshots credited to their original creators.

## 📦 Code & models in this repository

**898 tracked upstream files** — เปิดโฟลเดอร์ด้านล่างเพื่อดูโค้ดจริง โมเดล และตัวอย่างต้นฉบับ

| Project files | Files | Pinned source | License / notices |
| --- | ---: | --- | --- |
| [pink-kinematics/pink](projects/pink) | 128 | [`2c333161fc26`](https://github.com/pink-kinematics/pink/tree/2c333161fc260de93fb7be1dd8d6519d2ecc2df2) | [LICENSE](projects/pink/LICENSE) · [NOTICE](projects/pink/NOTICE) |
| [loco-3d/crocoddyl](projects/crocoddyl) | 770 | [`0d3b1ae9c2f0`](https://github.com/loco-3d/crocoddyl/tree/0d3b1ae9c2f062305f50a01058df12eb28ae0500) | [LICENSE](projects/crocoddyl/LICENSE) |

[Source manifest](source-manifest.json) records commits, file counts, Git LFS pointers and external submodules. [Per-file manifests](source-manifests/) record original Git blob hashes. `projects/` keeps upstream code, README files, licenses and notices; SSRobotics supplies the organization and guides.

```bash
git clone --depth 1 https://github.com/SSRobotic/humanoid-algorithms.git
cd humanoid-algorithms
```

### ▶️ Run the SSRobotics CPU examples

[Implementation](ssrobotics_algorithms/) · [Examples](examples/) · [Behavior tests](tests/)

```bash
python -m pip install -r requirements.txt
python examples/run_ik.py
python examples/run_mpc.py
python examples/run_rrt.py
python -m unittest discover -s tests -v
```

The original SSRobotics examples implement damped least-squares IK for a planar two-link arm, **unconstrained** finite-horizon linear MPC for a 1D double integrator, and collision-checked RRT for a 2D point robot. They require only NumPy and run on CPU. They are learning examples, with no humanoid balance or hardware integration. Pink and Crocoddyl provide the imported robotics implementations; install their dependencies using their retained README files.

Crocoddyl’s pinned `cmake` submodule is listed in its manifest and `.gitmodules`, and is not bundled as a nested Git repository. Use the original upstream checkout with `--recurse-submodules` for source builds that need it.

## 🚀 Start here

Use the [algorithm map and engineering guide](GUIDE.md) to choose between IK, inverse dynamics, MPC / DDP, planning, numerical optimization and RL.

## 🗂️ Resource directory

| Resource | What it provides | Keywords | Original source |
| --- | --- | --- | --- |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio) | Rigid-body dynamics and kinematics algorithms for robotics, including model-based control research. | Kinematics, Dynamics, C++ | [stack-of-tasks](https://github.com/stack-of-tasks/pinocchio) |
| [Drake](https://github.com/RobotLocomotion/drake) | Model-based robotics toolbox for dynamics, optimization, planning and control. | Dynamics, Optimization, Planning | [RobotLocomotion](https://github.com/RobotLocomotion/drake) |
| [RSL RL](https://github.com/leggedrobotics/rsl_rl) | Learning-algorithm implementations used in robotics training workflows. | RL, Training, Robotics | [leggedrobotics](https://github.com/leggedrobotics/rsl_rl) |
| [MoveIt 2](https://github.com/moveit/moveit2) | ROS 2 motion-planning and manipulation framework. | ROS 2, Motion planning, Manipulation | [moveit](https://github.com/moveit/moveit2) |
| [TSID](https://github.com/stack-of-tasks/tsid) | Task-space inverse-dynamics control using Pinocchio, with robotics exercises and examples. | Inverse dynamics, Whole-body control, QP | [stack-of-tasks](https://github.com/stack-of-tasks/tsid) |
| [Crocoddyl](https://github.com/loco-3d/crocoddyl) | Contact-aware optimal control with DDP-family solvers for robot motion. | DDP, Optimal control, Contacts | [loco-3d](https://github.com/loco-3d/crocoddyl) |
| [OCS2](https://github.com/leggedrobotics/ocs2) | Optimal-control and MPC tooling for systems with switching dynamics. | MPC, Optimal control, Switched systems | [leggedrobotics](https://github.com/leggedrobotics/ocs2) |
| [Pink Inverse Kinematics](https://github.com/pink-kinematics/pink) | Differential inverse kinematics in Python using Pinocchio and QP solvers. | IK, Pinocchio, QP | [pink-kinematics](https://github.com/pink-kinematics/pink) |
| [OMPL](https://github.com/ompl/ompl) | Sampling-based motion-planning algorithms; robot collision checking and execution are supplied by integrations. | Motion planning, RRT, Sampling | [ompl](https://github.com/ompl/ompl) |
| [cuRobo](https://github.com/NVlabs/curobo) | GPU-accelerated robot motion-generation tools; verify the API version and supported robot setup upstream. | CUDA, Trajectory optimization, Motion planning | [NVlabs](https://github.com/NVlabs/curobo) |
| [OSQP](https://github.com/osqp/osqp) | Numerical solver for convex quadratic programs, useful as a component in IK and control formulations. | QP, Optimization, Solver | [osqp](https://github.com/osqp/osqp) |
| [CasADi](https://github.com/casadi/casadi) | Symbolic differentiation and optimization tools for formulating nonlinear control and trajectory problems. | NLP, Automatic differentiation, MPC | [casadi](https://github.com/casadi/casadi) |
| [Stable-Baselines3](https://github.com/DLR-RM/stable-baselines3) | PyTorch reinforcement-learning implementations, including PPO, SAC and TD3; supply a compatible task environment. | RL, PPO, SAC | [DLR-RM](https://github.com/DLR-RM/stable-baselines3) |

[resources.json](resources.json) contains the structured entries. [GUIDE.md](GUIDE.md) connects tools to tasks and records the information needed to reproduce an experiment.

## 🔗 Explore the ecosystem

<details>
<summary>🌐 Browse all 14 categories</summary>

[🦾 Models & URDF](https://github.com/SSRobotic/humanoid-models) · [🌐 Simulation](https://github.com/SSRobotic/humanoid-simulation) · [🧠 Robot Learning](https://github.com/SSRobotic/humanoid-robot-learning) · [⚙️ Whole-body Control](https://github.com/SSRobotic/humanoid-whole-body-control) · [👁️ Vision & 3D Perception](https://github.com/SSRobotic/humanoid-vision) · [🤏 Manipulation & Planning](https://github.com/SSRobotic/humanoid-manipulation) · [🎮 Teleoperation](https://github.com/SSRobotic/humanoid-teleoperation) · [💾 Datasets & Demonstrations](https://github.com/SSRobotic/humanoid-datasets) · [💬 Vision-Language-Action](https://github.com/SSRobotic/humanoid-vla) · [🔌 Hardware & SDKs](https://github.com/SSRobotic/humanoid-hardware-sdks) · [📡 ROS & Integration](https://github.com/SSRobotic/humanoid-ros2) · [📚 Research & Benchmarks](https://github.com/SSRobotic/humanoid-research) · [🟢 Isaac Sim & Isaac Lab](https://github.com/SSRobotic/humanoid-isaac-sim) · [🧮 Robotics Algorithms](https://github.com/SSRobotic/humanoid-algorithms)

</details>

[🏠 Main Hub](https://github.com/SSRobotic/humanoid-robot-hub) · [🤖 Profile](https://github.com/SSRobotic)

## 🙌 Help this collection grow

⭐ [Star the Hub](https://github.com/SSRobotic/humanoid-robot-hub) · 👤 [Follow SSRobotic](https://github.com/SSRobotic) · 💡 [Suggest a resource](https://github.com/SSRobotic/humanoid-robot-hub/issues/new/choose)

Suggestions and corrections follow the [Hub contribution guide](https://github.com/SSRobotic/humanoid-robot-hub/blob/main/CONTRIBUTING.md). Linked software, assets and documentation keep their original licenses. The original SSRobotics logo is supplied by its creator; the root MIT license covers our original descriptions, guides and SSRobotics example code only. Imported `projects/` files retain their own licenses.
