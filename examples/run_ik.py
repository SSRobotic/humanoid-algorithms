"""Run from the repository root: python examples/run_ik.py."""
import sys, pathlib, json
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from ssrobotics_algorithms import solve_ik, forward_kinematics

result = solve_ik((1.2, 0.8))
print(json.dumps({"target": [1.2, 0.8], "q_radians": result.q.tolist(),
                  "end_effector": forward_kinematics(result.q).tolist(),
                  "error": result.error, "iterations": result.iterations,
                  "converged": result.converged}, indent=2))
sys.exit(0 if result.converged else 1)
