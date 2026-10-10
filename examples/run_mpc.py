"""CPU-only unconstrained double-integrator MPC, not a humanoid controller."""
import sys, pathlib, json
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np
from ssrobotics_algorithms import mpc_action, double_integrator

state = np.zeros(2)
a, b = double_integrator(0.1)
trajectory = [state.tolist()]
for _ in range(100):
    u = mpc_action(state)
    state = a @ state + b[:, 0] * u
    trajectory.append(state.tolist())
print(json.dumps({"model": "1D double integrator", "constraints": "none",
                  "target": [1.0, 0.0], "final_state": state.tolist(),
                  "final_error": float(np.linalg.norm(state-[1, 0])),
                  "trajectory": trajectory}, indent=2))
