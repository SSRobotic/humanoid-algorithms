"""CPU-only RRT path planning for a 2D point robot."""
import sys, pathlib, json
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from ssrobotics_algorithms import plan_rrt

obstacles = [(0.4, 0.0, 0.6, 0.7)]
path = plan_rrt((0.1, 0.1), (0.9, 0.9), obstacles, seed=7)
print(json.dumps({"seed": 7, "obstacles": obstacles, "path": path.tolist()}, indent=2))
