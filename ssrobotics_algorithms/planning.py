"""Seeded 2D RRT with exact segment/rectangle collision tests for a point robot.

Rectangles include their boundaries. No robot footprint or dynamics is modeled.
"""
import numpy as np
from .kinematics import vector2


def segment_free(start, end, obstacles):
    start, end = vector2(start, "start"), vector2(end, "end")
    direction = end - start
    for rect in obstacles:
        rect = np.asarray(rect, dtype=float)
        if rect.shape != (4,) or not np.isfinite(rect).all() or np.any(rect[:2] > rect[2:]):
            raise ValueError("rectangle must be finite [xmin, ymin, xmax, ymax]")
        lo, hi = 0.0, 1.0
        for axis in range(2):
            if abs(direction[axis]) < 1e-14:
                if start[axis] < rect[axis] or start[axis] > rect[axis+2]:
                    lo, hi = 1.0, 0.0
                    break
            else:
                t1, t2 = sorted(((rect[axis]-start[axis])/direction[axis],
                                 (rect[axis+2]-start[axis])/direction[axis]))
                lo, hi = max(lo, t1), min(hi, t2)
        if lo <= hi:
            return False
    return True


def plan_rrt(start, goal, obstacles=(), bounds=(0, 0, 1, 1), *,
             step_size=0.08, max_iterations=5000, seed=7):
    start, goal = vector2(start, "start"), vector2(goal, "goal")
    bounds = np.asarray(bounds, dtype=float)
    if bounds.shape != (4,) or not np.isfinite(bounds).all() or np.any(bounds[:2] >= bounds[2:]):
        raise ValueError("bounds must be finite [xmin, ymin, xmax, ymax]")
    if not np.isfinite(step_size) or step_size <= 0 or max_iterations < 1:
        raise ValueError("step_size and max_iterations must be positive")
    obstacles = tuple(obstacles)
    for point in (start, goal):
        if np.any(point < bounds[:2]) or np.any(point > bounds[2:]) or not segment_free(point, point, obstacles):
            raise ValueError("start and goal must be inside free space")
    rng = np.random.default_rng(seed)
    nodes, parents = [start.copy()], [-1]
    for _ in range(max_iterations):
        sample = goal if rng.random() < 0.15 else rng.uniform(bounds[:2], bounds[2:])
        nearest = int(np.argmin(np.linalg.norm(np.array(nodes)-sample, axis=1)))
        delta = sample - nodes[nearest]
        distance = np.linalg.norm(delta)
        if distance < 1e-12:
            continue
        candidate = nodes[nearest] + delta * min(1.0, step_size/distance)
        if not segment_free(nodes[nearest], candidate, obstacles):
            continue
        nodes.append(candidate)
        parents.append(nearest)
        if np.linalg.norm(candidate-goal) <= step_size and segment_free(candidate, goal, obstacles):
            path, current = [goal.copy()], len(nodes)-1
            while current >= 0:
                path.append(nodes[current])
                current = parents[current]
            return np.array(path[::-1])
    raise RuntimeError("no path found within the iteration budget")
