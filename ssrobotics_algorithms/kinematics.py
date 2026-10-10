"""Damped least-squares IK for a planar two-link arm (angles in radians)."""
from dataclasses import dataclass
import numpy as np


def vector2(value, name):
    out = np.asarray(value, dtype=float)
    if out.shape != (2,) or not np.isfinite(out).all():
        raise ValueError(f"{name} must contain two finite numbers")
    return out


def forward_kinematics(q, lengths=(1.0, 1.0)):
    q, lengths = vector2(q, "q"), vector2(lengths, "lengths")
    if np.any(lengths <= 0):
        raise ValueError("link lengths must be positive")
    angles = np.array([q[0], q.sum()])
    return np.array([lengths @ np.cos(angles), lengths @ np.sin(angles)])


def jacobian(q, lengths=(1.0, 1.0)):
    q, lengths = vector2(q, "q"), vector2(lengths, "lengths")
    if np.any(lengths <= 0):
        raise ValueError("link lengths must be positive")
    a, b = q[0], q.sum()
    l1, l2 = lengths
    return np.array([[-l1*np.sin(a)-l2*np.sin(b), -l2*np.sin(b)],
                     [l1*np.cos(a)+l2*np.cos(b), l2*np.cos(b)]])


@dataclass
class IKResult:
    q: np.ndarray
    error: float
    iterations: int
    converged: bool


def solve_ik(target, lengths=(1.0, 1.0), q0=(0.1, 0.1), *,
             damping=0.03, tolerance=1e-6, max_iterations=500):
    target, lengths, q = (vector2(target, "target"), vector2(lengths, "lengths"),
                          vector2(q0, "q0").copy())
    if np.any(lengths <= 0):
        raise ValueError("link lengths must be positive")
    radius = np.linalg.norm(target)
    if radius > lengths.sum()+tolerance or radius < abs(lengths[0]-lengths[1])-tolerance:
        raise ValueError("target is outside the two-link workspace")
    if not np.isfinite([damping, tolerance]).all() or damping <= 0 or tolerance <= 0 or max_iterations < 1:
        raise ValueError("damping, tolerance and max_iterations must be positive")
    for iteration in range(max_iterations + 1):
        error = target - forward_kinematics(q, lengths)
        norm = float(np.linalg.norm(error))
        if norm <= tolerance or iteration == max_iterations:
            return IKResult(q, norm, iteration, norm <= tolerance)
        j = jacobian(q, lengths)
        step = j.T @ np.linalg.solve(j @ j.T + damping**2 * np.eye(2), error)
        q += step * min(1.0, 0.25 / max(float(np.linalg.norm(step)), 1e-12))
    raise AssertionError("unreachable loop termination")
