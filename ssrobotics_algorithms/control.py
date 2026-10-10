"""Unconstrained finite-horizon linear MPC for 1D position/velocity tracking.

This example has no acceleration, state or contact constraints. It is an
educational double-integrator controller, not a humanoid balance controller.
"""
import numpy as np
from .kinematics import vector2


def double_integrator(dt=0.1):
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and positive")
    return np.array([[1.0, dt], [0.0, 1.0]]), np.array([[0.5*dt*dt], [dt]])


def mpc_action(state, target=(1.0, 0.0), *, dt=0.1, horizon=30,
               position_weight=10.0, velocity_weight=1.0, effort_weight=0.1):
    state, target = vector2(state, "state"), vector2(target, "target")
    if not isinstance(horizon, int) or horizon < 1:
        raise ValueError("horizon must be a positive integer")
    weights = np.array([position_weight, velocity_weight, effort_weight])
    if not np.isfinite(weights).all() or np.any(weights <= 0):
        raise ValueError("cost weights must be finite and positive")
    a, b = double_integrator(dt)
    q = np.diag(weights[:2])
    # V(x) = x' P x + 2 p' x + c; the affine term also tracks nonzero velocity.
    p_matrix, p_vector = q.copy(), -q @ target
    for _ in range(horizon):
        h = effort_weight + float((b.T @ p_matrix @ b).item())
        gain = b.T @ p_matrix @ a / h
        bias = float((b.T @ p_vector).item()) / h
        new_vector = -q @ target + a.T @ p_vector - a.T @ p_matrix @ b[:, 0] * bias
        p_matrix = q + a.T @ p_matrix @ a - a.T @ p_matrix @ b @ gain
        p_vector = new_vector
    return -float((gain @ state).item()) - bias
