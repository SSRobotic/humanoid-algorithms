import unittest
import numpy as np
from ssrobotics_algorithms import forward_kinematics, solve_ik, mpc_action, double_integrator, plan_rrt, segment_free


class AlgorithmTests(unittest.TestCase):
    def test_ik_reaches_targets(self):
        for target in ((1.2, 0.8), (-0.5, 1.4), (0.8, -0.6)):
            result = solve_ik(target)
            self.assertTrue(result.converged)
            np.testing.assert_allclose(forward_kinematics(result.q), target, atol=1e-6)

    def test_ik_reports_unreachable_and_nonfinite(self):
        for target in ((3, 0), (float("nan"), 0)):
            with self.assertRaises(ValueError):
                solve_ik(target)
        result = solve_ik((1.2, 0.8), max_iterations=1)
        self.assertFalse(result.converged)

    def test_mpc_closed_loop_reaches_stationary_target(self):
        a, b = double_integrator()
        for target in ((1.0, 0.0), (-2.0, 0.0)):
            state = np.array([0.0, 0.4])
            for _ in range(120):
                state = a @ state + b[:, 0] * mpc_action(state, target)
            np.testing.assert_allclose(state, target, atol=1e-5)

    def test_controller_rejects_invalid_parameters(self):
        with self.assertRaises(ValueError):
            double_integrator(0)
        with self.assertRaises(ValueError):
            mpc_action((0, 0), horizon=0)

    def test_rrt_connects_without_crossing_obstacle(self):
        obstacles = [(0.4, 0, 0.6, 0.7)]
        path = plan_rrt((0.1, 0.1), (0.9, 0.9), obstacles, seed=7)
        np.testing.assert_allclose(path[[0, -1]], [[0.1, 0.1], [0.9, 0.9]])
        self.assertTrue(np.all((path >= 0) & (path <= 1)))
        for start, end in zip(path[:-1], path[1:]):
            self.assertTrue(segment_free(start, end, obstacles))
        np.testing.assert_allclose(path, plan_rrt((0.1, 0.1), (0.9, 0.9), obstacles, seed=7))

    def test_rrt_does_not_cross_solid_wall(self):
        obstacles = [(0.4, 0, 0.6, 1)]
        self.assertFalse(segment_free((0.1, 0.5), (0.9, 0.5), obstacles))
        with self.assertRaises(RuntimeError):
            plan_rrt((0.1, 0.5), (0.9, 0.5), obstacles, max_iterations=500)


if __name__ == "__main__":
    unittest.main()
