import unittest
import math
from rmf_fleetmind_edge_ai.collision_predictor import (
    calculate_ttc,
    MLCollisionPredictor
)


class TestCollisionPredictor(unittest.TestCase):

    def test_head_on_collision(self):
        # AMR 1 at x=0, moving right at 1m/s
        # AMR 2 at x=2, moving left at 1m/s
        # Combined speed = 2 m/s, distance = 2m -> collision at t=1s (or before due to radii)
        ttc, min_sep, risk = calculate_ttc(
            0.0, 0.0, 1.0, 0.0,
            2.0, 0.0, -1.0, 0.0,
            robot_radius=0.3,
            safety_margin=0.5,
            horizon=5.0
        )
        self.assertIsNotNone(ttc)
        self.assertLessEqual(ttc, 1.0)
        self.assertEqual(risk, "CRITICAL")

    def test_diverging_safe_paths(self):
        # AMR 1 moving right, AMR 2 moving further right faster
        ttc, min_sep, risk = calculate_ttc(
            0.0, 0.0, 0.5, 0.0,
            5.0, 0.0, 1.0, 0.0,
            horizon=5.0
        )
        self.assertIsNone(ttc)
        self.assertEqual(risk, "SAFE")

    def test_cross_traffic_warning(self):
        # AMR 1 moving on x axis, AMR 2 moving on y axis
        ttc, min_sep, risk = calculate_ttc(
            0.0, 0.0, 1.0, 0.0,
            1.5, -2.0, 0.0, 1.0,
            horizon=5.0
        )
        self.assertIn(risk, ["WARNING", "CRITICAL"])

    def test_ml_predictor_fallback(self):
        # Without valid model file, should gracefully use deterministic calculation
        pred = MLCollisionPredictor(model_path="/nonexistent/model.joblib")
        self.assertFalse(pred.ml_loaded)

        ttc, min_sep, risk, method = pred.predict_risk(
            0.0, 0.0, 1.0, 0.0,
            1.0, 0.0, -1.0, 0.0
        )
        self.assertEqual(method, "DETERMINISTIC")
        self.assertEqual(risk, "CRITICAL")


if __name__ == '__main__':
    unittest.main()
