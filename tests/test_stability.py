"""Unit tests for common.stability — settled-detection over a position window.

Runs under system python. Mirrors the stability convention used in
samples/freefall.py (strict < epsilon on each axis) but factored into a
pure helper so the capture loop can call it without duplicating logic.
"""
import unittest

from common.stability import is_settled


class TestIsSettled(unittest.TestCase):
    def test_identical_window_settled(self):
        window = [(0.1, 0.2, 0.3)] * 5
        self.assertTrue(is_settled(window, epsilon=1e-4, required_len=5))

    def test_one_outlier_not_settled(self):
        window = [(0.1, 0.2, 0.3)] * 4 + [(0.5, 0.2, 0.3)]
        self.assertFalse(is_settled(window, epsilon=1e-4, required_len=5))

    def test_window_shorter_than_required_not_settled(self):
        window = [(0.1, 0.2, 0.3)] * 3
        self.assertFalse(is_settled(window, epsilon=1e-4, required_len=5))

    def test_empty_window_not_settled(self):
        self.assertFalse(is_settled([], epsilon=1e-4, required_len=5))

    def test_drift_below_epsilon_settled(self):
        window = [(0.0, 0.0, z) for z in (0.0, 1e-5, 2e-5, 3e-5, 4e-5)]
        self.assertTrue(is_settled(window, epsilon=1e-4, required_len=5))

    def test_drift_above_epsilon_not_settled(self):
        window = [(0.0, 0.0, z) for z in (0.0, 0.0, 0.0, 0.0, 2e-4)]
        self.assertFalse(is_settled(window, epsilon=1e-4, required_len=5))

    def test_drift_in_x_axis_counts(self):
        window = [(x, 0.0, 0.0) for x in (0.0, 0.0, 0.0, 0.0, 2e-4)]
        self.assertFalse(is_settled(window, epsilon=1e-4, required_len=5))

    def test_drift_in_y_axis_counts(self):
        window = [(0.0, y, 0.0) for y in (0.0, 0.0, 0.0, 0.0, 2e-4)]
        self.assertFalse(is_settled(window, epsilon=1e-4, required_len=5))

    def test_exactly_required_len_settled(self):
        window = [(0.1, 0.2, 0.3)] * 5
        self.assertTrue(is_settled(window, epsilon=1e-4, required_len=5))


if __name__ == "__main__":
    unittest.main()
