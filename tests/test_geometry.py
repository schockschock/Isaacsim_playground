"""Unit tests for common.geometry — pure-python stereo rig vector math.

Runs under system python (no numpy, no Isaac Sim). Verifies the geometry
helpers that setup_stereo_rig depends on.
"""
import math
import unittest

from common.geometry import (
    angular_velocity_rad_s_to_deg_s,
    compute_eye_positions,
    rig_right_vector,
)


def approx_eq(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def length(v):
    return math.sqrt(sum(c * c for c in v))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def dist(a, b):
    return length(sub(a, b))


class TestRigRightVector(unittest.TestCase):
    def test_nominal_view_along_negative_y(self):
        # Camera at +Y looking toward origin: forward = -Y, up = +Z.
        # right = cross(forward, up) = cross((0,-1,0),(0,0,1)) = (-1,0,0)
        right = rig_right_vector(base=(0.0, 1.0, 0.0), look_at=(0.0, 0.0, 0.0))
        self.assertTrue(approx_eq(right, (-1.0, 0.0, 0.0)))

    def test_is_unit_length(self):
        right = rig_right_vector(base=(1.0, 2.0, 0.5), look_at=(-1.0, 0.0, 0.2))
        self.assertAlmostEqual(length(right), 1.0, places=9)

    def test_degenerate_look_straight_down_returns_fallback_no_nan(self):
        # forward parallel to up -> cross product is zero. Must fall back to a
        # valid unit axis, never return NaN or a zero vector.
        right = rig_right_vector(base=(0.0, 0.0, 1.0), look_at=(0.0, 0.0, 0.0))
        self.assertEqual(length(right), 1.0)
        for c in right:
            self.assertFalse(math.isnan(c), f"NaN component in {right}")

    def test_degenerate_look_straight_up_returns_fallback_no_nan(self):
        right = rig_right_vector(base=(0.0, 0.0, 0.0), look_at=(0.0, 0.0, 1.0))
        self.assertEqual(length(right), 1.0)
        for c in right:
            self.assertFalse(math.isnan(c), f"NaN component in {right}")


class TestComputeEyePositions(unittest.TestCase):
    def test_zero_baseline_both_eyes_equal_base(self):
        base = (0.0, 1.0, 0.0)
        left, right = compute_eye_positions(base=base, look_at=(0.0, 0.0, 0.0), baseline=0.0)
        self.assertTrue(approx_eq(left, base))
        self.assertTrue(approx_eq(right, base))

    def test_interocular_distance_equals_baseline(self):
        base = (0.0, 1.0, 0.0)
        baseline = 0.06
        left, right = compute_eye_positions(base=base, look_at=(0.0, 0.0, 0.0), baseline=baseline)
        self.assertAlmostEqual(dist(left, right), baseline, places=9)

    def test_eyes_symmetric_about_base(self):
        base = (0.0, 1.0, 0.0)
        baseline = 0.06
        left, right = compute_eye_positions(base=base, look_at=(0.0, 0.0, 0.0), baseline=baseline)
        mid = tuple((l + r) / 2.0 for l, r in zip(left, right))
        self.assertTrue(approx_eq(mid, base))

    def test_left_is_on_camera_left_for_view_along_negative_y(self):
        # Facing -Y, up +Z -> right = -X, so camera-left = +X.
        base = (0.0, 1.0, 0.0)
        left, right = compute_eye_positions(base=base, look_at=(0.0, 0.0, 0.0), baseline=0.06)
        self.assertGreater(left[0], right[0])

    def test_degenerate_view_does_not_crash(self):
        left, right = compute_eye_positions(base=(0.0, 0.0, 1.0), look_at=(0.0, 0.0, 0.0), baseline=0.06)
        self.assertEqual(dist(left, right), 0.06)


class TestAngularVelocityConversion(unittest.TestCase):
    def test_zero_returns_zero(self):
        result = angular_velocity_rad_s_to_deg_s((0.0, 0.0, 0.0))
        self.assertEqual(result, (0.0, 0.0, 0.0))

    def test_pi_is_180(self):
        result = angular_velocity_rad_s_to_deg_s((0.0, 0.0, math.pi))
        self.assertAlmostEqual(result[0], 0.0, places=9)
        self.assertAlmostEqual(result[1], 0.0, places=9)
        self.assertAlmostEqual(result[2], 180.0, places=9)

    def test_two_pi_is_360(self):
        result = angular_velocity_rad_s_to_deg_s((0.0, 0.0, 2.0 * math.pi))
        self.assertAlmostEqual(result[2], 360.0, places=9)

    def test_one_radian_is_about_57_3_degrees(self):
        result = angular_velocity_rad_s_to_deg_s((1.0, 0.0, 0.0))
        self.assertAlmostEqual(result[0], 180.0 / math.pi, places=9)
        self.assertAlmostEqual(result[1], 0.0, places=9)
        self.assertAlmostEqual(result[2], 0.0, places=9)

    def test_returns_3_tuple_of_float(self):
        result = angular_velocity_rad_s_to_deg_s((0.5, 5.0, 286.5))
        self.assertEqual(len(result), 3)
        for c in result:
            self.assertIsInstance(c, float)

    def test_negative_handled(self):
        result = angular_velocity_rad_s_to_deg_s((0.0, 0.0, -math.pi))
        self.assertAlmostEqual(result[2], -180.0, places=9)


if __name__ == "__main__":
    unittest.main()
