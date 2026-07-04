"""Tests for common.ground_truth — pure-logic, numpy only."""
import unittest
import numpy as np

from common.ground_truth import compute_disparity, derive_f_px


class TestDeriveFPx(unittest.TestCase):
    def test_known_values(self):
        f_px = derive_f_px(focal_length_mm=24.0, resolution=(720, 720),
                           aperture=(20.955, 15.291))
        self.assertAlmostEqual(f_px, 24.0 * 720 / 20.955, places=1)

    def test_non_square_resolution(self):
        f_px = derive_f_px(focal_length_mm=50.0, resolution=(640, 480),
                           aperture=(32.0, 24.0))
        self.assertAlmostEqual(f_px, 50.0 * 640 / 32.0, places=1)


class TestComputeDisparity(unittest.TestCase):
    def setUp(self):
        self.f_px = 800.0
        self.B = 0.08
        self.H, self.W = 120, 160

    def test_fronto_parallel_plane(self):
        Z = 0.5
        left_depth = np.full((self.H, self.W), Z, dtype=np.float32)
        right_depth = np.full((self.H, self.W), Z, dtype=np.float32)
        expected_d = self.f_px * self.B / Z

        disparity = compute_disparity(left_depth, right_depth, self.B, self.f_px)
        self.assertEqual(disparity.shape, (self.H, self.W))
        self.assertEqual(disparity.dtype, np.float32)
        np.testing.assert_allclose(disparity, expected_d, rtol=1e-5)

    def test_varying_depth(self):
        y = np.arange(self.H, dtype=np.float32).reshape(-1, 1)
        x = np.arange(self.W, dtype=np.float32).reshape(1, -1)
        Z = 0.5 + 0.1 * (y + x) / max(self.H, self.W)
        left_depth = Z.astype(np.float32)
        right_depth = Z.astype(np.float32)

        disparity = compute_disparity(left_depth, right_depth, self.B, self.f_px)
        expected = self.f_px * self.B / Z
        np.testing.assert_allclose(disparity, expected, rtol=1e-4)

    def test_zero_baseline_gives_zero_disparity(self):
        Z = np.full((self.H, self.W), 1.0, dtype=np.float32)
        disparity = compute_disparity(Z, Z, 0.0, self.f_px)
        np.testing.assert_allclose(disparity, 0.0, atol=1e-12)

    def test_handles_infinite_depth(self):
        Z = np.full((self.H, self.W), np.inf, dtype=np.float32)
        disparity = compute_disparity(Z, Z, self.B, self.f_px)
        np.testing.assert_allclose(disparity, 0.0, atol=1e-12)

    def test_handles_nan_depth(self):
        Z = np.full((self.H, self.W), np.nan, dtype=np.float32)
        disparity = compute_disparity(Z, Z, self.B, self.f_px)
        self.assertTrue(np.all(np.isnan(disparity)))


if __name__ == "__main__":
    unittest.main()
