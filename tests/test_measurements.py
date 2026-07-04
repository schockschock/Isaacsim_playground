"""Tests for common.measurements — pure-logic, numpy only."""
import unittest
import numpy as np

from common.measurements import (body_inertia_from_column_major, world_inertia,
                                  kinetic_energies, potential_energy,
                                  momenta, dissipating_flag)


class TestBodyInertiaFromColumnMajor(unittest.TestCase):
    def test_identity_3x3(self):
        I9 = np.array([1, 0, 0, 0, 2, 0, 0, 0, 3], dtype=np.float64)
        expected = np.array([[1, 0, 0], [0, 2, 0], [0, 0, 3]], dtype=np.float64)
        result = body_inertia_from_column_major(I9)
        np.testing.assert_array_almost_equal(result, expected)

    def test_full_matrix(self):
        I9 = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9], dtype=np.float64)
        result = body_inertia_from_column_major(I9)
        self.assertEqual(result.shape, (3, 3))
        self.assertEqual(result[0, 0], 1)
        self.assertEqual(result[1, 0], 2)
        self.assertEqual(result[2, 2], 9)


class TestWorldInertia(unittest.TestCase):
    def test_identity_quaternion_preserves_inertia(self):
        I_body = np.array([[1, 0, 0], [0, 2, 0], [0, 0, 3]], dtype=np.float64)
        q = np.array([1.0, 0.0, 0.0, 0.0])
        result = world_inertia(I_body, q)
        np.testing.assert_array_almost_equal(result, I_body)

    def test_90_deg_rotation_symmetry(self):
        I_body = np.diag([1.0, 2.0, 3.0])
        q = np.array([0.70710678, 0.0, 0.0, 0.70710678])
        result = world_inertia(I_body, q)
        self.assertEqual(result.shape, (3, 3))
        np.testing.assert_allclose(result @ result.T, result.T @ result, atol=1e-10)


class TestKineticEnergies(unittest.TestCase):
    def test_spinning_sphere(self):
        m = 1.0
        I = np.diag([0.4, 0.4, 0.4])
        w = np.array([10.0, 0.0, 0.0])
        v = np.array([0.0, 0.0, 0.0])
        ke_trans, ke_rot = kinetic_energies(m, v, I, w)
        self.assertAlmostEqual(ke_trans, 0.0)
        self.assertAlmostEqual(ke_rot, 0.5 * 0.4 * 100.0)

    def test_translating_cube(self):
        m = 2.0
        I = np.diag([1.0, 1.0, 1.0])
        w = np.array([0.0, 0.0, 0.0])
        v = np.array([3.0, 0.0, 0.0])
        ke_trans, ke_rot = kinetic_energies(m, v, I, w)
        self.assertAlmostEqual(ke_trans, 0.5 * 2.0 * 9.0)
        self.assertAlmostEqual(ke_rot, 0.0)

    def test_returns_float(self):
        m = 1.0
        I = np.eye(3)
        v = np.array([1.0, 0.0, 0.0])
        w = np.array([0.0, 1.0, 0.0])
        ke_trans, ke_rot = kinetic_energies(m, v, I, w)
        self.assertIsInstance(ke_trans, float)
        self.assertIsInstance(ke_rot, float)


class TestPotentialEnergy(unittest.TestCase):
    def test_ground_level(self):
        self.assertAlmostEqual(potential_energy(2.0, 0.0), 0.0)

    def test_positive_height(self):
        pe = potential_energy(3.0, 5.0)
        self.assertAlmostEqual(pe, 3.0 * 9.81 * 5.0)

    def test_custom_g(self):
        pe = potential_energy(1.0, 10.0, g=10.0)
        self.assertAlmostEqual(pe, 100.0)


class TestMomenta(unittest.TestCase):
    def test_stationary(self):
        m = 2.0
        v = np.array([0.0, 0.0, 0.0])
        I_world = np.eye(3)
        w = np.array([0.0, 0.0, 0.0])
        I_body = np.eye(3)
        p, L_world, L_body = momenta(m, v, I_world, w, I_body)
        np.testing.assert_array_almost_equal(p, [0, 0, 0])
        np.testing.assert_array_almost_equal(L_world, [0, 0, 0])
        np.testing.assert_array_almost_equal(L_body, [0, 0, 0])

    def test_linear_only(self):
        m = 2.0
        v = np.array([3.0, 0.0, 0.0])
        I_world = np.eye(3)
        w = np.array([0.0, 0.0, 0.0])
        I_body = np.eye(3)
        p, L_world, L_body = momenta(m, v, I_world, w, I_body)
        np.testing.assert_array_almost_equal(p, [6.0, 0.0, 0.0])
        np.testing.assert_array_almost_equal(L_world, [0, 0, 0])

    def test_angular_offset_axis(self):
        m = 1.0
        v = np.array([0.0, 0.0, 0.0])
        I_world = np.diag([2.0, 2.0, 4.0])
        w = np.array([0.0, 0.0, 5.0])
        I_body = I_world
        p, L_world, L_body = momenta(m, v, I_world, w, I_body)
        np.testing.assert_array_almost_equal(L_world, [0.0, 0.0, 20.0])


class TestDissipatingFlag(unittest.TestCase):
    def test_constant_energy_not_dissipating(self):
        E = np.array([10.0, 10.0, 10.0, 10.0])
        result = dissipating_flag(E, eps=0.01)
        np.testing.assert_array_equal(result, [False, False, False, False])

    def test_large_jump_is_dissipating(self):
        E = np.array([10.0, 10.0, 5.0, 5.0])
        result = dissipating_flag(E, eps=0.1)
        np.testing.assert_array_equal(result, [False, False, True, False])

    def test_small_drift_not_dissipating(self):
        E = np.array([10.0, 10.001, 10.0, 10.002])
        result = dissipating_flag(E, eps=0.01)
        np.testing.assert_array_equal(result, [False, False, False, False])

    def test_one_big_jump(self):
        E = np.array([10.0, 10.0, 15.0, 14.9])
        result = dissipating_flag(E, eps=1.0)
        np.testing.assert_array_equal(result, [False, False, True, False])

    def test_empty_input(self):
        result = dissipating_flag(np.array([]), eps=0.01)
        self.assertEqual(len(result), 0)

    def test_single_element(self):
        result = dissipating_flag(np.array([5.0]), eps=0.01)
        np.testing.assert_array_equal(result, [False])


if __name__ == "__main__":
    unittest.main()
