"""Tests for common.dataset_writer — pure-logic, h5py + numpy only."""
import os
import tempfile
import unittest

import numpy as np

from common.dataset_writer import DatasetWriter


class TestDatasetWriter(unittest.TestCase):

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.h5_path = os.path.join(self.tmpdir, "test.h5")

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_creates_hdf5_with_attrs(self):
        writer = DatasetWriter(
            self.h5_path,
            scenario="freefall",
            fps=120,
            dt_phys=1 / 480,
            solver_substeps=4,
            baseline_m=0.06,
            material={"restitution": 0.4, "friction": 0.5, "density": 1000},
            potato_usd_path="/data2/adrien/clean_dataset/2R1-1/2R1-1_centered.usd",
        )
        self.assertTrue(os.path.isfile(self.h5_path))

        import h5py
        with h5py.File(self.h5_path, "r") as f:
            self.assertIn("/state", f)
            for key in ("scenario", "fps", "dt_phys", "solver_substeps",
                         "t_offset_frames", "baseline_m", "potato_usd_path"):
                self.assertIn(key, f.attrs, f"missing attr: {key}")
            self.assertEqual(f.attrs["scenario"], "freefall")
            self.assertEqual(f.attrs["fps"], 120)
            self.assertEqual(f.attrs["potato_usd_path"],
                             "/data2/adrien/clean_dataset/2R1-1/2R1-1_centered.usd")

        writer.finalize()

    def test_append_state_writes_rows(self):
        writer = DatasetWriter(self.h5_path, scenario="test")
        state0 = {
            "time": 0.0,
            "position": np.array([0.0, 0.0, 0.5]),
            "orientation": np.array([1.0, 0.0, 0.0, 0.0]),
            "lin_velocity": np.array([0.0, 0.0, -0.1]),
            "ang_velocity": np.array([1.0, 0.0, 0.0]),
        }
        writer.append_state(state0)

        state1 = {
            "time": 1.0 / 120,
            "position": np.array([0.0, 0.0, 0.499]),
            "orientation": np.array([1.0, 0.0, 0.0, 0.0]),
            "lin_velocity": np.array([0.0, 0.0, -0.2]),
            "ang_velocity": np.array([1.0, 0.0, 0.0]),
        }
        writer.append_state(state1)
        writer.finalize()

        import h5py
        with h5py.File(self.h5_path, "r") as f:
            ds = f["/state"]
            self.assertEqual(ds.shape, (2,))
            row0 = ds[0]
            self.assertAlmostEqual(row0["time"], 0.0)
            np.testing.assert_array_almost_equal(row0["position"], state0["position"])
            np.testing.assert_array_almost_equal(row0["orientation"], state0["orientation"])
            self.assertAlmostEqual(row0["lin_velocity"][2], -0.1)

    def test_t_offset_frames_attr(self):
        writer = DatasetWriter(self.h5_path, scenario="test",
                               t_offset_frames=5)
        writer.finalize()
        import h5py
        with h5py.File(self.h5_path, "r") as f:
            self.assertEqual(f.attrs["t_offset_frames"], 5)

    def test_finalize_creates_state_metadata(self):
        writer = DatasetWriter(self.h5_path, scenario="test")
        for i in range(3):
            writer.append_state({
                "time": float(i),
                "position": np.array([0.0, 0.0, 1.0]),
                "orientation": np.array([1.0, 0.0, 0.0, 0.0]),
                "lin_velocity": np.array([0.0, 0.0, 0.0]),
                "ang_velocity": np.array([0.0, 0.0, 0.0]),
            })
        writer.finalize()
        import h5py
        with h5py.File(self.h5_path, "r") as f:
            self.assertEqual(f.attrs["n_state_rows"], 3)
            self.assertEqual(f.attrs["n_rgb_frames"], 0)


if __name__ == "__main__":
    unittest.main()
