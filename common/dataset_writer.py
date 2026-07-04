"""Per-run HDF5 dataset writer for PhysX rigid-body state.

Writes per-frame state as a structured numpy array in an HDF5 file,
with run metadata stored as top-level attributes.

See ADR-0005 for the per-run HDF5 schema.
"""
import h5py
import numpy as np

STATE_DTYPE = np.dtype([
    ("time", np.float64),
    ("position", np.float64, (3,)),
    ("orientation", np.float64, (4,)),
    ("lin_velocity", np.float64, (3,)),
    ("ang_velocity", np.float64, (3,)),
    ("mass", np.float64),
    ("inertia_world", np.float64, (3, 3)),
    ("E_trans", np.float64),
    ("E_rot", np.float64),
    ("PE_grav", np.float64),
    ("E_total", np.float64),
    ("p_lin", np.float64, (3,)),
    ("L_ang_world", np.float64, (3,)),
    ("L_ang_body", np.float64, (3,)),
    ("dissipating", np.bool_),
    ("in_contact", np.bool_),
])

CONTACTS_DTYPE = np.dtype([
    ("step", np.int32),
    ("position", np.float64, (3,)),
    ("normal", np.float64, (3,)),
    ("impulse", np.float64, (3,)),
    ("separation", np.float64),
    ("face_index0", np.int32),
    ("face_index1", np.int32),
])


class DatasetWriter:
    def __init__(self, path, scenario, fps=120, dt_phys=1.0 / 480,
                 solver_substeps=4, t_offset_frames=0,
                 baseline_m=None, material=None, potato_usd_path=None):
        self._file = h5py.File(path, "w")
        self._file.attrs["scenario"] = scenario
        self._file.attrs["fps"] = fps
        self._file.attrs["dt_phys"] = dt_phys
        self._file.attrs["solver_substeps"] = solver_substeps
        self._file.attrs["t_offset_frames"] = t_offset_frames

        if baseline_m is not None:
            self._file.attrs["baseline_m"] = baseline_m
        if material is not None:
            for k, v in material.items():
                self._file.attrs[f"material_{k}"] = v
        if potato_usd_path is not None:
            self._file.attrs["potato_usd_path"] = potato_usd_path

        self._file.create_dataset(
            "/state", shape=(0,), maxshape=(None,), dtype=STATE_DTYPE
        )
        self._file.create_dataset(
            "/contacts", shape=(0,), maxshape=(None,), dtype=CONTACTS_DTYPE
        )
        self._state_rows = []
        self._contact_rows = []
        self._n_rgb_frames = 0

    def append_state(self, state_dict):
        row = np.zeros(1, dtype=STATE_DTYPE)
        for field in STATE_DTYPE.names:
            val = state_dict.get(field)
            if val is not None:
                row[field] = val
        self._state_rows.append(row)

    def append_contacts(self, step_idx, contacts):
        for c in contacts:
            row = np.zeros(1, dtype=CONTACTS_DTYPE)
            row["step"] = step_idx
            row["position"] = c["position"]
            row["normal"] = c["normal"]
            row["impulse"] = c["impulse"]
            row["separation"] = c["separation"]
            row["face_index0"] = c["face_index0"]
            row["face_index1"] = c["face_index1"]
            self._contact_rows.append(row)

    def set_body_invariants(self, com_offset, inertia_body_col9):
        self._file.attrs["body_com_offset"] = np.asarray(com_offset, dtype=np.float64)
        self._file.attrs["body_inertia_body_col9"] = np.asarray(inertia_body_col9, dtype=np.float64)

    def set_n_rgb_frames(self, n):
        self._n_rgb_frames = n

    def finalize(self):
        if self._state_rows:
            data = np.concatenate(self._state_rows, axis=0)
            self._file["/state"].resize(len(data), axis=0)
            self._file["/state"][:] = data
        if self._contact_rows:
            contact_data = np.concatenate(self._contact_rows, axis=0)
            self._file["/contacts"].resize(len(contact_data), axis=0)
            self._file["/contacts"][:] = contact_data
        self._file.attrs["n_state_rows"] = len(self._state_rows)
        self._file.attrs["n_rgb_frames"] = self._n_rgb_frames
        self._file.close()
