"""Pure-python stereo rig geometry helpers (no numpy / Isaac Sim).

Used by common.replicator.setup_stereo_rig to compute left/right eye
positions from a base camera pose and a baseline. Kept dependency-free
so it can be unit-tested under system python.
"""
import math

__all__ = ["angular_velocity_rad_s_to_deg_s", "compute_eye_positions", "rig_right_vector"]

_UP = (0.0, 0.0, 1.0)
_FALLBACK_RIGHT = (1.0, 0.0, 0.0)


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _scale(v, s):
    return tuple(c * s for c in v)


def _add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def _length(v):
    return math.sqrt(sum(c * c for c in v))


def _normalize(v):
    n = _length(v)
    if n == 0.0:
        raise ValueError("cannot normalize a zero vector")
    return tuple(c / n for c in v)


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def rig_right_vector(base, look_at, up=_UP):
    """Unit vector pointing to the camera's right, given base/look_at/up.

    right = normalize(cross(forward, up)). When forward is parallel to up
    (camera looking straight up/down) the cross product vanishes; we fall
    back to +X so callers never see NaN or a zero vector.

    Args:
        base: camera position (x, y, z).
        look_at: target point the camera is aimed at (x, y, z).
        up: world up vector; defaults to +Z.

    Returns:
        Unit-length (x, y, z) tuple pointing rightward in camera space.
    """
    forward = _normalize(_sub(look_at, base))
    raw = _cross(forward, up)
    if _length(raw) < 1e-9:
        return _FALLBACK_RIGHT
    return _normalize(raw)


def compute_eye_positions(base, look_at, baseline, up=_UP):
    """Return (left_eye, right_eye) positions for a toed-in stereo rig.

    Both eyes share the same look_at target (toed-in convergence). The eyes
    are offset by +/- baseline/2 along the camera's right axis.

    Args:
        base: rig center position (x, y, z).
        look_at: shared convergence target (x, y, z).
        baseline: distance between the two eyes, in meters.
        up: world up vector; defaults to +Z.

    Returns:
        Tuple of (left_eye, right_eye), each an (x, y, z) tuple.
    """
    right = rig_right_vector(base, look_at, up)
    half = baseline / 2.0
    left = _sub(base, _scale(right, half))
    right_eye = _add(base, _scale(right, half))
    return left, right_eye


_RAD_TO_DEG = 180.0 / math.pi


def angular_velocity_rad_s_to_deg_s(vec):
    """Convert an angular velocity vector from radians/second to degrees/second.

    The USD Physics schema defines ``physics:angularVelocity`` in degrees/second
    (see ``usdPhysics/schema.usda`` line 193), so values set in rad/s must be
    converted before authoring the attribute.

    Args:
        vec: (x, y, z) angular velocity in rad/s.

    Returns:
        (x, y, z) angular velocity in deg/s.
    """
    return tuple(v * _RAD_TO_DEG for v in vec)
