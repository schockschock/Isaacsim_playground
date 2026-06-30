"""Pure-python settled-detection helper (no numpy / Isaac Sim).

Used by common.loop.run_capture_loop to decide when a rigid body has come
to rest. Convention matches samples/freefall.py: a body is settled when,
over a sliding window of `required_len` recent positions, the per-axis
spread (max - min) is strictly below `epsilon` on every axis.
"""
__all__ = ["is_settled"]


def is_settled(window, epsilon, required_len):
    """Return True if the position window indicates the body has settled.

    Args:
        window: list of recent positions, each an (x, y, z) tuple.
        epsilon: max allowed per-axis drift (meters) over the window.
        required_len: minimum number of samples required before deciding.

    Returns:
        True only if len(window) >= required_len AND every axis spread is
        strictly less than epsilon. False otherwise (including empty/short
        windows).
    """
    if len(window) < required_len:
        return False
    xs = [p[0] for p in window]
    ys = [p[1] for p in window]
    zs = [p[2] for p in window]
    return (
        (max(xs) - min(xs)) < epsilon
        and (max(ys) - min(ys)) < epsilon
        and (max(zs) - min(zs)) < epsilon
    )
