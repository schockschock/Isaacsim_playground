"""PNG sequence -> MP4 encoding via ffmpeg subprocess.

Pure stdlib (subprocess + os) so it runs under any python, including the
system python used by the unit tests. imageio is only available inside
Isaac Sim's bundled python; shelling out to ffmpeg avoids that dependency
and lets the encoder be unit-tested.
"""
import os
import subprocess

__all__ = ["pngs_to_mp4", "stereo_pair_to_mp4"]


def _run(cmd):
    """Run cmd, raising CalledProcessError with stderr on failure."""
    subprocess.run(cmd, check=True, capture_output=True)


def pngs_to_mp4(frame_dir, out_path, fps=120, pattern="rgb_%04d.png"):
    """Encode a numbered PNG sequence into an MP4 via ffmpeg.

    Args:
        frame_dir: directory containing the PNG frames.
        out_path: output .mp4 path.
        fps: frame rate of the output video.
        pattern: printf-style filename pattern inside frame_dir.

    Returns:
        out_path on success. Raises subprocess.CalledProcessError if ffmpeg
        fails (e.g. no frames match the pattern).
    """
    input_seq = os.path.join(frame_dir, pattern)
    _run(
        [
            "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
            "-framerate", str(fps),
            "-i", input_seq,
            "-c:v", "libx264",
            "-pix_fmt", "yuv420p",
            out_path,
        ]
    )
    return out_path


def stereo_pair_to_mp4(left_dir, right_dir, out_path, fps=120,
                       pattern="rgb_%04d.png", mode="sbs"):
    """Encode left/right PNG sequences into a stereo MP4.

    Args:
        left_dir: directory of left-eye PNGs.
        right_dir: directory of right-eye PNGs.
        out_path: output .mp4 path.
        fps: frame rate.
        pattern: printf-style filename pattern inside each dir.
        mode: "sbs" = side-by-side (horizontal stack, 2x width). Only
            side-by-side is currently supported.

    Returns:
        out_path on success.
    """
    if mode != "sbs":
        raise ValueError(f"unsupported stereo mode: {mode!r} (only 'sbs')")
    left_seq = os.path.join(left_dir, pattern)
    right_seq = os.path.join(right_dir, pattern)
    _run(
        [
            "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
            "-framerate", str(fps),
            "-i", left_seq,
            "-framerate", str(fps),
            "-i", right_seq,
            "-filter_complex", "hstack=inputs=2",
            "-c:v", "libx264",
            "-pix_fmt", "yuv420p",
            out_path,
        ]
    )
    return out_path
