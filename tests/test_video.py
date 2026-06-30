"""Unit tests for common.video — PNG sequence -> MP4 via ffmpeg subprocess.

Runs under system python. Generates synthetic PNGs with ffmpeg's lavfi
testsrc, encodes them with common.video.pngs_to_mp4, then verifies the
output with ffprobe. Requires ffmpeg+ffprobe+libx264 on PATH (true on
this host: ffmpeg 4.4.2).
"""
import os
import subprocess
import tempfile
import unittest

from common.video import pngs_to_mp4, stereo_pair_to_mp4

NUM_FRAMES = 5
FRAME_W = 320
FRAME_H = 240
RATE = 10


def _generate_pngs(dir_path, n=NUM_FRAMES, w=FRAME_W, h=FRAME_H, rate=RATE,
                   pattern="rgb_%04d.png"):
    """Generate n synthetic PNGs into dir_path using ffmpeg lavfi testsrc."""
    subprocess.run(
        [
            "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
            "-f", "lavfi", "-i", f"testsrc=size={w}x{h}:rate={rate}",
            "-vframes", str(n),
            os.path.join(dir_path, pattern),
        ],
        check=True,
    )


def _probe_frames(video_path):
    """Return the number of video frames reported by ffprobe."""
    out = subprocess.run(
        [
            "ffprobe", "-v", "error", "-count_frames",
            "-select_streams", "v:0",
            "-show_entries", "stream=nb_read_frames",
            "-of", "csv=p=0", video_path,
        ],
        capture_output=True, text=True, check=True,
    )
    return int(out.stdout.strip())


def _probe_dimensions(video_path):
    out = subprocess.run(
        [
            "ffprobe", "-v", "error",
            "-select_streams", "v:0",
            "-show_entries", "stream=width,height",
            "-of", "csv=p=0", video_path,
        ],
        capture_output=True, text=True, check=True,
    )
    w, h = out.stdout.strip().split(",")
    return int(w), int(h)


class TestPngsToMp4(unittest.TestCase):
    def test_encodes_all_frames(self):
        with tempfile.TemporaryDirectory() as d:
            _generate_pngs(d)
            out = os.path.join(d, "out.mp4")
            pngs_to_mp4(d, out, fps=RATE)
            self.assertTrue(os.path.isfile(out))
            self.assertEqual(_probe_frames(out), NUM_FRAMES)

    def test_output_dimensions_match_input(self):
        with tempfile.TemporaryDirectory() as d:
            _generate_pngs(d)
            out = os.path.join(d, "out.mp4")
            pngs_to_mp4(d, out, fps=RATE)
            w, h = _probe_dimensions(out)
            self.assertEqual((w, h), (FRAME_W, FRAME_H))

    def test_missing_frames_raises(self):
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, "out.mp4")
            with self.assertRaises(subprocess.CalledProcessError):
                pngs_to_mp4(d, out, fps=RATE)


class TestStereoPairToMp4(unittest.TestCase):
    def test_sbs_double_width(self):
        with tempfile.TemporaryDirectory() as d:
            left = os.path.join(d, "Left")
            right = os.path.join(d, "Right")
            os.makedirs(left)
            os.makedirs(right)
            _generate_pngs(left)
            _generate_pngs(right)
            out = os.path.join(d, "sbs.mp4")
            stereo_pair_to_mp4(left, right, out, fps=RATE, mode="sbs")
            self.assertTrue(os.path.isfile(out))
            w, h = _probe_dimensions(out)
            self.assertEqual((w, h), (FRAME_W * 2, FRAME_H))
            self.assertEqual(_probe_frames(out), NUM_FRAMES)


if __name__ == "__main__":
    unittest.main()
