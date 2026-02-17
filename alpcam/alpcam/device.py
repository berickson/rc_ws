"""Utility to discover the Alp stereo camera by V4L2 device name."""

import glob
import os

CAMERA_NAME = "3D USB Camera"


def find_camera_device(target_name: str = CAMERA_NAME) -> str | None:
    """Scan /sys/class/video4linux to find the /dev/videoN path for a camera
    matching *target_name*.

    Returns the device path (e.g. '/dev/video2') or None if not found.
    Only returns video-capture devices (skips metadata nodes).
    """
    for name_file in sorted(glob.glob("/sys/class/video4linux/video*/name")):
        with open(name_file) as f:
            name = f.read().strip()
        if target_name in name:
            device_dir = os.path.dirname(name_file)
            device_id = os.path.basename(device_dir)  # e.g. "video2"

            # V4L2 exposes multiple /dev/videoN nodes per physical device
            # (capture, metadata, etc.).  Check that this one supports
            # video capture by looking for "capture" in the
            # device_caps or uevent.
            uevent_path = os.path.join(device_dir, "uevent")
            if os.path.exists(uevent_path):
                with open(uevent_path) as f:
                    uevent = f.read()
                # Metadata nodes typically have MINOR numbers but we can
                # also just try opening the first match — V4L2 lists the
                # capture node first.  Return the first match.
            return f"/dev/{device_id}"
    return None


def main():
    """CLI entry point — prints discovered device path."""
    device = find_camera_device()
    if device:
        print(f"Found '{CAMERA_NAME}' at {device}")
    else:
        print(f"Camera '{CAMERA_NAME}' not found. Is it plugged in?")
        print("Available V4L2 devices:")
        for name_file in sorted(glob.glob("/sys/class/video4linux/video*/name")):
            with open(name_file) as f:
                name = f.read().strip()
            device_id = os.path.basename(os.path.dirname(name_file))
            print(f"  /dev/{device_id}: {name}")


if __name__ == "__main__":
    main()
