This package is to support my Alp stereo camera for the red-crash project.  The camera will be used for create point clouds, for VSLAM, and for FPV teleoperation through a Quest 3 VR headset.

The camera is USB and sends synchronized images in a single frame with the left half. It supports images up to 3200x1200 and at lower resolutions it supports up to 120hz.

The camera device name is 3D USB Camera (V4L2)

The target environment is ROS Jazzy

## Dependencies

### System packages (apt)

```bash
sudo apt install python3-opencv python3-pyqt5 ros-jazzy-cv-bridge ros-jazzy-camera-calibration ros-jazzy-sensor-msgs
```

### ROS packages

Declared in `package.xml` and resolved via `rosdep`:

```bash
cd ~/projects/rc_ws
rosdep install --from-paths alpcam --ignore-src -y
```

### Notes

- **Conda / PlatformIO**: If you use conda or PlatformIO, make sure their Python environments are deactivated so that the system `python3` (with `python3-opencv`) is used. The PlatformIO venv and conda base environment shadow system packages and will cause `ModuleNotFoundError` for `cv2`.
- The camera device is discovered by name via V4L2 sysfs, so the `/dev/videoN` index does not need to be hardcoded.