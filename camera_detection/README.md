# Manipulation Camera

*Jeff Jewett*

This package configures a USB camera for the purpose of being an external mounted camera that localizes objects to a manipulator arm workspace.

## Installation

```bash
cd ~/ros2_ws
rosdep install --from-paths src/ --ignore-src -r
colcon build
source install/setup.bash
```

If you run into issues, you may need to additionally run

```bash
pip install opencv-python "numpy<2"
```

## Usage

### Calibration

Before using, you need to calibrate the camera to remove distortions. This requires a physical checkerboard pattern, such as https://github.com/kyle-bersani/opencv-examples/blob/master/CalibrationByChessboard/chessboard-to-print.pdf. Run

```
ros2 launch camera_detection calibrate.py
```

Move the checkerboard and/or camera around to get lots of angles. Once it is sufficient, hit calibrate. It should then give a distortion estimate, which should hopefully be below 0.3px. When satisfied, hit Save and Commit. It should save in `config/camera_info.yaml`. If it crashes, [don't blame me](https://github.com/ros-perception/image_pipeline/issues/1056)--you can extract it manually with

```
tar -zxvf /tmp/calibrationdata.tar.gz -O ost.yaml > ~/ros2_ws/ROS-Warehouse-Demo/camera_info.yaml
```

### Using the camera

Currently, there is not much to do other than view it in `rviz2`. Run

```bash
ros2 launch camera_detection camera.py
```

The raw camera feed is on `/camera/image_raw`. The undistorted camera feed is on `/camera/image_rect`. To make it easier to verify the camera is undistorted, `/camera/image_rect_grid` shows a grid overlay.