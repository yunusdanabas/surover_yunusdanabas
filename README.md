# Camera Testing and ArUco Tag Detection

This repository contains a collection of small ROS launch files for testing camera configurations (Intel RealSense and ZED2) and ArUco tag detection. These tests were performed as part of integrating camera inputs into our rover's SLAM system. Note that this repo only includes camera configuration and detection testing code—not the full rover SLAM system.

## Features

- **Aruco Tag Detection:** Uses the `aruco_detect` package to detect ArUco tags.
- **RealSense Testing:** Launch files for testing the Intel RealSense camera with ArUco detection.
  - `realsense_aruco_detect.launch`
  - `realsense_aruco_detect_fixedcamera.launch`
- **ZED2 Testing:** Launch file for testing the ZED2 camera with ArUco detection.
  - `zed2_aruco_detect.launch`
- **Additional Testing:** Other sample launch files (e.g., `singlerobot_manuel.launch`, `fiducial_slam_turtlebot3.launch`) for experimenting with camera integration in a simulated rover environment.

## Usage

To test a specific configuration, simply run the corresponding launch file. For example, to test the ZED2 camera with ArUco detection:

```bash
roslaunch <your_package_name> zed2_aruco_detect.launch
```

Replace `<your_package_name>` with the name of your package.

## Dependencies

Ensure you have the following ROS packages installed:
- `aruco_detect`
- `fiducial_slam`
- `zed_wrapper` (for ZED2 camera)
- `realsense2_camera` (for Intel RealSense)

## Author

**Yunus Emre Danabas**  
Email: [yunusemredanabas@gmail.com](mailto:yunusemredanabas@gmail.com)
```
