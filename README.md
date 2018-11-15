# Camera Lidar Fusion
This project was created to calibrate camera and lidar. It calculates transformation between RGB camera frame and Lidar point cloud frame, projects a point cloud onto RGB image and, and projects RGB image pixels onto a point cloud.


# 1. Quickstart / Minimal Setup

The calibration consists three main steps:
1) camera intrinsic parameters calibration
2) camera-lidar extrinsic parameters calibration
3) camera-lidar projection
4) lidar-camera projection
5) visualization

## 1.1 Camera intrinsic parameters calibration

The camera intrinsic parameters can be estimated by using the [cameracalibrator tool](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) from ROS camera_calibration package.


Start this procedure by starting roscore:
        roscore 
