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

Open another terminal window and play the rosbag:

    rosbag play rosbag_file_name.bag


Open another terminal window and start the ROS cameracalibrator:

    rosrun camera_calibration cameracalibrator.py --size 5x7 --square 0.05 image:=/sensors/camera/image_color

A GUI window will show up and start detecting chessboard. As the chessboard being moved around the calibration sidebars increase in length. When the CALIBRATE button turns green, it mean the calibrator is ready to calulate the camera intrinsic parameters. Click on the CALIBRATE button to start calculation. When this is done, Click SAVE to save the result and click COMMIT to apply the new camera intrinsic parameters to the camera info.


It may happen that the chessboard in video moved too fast that the calibrate cannot capture enough frames for calibration. One solution is slowing down the framerate by add -r 0.3 flag to rosbag play to play the bag at X0.3 speed:

    rosbag play -r 0.3 rosbag_file_name.bag


