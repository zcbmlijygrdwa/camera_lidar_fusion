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

<p align="center">
    <img src="http://zhenyuyang.usite.pro/rgbd_calib/calibrator_all_button.png">
</p>


A GUI window will show up and start detecting chessboard. As the chessboard being moved around the calibration sidebars increase in length. When the CALIBRATE button turns green, it mean the calibrator is ready to calulate the camera intrinsic parameters. Click on the CALIBRATE button to start calculation. When this is done, click COMMIT to apply the new camera intrinsic parameters to the camera info. Click SAVE to save the calibration results, results will be saved as /tmp/calibrationdata.tar.gz. This compressed file contains ost.yaml files, which will be used to rectify the RGB camera frames.


It may happen that the chessboard in video moved too fast that the calibrate cannot capture enough frames for calibration. One solution is slowing down the framerate by add -r 0.3 flag to rosbag play to play the bag at X0.3 speed:

    rosbag play -r 0.3 rosbag_file_name.bag



## 1.2 Update the camera intrinsic parameters in rosbag data

As the camera intrinsic parameters are obtained, the original rosbag data can be updated by using bag_tools from ROS.

If you are using ROS Indigo, you can try to install the package by using apt-get:

    sudo apt-get install ros-indigo-bag-tools

If you are using ROS Kinetic, you can try to install the package by building from the [source](https://github.com/srv/srv_tools):

Go to /src in your ROS workspace

    cd catkin_ws/src

Clone srv_tools repo:

    git clone https://github.com/srv/srv_tools.git

Install:

    cd ..
    catkin_make
    source devel/setup.bash



Now we can update the rosbag camera_info data by creating a new one based on the yaml file obtained from camera calibrator:

    python change_camera_info.py /path_to_original_bag_data.bag /path_to_new_bag_data.bag /sensors/camera/camera_info=/path_to_yaml_file.yaml

With the generated bag file being played by rosbag, rectification can be done by using the image_proc package from ROS:

    ROS_NAMESPACE=sensors/camera rosrun image_proc image_proc image_raw:=image_color

Use image_view package from ROS for viualizing the rectified image:

    rosrun image_view image_view image:=/sensors/camera/imaget_color

Compared with the original image, it is clear that the RGB image is rectified and the edge of chessboard is straight(shown below).

Original image:
<p align="center">
    <img src="http://zhenyuyang.usite.pro/rgbd_calib/calibrator_before.png">
</p>


Rectified image:
<p align="center">
    <img src="http://zhenyuyang.usite.pro/rgbd_calib/calibrator_after.png">
</p>



 
