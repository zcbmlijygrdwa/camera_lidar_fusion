# Camera Lidar Fusion
This project was created to calibrate camera and lidar. It calculates transformation between RGB camera frame and Lidar point cloud frame, projects a point cloud onto RGB image and, and projects RGB image pixels onto a point cloud.


### Related Papers

* **LSD-SLAM: Large-Scale Direct Monocular SLAM**, *J. Engel, T. Schöps, D. Cremers*, ECCV '14

* **Semi-Dense Visual Odometry for a Monocular Camera**, *J. Engel, J. Sturm, D. Cremers*, ICCV '13




# 1. Quickstart / Minimal Setup

Start roscore:

    roscore

Play rosbag:

    rosbag play rosbag_file_name.bag

Use camera_calibration tool to perform calibration, and save the results:

    rosrun camera_calibration cameracalibrator.py --size 5x7 --square 0.05 image:=/sensors/camera/image_color

Fetch yaml file from the results:

    roscd camera_lidar_fusion/data/
    cp /tmp/calibrationdata.tar.gz ./
    tar xvzf calibrationdata.tar.gz ost.yaml
    rm calibrationdata.tar.gz

Create a new rosbag data file with updated camera_info based on the yaml file.

    roscd camera_lidar_fusion/scripts/
    python change_camera_info.py /path_to_original_bag_data.bag /path_to_new_bag_data.bag /sensors/camera/camera_info=../data/ost.yaml


Play the newly generated rosbag file and use image_proc tool to recify the rgb image and publish the results:

    rosbag play /path_to_new_bag_data.bag
    
In a new terminal window use image_proc tool to recify the rgb image and publish the results:

    ROS_NAMESPACE=sensors/camera rosrun image_proc image_proc image_raw:=image_color
    
Use image_view2 in image_view2 ROS package to get pixel coordinates;

    apt-get install ros-kinetic-image-view2
    rosrun image_view2 image_view2 image:=/sensors/camera/image_rect_color

Use Publish Point in rviz to get coordinates of corresponding 3D points in point cloud:

    rviz

Write the 2D-3D point pairs in /data/corr.txt. At lease 9 pairs are needed.

Run transformation_estimator to estimate the static transform between camera and lidar:

    rosrun camera_lidar_fusion transformation_estimator.py

Start projection node:

    rosrun camera_lidar_fusion projection.py

Visulize the projections in rivz.


# 2. Details


The calibration consists three main steps:
1) camera intrinsic parameters calibration
2) camera-lidar extrinsic parameters calibration
3) camera-lidar and lidar-camera projection
4) visualization

## 2.1.1 Camera intrinsic parameters calibration

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



## 2.1.2 Update the camera intrinsic parameters in rosbag data

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



## 2.2 camera-lidar extrinsic parameters calibration

The camera-lidar extrinsic parameters calibration is based on the correspondence between a set of 2D points in image and a set of 3D points in point cloud. This correspondence can be obtained by directly selecting points on a pair of 2D frame and 3D frame. In my approach I store the correspondence points in a txt file called corr.txt with the following format:

point2D_x,point2D_y,point3D_x,point3D_y,point3D_z

Node transformation_estimator was created to perform calculation of transformation between camera and lidar. The idea here is to give an initail transformation and project the 3D point into 2D space, then get the error as sum of the eurclidian distance between each selected 2D point and projected 2D point pair, and then minimize the error to get a estimation of trasformation between camera and lidar.

Run this node with rosrun, and when a estimated transformation is found, this node will publish the transformation as a Pose message as topic /lidar_pose.

    rosrun camera_lidar_fusion transformation_estimator.py


 
 
## 2.3 Projections

As the static transform between the camera and the lidar is obtained, projection can be made. The projection node subscribe to the /lidar_pose topic, and uses the transform information to project 3D points onto RGB image and find the corresponding colors for 3D points. This node publishes results as /image_color_with_proj (image with 3D points projection) and /pointcloud_color (point cloud data with color information). 

    rosrun camera_lidar_fusion projection.py


 
## 2.4 Visualization
We can use RVIZ to visulization the results. Open the rivz:

    rviz

With the rviz GUI loaded, we can display the RGB image with projected 3D points by adding visulization by topic /image_color_with_proj.  We can display the colored point cloud by adding visulization by topic /pointcloud_color. Note that to display the rgb color of the point cloud, please select RGBF32 option (by default it is Intensity) for Color Transformer. Increase Size(m) for better visibility. 


<p align="center">
    <img src="http://zhenyuyang.usite.pro/rgbd_calib/final_rgb.png">
</p>
 
<p align="center">
    <img src="http://zhenyuyang.usite.pro/rgbd_calib/final_pc.png">
</p>
 
