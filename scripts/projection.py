#!/usr/bin/env python
# license removed for brevity


from image_geometry import PinholeCameraModel
import cv2
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
from scipy.optimize import minimize
from sensor_msgs.msg import Image, PointField, CameraInfo
import numpy as np
import tf
import commentjson as json
import math
import random
import sensor_msgs.point_cloud2
import message_filters   #to synchronize topic subscription

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs.point_cloud2 import read_points
from tf.transformations import euler_from_quaternion, quaternion_from_euler


cameraModel = PinholeCameraModel()

pub_image = {}
pub_pc = {}
pub_pose = rospy.Publisher('lidar_pose', Pose, queue_size=10)

isRotMatSet = False
rotationMatrix = []
cv_image = []
bridge = {}

def create_pc_fields():
    fields = []
    fields.append( PointField( 'x', 0, PointField.FLOAT32, 1 ) )
    fields.append( PointField( 'y', 4, PointField.FLOAT32, 1 ) )
    fields.append( PointField( 'z', 8, PointField.FLOAT32, 1 ) )
    fields.append( PointField( 'intensity', 12, PointField.FLOAT32, 1 ) )
    fields.append( PointField( 'r', 16, PointField.FLOAT32, 1 ) )
    fields.append( PointField( 'g', 20, PointField.FLOAT32, 1 ) )
    fields.append( PointField( 'b', 24, PointField.FLOAT32, 1 ) )
    return fields

def RGBD_callback(image_data,pointCloud_data):
    global cv_image
    global bridge
    global cameraModel
    global isRotMatSet
    global rotationMatrix
    global pub_image
    global pub_pc

    print("new frame received.")

    try:
      cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")
      width, height = cv_image.shape[:2]
      #print "cv_image w h = "+str(width) +", "+ str(height)
    except CvBridgeError as e:
      print(e)

    if(isRotMatSet):
        cv_temp = []
        cv_temp = cv_image.copy();
        width, height = cv_temp.shape[:2]
        
        new_points = []
        for point in (read_points(pointCloud_data, skip_nans=True)):

                pointXYZ = [point[0],point[1],point[2],1.0]
                intensity = point[3]
                intensityInt = int(intensity*intensity*intensity)
                transformedPoint = rotationMatrix.dot(pointXYZ)
                if transformedPoint[ 2 ] < 0:
                    continue
                projected_2d_point = cameraModel.project3dToPixel( transformedPoint )
                #projection
                if projected_2d_point[ 0 ] >= 0 and projected_2d_point[ 0 ] <= width and projected_2d_point[ 1 ] >= 0 and projected_2d_point[ 1 ] <= height:
                    cv2.circle(cv_temp, (int(projected_2d_point[0]), int(projected_2d_point[1])), 5,  (intensityInt % 255, ( intensityInt / 255 ) % 255, ( intensityInt / 255 / 255 )), thickness = -1 )
                    [b,g,r] = cv_image[int(projected_2d_point[1]),int(projected_2d_point[0 ])]
                    new_points.append([point[0],point[1],point[2],intensity,r/255.0,g/255.0,b/255.0])
        try:
            pub_image.publish(bridge.cv2_to_imgmsg(cv_temp, "bgr8"))
            new_pointCloud = sensor_msgs.point_cloud2.create_cloud(pointCloud_data.header, create_pc_fields(), new_points)
            pub_pc.publish(new_pointCloud)
        except CvBridgeError as e:
            print(e)

    else:
        print( 'Waiting for pose info from /' + sub_pose )


def poseCallback( data ):
    global isRotMatSet
    global rotationMatrix
    pose = data
    #print("lidarToRGB, pose received")
    #print(pose)
    quaternion = (
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    
    translation = [pose.position.x, pose.position.y, pose.position.z, 1.0 ]
    rotationMatrix = tf.transformations.euler_matrix(roll,pitch,yaw)
    transformMatrix = rotationMatrix
    transformMatrix[ :, 3 ] = translation

    isRotMatSet = True


def cameraCallback( data ):
    global cameraModel
    cameraModel.fromCameraInfo(data)

def lidarToRGB():
    global pub_image
    global pub_pc
    global bridge
    rospy.init_node('lidar_to_rgb', anonymous=True)

    bridge = CvBridge()

    sub_pose = rospy.resolve_name( 'lidar_pose' )


    # subscribe to camera
    sub_pose = rospy.Subscriber( sub_pose, Pose, callback = poseCallback, queue_size = 1 )
    camera = rospy.Subscriber(rospy.resolve_name( 'sensors/camera/camera_info'), CameraInfo, callback = cameraCallback, queue_size = 1 )
    
    pub_image = rospy.Publisher("image_color_with_proj",Image)
    pub_pc = rospy.Publisher("pointcloud_color", PointCloud2, queue_size = 1 )

    sub_image = message_filters.Subscriber('sensors/camera/image_color', Image)
    sub_pointcloud = message_filters.Subscriber('sensors/velodyne_points', PointCloud2)

    ts = message_filters.ApproximateTimeSynchronizer([sub_image, sub_pointcloud], 1,0.1)
    ts.registerCallback(RGBD_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        lidarToRGB()
    except rospy.ROSInterruptException:
        pass
