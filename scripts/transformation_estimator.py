#!/usr/bin/env python
# license removed for brevity


from image_geometry import PinholeCameraModel
import cv2
import cv_bridge
from scipy.optimize import minimize
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import tf
import math
import random

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler

cameraModel = PinholeCameraModel()
pub_pose = rospy.Publisher('lidar_pose', Pose, queue_size=10)
pose_fusion = {}
isRTFound = False

PI = 3.141592653
myBounds = [[-1, 1], [-1, 1], [-1, 1], [0, 2*PI], [0, 2*PI], [0, 2*PI]]

def generateRandTsfm():
        randomTransform = [0]*len(myBounds)
        #generate random initial transform
        for i in range( 0, len(randomTransform) ):
            randomTransform[i] = random.uniform(myBounds[ i ][ 0 ], myBounds[ i ][ 1 ] )
        return randomTransform

def costCalculation( sol, sign = 1.0, debug = False ):
    global cameraModel
    translation = [ sol[ 0 ], sol[ 1 ], sol[ 2 ], 1.0 ]
    rotationMatrix = tf.transformations.euler_matrix( sol[ 5 ], sol[ 4 ], sol[ 3 ] )
    trasformMatrix = rotationMatrix
    trasformMatrix[ :, 3 ] = translation

    error = 0
    for i in range( 0, len(pointSet_3D) ):
        point = np.concatenate((np.asarray(pointSet_3D[i]),1.0), axis=None)
        selected_2d_point = np.asarray(pointSet_2D[i])
        rotated_3d_point = trasformMatrix.dot( point )
        projected_2d_point = cameraModel.project3dToPixel( rotated_3d_point )

        tempError =  math.sqrt( np.sum( (np.asarray( projected_2d_point ) - np.array( selected_2d_point )) ** 2))
        error = error + tempError
    return sign * error



def cameraCallback( data ):
    global cameraModel
    global pose_fusion
    global isRTFound

    if(not isRTFound):
        cameraModel.fromCameraInfo(data)
        result = minimize( costCalculation, generateRandTsfm(), args = ( 1.0, False ), bounds = myBounds, method = 'SLSQP', options = { 'disp': False, 'maxiter': 10000 } )
        minE = 100
        while not result.success or result.fun > 40:
            result = minimize( costCalculation, generateRandTsfm(), args = ( 1.0, False ), bounds = myBounds, method = 'SLSQP', options = { 'disp': False, 'maxiter': 10000 } )
            if(result.fun<minE):
                minE  = result.fun
                print "new minimal error found: " + str(minE)

        final_sol = result.x
        print( "Tranform found: x:"+str(final_sol[0])+", y = "+str(final_sol[1])+", z = "+str(final_sol[2])+", roll = "+str(final_sol[5])+", pithc = "+str(final_sol[4])+", yaw = "+str(final_sol[ 3 ]))
        print( '' )
        print( 'Final transform:' )
        print( result.x)


        translation = [final_sol[0],final_sol[1],final_sol[2],1.0]
        rotationMatrix = tf.transformations.euler_matrix( final_sol[ 5 ], final_sol[ 4 ], final_sol[ 3 ] )
        trasformMatrix = rotationMatrix
        trasformMatrix[ :, 3 ] = translation

        pose_fusion = Pose(Point(final_sol[ 0 ].item(), final_sol[ 1 ].item(), final_sol[ 2 ].item()),Quaternion(*quaternion_from_euler(final_sol[ 5 ].item(), final_sol[ 4 ].item(), final_sol[ 3 ].item())))
        isRTFound = True
        
    pub_pose.publish(pose_fusion)


def rgbd_fuser():
    rospy.init_node('transformation_estimator', anonymous=True)
    camera_info = rospy.resolve_name( 'sensors/camera/camera_info' )

    camera = rospy.Subscriber( camera_info, CameraInfo, callback = cameraCallback, queue_size = 1 )

    rospy.spin()

if __name__ == '__main__':
    try:
        File_object = open("/home/zhenyu/catkin_ws/src/camera_lidar_fusion/data/corr.txt","r")
        corr_data_str = File_object.readlines()

        pointSet_2D = [0]*len(corr_data_str)
        pointSet_3D = [0]*len(corr_data_str)

        for i in range(len(corr_data_str)):
            print "spliting: "+corr_data_str[i]
            tempSpli = corr_data_str[i].split(',')
            for j in range(len(tempSpli)):
                print "tempSpli[] = " +str(tempSpli[j])        
            pointSet_2D[i] = [float(tempSpli[0]),float(tempSpli[1])]
            pointSet_3D[i] = [float(tempSpli[2]),float(tempSpli[3]),float(tempSpli[4])]
        print(corr_data_str)
        print "pointSet_2D = " + str(pointSet_2D)
        print "pointSet_3D = " + str(pointSet_3D)

        rgbd_fuser()
    except rospy.ROSInterruptException:
        pass
