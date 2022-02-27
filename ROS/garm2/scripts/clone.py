#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from math import sin, cos, pi, pow, sqrt
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2, PointCloud
from sys import getsizeof
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
#from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np

class odometry_param:
    def __init__(self):
        self.pose_x = 0
        self.pose_y = 0
        self.pose_z = 0

        self.orientation_x = 0
        self.orientation_y = 0
        self.orientation_z = 0
        self.orientation_w = 0

        self.vel_linear_x = 0
        self.vel_linear_y = 0
        self.vel_linear_z = 0

        self.vel_angular_x = 0
        self.vel_angular_y = 0
        self.vel_angular_z = 0


odom_param = odometry_param()


class inertialunit_param:
    def __init__(self):
        self.orientation_x = 0
        self.orientation_y = 0
        self.orientation_z = 0
        self.orientation_w = 0

        self.angular_velocity_x = 0
        self.angular_velocity_y = 0
        self.angular_velocity_z = 0

        self.linear_acceleration_x = 0
        self.linear_acceleration_y = 0
        self.linear_acceleration_z = 0

imu_param = inertialunit_param()



def odom_callback(data):
    odom_param.pose_x = data.pose.pose.position.x
    odom_param.pose_y = data.pose.pose.position.y
    odom_param.pose_z = data.pose.pose.position.z

    odom_param.orientation_x = data.pose.pose.orientation.x
    odom_param.orientation_y = data.pose.pose.orientation.y
    odom_param.orientation_z = data.pose.pose.orientation.z
    odom_param.orientation_w = data.pose.pose.orientation.w

    odom_param.vel_linear_x = data.twist.twist.linear.x
    odom_param.vel_linear_y = data.twist.twist.linear.y
    odom_param.vel_linear_z = data.twist.twist.linear.z

    odom_param.vel_angular_x = data.twist.twist.angular.x
    odom_param.vel_angular_y = data.twist.twist.angular.y
    odom_param.vel_angular_z = data.twist.twist.angular.z


def imu_callback(data):
    imu_param.orientation_x = data.orientation.x
    imu_param.orientation_y = data.orientation.y
    imu_param.orientation_z = data.orientation.z
    imu_param.orientation_w = data.orientation.w

    imu_param.angular_velocity_x = data.angular_velocity.x
    imu_param.angular_velocity_y = data.angular_velocity.y
    imu_param.angular_velocity_z = data.angular_velocity.z

    imu_param.linear_acceleration_x = data.linear_acceleration.x
    imu_param.linear_acceleration_y = data.linear_acceleration.y
    imu_param.linear_acceleration_z = data.linear_acceleration.z




        
rospy.init_node('lcp_clone', anonymous=False)
rospy.Subscriber("/odom", Odometry, odom_callback)
rospy.Subscriber("/imu/data", Imu, imu_callback)




current_time = rospy.Time.now()
br = tf.TransformBroadcaster()
odom_pub = rospy.Publisher("odom_clone", Odometry, queue_size=50)
imu_pub = rospy.Publisher("imu/data_clone", Imu, queue_size=50)


##############################################################################################
#ODOM#
##############################################################################################
# IMU and GPS publisher

rate = rospy.Rate(100) # 100hz
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    '''
    br.sendTransform(
        (odom_param.pose_x, odom_param.pose_y, odom_param.pose_z),
        (odom_param.orientation_x, odom_param.orientation_y, odom_param.orientation_z, odom_param.orientation_w),
        current_time,
        "my_base_link",
        "my_odom"
    )
    '''

    odom = Odometry()
    odom.header.stamp = current_time
    #odom.header.frame_id = "my_odom"
    #odom.child_frame_id = "my_base_link"


    odom.pose.pose = Pose(Point(odom_param.pose_x, odom_param.pose_y, odom_param.pose_z), Quaternion(odom_param.orientation_x, odom_param.orientation_y, odom_param.orientation_z, odom_param.orientation_w))
    odom.twist.twist = Twist(Vector3(odom_param.vel_linear_x, odom_param.vel_linear_y, odom_param.vel_linear_z), Vector3(odom_param.vel_angular_x, odom_param.vel_angular_y, odom_param.vel_angular_z))
    
    # publish the message
    odom_pub.publish(odom)


    ##############################################################################################
    #IMU#
    ##############################################################################################
    
    imu = Imu()
    imu.header.stamp = current_time
    imu.header.frame_id = "imu_clone"


    imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = imu_param.orientation_x, imu_param.orientation_y, imu_param.orientation_z, imu_param.orientation_w
    imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = imu_param.angular_velocity_x, imu_param.angular_velocity_y, imu_param.angular_velocity_z
    imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = imu_param.linear_acceleration_x, imu_param.linear_acceleration_y, imu_param.linear_acceleration_z

    # publish the message
    imu_pub.publish(imu)
    
    br.sendTransform(
        (0, 0, 0),
        (0, 0, 0, 1),
        current_time,
        imu.header.frame_id,
        "base_footprint_filtered"
    )

    rate.sleep()
    


   

    

    

    
    

