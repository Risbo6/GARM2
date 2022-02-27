from controller import Robot
from controller import LidarPoint

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

class robot_param:
    def __init__(self):
        self.vel_l = 0
        self.vel_r = 0
        self.wheel_bias = 0.28*2
        self.wheel_radius = 0.1

r_param = robot_param()





def cmd_vel_callback(msg):
    r_param.vel_l = ((msg.linear.x - (msg.angular.z * r_param.wheel_bias / 2.0)) / r_param.wheel_radius)
    r_param.vel_r = ((msg.linear.x + (msg.angular.z * r_param.wheel_bias / 2.0)) / r_param.wheel_radius)


        
rospy.init_node('Garm2_webots', anonymous=False)


rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)



TIME_STEP = 32
robot = Robot()

wheels = []
wheelsNames = ['FL_WHEEL', 'BL_WHEEL', 'L_WHEEL','FR_WHEEL', 'BR_WHEEL', 'R_WHEEL']

inertial_unit = robot.getDevice('inertial_unit')
inertial_unit.enable(TIME_STEP)

front_lidar = robot.getDevice('front_lidar')
front_lidar.enable(TIME_STEP)
#front_lidar.enablePointCloud()


top_lidar = robot.getDevice('VelodyneHDL-32E')
top_lidar.enable(TIME_STEP)
top_lidar.enablePointCloud()

accel = robot.getDevice('accel')
accel.enable(TIME_STEP)

gyro = robot.getDevice('gyro')
gyro.enable(TIME_STEP)

camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)




for i in range(6):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)



current_time = rospy.Time.now()
br = tf.TransformBroadcaster()
##############################################################################################
#ODOM#
##############################################################################################
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)


x = 0.0
y = 0.0
th = 0.0

vx = 0
vy = 0
vth = 0
ul = 0
ur = 0


##############################################################################################
#ODOM#
##############################################################################################

# IMU and GPS publisher
imu_pub = rospy.Publisher("imu/data", Imu, queue_size=50)
navsatfix_pub = rospy.Publisher("gps/fix", NavSatFix, queue_size=50)
lidar_pub = rospy.Publisher("velodyne/cloud", PointCloud2, queue_size=50)










while robot.step(TIME_STEP) != -1:
    current_time = rospy.Time.now()


    ##############################################################################################
    #ODOM#
    ##############################################################################################
    #Compute speed

    ul = wheels[2].getVelocity() # Left wheel
    ur = wheels[5].getVelocity() # Right wheel

    vx = (r_param.wheel_radius/2)*(ul+ur)*cos(th)
    vy = (r_param.wheel_radius/2)*(ul+ur)*sin(th)

    v = sqrt((pow(vx, 2)+pow(vy, 2)))

    #Erreur ici

    vth = (r_param.wheel_radius/r_param.wheel_bias)*(ur-ul)

    # compute odometry in a typical way given the velocities of the robot
    dt = TIME_STEP/1000
    
    #delta_x = (vx * cos(th) - vy * sin(th)) * dt
    #delta_y = (vx * sin(th) + vy * cos(th)) * dt

    delta_x = vx * dt
    delta_y = vy * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th


    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    
    
    br.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )
    





    

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    #odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)



    ##############################################################################################
    #ODOM#
    ##############################################################################################


    ##############################################################################################
    #IMU#
    ##############################################################################################
    imu = Imu()
    imu.header.stamp = current_time
    imu.header.frame_id = "imu"


    # Get quaternion
    roll, pitch, yaw = inertial_unit.getRollPitchYaw()
    imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro.getValues()
    imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = accel.getValues() 

    #imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = inertial_unit.getQuaternion()

    # No angular velocities or linear accel, so cov = -1
    #imu.linear_acceleration_covariance = [-1,-1,-1,-1,-1,-1,-1,-1,-1]
    #imu.angular_velocity_covariance = [-1,-1,-1,-1,-1,-1,-1,-1,-1]

    # publish the message
    imu_pub.publish(imu)

    br.sendTransform(
        (0, 0, 0),
        (0, 0, 0, 1),
        current_time,
        imu.header.frame_id,
        "base_link"
    )


    ##############################################################################################
    #IMU#
    ##############################################################################################



    ##############################################################################################
    #GPS#
    ##############################################################################################
    navsatfix = NavSatFix()
    navsatfix.header.stamp = current_time
    navsatfix.header.frame_id = "gps"

    navsatfix.latitude, navsatfix.longitude, navsatfix.altitude = gps.getValues()

    # Covariance type unknown
    navsatfix.position_covariance_type = 0

    # publish the message
    navsatfix_pub.publish(navsatfix)

    br.sendTransform(
        (0, 0, 0),
        (0, 0, 0, 1),
        current_time,
        navsatfix.header.frame_id,
        "base_link"
    )
    ##############################################################################################
    #GPS#
    ##############################################################################################


    ##############################################################################################
    #TOP LIDAR#
    ##############################################################################################
    #Lidar data
    #point_cloud = LidarPoint()

    
    pointCloud = top_lidar.getPointCloud('list')
    mylist = []
    for i in range (top_lidar.getNumberOfPoints()):
        xyz = [-pointCloud[i].x, pointCloud[i].z ,pointCloud[i].y]
        #xyz = [pointCloud[i].x, pointCloud[i].y ,pointCloud[i].z]
        mylist.append(xyz)

    #lidar_header = Header()
    #lidar_header.frame_id = "lidar"
    pointcloud = PointCloud2()
    pointcloud.header.stamp = current_time
    pointcloud.header.frame_id = "lidar"

    pointcloud2 = point_cloud2.create_cloud_xyz32(pointcloud.header, mylist)

    lidar_pub.publish(pointcloud2)

    lidar_quat = tf.transformations.quaternion_from_euler(-pi/2, pi, 0)
    #print(lidar_quat)
    #lidar_quat = tf.transformations.quaternion_from_euler(0, 0, 0)


    
    br.sendTransform(
        (-0.2, 0, 1),
        lidar_quat,
        current_time,
        pointcloud.header.frame_id,
        "base_link"
    )
    
    

    ##############################################################################################
    #TOP LIDAR#
    ##############################################################################################


    



    wheels[2].setVelocity(r_param.vel_l) # Left wheel
    wheels[5].setVelocity(r_param.vel_r) # Right wheel


    


    
    


   

    

    

    
    

