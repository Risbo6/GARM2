#!/usr/bin/env python3
import trio
from itertools import count
import rospy
from rospy import client
from std_msgs.msg import String, Int16, Float64MultiArray, Int8
from sensor_msgs.msg import Imu, NavSatFix
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from random import randint
from struct import *
import math

PORT = 8080
client_number = 0

#Cette classe contient toutes les variables qui vont être envoyées/reçues avec ROS
class message():
    def __init__(self):
        self.yaw = 0
        self.pitch = 0.0
        self.roll = 0.0
        self.gps_longitude = 7.1471
        self.gps_latitude = 46.7967
        self.linear_current_speed = 0.0
        self.angular_current_speed = 0.0
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.battery_status = 0
        self.move_base_status = 0
        self.manual_mode_switch = False
        self.move_base_goal_x = 0
        self.move_base_goal_y = 0
        self.move_base_goal_theta = 0





# Echo server. Renvoie un message à chaque fois qu'il en reçoit un.
# Le client fixe donc le tickrate du serveur. 
async def echo(server_stream):
    global msg
    async for data in server_stream:
        # Réception
        if len(data) == 17 : #Si mauvaise connection, le serveur envoie deux pack d'un coup et tout crash
            data_received_unpacked = unpack('dd?', data)
            msg.linear_speed = data_received_unpacked[0]
            msg.angular_speed = data_received_unpacked[1]
            msg.manual_mode_switch = data_received_unpacked[2]

        # Envoi
        #data_tosend_packed = pack('ddd', 5.5, 5.6, 5.6)
        #msg.gps_longitude-=0.000001 
        #msg.gps_latitude -=0.000001 
        #msg.yaw+=0.01
        data_tosend_packed = pack('dddddddiddd', msg.yaw, msg.pitch, msg.roll, msg.gps_longitude, msg.gps_latitude, msg.linear_current_speed, msg.angular_current_speed, msg.move_base_status, msg.move_base_goal_x, msg.move_base_goal_y, msg.move_base_goal_theta)
        await server_stream.send_all(data_tosend_packed)








# Quaternions to Euler
def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z 

# Différents callback ROS liés aux subscribers
def imu_callback(data):
    euler_angle_rad = euler_from_quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    msg.roll = euler_angle_rad[0]
    msg.pitch = euler_angle_rad[1]
    msg.yaw = euler_angle_rad[2]


def battery_callback(data):
    pass


def gps_callback(data):
    msg.gps_latitude = data.latitude
    msg.gps_longitude = data.longitude
    #altitude = data.altitude
    

def odom_callback(data):
    msg.linear_current_speed = data.twist.twist.linear.x
    msg.angular_current_speed = data.twist.twist.angular.z


def goalstatus_callback(data):
    status = 0
    l = len(data.status_list)
    #last is data.status_list[l-1].goal_id
    for i in range(0, l) :  
        #print(data.status_list[i].status)
        status = data.status_list[i].status

    msg.move_base_status = status


def currentgoal_callback(data):
    msg.move_base_goal_x = data.goal.target_pose.pose.position.x
    msg.move_base_goal_y = data.goal.target_pose.pose.position.y
    euler_angle_rad = euler_from_quaternion(data.goal.target_pose.pose.orientation.x, data.goal.target_pose.pose.orientation.y, data.goal.target_pose.pose.orientation.z, data.goal.target_pose.pose.orientation.w)
    msg.move_base_goal_theta = euler_angle_rad[2]




# Tout ce qui concerne ROS (node, publisher, receiver, etc) est déclaré ici.
# La fonction s'execute de manière asynchrone, et peut donc tourner en // du server
# Utiliser classe message pour les datas.
async def ros():
    frequency = 100
    
    rospy.init_node('server', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    #rospy.Subscriber("send_multiple_float", Float64MultiArray, callback)
    rospy.Subscriber("/imu/data", Imu, imu_callback)
    rospy.Subscriber("/capacity_percent", Int8, battery_callback)
    rospy.Subscriber("/gps/fix", NavSatFix, gps_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/move_base/status", GoalStatusArray, goalstatus_callback)
    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, currentgoal_callback)
    

    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    
    while not rospy.is_shutdown():        
        #pub.publish('Ping!')
        vel_msg.linear.x = msg.linear_speed
        vel_msg.angular.z = msg.angular_speed
        if msg.manual_mode_switch :
            pub_vel.publish(vel_msg)
        await trio.sleep(1/frequency)




# Fonction serveur. La nursery s'ouvre que lorqu'au moins un client s'est connecté.
# ROS ne fonctionne donc pas tant que personne n'est connecté.
async def server(server_stream):
    global msg
    global client_number
    client_number+=1 #Number of client connected
    print("Connection started.")

    async with trio.open_nursery() as nursery:
        nursery.start_soon(echo, server_stream)
        if client_number == 1: #ROS only need to be open once on the first client. Else, we'll have multiple instance of the same subscriber/publisher
            nursery.start_soon(ros)
        
    print("Connection closed.")
    msg.linear_speed = 0
    msg.angular_speed = 0

    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub.publish('Error')





async def main():
    await trio.serve_tcp(server, PORT)
    
    
if __name__ == '__main__':
    msg = message()
    trio.run(main)

