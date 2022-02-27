#!/usr/bin/env python3

import trio
from itertools import count
import rospy
from rospy import client
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String, Int16, Float64MultiArray, Int8
from sensor_msgs.msg import Imu, NavSatFix
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from random import randint
from struct import *
import math

PORT = 8081
client_number = 0

ONE_WAY = 1
PATROL = 2
CIRCLE = 3
GOAL_REACHED = 3

#Cette classe contient toutes les variables qui vont être envoyées/reçues avec ROS
class message():
    def __init__(self):
        self.lat_goal = 0.0
        self.lon_goal = 0.0
        self.lat_goal_old = 0.0
        self.lon_goal_old = 0.0

        self.current_lat = 0.1
        self.current_lon = 0.5

        self.yaw = 0

        self.waypoint_lat = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.waypoint_lon = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.waypoint_number = 0
        self.current_waypoint = 0
        self.autopilot_mode = 0
        self.cancel_goal = 0

        self.move_base_status = 0



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


# Echo server. Renvoie un message à chaque fois qu'il en reçoit un.
# Le client fixe donc le tickrate du serveur. 
async def echo(server_stream):
    global msg
    async for data in server_stream:
        # Réception
        if len(data) == 8*21 : #Si mauvaise connection, le serveur envoie deux pack d'un coup et tout crash
            data_received_unpacked = unpack('9d9dddd', data)

            for i in range(9):
                msg.waypoint_lat[i] = data_received_unpacked[i] 
                msg.waypoint_lon[i] = data_received_unpacked[i+9] 


            msg.autopilot_mode = data_received_unpacked[18] 
            msg.cancel_goal = data_received_unpacked[19]
            msg.waypoint_number = data_received_unpacked[20]


            #print(msg.autopilot_mode, msg.cancel_goal, msg.waypoint_number)

        data_tosend_packed = pack('dddd', msg.current_lat, msg.current_lon, msg.yaw, msg.current_waypoint)

        await server_stream.send_all(data_tosend_packed)

    

def gps_callback(data):
    msg.current_lat = data.latitude
    msg.current_lon = data.longitude
    #altitude = data.altitude
    


def imu_callback(data):
    euler_angle_rad = euler_from_quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    msg.yaw = euler_angle_rad[2]


def goalstatus_callback(data):
    status = 0
    l = len(data.status_list)
    #last is data.status_list[l-1].goal_id
    for i in range(0, l) :  
        #print(data.status_list[i].status)
        status = data.status_list[i].status

    msg.move_base_status = status



# Tout ce qui concerne ROS (node, publisher, receiver, etc) est déclaré ici.
# La fonction s'execute de manière asynchrone, et peut donc tourner en // du server
# Utiliser classe message pour les datas.
async def ros():
    frequency = 100
    
    rospy.init_node('map_server', anonymous=True)

    navsatfix_goal_pub = rospy.Publisher("gps_goal_fix", NavSatFix, queue_size=10)

    rospy.Subscriber("/gps/fix", NavSatFix, gps_callback)
    rospy.Subscriber("/imu/data", Imu, imu_callback)
    rospy.Subscriber("/move_base/status", GoalStatusArray, goalstatus_callback)
    
    cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    cancel_msg = GoalID()


    while not rospy.is_shutdown():   

        navsatfix = NavSatFix()
        navsatfix.altitude = 0

        #print(msg.move_base_status) 


        if msg.autopilot_mode == ONE_WAY :
            if msg.waypoint_lat[msg.current_waypoint] != msg.lat_goal_old or msg.waypoint_lon[msg.current_waypoint] != msg.lon_goal_old :
                print("going to waypoint number: ", msg.current_waypoint)
                
                navsatfix.latitude = msg.waypoint_lat[msg.current_waypoint]
                navsatfix.longitude = msg.waypoint_lon[msg.current_waypoint]
                navsatfix_goal_pub.publish(navsatfix)
                msg.lat_goal_old = msg.waypoint_lat[msg.current_waypoint]
                msg.lon_goal_old = msg.waypoint_lon[msg.current_waypoint]
                await trio.sleep(0.5) #Laisse le temps au move_base de generer une nouvelle trajectoire


            if msg.move_base_status == GOAL_REACHED:
                print("GOAL REACHED")
                cancel_pub.publish(cancel_msg)
                msg.current_waypoint +=1
                

        if msg.autopilot_mode == CIRCLE :
            if msg.waypoint_lat[msg.current_waypoint] != msg.lat_goal_old or msg.waypoint_lon[msg.current_waypoint] != msg.lon_goal_old :
                print("going to waypoint number: ", msg.current_waypoint)
                
                navsatfix.latitude = msg.waypoint_lat[msg.current_waypoint]
                navsatfix.longitude = msg.waypoint_lon[msg.current_waypoint]
                navsatfix_goal_pub.publish(navsatfix)
                msg.lat_goal_old = msg.waypoint_lat[msg.current_waypoint]
                msg.lon_goal_old = msg.waypoint_lon[msg.current_waypoint]
                await trio.sleep(0.5) #Laisse le temps au move_base de generer une nouvelle trajectoire


            if msg.move_base_status == GOAL_REACHED:
                print("GOAL REACHED")
                cancel_pub.publish(cancel_msg)
                if msg.current_waypoint == (msg.waypoint_number-1):
                    msg.current_waypoint = 0

                else :
                    msg.current_waypoint +=1

               

        if msg.autopilot_mode == PATROL :
            if msg.waypoint_lat[msg.current_waypoint] != msg.lat_goal_old or msg.waypoint_lon[msg.current_waypoint] != msg.lon_goal_old :
                print("going to waypoint number: ", msg.current_waypoint)
                
                navsatfix.latitude = msg.waypoint_lat[msg.current_waypoint]
                navsatfix.longitude = msg.waypoint_lon[msg.current_waypoint]
                navsatfix_goal_pub.publish(navsatfix)
                msg.lat_goal_old = msg.waypoint_lat[msg.current_waypoint]
                msg.lon_goal_old = msg.waypoint_lon[msg.current_waypoint]
                await trio.sleep(0.5) #Laisse le temps au move_base de generer une nouvelle trajectoire


            if msg.move_base_status == GOAL_REACHED:
                print("GOAL REACHED")
                cancel_pub.publish(cancel_msg)
                if msg.current_waypoint == (msg.waypoint_number-1):
                    decrement = True

                if msg.current_waypoint == 0:
                    decrement = False

                if decrement :
                    msg.current_waypoint -=1
                else :
                    msg.current_waypoint +=1


        # publish the goal
        '''
        if msg.lat_goal != msg.lat_goal_old or msg.lon_goal != msg.lon_goal_old :
            cancel_pub.publish(cancel_msg)
            navsatfix_goal_pub.publish(navsatfix)
            msg.lat_goal_old = msg.lat_goal
            msg.lon_goal_old = msg.lon_goal


        '''

     
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



async def main():
    await trio.serve_tcp(server, PORT)
    
    
if __name__ == '__main__':
    msg = message()
    trio.run(main)

