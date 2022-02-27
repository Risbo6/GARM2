#!/usr/bin/python3

import pygame
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('logitech_g29', anonymous=False)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

pygame.init()

# Initialize the joysticks.
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
rospy.loginfo(f'{joystick.get_name()} initialized.')

rate = rospy.Rate(100) 
while not rospy.is_shutdown():
    # ROS message
    twist = Twist()

    #Get events
    for event in pygame.event.get(): # User did something.
        pass

    

    try:
        jid = joystick.get_instance_id()
    except AttributeError:
        jid = joystick.get_id()


    for i in range(joystick.get_numaxes()):
        axis = joystick.get_axis(i)
        #print("Axis {} value: {:>6.3f}".format(i, axis))
        if i == 0:
            twist.angular.z = axis

        elif i == 2:
            twist.linear.x = axis

    cmd_vel_pub.publish(twist)

    rate.sleep()



