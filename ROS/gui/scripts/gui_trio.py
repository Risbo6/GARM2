#!/usr/bin/env python3.8
import rospy
from rospy import client
from std_msgs.msg import String, Int16, Float64MultiArray
from sensor_msgs.msg import Joy
from math import degrees, atan2
import numpy as np


'''
print("IP: 160.98.",end ='')
IP=input()
IP="160.98."+IP
IP='192.168.2.101'
'''

IP=rospy.get_param('ip')


import kivy        
from kivymd.app import MDApp
from kivymd.uix.behaviors.toggle_behavior import MDToggleButton
from kivymd.uix.button import MDRectangleFlatButton
from kivy.lang import Builder
from kivy.uix.image import Image
from kivy.uix.widget import Widget
from kivy.graphics.texture import Texture
from kivy.clock import Clock, mainthread
from kivy.core.window import Window
import trio
from timeit import default_timer as timer, main
from struct import *

Window.fullscreen = False




#IP = '10.193.21.28'
#IP = '192.168.2.3'
#IP = '127.0.0.1'
#IP = '160.98.112.232'
PORT = 8080

#IP = '178.197.201.241'
#PORT = 44444

class message():
    def __init__(self):
        self.sens = None
        self.rand = 2
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.gps_longitude = 0.0
        self.gps_latitude = 0.0
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


# Class pour le GUI
class GARM2(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.button_status = 0
        self.deadManSwitchMotion = False
        self.linear_speed_limit = 80
        self.angular_speed_limit = 50
        self.max_linear_speed = 1.5 #m/s
        self.max_angular_speed = 1 #rad/s
        self.yaw_icon = 0
        self.calibration_button_status = 0
        self.orientation_offset = 0
        


    # Builder en fonction du fichier KV
    def build(self):
        self.theme_cls.theme_style = "Dark"
        #return Builder.load_file('/home/other/catkin_ws/src/gui/scripts/gui.kv') #Fichier contenant les layout
        #return Builder.load_file('/home/boris/Documents/Trio/GUI/gui.kv')
        return Builder.load_file('/home/other/Documents/Trio/GUI/gui_nocam.kv') #Fichier contenant les layout


    # Trio 
    async def app_func(self):
        client_stream = await trio.open_tcp_stream(IP, PORT)

        async with client_stream:
            async with trio.open_nursery() as nursery:
                async def run_wrapper():
                    # trio needs to be set so that it'll be used for the event loop
                    await self.async_run(async_lib='trio')
                    print('App done')
                    nursery.cancel_scope.cancel()

                nursery.start_soon(run_wrapper)
                nursery.start_soon(self.sender, client_stream)
                nursery.start_soon(self.receiver, client_stream)
                nursery.start_soon(self.ros)
                nursery.start_soon(self.calibration)




        
    @mainthread
    def joy_callback(self, data): 
        global msg
        ##Dead man switch LT
        if data.buttons[5] == 1 :
                self.root.ids.left_joystick.joy_inside_color = 0, 1, 0, 0.5
                self.root.ids.right_joystick.joy_inside_color = 0, 1, 0, 0.5
                self.deadManSwitchMotion = True # deadmanswtich appuye
        else : 
                self.root.ids.left_joystick.joy_inside_color = 1, 0, 0, 0.5
                self.root.ids.right_joystick.joy_inside_color = 1, 0, 0, 0.5
                self.deadManSwitchMotion = False # deadmanswtich relache

        ##Affichage des joysticks
        self.root.ids.left_joystick.joystick_x = 0
        self.root.ids.left_joystick.joystick_y = data.axes[1] #crash?
        self.root.ids.right_joystick.joystick_x = -data.axes[3] #crash?
        self.root.ids.right_joystick.joystick_y = 0


        ##Reglage du speed linear limiter
        if round(data.axes[7]) == 1 :
                self.linear_speed_limit += 10
                if self.linear_speed_limit > 100 :
                        self.linear_speed_limit = 100

        if round(data.axes[7]) == -1 :
                self.linear_speed_limit -= 10
                if self.linear_speed_limit < 0 :
                        self.linear_speed_limit = 0

        self.root.ids.linear_speed_limiter.prog_value = self.linear_speed_limit
        self.root.ids.linear_speed_limit_label.text = (f'{self.linear_speed_limit:.0f}%')

        ##Reglage du speed angular limiter
        if round(data.axes[6]) == -1 :
                self.angular_speed_limit += 10
                if self.angular_speed_limit > 100 :
                        self.angular_speed_limit = 100

        if round(data.axes[6]) == 1 :
                self.angular_speed_limit -= 10
                if self.angular_speed_limit < 0 :
                        self.angular_speed_limit = 0



        self.root.ids.angular_speed_limiter.prog_value = self.angular_speed_limit
        self.root.ids.angular_speed_limit_label.text = (f'{self.angular_speed_limit:.0f}%')

        ## Envoi des consignes de vitesse lin et rot, seulement si dead man switch presse
        if self.deadManSwitchMotion == True :
        #if True :
                msg.linear_speed  = data.axes[1]*self.linear_speed_limit*self.max_linear_speed/100
                msg.angular_speed = data.axes[3]*self.angular_speed_limit*self.max_angular_speed/100
                
                
        else :
                msg.linear_speed = 0
                msg.angular_speed = 0


    async def ros(self):
        rospy.init_node('gui', anonymous=True)
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        


    async def calibration(self):
        global msg
        while True:
            await trio.sleep(1/100)
            if self.calibration_button_status == True :
                self.calibration_button_status = False

                if  self.deadManSwitchMotion == True:
                    lat_start = msg.gps_latitude #Get gps at the start of the movement
                    lon_start = msg.gps_longitude
                    msg.linear_speed = 1 #Start moving

                    for t in range(1, 11): #Wait for 4 sec
                        self.root.ids.calibrate_button.text = "Moving...("+str(t*10)+'%)'
                        await trio.sleep(0.4)


                    lat_end = msg.gps_latitude #Get gps at the end of the movement
                    lon_end = msg.gps_longitude
                    msg.linear_speed = 0 #Stop moving after x sec.
                    
                    #Vrai si GPS est off (même data au début qu'à la fin)
                    if lat_end-lat_start + lon_end-lon_start == 0: 
                        self.root.ids.calibrate_button.text = "Error, GPS is probably not working."

                    else :
                        v = lat_end-lat_start, lon_end-lon_start
                        v = np.array(v)
                        v = v/np.linalg.norm(v) #Normalized
                        angle = round(degrees(atan2(v[0], v[1])))
                        self.orientation_offset = angle-msg.yaw
                        self.root.ids.calibrate_button.text = "Angle is: "+str(angle)+"°"    

    

                    await trio.sleep(3)
                    self.root.ids.calibrate_button.text = "Redo Compass Calibration"

                else :
                    self.root.ids.calibrate_button.text = "Dead man's switch not pressed!"
                    await trio.sleep(2)
                    self.root.ids.calibrate_button.text = "Compass Calibration"

        

    # Envoi des infos au robot
    async def sender(self, client_stream):
        server_tickrate = 100
        while True:
                #self.temp +=5
                #self.root.ids.front_axe.alpha = round(self.temp)
                #self.root.ids.roll_deg.text = (f'{round(self.temp):.0f}°')
                #self.button_status = 0

            data_tosend_packed  = pack('dd?', msg.linear_speed, msg.angular_speed, msg.manual_mode_switch)
            await client_stream.send_all(data_tosend_packed )


            await trio.sleep(1/server_tickrate)



    # Réception des infos du robot
    async def receiver(self, client_stream):
        global msg
        async for data in client_stream:
            #data_received_unpacked = unpack('ddd', data)

            if len(data) == 88 : #Si lag, deux pack peuvent être envoyé en un et donc plus possible de unpack.
                data_received_unpacked = unpack('dddddddiddd', data)

                msg.yaw = degrees(data_received_unpacked[0])
                msg.pitch = degrees(data_received_unpacked[1])
                msg.roll = degrees(data_received_unpacked[2])

                msg.gps_longitude = data_received_unpacked[3]
                msg.gps_latitude = data_received_unpacked[4]

                msg.linear_current_speed = data_received_unpacked[5] 
                msg.angular_current_speed = data_received_unpacked[6]

                msg.move_base_status = data_received_unpacked[7]

                msg.move_base_goal_x = data_received_unpacked[8]
                msg.move_base_goal_y = data_received_unpacked[9]
                msg.move_base_goal_theta = data_received_unpacked[10]


            # Move_base status
            if msg.move_base_status == 0:
                self.root.ids.move_base_status.text = "The goal has yet to be processed by the action server."

            elif msg.move_base_status == 1:
                self.root.ids.move_base_status.text = "The goal is currently being processed by the action server."

            elif msg.move_base_status == 2:
                self.root.ids.move_base_status.text = "The goal received a cancel request after it started executing and has since completed its execution."

            elif msg.move_base_status == 3:
                self.root.ids.move_base_status.text = "The goal was achieved successfully by the action server."

            elif msg.move_base_status == 4:
                self.root.ids.move_base_status.text = "The goal was aborted during execution by the action server due to some failure."

            elif msg.move_base_status == 5:
                self.root.ids.move_base_status.text = "The goal was rejected by the action server without being processed, because the goal was unattainable or invalid."

            elif msg.move_base_status == 6:
                self.root.ids.move_base_status.text = "The goal received a cancel request after it started executing and has not yet completed execution."
            
            elif msg.move_base_status == 7:
                self.root.ids.move_base_status.text = "The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled."

            elif msg.move_base_status == 8:
                self.root.ids.move_base_status.text = "The goal received a cancel request before it started executing and was successfully cancelled."

            elif msg.move_base_status == 9:
                self.root.ids.move_base_status.text = "An action client can determine that a goal is LOST. This should not be sent over the wire by an action server."

            else: 
                self.root.ids.move_base_status.text = "Error. Unvalid move_base status."


            #Move base goal
            self.root.ids.move_base_goal_x.text = (f'x = {msg.move_base_goal_x:.2f}m')
            self.root.ids.move_base_goal_y.text = (f'y = {msg.move_base_goal_y:.2f}m')
            self.root.ids.move_base_goal_theta.text = (f'theta = {degrees(msg.move_base_goal_theta):.1f}°')



            # Roll animation
            msg.roll = -4
            self.root.ids.front_axe.alpha = round(msg.roll)
            self.root.ids.roll_deg.text = (f'{round(msg.roll):.0f}°')

            # Pitch animation
            msg.pitch = 7
            self.root.ids.side_axe.alpha = round(msg.pitch)
            self.root.ids.pitch_deg.text = (f'{round(msg.pitch):.0f}°')

            # Yaw icons
            self.yaw_icon = round((msg.yaw+self.orientation_offset) % 360)
            self.root.ids.MyMapView.name = 'images/icons/icon'+str(self.yaw_icon)+'.png'

            # Yaw animation   
            yaw_with_offset = ((msg.yaw+90+self.orientation_offset)%360)-180

            self.root.ids.top_axe.alpha = round(yaw_with_offset)
            self.root.ids.yaw_deg.text = (f'{round(yaw_with_offset+self.orientation_offset):.0f}°')


            #Affichage du point cardinal sur le GUI
            if yaw_with_offset < 22.5 and yaw_with_offset > -22.5 :     
                    self.root.ids.yaw_points_card.text = "North" 

            elif yaw_with_offset < 67.5 and yaw_with_offset > 22.5 :     
                    self.root.ids.yaw_points_card.text = "North West" 

            elif yaw_with_offset < 112.5 and yaw_with_offset > 67.5 :     
                    self.root.ids.yaw_points_card.text = "West" 

            elif yaw_with_offset < 157.5 and yaw_with_offset > 112.5 :     
                    self.root.ids.yaw_points_card.text = "South West" 

            elif yaw_with_offset > -67.5 and yaw_with_offset< -22.5 :     
                    self.root.ids.yaw_points_card.text = "North East" 

            elif yaw_with_offset > -112.5 and yaw_with_offset < -67.5 :     
                    self.root.ids.yaw_points_card.text = "East" 

            elif yaw_with_offset > -157.5 and yaw_with_offset < -112.5 :     
                    self.root.ids.yaw_points_card.text = "South East" 

            else : 
                    self.root.ids.yaw_points_card.text = "South" 

            # Linear current speed
            self.root.ids.linear_current_speed.prog_value = abs(msg.linear_current_speed*100/self.max_linear_speed)
            self.root.ids.linear_current_speed_label.text = (f'{msg.linear_current_speed:.1f} m/s')

            # Angular current speed
            self.root.ids.angular_current_speed.prog_value = abs(msg.angular_current_speed*100/self.max_angular_speed)
            self.root.ids.angular_current_speed_label.text = (f'{msg.angular_current_speed:.1f} rad/s')

            # GPS
            self.root.ids.MyMapView.center_on(msg.gps_latitude, msg.gps_longitude)
            self.root.ids.MyMapView.current_lat = msg.gps_latitude
            self.root.ids.MyMapView.current_lon = msg.gps_longitude
            self.root.ids.latitude_label.text = (f'Latitude: {msg.gps_latitude:.7f}')
            self.root.ids.longitude_label.text = (f'Longitude: {msg.gps_longitude:.7f}')






    def calibrate_press(self, *args):
        self.calibration_button_status = True
        
    def mode_switch(self, *args):
        if msg.manual_mode_switch :
            msg.manual_mode_switch = False
            print("On")

        else :
            msg.manual_mode_switch = True
            print("Off")

        



if __name__ == '__main__':
    msg = message()
    trio.run(GARM2().app_func)





