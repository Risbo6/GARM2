#!/usr/bin/env python3.8
from kivy.app import App
from kivy_garden.mapview import MapMarker








import kivy        
from kivymd.app import MDApp
from kivymd.uix.behaviors.toggle_behavior import MDToggleButton
from kivymd.uix.button import MDRectangleFlatButton
from kivy.lang import Builder
from kivy.uix.image import Image
from kivy.uix.widget import Widget
from kivy.clock import Clock, mainthread
from kivy.core.window import Window
from math import degrees
import trio
from timeit import default_timer as timer, main
from struct import *
import rospy

#print("IP: 160.98.",end ='')
#IP=input()
#IP="160.98."+IP
IP='192.168.2.101'
IP='127.0.0.1'
IP='192.168.1.106'
#IP='160.98.113.14'
IP=rospy.get_param('ip')

PORT = 8081

#IP = '178.197.201.241'
#PORT = 44444

ONE_WAY = 1
PATROL = 2
CIRCLE = 3

class message():
    def __init__(self):
        self.lat_goal = 0
        self.lon_goal = 0.0

        self.current_lat = 46.79682
        self.current_lon = 7.14723
        self.yaw = 200

        self.waypoint_lat = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.waypoint_lon = [0, 0, 0, 0, 0, 0, 0, 0, 0]


        self.waypoint_number = 0
        self.current_waypoint = 0
        self.autopilot_mode = 0
        self.cancel_goal = 0



        

        

        


# Class pour le GUI
class MapApp(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs) 
        self.marker_old = None
        self.marker_robot_old = None 
        self.yaw_icon = 90
        self.marker_list = []
        self.waypoint_list = []
        self.waypoint_list_green = []
        self.current_waypoint_old = 0


    def build(self):
        self.theme_cls.theme_style = "Dark"
        #self.theme_cls.primary_palette = "Red" 
        return self.root

    def touch_down(self, args):
        if args.button == 'left':       
            if msg.waypoint_number < 9:
                msg.waypoint_number+=1

                mouse_coord = self.root.ids.mapview.get_latlon_at(args.pos[0], args.pos[1]-self.root.ids.button_box.height)
                marker = MapMarker(lon=mouse_coord.lon, lat=mouse_coord.lat, source='flag/'+str(msg.waypoint_number)+'.png')
                self.waypoint_list.append(marker)

                marker_green = MapMarker(lon=mouse_coord.lon, lat=mouse_coord.lat, source='flag/'+str(msg.waypoint_number)+'_green.png')
                self.waypoint_list_green.append(marker_green)

                self.root.ids.mapview.add_marker(marker)

                #self.root.ids.mapview.remove_marker(marker)
                #print(self.marker_list)








    def one_way_press(self, *args):
        if len(self.waypoint_list) > 0:
            for i in range (len(self.waypoint_list)):
                msg.waypoint_lat[i] = self.waypoint_list[i].lat
                msg.waypoint_lon[i] = self.waypoint_list[i].lon


            self.root.ids.autopilot_label.text = "Autopilot Mode: One way"
            self.root.ids.status_label.text = "Status: Running"


            msg.autopilot_mode = ONE_WAY
            msg.cancel_goal = 0



    def patrol_press(self, *args):

        if len(self.waypoint_list) > 1:
            for i in range (len(self.waypoint_list)):
                msg.waypoint_lat[i] = self.waypoint_list[i].lat
                msg.waypoint_lon[i] = self.waypoint_list[i].lon


            self.root.ids.autopilot_label.text = "Autopilot Mode: Patrol"
            self.root.ids.status_label.text = "Status: Running"

            msg.autopilot_mode = PATROL
            msg.cancel_goal = 0

    def circle_press(self, *args):

        if len(self.waypoint_list) > 1:
            for i in range (len(self.waypoint_list)):
                msg.waypoint_lat[i] = self.waypoint_list[i].lat
                msg.waypoint_lon[i] = self.waypoint_list[i].lon


            self.root.ids.autopilot_label.text = "Autopilot Mode: Circle"
            self.root.ids.status_label.text = "Status: Running"

            msg.autopilot_mode = CIRCLE
            msg.cancel_goal = 0

        msg.current_waypoint += 1

    def cancel_press(self, *args):
        for i in range (9):
            msg.waypoint_lat[i] = 0
            msg.waypoint_lon[i] = 0

        for i in range (msg.waypoint_number):
            self.root.ids.mapview.remove_marker(self.waypoint_list[i])

        for i in range(msg.waypoint_number):
            self.root.ids.mapview.remove_marker(self.waypoint_list_green[i])

        msg.current_waypoint = 0
        self.current_waypoint_old = 0
        self.waypoint_list_green = []
        
        msg.cancel_goal = 1
        msg.autopilot_mode = 0
        msg.waypoint_number = 0
        self.waypoint_list = []


        self.root.ids.autopilot_label.text = "Autopilot Mode: None"
        self.root.ids.status_label.text = "Status: Canceled"

        

        

        #self.current_waypoint_old = 0

        
        


        



    
        


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


    

    # Envoi des infos au robot
    async def sender(self, client_stream):
        server_tickrate = 10
        while True:
            data_tosend_packed  = pack('9d9dddd', msg.waypoint_lat[0], msg.waypoint_lat[1], msg.waypoint_lat[2], msg.waypoint_lat[3], msg.waypoint_lat[4], msg.waypoint_lat[5]
            ,msg.waypoint_lat[6], msg.waypoint_lat[7], msg.waypoint_lat[8]
            ,msg.waypoint_lon[0], msg.waypoint_lon[1] ,msg.waypoint_lon[2], msg.waypoint_lon[3], msg.waypoint_lon[4], msg.waypoint_lon[5], msg.waypoint_lon[6]
            ,msg.waypoint_lon[7], msg.waypoint_lon[8]
            ,msg.autopilot_mode, msg.cancel_goal, msg.waypoint_number)
            await client_stream.send_all(data_tosend_packed )

            

            await trio.sleep(1/server_tickrate)



    # Réception des infos du robot
    async def receiver(self, client_stream):
        global msg
        async for data in client_stream:

            if len(data) == 8*4 : #Si lag, deux pack peuvent être envoyé en un et donc plus possible de unpack.
                data_received_unpacked = unpack('dddd', data)
                msg.current_lat = data_received_unpacked[0]
                msg.current_lon = data_received_unpacked[1]
                msg.yaw = degrees(data_received_unpacked[2])
                msg.current_waypoint = data_received_unpacked[3]+1 # Waypoint[0] = waypoint numéro 1






            #Affichage du current waypoint
            if msg.current_waypoint != self.current_waypoint_old:
                if msg.current_waypoint > 1:
                    #self.root.ids.mapview.remove_marker(self.waypoint_list_green[int(msg.current_waypoint-2)])
                    pass

                for i in range(len(self.waypoint_list_green)):
                    self.root.ids.mapview.remove_marker(self.waypoint_list_green[i])


                if len(self.waypoint_list_green) > 0 :
                    if msg.autopilot_mode == ONE_WAY or msg.autopilot_mode == PATROL or msg.autopilot_mode == CIRCLE:
                        self.root.ids.mapview.add_marker(self.waypoint_list_green[int(msg.current_waypoint-1)])
                        self.current_waypoint_old = msg.current_waypoint

                        self.root.ids.path_label.text = "Path: Heading towards waypoint "+str(int(msg.current_waypoint))


                

            
            #Marker qui représente la position actuelle du robot
            current_lon = msg.current_lon
            current_lat = msg.current_lat
            
            self.yaw_icon = round((msg.yaw+0) % 360)

            print(current_lon, current_lat, self.yaw_icon)
            marker_robot = MapMarker(lon=current_lon, lat=current_lat, source='icons/icon'+str(self.yaw_icon)+'.png')
            self.root.ids.mapview.add_marker(marker_robot)

            if self.marker_robot_old is not None :
                self.root.ids.mapview.remove_marker(self.marker_robot_old)
            self.marker_robot_old = marker_robot

            #Markers qui représentent les précédentes positions du robot
            marker_trail = MapMarker(lon=current_lon, lat=current_lat, source='blue_transparent.png')
            self.root.ids.mapview.add_marker(marker_trail)
            self.marker_list.append(marker_trail)

            if len(self.marker_list) >= 500 :
                self.root.ids.mapview.remove_marker(self.marker_list[0])
                del self.marker_list[0]



            


            










        



if __name__ == '__main__':
    msg = message()
    trio.run(MapApp().app_func)