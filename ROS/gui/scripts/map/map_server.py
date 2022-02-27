import trio
from itertools import count
from random import randint
from struct import *
import math

PORT = 8082
client_number = 0

#Cette classe contient toutes les variables qui vont être envoyées/reçues avec ROS
class message():
    def __init__(self): 
        self.lat_goal = 0.0
        self.lon_goal = 0.0
        self.lat_goal_old = 0.0
        self.lon_goal_old = 0.0

        self.current_lat = 0
        self.current_lon = 0

        self.yaw = 0

        self.waypoint_lat = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.waypoint_lon = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.waypoint_number = 0
        self.current_waypoint = 0
        self.autopilot_mode = 0
        self.cancel_goal = 0






# Echo server. Renvoie un message à chaque fois qu'il en reçoit un.
# Le client fixe donc le tickrate du serveur. 
async def echo(server_stream):
    global msg
    async for data in server_stream:
        # Réception
        if len(data) == 8*20 : #Si mauvaise connection, le serveur envoie deux pack d'un coup et tout crash
            data_received_unpacked = unpack('9d9dddd', data)

            for i in range(9):
                msg.waypoint_lat[i] = data_received_unpacked[i] 
                msg.waypoint_lon[i] = data_received_unpacked[i+9] 


            msg.autopilot_mode = data_received_unpacked[18] 
            msg.cancel_goal = data_received_unpacked[19]
            msg.waypoint_number = data_received_unpacked[20]


            print(msg.autopilot_mode, msg.cancel_goal)





        data_tosend_packed = pack('ddd', msg.current_lat, msg.current_lon, msg.yaw)

        await server_stream.send_all(data_tosend_packed)

    




# Fonction serveur. La nursery s'ouvre que lorqu'au moins un client s'est connecté.
# ROS ne fonctionne donc pas tant que personne n'est connecté.
async def server(server_stream):
    global msg
    global client_number
    client_number+=1 #Number of client connected
    print("Connection started.")

    async with trio.open_nursery() as nursery:
        nursery.start_soon(echo, server_stream)
        
    print("Connection closed.")





async def main():
    await trio.serve_tcp(server, PORT)
    
    
if __name__ == '__main__':
    msg = message()
    trio.run(main)

