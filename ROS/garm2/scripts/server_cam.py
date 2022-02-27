#!/usr/bin/env python3
import trio
import cv2
import numpy as np

# Video capture
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

PORT = 8082
image_bytes = bytearray(0)

# Rescale because there is no compression.
def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)


async def capture():
    global image_bytes
    frame_id = 1
    while True :
        ret, frame = cap.read()
        frame = rescale_frame(frame, 50)

        image_bytes = cv2.imencode('.jpg', frame)[1].tobytes()
        image_bytes = bytearray(image_bytes)
        image_bytes = bytearray(len(image_bytes).to_bytes(length=3, byteorder="big")+image_bytes)
        image_bytes = bytearray(frame_id.to_bytes(length=5, byteorder="big")+image_bytes)


        await trio.sleep(1/200)


async def echo(server_stream):
    async for data in server_stream:
        await server_stream.send_all(image_bytes)
    


async def server(server_stream):
    print("Connection started.")

    async with trio.open_nursery() as nursery:
        nursery.start_soon(echo, server_stream)
        nursery.start_soon(capture)
                
    print("Connection closed.")


async def main():
    await trio.serve_tcp(server, PORT)
    

trio.run(main)

