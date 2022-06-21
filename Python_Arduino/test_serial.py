# Importing Libraries
import cv2  # Import the OpenCV library
import numpy as np # Import Numpy library
import time
import os
# import argparse

import serial  # pip install pyserial
import time

# Start the video stream
arduino = serial.Serial(port='COM11', baudrate=9600, timeout=.1)
moves = ["moveForward", "moveSidewaysLeft",  "moveSidewaysRight", "moveSidewaysLeft", "moveRightForward",
          "moveLeftForward", "moveRightBackward", "moveLeftBackward", "rotateRight", "rotateLeft", "rotateLeft",
         "stopMoving"]





def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data
while True:
    num = input("Enter a number: ") # Taking input from user

    num = "-051 022 033 044"  #
    value = write_read(num)
    print(value)  # printing the value



