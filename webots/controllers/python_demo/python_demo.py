"""Sample Webots controller for the visual tracking benchmark."""

from controller import Robot, Node, Display
import base64
import os
import sys
import tempfile

## for image processing
import numpy as np
import cv2

# Get pointer to the robot.
robot = Robot()

# Set the controller time step based on the current world's time step.
timestep = int(robot.getBasicTimeStep() * 4)

camera  = None
display = None
num = robot.getNumberOfDevices()

for i in range(num):
    dev = robot.getDeviceByIndex(i)
    if dev.getNodeType() == Node.CAMERA:
        camera = dev
        width  = camera.getWidth()
        height = camera.getHeight()
        print 'camera found {} / {}x{}'.format(dev.getName(), width, height)

    if dev.getNodeType() == Node.DISPLAY:
        print 'display found {}'.format(dev.getName())
        display = dev

if camera:
    camera.enable(timestep)

# Show camera image in the display background.
if display:
    display.attachCamera(camera)
    display.setColor(0xFF0000)

while robot.step(timestep) != -1:

    # Get camera image.
    img = np.array(camera.getImageArray(), np.uint8)

    #gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hsv_img = cv2.cvtColor(img,  cv2.COLOR_RGB2HSV)

    # define range of color in HSV
    lower_col = np.array([ 0,  40, 100])
    upper_col = np.array([15, 255, 255])

    # Threshold the HSV image to get only colors
    mask = cv2.inRange(hsv_img, lower_col, upper_col)

    #alpha = np.full(mask.shape[:2], 127, np.uint8)
    zero = np.full(mask.shape[:2], 0, np.uint8)

    # Show detected blob in the display: draw the circle and centroid.
    if display:
        #display.setColor(0xFF0000)
        #display.fillOval(int(160), int(160), 5, 5)
        ir = display.imageNew(cv2.merge((mask, zero, zero, mask)).tolist(), Display.RGBA, width, height)
        #display.setAlpha(0.6)
        display.imagePaste(ir, 0, 0, False)
        display.imageDelete(ir)

    # Send the display image to the robot window.
    pass
