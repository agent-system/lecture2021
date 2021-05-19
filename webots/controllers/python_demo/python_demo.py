"""Sample Webots controller for the visual tracking benchmark."""

from controller import Robot, Node, Display
import os
import sys
import math

## for image processing
import numpy as np
import cv2

node_type_map = {
Node.NO_NODE: "NO_NODE",
Node.APPEARANCE: "APPEARANCE",
Node.BACKGROUND: "BACKGROUND",
Node.BOX: "BOX",
Node.CAPSULE: "CAPSULE",
Node.COLOR: "COLOR",
Node.CONE: "CONE",
Node.COORDINATE: "COORDINATE",
Node.CYLINDER: "CYLINDER",
Node.DIRECTIONAL_LIGHT: "DIRECTIONAL_LIGHT",
Node.ELEVATION_GRID: "ELEVATION_GRID",
Node.FOG: "FOG",
Node.GROUP: "GROUP",
Node.IMAGE_TEXTURE: "IMAGE_TEXTURE",
Node.INDEXED_FACE_SET: "INDEXED_FACE_SET",
Node.INDEXED_LINE_SET: "INDEXED_LINE_SET",
Node.MATERIAL: "MATERIAL",
Node.MESH: "MESH",
Node.MUSCLE: "MUSCLE",
Node.NORMAL: "NORMAL",
Node.PBR_APPEARANCE: "PBR_APPEARANCE",
Node.PLANE: "PLANE",
Node.POINT_LIGHT: "POINT_LIGHT",
Node.POINT_SET: "POINT_SET",
Node.SHAPE: "SHAPE",
Node.SPHERE: "SPHERE",
Node.SPOT_LIGHT: "SPOT_LIGHT",
Node.TEXTURE_COORDINATE: "TEXTURE_COORDINATE",
Node.TEXTURE_TRANSFORM: "TEXTURE_TRANSFORM",
Node.TRANSFORM: "TRANSFORM",
Node.VIEWPOINT: "VIEWPOINT",
Node.ROBOT: "ROBOT",
Node.DIFFERENTIAL_WHEELS: "DIFFERENTIAL_WHEELS",
Node.ACCELEROMETER: "ACCELEROMETER",
Node.BRAKE: "BRAKE",
Node.CAMERA: "CAMERA",
Node.COMPASS: "COMPASS",
Node.CONNECTOR: "CONNECTOR",
Node.DISPLAY: "DISPLAY",
Node.DISTANCE_SENSOR: "DISTANCE_SENSOR",
Node.EMITTER: "EMITTER",
Node.GPS: "GPS",
Node.GYRO: "GYRO",
Node.INERTIAL_UNIT: "INERTIAL_UNIT",
Node.LED: "LED",
Node.LIDAR: "LIDAR",
Node.LIGHT_SENSOR: "LIGHT_SENSOR",
Node.LINEAR_MOTOR: "LINEAR_MOTOR",
Node.PEN: "PEN",
Node.POSITION_SENSOR: "POSITION_SENSOR",
Node.PROPELLER: "PROPELLER",
Node.RADAR: "RADAR",
Node.RANGE_FINDER: "RANGE_FINDER",
Node.RECEIVER: "RECEIVER",
Node.ROTATIONAL_MOTOR: "ROTATIONAL_MOTOR",
Node.SPEAKER: "SPEAKER",
Node.TOUCH_SENSOR: "TOUCH_SENSOR",
Node.BALL_JOINT: "BALL_JOINT",
Node.BALL_JOINT_PARAMETERS: "BALL_JOINT_PARAMETERS",
Node.CHARGER: "CHARGER",
Node.CONTACT_PROPERTIES: "CONTACT_PROPERTIES",
Node.DAMPING: "DAMPING",
Node.FLUID: "FLUID",
Node.FOCUS: "FOCUS",
Node.HINGE_JOINT: "HINGE_JOINT",
Node.HINGE_JOINT_PARAMETERS: "HINGE_JOINT_PARAMETERS",
Node.HINGE_2_JOINT: "HINGE_2_JOINT",
Node.IMMERSION_PROPERTIES: "IMMERSION_PROPERTIES",
Node.JOINT_PARAMETERS: "JOINT_PARAMETERS",
Node.LENS: "LENS",
Node.LENS_FLARE: "LENS_FLARE",
Node.PHYSICS: "PHYSICS",
Node.RECOGNITION: "RECOGNITION",
Node.SLIDER_JOINT: "SLIDER_JOINT",
Node.SLOT: "SLOT",
Node.SOLID: "SOLID",
Node.SOLID_REFERENCE: "SOLID_REFERENCE",
Node.TRACK: "TRACK",
Node.TRACK_WHEEL: "TRACK_WHEEL",
Node.WORLD_INFO: "WORLD_INFO",
Node.ZOOM: "ZOOM",
Node.MICROPHONE: "MICROPHONE",
Node.RADIO: "RADIO",
Node.SKIN: "SKIN",
}

# Get pointer to the robot.
robot = Robot()

# Set the controller time step based on the current world's time step.
timestep = int(robot.getBasicTimeStep() * 4)

camera  = None
display = None
receiver = None

num = robot.getNumberOfDevices()

for i in range(num):
    dev = robot.getDeviceByIndex(i)
    tp = dev.getNodeType()
    if tp in node_type_map:
        tp = node_type_map[tp]
    print "Device #{} / {}({})".format(i, dev.getName(), tp)
    if dev.getNodeType() == Node.CAMERA:
        camera = dev
        width  = camera.getWidth()
        height = camera.getHeight()
        print 'camera found {} / {}x{}'.format(dev.getName(), width, height)

    if dev.getNodeType() == Node.DISPLAY:
        print 'display found {}'.format(dev.getName())
        display = dev

    if dev.getNodeType() == Node.RECEIVER:
        print 'receiver found {}'.format(dev.getName())
        receiver = dev

if camera:
    camera.enable(timestep)

# Show camera image in the display background.
if display:
    display.attachCamera(camera)
    display.setColor(0xFF0000)

if receiver:
    receiver.enable(100)

hp = robot.getDevice('head-neck-p')
lf = robot.getDevice('larm-shoulder-p')
rf = robot.getDevice('rarm-shoulder-p')
lb = robot.getDevice('lleg-crotch-p')
rb = robot.getDevice('rleg-crotch-p')

cycle_steps = 10

if hp:
    hp.setPosition(2.0 - math.pi * 0.3)

counter = 0
while robot.step(timestep) != -1:

    while receiver.getQueueLength() > 0:
        data = receiver.getData()
        print 'recv: {}'.format(data)
        receiver.nextPacket()

    if lf and rf and lb and rb:
        pos_a = math.sin(math.pi * (counter % cycle_steps ) / float(cycle_steps))
        pos_b = math.sin(math.pi * ((counter + cycle_steps/2) % cycle_steps ) / float(cycle_steps))
        lf.setPosition(0.3*pos_a + 0.9)
        rf.setPosition(0.3*pos_b + 0.9)
        lb.setPosition(1.1*pos_a + 0.7)
        rb.setPosition(1.1*pos_b + 0.7)

    # Get camera image.
    if camera:
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
    if display and camera:
        #display.setColor(0xFF0000)
        #display.fillOval(int(160), int(160), 5, 5)
        ir = display.imageNew(cv2.merge((mask, zero, zero, mask)).tolist(), Display.RGBA, width, height)
        #display.setAlpha(0.6)
        display.imagePaste(ir, 0, 0, False)
        display.imageDelete(ir)

    # Send the display image to the robot window.
    counter = counter + 1
