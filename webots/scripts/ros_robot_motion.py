#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from webots_ros.msg import BoolStamped, Float64Stamped, Int32Stamped, Int8Stamped, RadarTarget, RecognitionObject, StringStamped
from webots_ros.srv import set_bool, set_float, set_float_array, set_int, set_string, get_bool, get_float, get_float_array, get_int, get_string, get_uint64
import webots_ros.srv

from sensor_msgs.msg import Image

import subprocess

from webots_roslib import create_model_map, WbCamera, WbRangeFinder, WbMotor

rfw = None
lfw = None
rbw = None
lbw = None

def stop_moving():
    lfw.setVelocity(0.0)
    lbw.setVelocity(0.0)
    rfw.setVelocity(0.0)
    rbw.setVelocity(0.0)

def move_forward():
    lfw.setVelocity(4.0)
    lbw.setVelocity(4.0)
    rfw.setVelocity(-4.0)
    rbw.setVelocity(-4.0)

def turn_right():
    lfw.setVelocity(2.0)
    lbw.setVelocity(2.0)
    rfw.setVelocity(2.0)
    rbw.setVelocity(2.0)

def turn_right():
    lfw.setVelocity(-2.0)
    lbw.setVelocity(-2.0)
    rfw.setVelocity(-2.0)
    rbw.setVelocity(-2.0)

def motion_callback(msg):
    motion = msg.data
    ## print('callback')
    if motion == 'stop':
        # stop_moving()
        lfw.setVelocity(0.0)
        lbw.setVelocity(0.0)
        rfw.setVelocity(0.0)
        rbw.setVelocity(0.0)
    elif motion == 'move_forward':
        # move_forward()
        lfw.setVelocity(4.0)
        lbw.setVelocity(4.0)
        rfw.setVelocity(-4.0)
        rbw.setVelocity(-4.0)
    elif motion == 'turn_right':
        # turn_right()
        lfw.setVelocity(2.0)
        lbw.setVelocity(2.0)
        rfw.setVelocity(2.0)
        rbw.setVelocity(2.0)
    elif motion == 'turn_left':
        # turn_left()
        lfw.setVelocity(-2.0)
        lbw.setVelocity(-2.0)
        rfw.setVelocity(-2.0)
        rbw.setVelocity(-2.0)

if __name__ == '__main__':
    rospy.init_node('webots_ros_motion_client', anonymous=True)

    model_map = create_model_map()

    for k in model_map.values():
        print('model name: {}'.format(k.name))
        for dv in k.device_map.values():
            if type(dv) == WbCamera:
                dv.enable()
            if type(dv) == WbRangeFinder:
                dv.enable()

    name = 'kxrl4_urdf'

    if name in model_map:
        rc = model_map[name]
    else:
        exit(0)

    if 'rfw' in rc.device_map:
        rfw = rc.device_map['rfw']
        rfw.setPosition(float('inf'))
        rfw.setVelocity(0.0)
    if 'lfw' in rc.device_map:
        lfw = rc.device_map['lfw']
        lfw.setPosition(float('inf'))
        lfw.setVelocity(0.0)
    if 'rbw' in rc.device_map:
        rbw = rc.device_map['rbw']
        rbw.setPosition(float('inf'))
        rbw.setVelocity(0.0)
    if 'lbw' in rc.device_map:
        lbw = rc.device_map['lbw']
        lbw.setPosition(float('inf'))
        lbw.setVelocity(0.0)

    if 'neck_p' in rc.device_map:
        neck_p = rc.device_map['neck_p']
        neck_p.setPosition(-0.5)

    sub = rospy.Subscriber("motion_command", String, motion_callback)

    r = rospy.Rate(1000) # 1000hz
    while not rospy.is_shutdown():
        rc.time_step(4)
        r.sleep()
