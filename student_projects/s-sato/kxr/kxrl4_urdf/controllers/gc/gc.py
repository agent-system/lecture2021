"""Sample Webots controller for the visual tracking benchmark."""

from controller import Robot, Node, Display
import os
import sys
import math

## for smach
import smach_def
from motionlib import MotionLib

# Get pointer to the robot.
robot = Robot()

mlib = MotionLib(robot)

sm = smach_def.create_state_machine(count=1000, lib=mlib)

sm.execute()
