#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from opencv_apps.msg import MomentArrayStamped

import smach_def_with_two_propellers
import smach
import smach_ros

pub = None

def get_m00(c):
    return c.m00

class RosMain(object):
    def __init__(self):
        self.pub = rospy.Publisher('motion_command', String, queue_size=10)
        self.sub_goal  = rospy.Subscriber('goal_moments/moments', MomentArrayStamped, self.goal_callback)
        self.sub_trash = rospy.Subscriber('trash_moments/moments', MomentArrayStamped, self.trash_callback)
        self.goal_detect_result = None
        self.trash_detect_result = None
        self.rate = rospy.Rate(1000) # 1000hz

        self.motion_count = 100

    def goal_callback(self, msg):
        self.goal_detect_result = msg.moments
    def trash_callback(self, msg):
        self.trash_detect_result = msg.moments

    def position_of_main_area(self, contours):
        contours.sort(key = get_m00, reverse=True)
        for c in contours:
            print c.m00
        M = contours[0]
        result = 'turn_left' ## initialize result
        if M:
            m00 = M.m00
            m10 = M.m10
            m01 = M.m01
            if m00 > 6000:
                result = 'other_state' ## ???
            elif m00 > 0:
                ypos = m01/m00
                xpos = m10/m00
                print m00, xpos, ypos
                diff = 10
                if xpos > (160 + diff):
                    result = 'turn_right'
                elif xpos < (160 - diff):
                    result = 'turn_left'
                else:
                    result = 'swim_forward'
        return result

    def detect_trash(self):
        result = 'turn_left'
        # result = 'swim_forward'
        if self.trash_detect_result:
            contours = self.trash_detect_result
            self.trash_detect_result = None
            result = self.position_of_main_area(contours)
        return result

    def detect_goal(self):
        global collected_counter
        result = 'turn_left'
        if self.goal_detect_result:
            contours = self.goal_detect_result
            self.goal_detect_result = None
            result = self.position_of_main_area(contours)
            if result == 'other_state':
                collected_counter +=1
                print"collected_counter +1"
        return result

    def move_forward(self):
        self.pub.publish('move_forward')
        for i in range(self.motion_count):
            self.rate.sleep()

    def move_back(self):
        self.pub.publish('move_back')
        for i in range(self.motion_count):
            self.rate.sleep()      

    def turn_left(self):
        self.pub.publish('turn_left')
        for i in range(self.motion_count):
            self.rate.sleep()

    def turn_right(self):
        self.pub.publish('turn_right')
        for i in range(self.motion_count):
            self.rate.sleep()

    def swim_forward(self):
        self.pub.publish('swim_forward')
        for i in range(self.motion_count*2):
            self.rate.sleep()
            
if __name__ == '__main__':
    rospy.init_node('webots_ros_main', anonymous=True)

    rm = RosMain()

    #sm = smach_def.create_state_machine(count=1000, lib=rm)
    sm = smach_def_with_two_propellers.create_layerd_state_machine(count=1000, lib=rm)
    #motion test
    # sm = smach_def_with_two_propellers.test_state_machine(count=1000, lib=rm)
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    rospy.sleep(1)

    result = sm.execute()

    rospy.spin()
