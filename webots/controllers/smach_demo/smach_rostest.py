#!/usr/bin/env python
import rospy
import smach_ros

import smach_def

if __name__ == '__main__':
    rospy.init_node('action_average')

    sm = smach_def.create_state_machine(count=3, lib=None)

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    rospy.sleep(1)

    result = sm.execute()

    rospy.loginfo('result: %s'%(result))
    rospy.spin()
    sis.stop()
