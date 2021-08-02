#!/usr/bin/env python
import rospy
import smach

class detect_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move_forward', 'turn_left', 'turn_right', 'finish'],
                             input_keys=['counter', 'lib'], output_keys = ['lib'])
        self.counter = 0
    def execute(self, userdata):
        if self.counter > userdata.counter:
            userdata.counter_out = self.counter
            return 'finish'
        self.counter = self.counter + 1

        result = 'move_forward'
        if userdata.lib:
            lib = userdata.lib
            result = lib.detect()
        else:
            rospy.sleep(1)

        return result

class move_forward_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_next'],
                             input_keys=['lib'], output_keys = ['lib'])

    def execute(self, userdata):
        if userdata.lib:
            result = userdata.lib.move_forward()
        else:
            rospy.sleep(1)

        return 'to_next'

class turn_left_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_next'],
                             input_keys=['lib'], output_keys = ['lib'])

    def execute(self, userdata):
        if userdata.lib:
            result = userdata.lib.turn_left()
        else:
            rospy.sleep(1)

        return 'to_next'

class turn_right_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_next'],
                             input_keys=['lib'], output_keys = ['lib'])

    def execute(self, userdata):
        if userdata.lib:
            result = userdata.lib.turn_right()
        else:
            rospy.sleep(1)

        return 'to_next'


def log_none(msg):
    pass

def create_state_machine(count = 0, lib = None):
    smach.set_loggers(smach.loginfo, smach.logwarn, log_none, smach.logerr)

    sm = smach.StateMachine(outcomes=['finish'])

    with sm:
        smach.StateMachine.add('detect_state', detect_state(),
                               transitions={'move_forward':'move_forward_state',
                                            'turn_left'   :'turn_left_state',
                                            'turn_right'  :'turn_right_state',
                                            'finish'      :'finish'} )
        smach.StateMachine.add('move_forward_state', move_forward_state(),
                               transitions={'to_next':'detect_state'})

        smach.StateMachine.add('turn_left_state', turn_left_state(),
                               transitions={'to_next':'detect_state'})
        smach.StateMachine.add('turn_right_state', turn_right_state(),
                               transitions={'to_next':'detect_state'})

    sm.userdata.counter = count
    sm.userdata.lib = lib

    return sm

#    result = sm.execute()
#    print result
