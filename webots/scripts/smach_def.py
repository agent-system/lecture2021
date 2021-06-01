#!/usr/bin/env python
import rospy
import smach

class detect_state(smach.State):
    def __init__(self, func):
        smach.State.__init__(self, outcomes=['move_forward', 'turn_left', 'turn_right', 'finish', 'other_state'],
                             input_keys=['counter', 'lib'], output_keys = ['lib'])
        self.counter = 0
        self.func = func
    def execute(self, userdata):
        if self.counter > userdata.counter:
            userdata.counter_out = self.counter
            return 'finish'
        self.counter = self.counter + 1

        result = 'move_forward'
        result = self.func()

        return result

class move_forward_state(smach.State):
    def __init__(self, func):
        smach.State.__init__(self, outcomes=['to_next'],
                             input_keys=['lib'], output_keys = ['lib'])
        self.func = func

    def execute(self, userdata):
        self.func()

        return 'to_next'

class turn_left_state(smach.State):
    def __init__(self, func):
        smach.State.__init__(self, outcomes=['to_next'],
                             input_keys=['lib'], output_keys = ['lib'])
        self.func = func

    def execute(self, userdata):
        self.func()

        return 'to_next'

class turn_right_state(smach.State):
    def __init__(self, func):
        smach.State.__init__(self, outcomes=['to_next'],
                             input_keys=['lib'], output_keys = ['lib'])
        self.func = func

    def execute(self, userdata):
        self.func()

        return 'to_next'


def log_none(msg):
    pass

def create_state_machine(count = 0, lib = None):
    smach.set_loggers(smach.loginfo, smach.logwarn, log_none, smach.logerr)

    sm = smach.StateMachine(outcomes=['finish', 'other_state'])

    with sm:
        smach.StateMachine.add('detect_state', detect_state(lib.detect_trash),
                               transitions={'move_forward':'move_forward_state',
                                            'turn_left'   :'turn_left_state',
                                            'turn_right'  :'turn_right_state',
                                            'other_state' :'other_state',
                                            'finish'      :'finish'} )
        smach.StateMachine.add('move_forward_state', move_forward_state(lib.move_forward),
                               transitions={'to_next':'detect_state'})

        smach.StateMachine.add('turn_left_state', turn_left_state(lib.turn_left),
                               transitions={'to_next':'detect_state'})
        smach.StateMachine.add('turn_right_state', turn_right_state(lib.turn_right),
                               transitions={'to_next':'detect_state'})

    sm.userdata.counter = count
    sm.userdata.lib = 'lib'

    return sm

def create_layerd_state_machine(count = 0, lib = None):
    smach.set_loggers(smach.loginfo, smach.logwarn, log_none, smach.logerr)

    sm = smach.StateMachine(outcomes=['finish'])

    with sm:

        sm_sub0 = smach.StateMachine(outcomes=['other_state', 'finish'])

        with sm_sub0:
            smach.StateMachine.add('detect_state', detect_state(lib.detect_trash),
                                   transitions={'move_forward':'move_forward_state',
                                                'turn_left'   :'turn_left_state',
                                                'turn_right'  :'turn_right_state',
                                                'other_state' :'other_state',
                                                'finish'      :'finish'} )
            smach.StateMachine.add('move_forward_state', move_forward_state(lib.move_forward),
                                   transitions={'to_next':'detect_state'})
            smach.StateMachine.add('turn_left_state', turn_left_state(lib.turn_left),
                                   transitions={'to_next':'detect_state'})
            smach.StateMachine.add('turn_right_state', turn_right_state(lib.turn_right),
                                   transitions={'to_next':'detect_state'})
        sm_sub0.userdata.counter = count
        sm_sub0.userdata.lib = 'lib'
        smach.StateMachine.add('search_trash', sm_sub0,
                               transitions={'other_state':'bring_trash',
                                            'finish':'finish' })

        sm_sub1 = smach.StateMachine(outcomes=['other_state', 'finish'])

        with sm_sub1:
            smach.StateMachine.add('detect_state', detect_state(lib.detect_goal),
                                   transitions={'move_forward':'move_forward_state',
                                                'turn_left'   :'turn_left_state',
                                                'turn_right'  :'turn_right_state',
                                                'other_state' :'other_state',
                                                'finish'      :'finish'} )
            smach.StateMachine.add('move_forward_state', move_forward_state(lib.move_forward),
                                   transitions={'to_next':'detect_state'})
            smach.StateMachine.add('turn_left_state', turn_left_state(lib.turn_left),
                                   transitions={'to_next':'detect_state'})
            smach.StateMachine.add('turn_right_state', turn_right_state(lib.turn_right),
                                   transitions={'to_next':'detect_state'})
        sm_sub1.userdata.counter = count
        sm_sub1.userdata.lib = 'lib'
        smach.StateMachine.add('bring_trash', sm_sub1,
                               transitions={'other_state':'search_trash',
                                            'finish':'finish' })

    sm.userdata.counter = count
    sm.userdata.lib = 'lib'

    return sm
