import rospy
from std_msgs.msg import String
from webots_ros.msg import BoolStamped, Float64Stamped, Int32Stamped, Int8Stamped, RadarTarget, RecognitionObject, StringStamped
from webots_ros.srv import set_bool, set_float, set_float_array, set_int, set_string, get_bool, get_float, get_float_array, get_int, get_string, get_uint64
import webots_ros.srv

from sensor_msgs.msg import Image

import subprocess

model_names = []
model_map = {}

class WbDevice(object):
    def __init__(self, _name, _parent):
        self.name   = _name
        self.parent = _parent
        self.node_type = None
        self.proxy_map = {}
        self.type_map  = {}

    def start_subscribe(self):
        pass

    def callback(msg, args):
        pass

    def set_service(self, _svcname, proxy = None, svc_type = None, persistent=False):
        if not proxy and svc_type:
            proxy = rospy.ServiceProxy('/{}/{}/{}'.format(self.parent.name, self.name, _svcname),
                                       svc_type, persistent=persistent)
        self.proxy_map[_svcname] = proxy
        self.type_map [_svcname] = svc_type

class WbCamera(WbDevice):
    def __init__(self, *args):
        super(WbCamera, self).__init__(*args)
        self.set_service('enable', proxy=None, svc_type=set_int, persistent=True)
        self.sub_image = None

    def enable(self):
        self.proxy_map['enable'].call(1)
    def callback(msg, args):
        self.sub_image = msg
    def subscribe(self):
        sub = rospy.Subscriber('/{}/{}/image'.format(self.parent.name, self.name), Image, self.callback, callback_args=('image'))

class WbDisplay(WbDevice):
    def __init__(self, *args):
        super(WbDisplay, self).__init__(*args)
        self.set_service('attach_camera', proxy=None, svc_type=set_string, persistent=True)

class WbRangeFinder(WbDevice):
    def __init__(self, *args):
        super(WbRangeFinder, self).__init__(*args)
        self.set_service('enable', proxy=None, svc_type=set_int, persistent=True)
    def enable(self):
        self.proxy_map['enable'].call(1)
    def callback(msg, args):
        self.sub_image = msg
    def subscribe(self):
        sub = rospy.Subscriber('/{}/{}/image'.format(self.parent.name, self.name), Image, self.callback, callback_args=('image'))

class WbMotor(WbDevice):
    def __init__(self, *args):
        super(WbMotor, self).__init__(*args)
        self.set_service('set_position', proxy=None, svc_type=set_float, persistent=True)
        self.set_service('set_velocity', proxy=None, svc_type=set_float, persistent=True)
        self.set_pos_proxy = self.proxy_map['set_position']
        self.set_vel_proxy = self.proxy_map['set_velocity']
    def setPosition(self, pos):
        self.set_pos_proxy.call(pos)
    def setVelocity(self, vel):
        self.set_vel_proxy.call(vel)

class WbReceiver(WbDevice):
    def __init__(self, *args):
        super(WbReceiver, self).__init__(*args)

# class WbLidar(WbDevice):
# class WbGyro(WbDevice):
# class WbAccelerometer(WbDevice):
# class WbGps(WbDevice):
# class WbCompass(WbDevice):
# class WbPositionSensor(WbDevice):
# class WbDistanceSensor(WbDevice):
# class WbTouchSensor(WbDevice):
# class WbEmitter(WbDevice):

class RosClient(object):
    def __init__(self, _name):
        self.name = _name
        self.device_map = {}
        self.time_step_proxy = None
        self.getDevices()

    def getDevices(self):
        ret = rospy.ServiceProxy('/{}/robot/get_device_list'.format(self.name) ,
                                 webots_ros.srv.robot_get_device_list).call()
        for dev in ret.list:
            self.createDevice(dev)

    def createDevice(self, _devname):
        ret = rospy.ServiceProxy('/{}/{}/get_node_type'.format(self.name, _devname), get_int).call()
        if ret:
            ret = ret.value

        dev = None
        if ret == 35: ## camera
            dev = WbCamera(_devname, self)
        elif ret == 38: ## display
            dev = WbDisplay(_devname, self)
        elif ret == 52: ## rengefinder
            dev = WbRangeFinder(_devname, self)
        elif ret == 53: ## receiver
            dev = WbReceiver(_devname, self)
        elif ret == 54: ## motor
            dev = WbMotor(_devname, self)

        if dev:
            self.device_map[_devname] = dev

    def time_step(self, step):
        if not self.time_step_proxy:
            self.time_step_proxy = rospy.ServiceProxy('/{}/robot/time_step'.format(self.name),
                                                      set_int, persistent=True)
        self.time_step_proxy.call(step)

    def run_program(self, func, **kargs):
        func(self, **kargs)

def callback(data):
    #rospy.loginfo(data.data)
    model_names.append(data.data)

def create_model_map():
    sub = rospy.Subscriber("/model_name", String, callback)
    rospy.sleep(1.0)

    #print len(model_names)
    #print sub.get_num_connections()

    for nm in model_names:
        model_map[nm] = RosClient(nm)

    return model_map
