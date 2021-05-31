from controller import Node, Display

import cv2
import numpy as np
import math

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

def get_m00(c):
    return c['m00']

class MotionLib(object):
    def __init__(self, robot):
        self.robot = robot
        self.camera = None
        self.display = None
        num = self.robot.getNumberOfDevices()

        for i in range(num):
            dev = self.robot.getDeviceByIndex(i)
            tp = dev.getNodeType()
            if tp in node_type_map:
                tp = node_type_map[tp]
                print "Device #{} / {}({})".format(i, dev.getName(), tp)
            if dev.getNodeType() == Node.CAMERA:
                self.camera = dev
                print 'camera found {} / {}x{}'.format(dev.getName(),
                                                       self.camera.getWidth(),
                                                       self.camera.getHeight())

            if dev.getNodeType() == Node.DISPLAY:
                print 'display found {}'.format(dev.getName())
                self.display = dev
            if dev.getNodeType() == Node.GPS:
                print 'gps found {}'.format(dev.getName())
                self.gps = dev
        if self.camera:
            self.camera.enable(int(robot.getBasicTimeStep() * 4))
        # Show camera image in the display background.
        if self.display:
            self.display.attachCamera(self.camera)
        if self.gps:
            self.gps.enable(int(robot.getBasicTimeStep() * 2))

        self.lfw = self.robot.getDevice('lfw')
        self.rfw = self.robot.getDevice('rfw')
        self.lbw = self.robot.getDevice('lbw')
        self.rbw = self.robot.getDevice('rbw')
        self.lfw.setPosition(float('inf'))
        self.lfw.setVelocity(0.0)
        self.rfw.setPosition(float('inf'))
        self.rfw.setVelocity(0.0)
        self.lbw.setPosition(float('inf'))
        self.lbw.setVelocity(0.0)
        self.rbw.setPosition(float('inf'))
        self.rbw.setVelocity(0.0)

        self.lf_arm = self.robot.getDevice('lf_arm')
        self.rf_arm = self.robot.getDevice('rf_arm')
        self.lb_arm = self.robot.getDevice('lb_arm')
        self.rb_arm = self.robot.getDevice('rb_arm')

        self.neck_p = self.robot.getDevice('neck_p')
        self.neck_y = self.robot.getDevice('neck_y')

        self.neck_p_value = 0.0

        self.detect_count = 25
        self.counter = 0
        self.move_forward()

    def idle_step(self):
        step = self.robot.getBasicTimeStep()
        while (robot.step(step) != -1):
            pass

    def detect(self):
        self.counter += 1
        if self.robot.step(int(self.robot.getBasicTimeStep())) == -1:
            return 'finish'

        result = 'turn_right'
        if self.camera:
            img = np.array(self.camera.getImageArray(), np.uint8)

            #gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            hsv_img = cv2.cvtColor(img,  cv2.COLOR_RGB2HSV)

            # define range of color in HSV
            #lower_col = np.array([16, 40, 127]) # HSV 44, 39, 81
            #upper_col = np.array([28, 80, 255])
            lower_col = np.array([55, 100, 10]) # HSV 44, 39, 81
            upper_col = np.array([65, 255, 255])

            #lower_col = np.array([115,  80, 180]) # HSV 44, 39, 81
            #upper_col = np.array([125, 255, 220])

            # Threshold the HSV image to get only colors
            mask = cv2.inRange(hsv_img, lower_col, upper_col)

            imgEdge, contours, hierarchy = cv2.findContours(mask, 1, 2)
            contours = [ cv2.moments(c) for c in contours ]

            if contours:
                contours.sort(key = get_m00, reverse=True)
                M = contours[0]
                # print M
                if M:
                    m00 = M['m00']
                    m10 = M['m10']
                    m01 = M['m01']
                    if m00 > 8000:
                        result = 'move_forward'
                    elif m00 > 0:
                        xpos = m01/m00
                        ypos = m10/m00
                        print m00, xpos, ypos
                        diff = 10
                        if xpos > (hsv_img.shape[0] / 2.0 + diff):
                            result = 'turn_right'
                        elif xpos < (hsv_img.shape[0] / 2.0 - diff):
                            result = 'turn_left'
                        else:
                            result = 'move_forward'
                        if ypos > (hsv_img.shape[1] / 2.0 + diff):
                            self.neck_p_value -= 0.05
                        elif ypos < (hsv_img.shape[1] / 2.0 - diff):
                            self.neck_p_value += 0.05
            else:
                if self.neck_p_value < 0.03:
                    self.neck_p_value += 0.05
                elif self.neck_p_value > 0.03:
                    self.neck_p_value -= 0.05
            self.neck_p.setPosition(self.neck_p_value)
            #self.lf_arm.setPosition(0.4 * np.sin(counter / 1.0))
            #self.rf_arm.setPosition(0.4 * -np.sin(counter / 1.0))
            print(self.gps.getValues())

            #alpha = np.full(mask.shape[:2], 127, np.uint8)
            zero = np.full(mask.shape[:2], 0, np.uint8)

            # Show detected blob in the display: draw the circle and centroid.
            if self.display and self.camera:
                ir = self.display.imageNew(cv2.merge((mask, zero, zero, mask)).tolist(),
                                           Display.RGBA, self.camera.getWidth(), self.camera.getHeight())
                #display.setAlpha(0.6)
                self.display.imagePaste(ir, 0, 0, False)
                self.display.imageDelete(ir)
        return result

    def stop_moving(self):
        self.lfw.setVelocity(0.0)
        self.lbw.setVelocity(0.0)
        self.rfw.setVelocity(0.0)
        self.rbw.setVelocity(0.0)

        cntr = 0
        step = int(self.robot.getBasicTimeStep())
        while (self.robot.step(step) != -1):
            if cntr > 10:
                break
            cntr = cntr + 1

    def move_forward(self):
        if self.gps.getValues()[1] > 0.01:
            self.lfw.setVelocity(4.0)
            self.lbw.setVelocity(4.0)
            self.rfw.setVelocity(-4.0)
            self.rbw.setVelocity(-4.0)
        else:
            if self.counter%6 < 2:
                self.lf_arm.setPosition(0.2 *  np.sin(self.counter%6 * np.pi / 2.0))
                self.rf_arm.setPosition(0.2 * -np.sin(self.counter%6 * np.pi / 2.0))
                self.lb_arm.setPosition(0.5 * -np.sin(self.counter%6 * np.pi / 2.0))
                self.rb_arm.setPosition(0.5 *  np.sin(self.counter%6 * np.pi / 2.0))
            else:
                self.lf_arm.setPosition(0.2 *  np.sin((self.counter%6 + 2) * np.pi / 4.0))
                self.rf_arm.setPosition(0.2 * -np.sin((self.counter%6 + 2) * np.pi / 4.0))
                self.lb_arm.setPosition(0.5 * -np.sin((self.counter%6 + 2) * np.pi / 4.0))
                self.rb_arm.setPosition(0.5 *  np.sin((self.counter%6 + 2) * np.pi / 4.0))

        cntr = 0
        step = int(self.robot.getBasicTimeStep())
        while (self.robot.step(step) != -1):
            if cntr > self.detect_count:
                break
            cntr = cntr + 1

    def turn_right(self):
        if self.gps.getValues()[1] > 0.01:
            self.lfw.setVelocity(2.0)
            self.lbw.setVelocity(2.0)
            self.rfw.setVelocity(2.0)
            self.rbw.setVelocity(2.0)
        else:
            if self.counter%6 < 2:
                self.lf_arm.setPosition(0.2 *  np.sin(self.counter%6 * np.pi / 2.0))
                self.rf_arm.setPosition(0.1 * -np.sin(self.counter%6 * np.pi / 2.0))
                self.lb_arm.setPosition(0.5 * -np.sin(self.counter%6 * np.pi / 2.0))
                self.rb_arm.setPosition(0.1 *  np.sin(self.counter%6 * np.pi / 2.0))
            else:
                self.lf_arm.setPosition(0.2 *  np.sin((self.counter%6 + 2) * np.pi / 4.0))
                self.rf_arm.setPosition(0.1 * -np.sin((self.counter%6 + 2) * np.pi / 4.0))
                self.lb_arm.setPosition(0.5 * -np.sin((self.counter%6 + 2) * np.pi / 4.0))
                self.rb_arm.setPosition(0.1 *  np.sin((self.counter%6 + 2) * np.pi / 4.0))

        cntr = 0
        step = int(self.robot.getBasicTimeStep())
        while (self.robot.step(step) != -1):
            if cntr > self.detect_count:
                break
            cntr = cntr + 1

    def turn_left(self):
        if self.gps.getValues()[1] > 0.01:
            self.lfw.setVelocity(-2.0)
            self.lbw.setVelocity(-2.0)
            self.rfw.setVelocity(-2.0)
            self.rbw.setVelocity(-2.0)
        else:
            if self.counter%6 < 2:
                self.lf_arm.setPosition(0.1 *  np.sin(self.counter%6 * np.pi / 2.0))
                self.rf_arm.setPosition(0.2 * -np.sin(self.counter%6 * np.pi / 2.0))
                self.lb_arm.setPosition(0.1 * -np.sin(self.counter%6 * np.pi / 2.0))
                self.rb_arm.setPosition(0.5 *  np.sin(self.counter%6 * np.pi / 2.0))
            else:
                self.lf_arm.setPosition(0.1 *  np.sin((self.counter%6 + 2) * np.pi / 4.0))
                self.rf_arm.setPosition(0.2 * -np.sin((self.counter%6 + 2) * np.pi / 4.0))
                self.lb_arm.setPosition(0.1 * -np.sin((self.counter%6 + 2) * np.pi / 4.0))
                self.rb_arm.setPosition(0.5 *  np.sin((self.counter%6 + 2) * np.pi / 4.0))

        cntr = 0
        step = int(self.robot.getBasicTimeStep())
        while (self.robot.step(step) != -1):
            if cntr > self.detect_count:
                break
            cntr = cntr + 1
