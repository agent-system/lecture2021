import gym
from controller import Supervisor
from gym import spaces
import numpy as np
from collections import deque

PI = np.pi
PAST_STEP = 3
ACTION_REPEAT = 30

HEAD_MOTOR = 'Head'
MOTOR_NAMES = ['Left_Shoulder', 'Left_Arm1', 'Left_Hand',
               'Right_Shoulder', 'Right_Arm1', 'Right_Hand',
               'Left_Hip', 'Left_Leg1', 'Left_Leg3', 'Left_Ankle', 'Left_Foot',
               'Right_Hip', 'Right_Leg1', 'Right_leg3', 'Right_Ankle', 'Right_Foot']

# MOTOR_LOWER_BOUND = [-PI, -PI/4, -PI/2,
#                      -PI, -PI/2, -PI/2,
#                      -PI/4, -3*PI/4, -PI/2, -PI/2, -PI/12,
#                      -PI/36, -3*PI/4, -PI/2, -PI/2, -PI/4]
# MOTOR_UPPER_BOUND = [PI, PI/2, PI/2,
#                      PI, PI/4, PI/2,
#                      PI/36, 3*PI/4, PI/2, PI/2, PI/4,
#                      PI/4, 3*PI/4, PI/2, PI/2, PI/12]
MOTOR_LOWER_BOUND = [-PI, -PI/36, -PI/36,
                     -PI, -PI/36, -PI/36,
                     -PI/24, -PI/4, -PI/6, -PI/4, -PI/24,
                     0, -PI/4, -PI/6, -PI/4, -PI/24]
MOTOR_UPPER_BOUND = [PI, PI/36, PI/36,
                     PI, PI/36, PI/36,
                     0, PI/4, PI/6, PI/4, PI/24,
                     PI/24, PI/4, PI/6, PI/4, PI/24]

MOTOR_POSITION_SENSORS_NAME = ['']*16

for i in range(len(MOTOR_NAMES)):
    MOTOR_POSITION_SENSORS_NAME[i] = MOTOR_NAMES[i] + '_Sensor'


class KhrEnv(gym.Env):
    def __init__(self, timestep=None):
        super().__init__()
        self.supervisor = Supervisor()
        self.robot = self.supervisor.getSelf()
        if timestep is None:
            self.timestep = int(self.supervisor.getBasicTimeStep())
        else:
            self.timestep = timestep

        # observation state space: 44 = gps_position3 + linearvel3 + imurpy3
        #                               +gyroanglevel3 + motorpos16 + motorvel16
        self.observation_state_dim = 44
        observation_dim = self.observation_state_dim*PAST_STEP  # 44*3 = 132
        gps_pos_high_bound = np.array([5]*3)
        gps_vel_high_bound = np.array([3]*3)
        imu_rpy_high_bound = np.array([6*PI]*3)
        angle_vel_high_bound = np.array([2*PI]*3)
        motor_pos_high_bound = np.array(MOTOR_UPPER_BOUND)
        motor_vel_high_bound = np.array([8]*16)
        observation_high_bound = np.concatenate((gps_pos_high_bound,
                                                 gps_vel_high_bound,
                                                 imu_rpy_high_bound,
                                                 angle_vel_high_bound,
                                                 motor_pos_high_bound,
                                                 motor_vel_high_bound)*PAST_STEP)

        motor_pos_low_bound = np.array(MOTOR_LOWER_BOUND)
        observation_low_bound = np.concatenate((-gps_pos_high_bound,
                                                -gps_vel_high_bound,
                                                -imu_rpy_high_bound,
                                                -angle_vel_high_bound,
                                                motor_pos_low_bound,
                                                -motor_vel_high_bound)*PAST_STEP)
        assert (observation_low_bound.shape[0] == observation_high_bound.shape[0]
                == observation_dim), "observation dimension error"
        self.observation_space = spaces.Box(low=observation_low_bound,
                                            high=observation_high_bound,
                                            dtype=np.float32)

        # action_space: motor rotations:16
        self.action_space = spaces.Box(low=np.array(MOTOR_LOWER_BOUND),
                                       high=np.array(MOTOR_UPPER_BOUND),
                                       dtype=np.float32)

        # enable gps, imu and gyro
        self.gpsSensor = self.supervisor.getDevice("gps")
        assert self.gpsSensor, "gpsSensor name error"
        self.gpsSensor.enable(int(self.timestep))

        self.imuSensor = self.supervisor.getDevice("imu")
        assert self.imuSensor, "imuSensor name error"
        self.imuSensor.enable(int(self.timestep))

        self.gyroSensor = self.supervisor.getDevice("gyro")
        assert self.gyroSensor, "gyroSensor name error"
        self.gyroSensor.enable(int(self.timestep))

        # enable motors
        self.headMotor = self.supervisor.getDevice("Head")
        assert self.headMotor, "head motor name error"
        self.bodyMotors = []
        for motorname in MOTOR_NAMES:
            bodymotor = self.supervisor.getDevice(motorname)
            assert bodymotor, ('bodymotor name error', motorname)
            bodymotor.setPosition(0.0)
            bodymotor.setVelocity(0.0)
            self.bodyMotors.append(bodymotor)

        # enable motor position sensors
        self.bodyMotorSensors = []
        for sensorname in MOTOR_POSITION_SENSORS_NAME:
            motorsensor = self.supervisor.getDevice(sensorname)
            assert motorsensor, ("bodymotorsensor name error", sensorname)
            motorsensor.enable(int(self.timestep))
            self.bodyMotorSensors.append(motorsensor)

        # enable the motor position setting
        self.supervisor.step(int(self.timestep))

        self.observationBuffer = deque(maxlen=PAST_STEP)
        self.stepCount = 0
        self.subStepCount = 0
        self.reset()

    def render(self, mode='human'):
        pass

    def reset(self):
        self.supervisor.simulationReset()
        self.supervisor.simulationResetPhysics()
        self.observationBuffer.clear()
        print("reset and buffer clear")
        self.supervisor.step(int(self.timestep))
        self.stepCount = 0
        self.subStepCount = 0
        robot_pos = np.array(self.gpsSensor.getValues())
        robot_vel = np.zeros(3)
        robot_rpy = np.array(self.imuSensor.getRollPitchYaw())
        robot_angle_vel = np.zeros(3)
        body_motors_pos = []
        for k in range(len(self.bodyMotorSensors)):
            position = np.array([self.bodyMotorSensors[k].getValue()])
            body_motors_pos = np.concatenate((body_motors_pos, position))
        body_motor_vel = np.zeros(16)
        initialState = np.concatenate((robot_pos, robot_vel, robot_rpy,
                                       robot_angle_vel, body_motors_pos, body_motor_vel))

        for k in range(PAST_STEP):
            self.observationBuffer.append(initialState)
        return np.array(list(initialState)*PAST_STEP)

    def getObservation(self):
        newState = self.getRobotState()
        self.observationBuffer.append(newState)
        observations = []
        for k in range(PAST_STEP):
            observations = np.concatenate((observations, self.observationBuffer[k]))
        return observations

    # attention: the state data here is raw data without normalizition
    # we should do the normalizition before feeding it bo the NN
    def getRobotState(self):
        robotPosition = np.array(self.gpsSensor.getValues())
        robotVelocity = self.calculateRobotLinearVelocity(robotPosition)
        robotOrientation = np.array(self.imuSensor.getRollPitchYaw())
        robotAngleVelocity = np.array(self.gyroSensor.getValues())
        bodyMotorPositions = []
        for k in range(len(self.bodyMotorSensors)):
            position = np.array([self.bodyMotorSensors[k].getValue()])
            bodyMotorPositions = np.concatenate((bodyMotorPositions, position))
        bodyMotorVelocity = self.calculateBodyMotorsVelocity(bodyMotorPositions)
        robotCurrentState = np.concatenate((robotPosition, robotVelocity, robotOrientation,
                                            robotAngleVelocity, bodyMotorPositions,
                                            bodyMotorVelocity))
        return robotCurrentState

    def calculateRobotLinearVelocity(self, currentrobotpos):
        robotLinearVel = []
        currpos = currentrobotpos
        lastpos = self.observationBuffer[PAST_STEP-1][0:3]
        for k in range(3):
            vel = (currpos[k] - lastpos[k]) / (0.001*self.timestep*ACTION_REPEAT)
            robotLinearVel.append(vel)
        return np.array(robotLinearVel)

    def calculateBodyMotorsVelocity(self, currentmotorpos):
        bodyMotorsVel = []
        currpos = currentmotorpos
        lastpos = self.observationBuffer[PAST_STEP-1][12:28]
        for k in range(len(self.bodyMotorSensors)):
            vel = (currpos[k] - lastpos[k]) / (0.001*self.timestep*ACTION_REPEAT)
            bodyMotorsVel.append(vel)
        return np.array(bodyMotorsVel)

    def step(self, action):
        self.applyActions(action)
        for j in range(ACTION_REPEAT):
            self.subStepCount += 1
            if self.supervisor.step(int(self.timestep)) == -1:
                raise ValueError("webots step simulation error")
        self.stepCount += 1
        # print("stepcount", self.stepCount)
        observation = self.getObservation()
        reward = self.getReward()
        done = self.checkTerminateState()
        info = {}
        return observation, reward, done, info

    def applyActions(self, action):
        """
        position control
        """
        assert (len(action) == len(self.bodyMotors)), "action dimensions aren't equal to bodymotor numbers"
        for k in range(len(self.bodyMotors)):
            self.bodyMotors[k].setPosition(float(action[k]))

    def getReward(self):
        pos_z = self.gpsSensor.getValues()[2]
        vel_z = self.observationBuffer[PAST_STEP-1][5]
        reward = 2*vel_z
        if (3 - pos_z) < 0.5:
            reward = reward + 0.1
        return reward

    def checkTerminateState(self):
        done = False
        pos = self.gpsSensor.getValues()
        pos_x = pos[0]
        pos_z = pos[2]
        currentTime = self.timestep * 0.001 * self.stepCount * ACTION_REPEAT
        # print(self.timestep)
        # print(currentTime)
        ref_z = -3 + currentTime*0.2
        if (pos_z > 3.1) or (pos_z < -3.1):
            done = True
        if abs(pos_x) > 0.1:
            done = True
        if currentTime > 10:
            done = True
        if (ref_z - pos_z) > 1:
            done = True
        return done


if __name__ == 'main':
    env = KhrEnv()
    from stable_baselines3.common.env_checker import check_env
    check_env(env, warn=True)
