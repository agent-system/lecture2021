import gym
from controller import Supervisor
from gym import spaces
import numpy as np
from collections import deque


PAST_STEP = 4
BODY_MOTORS = ['motor_1', 'motor_2', 'motor_3', 'motor_4', 'motor_5', 'motor_6']
LEG_MOTORS = ['motor_leg_1', 'motor_leg_2', 'motor_leg_3', 'motor_leg_4']
BODY_MOTOR_POSITION_SENSORS = ['motor_1_position_sensor', 'motor_2_position_sensor',
                               'motor_3_position_sensor', 'motor_4_position_sensor',
                               'motor_5_position_sensor', 'motor_6_position_sensor']


class SlamanderRobotEnv(gym.Env):
    def __init__(self, timestep=None):
        super().__init__()
        self.supervisor = Supervisor()
        self.robot = self.supervisor.getSelf()
        if timestep is None:
            self.timestep = int(self.supervisor.getBasicTimeStep())
        else:
            self.timestep = timestep

        # observation state space: 24 = gps_position3 + gps_speed3 + imu_rpy3 + gyro3 +
        #                    position_body_motor_rotations 6 + velocity_body_motor6
        self.observation_state_space = 24
        observation_dim = self.observation_state_space*PAST_STEP  # 96
        gps_pos_high_bound = np.array([5]*3)
        gps_vel_hight_bound = np.array([5]*3)
        rpy_high_bound = np.array([np.inf]*3)
        angle_vel_high_bound = np.array([np.inf]*3)
        motor_pos_high_bound = np.array([1.13]*6)
        motor_vel_high_bound = np.array([20]*6)
        observation_high_bound = np.concatenate((gps_pos_high_bound,
                                                 gps_vel_hight_bound,
                                                 rpy_high_bound,
                                                 angle_vel_high_bound,
                                                 motor_pos_high_bound,
                                                 motor_vel_high_bound)*PAST_STEP)
        assert (observation_dim == observation_high_bound.shape[0]), "observation dimension error"
        self.observation_space = spaces.Box(low=-observation_high_bound,
                                            high=observation_high_bound,
                                            dtype=np.float32)

        # action_space: body_motor rotation 6
        self.action_space = spaces.Box(low=-motor_pos_high_bound,
                                       high=motor_pos_high_bound,
                                       dtype=np.float32)

        # enable gps and imu, gyro
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
        self.bodyMotors = []
        self.legMotors = []
        for bodyMotorName in BODY_MOTORS:
            bodymotor = self.supervisor.getDevice(bodyMotorName)
            assert bodymotor, ("bodymotor name error", bodyMotorName)
            bodymotor.setPosition(float(0))
            bodymotor.setVelocity(0.0)
            self.bodyMotors.append(bodymotor)
        for legMotorName in LEG_MOTORS:
            legmotor = self.supervisor.getDevice(legMotorName)
            assert legmotor, ("legmotor name error:", legMotorName)
            legmotor.setPosition(float(-np.pi*0.5))  # put legs backwards
            self.legMotors.append(legmotor)

        # enable body motor position sensors
        self.bodyMotorSensors = []
        for sensorName in BODY_MOTOR_POSITION_SENSORS:
            motorsensor = self.supervisor.getDevice(sensorName)
            assert motorsensor, ("bodymotorsensor name error", sensorName)
            motorsensor.enable(int(self.timestep))
            self.bodyMotorSensors.append(motorsensor)

        self.observationBuffer = deque(maxlen=PAST_STEP)
        self.stepCount = 0
        self.reset()

    def render(self, mode='human'):
        pass

    def reset(self):
        self.supervisor.simulationReset()
        self.supervisor.simulationResetPhysics()
        self.observationBuffer.clear()
        self.supervisor.step(int(self.timestep))
        self.stepCount = 0
        robot_pos = np.array(self.gpsSensor.getValues())
        robot_vel = np.zeros(3)
        robot_rpy = np.array(self.imuSensor.getRollPitchYaw())
        robot_angle_vel = np.zeros(3)
        body_motors_pos = []
        for i in range(len(self.bodyMotorSensors)):
            position = np.array([self.bodyMotorSensors[i].getValue()])
            body_motors_pos = np.concatenate((body_motors_pos, position))
        body_motors_pos = np.array(body_motors_pos)
        body_motor_vel = np.zeros(6)
        initialstate = np.concatenate((robot_pos, robot_vel, robot_rpy,
                                       robot_angle_vel, body_motors_pos, body_motor_vel))
        # print("reset, initialstate is : %", initialstate)
        for i in range(PAST_STEP):
            self.observationBuffer.append(initialstate)
        return np.concatenate((initialstate, initialstate, initialstate, initialstate))

    # attention: the state data here is raw data without normalizition
    # we should do the normalizition before feeding it bo the NN
    def getRobotState(self):
        robotPosition = np.array(self.gpsSensor.getValues())
        robotVelocity = self.calculateRobotLinearVelocity(robotPosition)
        robotOrientation = np.array(self.imuSensor.getRollPitchYaw())
        robotAngleVelocity = np.array(self.gyroSensor.getValues())
        bodyMotorPositions = []
        for i in range(len(self.bodyMotorSensors)):
            position = np.array([self.bodyMotorSensors[i].getValue()])
            bodyMotorPositions = np.concatenate((bodyMotorPositions, position))
        bodyMotorVelocity = self.calculateBodyMotorsVelocity(bodyMotorPositions)
        robotCurrentState = np.concatenate((robotPosition, robotVelocity, robotOrientation,
                                            robotAngleVelocity, bodyMotorPositions, bodyMotorVelocity))
        return robotCurrentState

    def getObservation(self):
        newState = self.getRobotState()
        self.observationBuffer.append(newState)
        observations = np.array([])
        for i in range(PAST_STEP):
            observations = np.concatenate((observations, self.observationBuffer[i]))
        return observations

    def calculateRobotLinearVelocity(self, currentrobotpos):
        robotLinearVel = []
        currpos = currentrobotpos
        lastpos = self.observationBuffer[PAST_STEP-1][0:3]
        for i in range(3):
            vel = (currpos[i] - lastpos[i]) / (0.001*self.timestep)
            robotLinearVel.append(vel)
        return np.array(robotLinearVel)

    def calculateBodyMotorsVelocity(self, currentmotorpos):
        bodyMotorsVel = []
        currpos = currentmotorpos
        lastpos = self.observationBuffer[PAST_STEP-1][12:18]
        for i in range(6):
            vel = (currpos[i] - lastpos[i]) / (0.001*self.timestep)
            bodyMotorsVel.append(vel)
        return np.array(bodyMotorsVel)

    def step(self, action):
        self.applyAction(action)
        self.leg_backward()
        if self.supervisor.step(int(self.timestep)) == -1:
            exit()
        self.stepCount += 1
        # print("stepcount", self.stepCount)
        observation = self.getObservation()
        reward = self.getReward()
        done = self.checkTerminateState()
        info = {}
        return observation, reward, done, info

    def applyAction(self, action):
        """
        apply the actions from NN output
        :param action: 1x6 array
        """
        for i in range(len(self.bodyMotors)):
            self.bodyMotors[i].setPosition(float(action[i]))

    def leg_backward(self):
        for i in range(len(self.legMotors)):
            self.legMotors[i].setPosition(float(-np.pi*0.5))
    # def getReward(self):
    #     pos_z = self.gpsSensor.getValues()[2]
    #     time_z = 0.1*(self.timestep*0.001*self.stepCount)
    #     reward = self.calculateReward(pos_z, time_z)
    #     return reward

    # def calculateReward(self, pos_z, time_z):
    #     reward = 0
    #     two_step_before_z = self.observationBuffer[0][2]
    #     if pos_z > two_step_before_z:
    #         reward += 1
    #     if pos_z > time_z:
    #         reward += 4
    #     if pos_z > 3*time_z:
    #         reward += 10
    #     # print(reward)
    #     return reward
    #
    # def checkTerminateState(self):
    #
    #     done = False
    #     pos_z = self.gpsSensor.getValues()[2]
    #     currentTime = self.timestep*0.001*self.stepCount
    #     ref_z = currentTime * 0.2-2
    #     if (pos_z > 3.2) or (pos_z < -2.15):
    #         done = True
    #     if (ref_z - pos_z) > 0.5:
    #         done = True
    #     return done

# ------------ another reward and terminate-------------------------------------

    def getReward(self):
        pos_z = self.gpsSensor.getValues()[2]
        vel_z = self.observationBuffer[PAST_STEP-1][5]
        reward = 10*vel_z
        # if (3 - pos_z) < 2:
        #     reward = reward + 0.1
        # if (3 - pos_z) < 1:
        #     reward = reward + 0.1
        # if (3 - pos_z) < 0.2:
        #     reward = reward + 0.1
        return reward

    def checkTerminateState(self):
        done = False
        pos_z = self.gpsSensor.getValues()[2]
        currentTime = self.timestep * 0.001 * self.stepCount
        ref_z = -2 + currentTime*0.5
        if (pos_z > 3.1) or (pos_z < -3.1):
            done = True
        if currentTime > 10:
            done = True
        if (ref_z - pos_z) > 1:
            done = True

        return done


if __name__ == 'main':
    env = SlamanderRobotEnv()
    from stable_baselines3.common.env_checker import check_env
    check_env(env, warn=True)

