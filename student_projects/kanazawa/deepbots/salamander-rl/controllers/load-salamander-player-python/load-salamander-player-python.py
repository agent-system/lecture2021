#!/usr/bin/env python3.6
from deepbots.supervisor.controllers.robot_supervisor import RobotSupervisor
from utilities import normalizeToRange, plotData
from PPO_agent import PPOAgent, Transition

from gym.spaces import Box, Discrete
import numpy as np
import math


class SalamanderRobot(RobotSupervisor):
    def __init__(self):
        super().__init__()
        self.observation_space = Box(low=np.array([-6, -4, -3.14,-3.14, -6, -4]),
                                     high=np.array([6, 4, 3.14,3.14, 6, 4]),
                                     dtype=np.float64)
        self.action_space = Discrete(2)

        # self.robot = self.getSelf()  # Grab the robot reference from the supervisor to access various robot methods
        # self.positionSensor = self.getDevice("polePosSensor")
        self.robot = self.supervisor.getSelf()  # Fix

        # self.positionSensor = self.supervisor.getDevice("polePosSensor")  # Fix
        # self.positionSensor.enable(self.timestep)

        self.ballpoint = self.supervisor.getFromDef("w-ball")  # Fix

        self.motors = []
        for motorName in ['motor_1','motor_2','motor_3','motor_4','motor_5','motor_6','motor_leg_1','motor_leg_2','motor_leg_3','motor_leg_4']:
            motor = self.supervisor.getDevice(motorName)  # Fix
            # motor.setPosition(float('inf'))  # Set starting position
            motor.setPosition(0.0)  # Set starting position #tekitou!!
            motor.setVelocity(0.0)  # Zero out starting velocity
            self.motors.append(motor)

        self.phase = 0
            
        self.stepsPerEpisode = 2000  # Max number of steps per episode
        self.episodeScore = 0  # Score accumulated during an episode
        self.episodeScoreList = []  # A list to save all the episode scores, used to check if task is solved
        self.preballpos = 0.0

    def get_observations(self): # TODO change normalizeToRange を理解して変えたい． 
        # Salamander Position on x axis
        salamanderPositionX = normalizeToRange(self.robot.getPosition()[0], -0.4, 0.4, -1.0, 1.0)
        # Salamander Position on z axis
        salamanderPositionZ = normalizeToRange(self.robot.getPosition()[2], -0.4, 0.4, -1.0, 1.0)
        
        # Salamander Rotation
        # print(self.robot.getOrientation())
        salamanderRotation0 = normalizeToRange(self.robot.getOrientation()[0], -0.4, 0.4, -1.0, 1.0)
        salamanderRotation2 = normalizeToRange(self.robot.getOrientation()[2], -0.4, 0.4, -1.0, 1.0)        
        # if self.robot.getOrientation()[0] > 0:
        #     salamanderRotation = normalizeToRange(self.robot.getSFRotation()[2], -0.4, 0.4, -1.0, 1.0)
        # else:
        #     salamanderRotation = normalizeToRange(-self.robot.getSFRotation()[2], -0.4, 0.4, -1.0, 1.0)
        
        # balllの場所など
        # Ball Position on x axis
        ballPositionX = normalizeToRange(self.ballpoint.getPosition()[0], -6.0, 6.0, -1.0, 1.0)
        # Ball Position on z axis
        ballPositionZ = normalizeToRange(self.ballpoint.getPosition()[2], -4.0, 4.0, -1.0, 1.0)

        return [salamanderPositionX, salamanderPositionZ, salamanderRotation0, salamanderRotation2, ballPositionX, ballPositionZ]

    def get_reward(self, action=None):
        ballPositionX = round(-self.ballpoint.getPosition()[0], 2)
        return ballPositionX
        # ballPositionX = round(-self.ballpoint.getPosition()[0], 2)
        # reward = ballPositionX - self.preballpos
        # self.preballpos = ballPositionX
        # return reward

    def is_done(self): # TODO change
        # if self.episodeScore > 195.0:
        #     return True

        # # ballがどこかの壁に近づいたらとか？
        # poleAngle = round(self.positionSensor.getValue(), 2)
        # if abs(poleAngle) > 0.261799388:  # 15 degrees off vertical
        #     return True

        # salamanderPositionX = round(self.robot.getPosition()[2], 2)  # Position on z axis
        # if abs(salamanderPositionX) > 0.39:
        #     return True
        ballPositionX = round(self.ballpoint.getPosition()[0], 2)         
        if ballPositionX > 5.5:
            return True

        ## 自分が飛んでいったら
        robotPositionX = round(self.robot.getPosition()[0], 2)         
        if robotPositionX > 6 or robotPositionX < -6:
            return True
        robotPositionZ = round(self.robot.getPosition()[2], 2)         
        if robotPositionZ > 4 or robotPositionZ < -4:
            return True

        return False

    def solved(self): # TODO change
        if len(self.episodeScoreList) > 100:  # Over 100 trials thus far
            if np.mean(self.episodeScoreList[-100:]) > 195.0:  # Last 100 episodes' scores average value
                return True
        return False

    def get_default_observation(self):
        return [0.0 for _ in range(self.observation_space.shape[0])]

    def apply_action(self, action): # TODO change! hzが高すぎる気がするからそこを何とかしたい．
        action = int(action[0])

        if action == 0:
            motorSpeed = 1.0
        else:
            motorSpeed = -1.0

        # motorSpeed = 1.0

        # for i in range(len(self.motors)):
            # self.wheels[i].setPosition(float('inf'))
            # self.wheels[i].setVelocity(motorSpeed)
	    #target_position[i] = SWIM_AMPL * ampl * sin(phase + i * (2 * M_PI / 6)) * ((i + 5) / 10.0) + spine_offset;

        for i in range(6):
            SWIM_AMPL = 1.0
            ampl = 1.0
            self.phase -= 0.5 / 1000.0 * 1.4 * 2.0 * math.pi
            target_pos = SWIM_AMPL * ampl * math.sin(self.phase + i * (2 * math.pi / 6)) * ((i + 5) / 10.0) + motorSpeed
            target_pos = max(self.motors[i].getMinPosition(),target_pos)
            target_pos = min(self.motors[i].getMaxPosition(),target_pos)
            self.motors[i].setPosition(target_pos)
            # if i==1:
            #     print(i, target_pos , self.phase)

            
    def render(self, mode='human'):
        print("render() is not used")

    def get_info(self):
        return None


env = SalamanderRobot()
agent = PPOAgent(numberOfInputs=env.observation_space.shape[0], numberOfActorOutputs=env.action_space.n)
#add for load
file_path = "/home/kanazawa/Desktop/codes/in_jsk/agent-system/lecture2021/student_projects/kanazawa/deepbots/salamander-rl/controllers/salamander-player-python/weights/202105240124"
agent.load(file_path)

observation = env.reset()
while True:
    selectedAction, actionProb = agent.work(observation, type_="selectActionMax")
    observation, _, _, _ = env.step([selectedAction])
