import gym
import sys
import os
import torch as th
import argparse
import matplotlib as plt
sys.path.append('.')

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.logger import Logger, HumanOutputFormat
from stable_baselines3.common.callbacks import BaseCallback


def build_env(env_name):
    gym.envs.register(
        id=env_name,
        entry_point='rl_env.salamander_env:SlamanderRobotEnv')

    log_dir = "tmp/"
    os.makedirs(log_dir, exist_ok=True)
    env = DummyVecEnv([lambda: Monitor(gym.make(env_name), log_dir)])
    env = VecNormalize(venv=env,
                       norm_obs=True,
                       norm_reward=False,
                       clip_obs=100.,
                       clip_reward=1000.)
    return env


# for train
def train(env, nn_structure, total_timesteps):
    model = PPO(policy='MlpPolicy',
                env=env,
                n_steps=256,
                batch_size=64,
                n_epochs=3,
                verbose=1,
                policy_kwargs=nn_structure,
                tensorboard_log='./log/tb_log')
    print("start training, total timesteps is %d" % total_timesteps)
    model.learn(total_timesteps=total_timesteps, tb_log_name='6_28_01', reset_num_timesteps=False)
    model.save('./trained_model/6_28_01')
    print("training process finished")


# for test
def test(env):
    model = PPO.load("ppo_salamandar_0625.zip")
    obs = env.reset()
    for i in range(10000):
        action, _ = model.predict(obs, deterministic=True)
        obs, rewards, dones, info = env.step(action)
        print(dones, rewards)


def main():
    NN_STRUCTURE = dict(activation_fn=th.nn.ReLU,
                        net_arch=[dict(pi=[512, 256], vf=[512, 256])])
    env_name = 'salamander_env-v1'
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--mode", dest="mode", type=str, default="test")
    arg_parser.add_argument("--total_timesteps",
                            dest="total_timesteps", type=int, default=1e6)
    args = arg_parser.parse_args()
    env = build_env(env_name=env_name)
    if args.mode == "train":
        print("start training, total timesteps is %d" % args.total_timesteps)
        train(env=env,
              nn_structure=NN_STRUCTURE,
              total_timesteps=args.total_timesteps)

    elif args.mode == "test":
        test(env=env)

    else:
        raise NotImplementedError


if __name__ == '__main__':
    main()

