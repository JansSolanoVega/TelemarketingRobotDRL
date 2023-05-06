#!/usr/bin/env python

import gym
import numpy
import time
import qlearn
from gym import wrappers
# ROS packages required
import rospy
import rospkg
# import our training environment
import telemarketing_empty
from stable_baselines import PPO2
from stable_baselines.common.policies import MlpPolicy, FeedForwardPolicy
from stable_baselines.bench import Monitor
from rl_ros_agents.utils.callbacks import SaveOnBestTrainingRewardCallback
import os 
from stable_baselines.common.vec_env import DummyVecEnv

LOGDIR = None
GAMMA = 0.95
LEARNING_RATE = 0.00025
N_STEPS = 4#BATCH_SIZE = N_STEPS*N_ENVIRONMENTS
MAX_GRAD_NORM = 0.1

TIME_STEPS = int(1e8)
REWARD_BOUND = 130

if __name__ == '__main__':

    rospy.init_node('transferLearning_ppo_empty', anonymous=True, log_level=rospy.WARN)
    env = gym.make('TelemarketingEmpty-v0')

    env = DummyVecEnv([lambda: env])
    rospy.loginfo("Gym environment done")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('telemarketing_openai')
    outdir = pkg_path + '/training_results'
    #env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo("Monitor Wrapper started")

    TIME_STEPS = int(1e8)

    defaul_log_dir = os.path.join("results", "PPO2_EmptyEnvironmentDiscrete_RestartHit_ReverseReward_100")
    os.makedirs(defaul_log_dir, exist_ok=True)
    logdir = defaul_log_dir
    
    path_temp_model = os.path.join(logdir, "best_model")
    if os.path.exists(path_temp_model+".zip"):
        print("continue training the model...")
        model = PPO2.load(path_temp_model+".zip", env=env)
        reset_num_timesteps = False
    else:
        print("Can't load the model with the path: {}, please check again!".format(path_temp_model))
        env.close()
        exit(-1)

    #TRAINING
    call_back = SaveOnBestTrainingRewardCallback(500, logdir, 1, REWARD_BOUND)
    try:
        print("EMPEZANDO")
        model.learn(TIME_STEPS, log_interval=200, callback=call_back, reset_num_timesteps=reset_num_timesteps)
        model.save(os.path.join(logdir, "PPO_final"))
    except KeyboardInterrupt:
        model.save(path_temp_model)
        print("KeyboardInterrupt: saved the current model to {}".format(path_temp_model))
    finally:
        env.close()
        exit(0)
