import numpy as np
from stable_baselines import DQN
from rl_ros_agents.env_wappers.arena2dEnv import get_arena_envs, Arena2dEnvWrapper
import os
import tensorflow as tf

# disable tensorflow deprecated information
tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

LOGDIR = None
GAMMA = 0.95
LEARNING_RATE = 1e-3
N_STEPS = 4
MAX_GRAD_NORM = 0.1

TIME_STEPS = int(1e8)
REWARD_BOUND = 130
use_reward_bound = True

def test(path_temp_model):
    #latest_log_dir = os.path.join("results",sorted(os.listdir("results"))[-1])
    #logdir = latest_log_dir
    #envs = get_arena_envs(log_dir=logdir)
    model = DQN.load(path_temp_model)
    reset_num_timesteps = False

    #observation
    obs = np.random.rand(1, 362)
    print(obs)
    action, _ = model.predict(obs)
    print(action)

if __name__ == "__main__":
    print(os.listdir("/."))
    test("best_model.zip")

