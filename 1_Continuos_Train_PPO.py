from stable_baselines3 import DDPG, PPO
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from sb3_contrib import RecurrentPPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy
import time 
import os
import torch as th
import numpy as np
from stable_baselines3.common.env_checker import check_env

import sys

# append a new directory to sys.path
sys.path.append('Training/Envirnments')

# SELECT TRAINING FREQUENCY
target_frequency = 0.5


# CHOOSE ENVIRONMENT
from Continuos_Eedward_official_environment import * # Continuous Sim s=(ang_err)
#from Continuos_Eedward_official_environment import * # Continuous Sim s=(ang_err)
#from Continuos_5_0_Eedward_official_environment import * # Continuous Sim s=(ang_err, p_diff_inlet, p_diff_outlet)
#from Continuos_2_0_Eedward_official_environment import * # Continuous Sim, s=(ang_err, vel_err, a_err)
#from Real_Continuos_Eedward_official_environment import * # Continuuos Real

env = PulleyEnv()

#env.set_target_freq(target_frequency)

check_env(env)

# #CHOOSE PRETRIANED MODEL IF NEEDED
# # models_dir = "models/PPO_cont-1709994520"
# # models_path = f"{models_dir}/800000.zip"

# models_dir = "models/PPO_cont-1709997458 - Copy"
# models_path = f"{models_dir}/800000.zip"
# log_dir = f"logs/PPO-custom-{int(time.time())}"
# # # #PRETRIAINED MODEL
# model = PPO.load(models_path, env=env, tensorboard_log=log_dir, device="cpu")

#MAKE NEW MODEL PATH
models_dir = f"models/PPO_cont-{int(time.time())}"
log_dir = f"logs/PPO-cont-{int(time.time())}"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)

if not os.path.exists(log_dir):
    os.makedirs(log_dir)



# #NEW MODEL
model = PPO("MlpPolicy", env, verbose=1,tensorboard_log=log_dir, device="cpu", ent_coef=1) 

env.reset()
TIMESTEPS = 100000
for i in range(1,1000000) :
    model.learn(total_timesteps=TIMESTEPS,  reset_num_timesteps=False, tb_log_name="PPO_custom")
    model.save(f"{models_dir}/{TIMESTEPS*i}")
env.close()