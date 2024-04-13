from stable_baselines3 import A2C, DQN, PPO, DDPG
import torch as th
import pandas as pd
import keyboard
import matplotlib.pyplot as plt
import math

import sys

#SELECT TARGET FREQUENCY
target_frequency =2

#SELECT MODEL PATH

# models_dir = "models/PPO-1709850196"
# models_path = f"{models_dir}/400000.zip"

# # #BEst one
# models_dir = "models/PPO_cont-1709997458"
# models_path = f"{models_dir}/800000.zip"

models_dir = "models/PPO_cont-1712160213"
models_path = f"{models_dir}/400000.zip"

# models_dir = "models/PPO_cont-1710183415"
# models_path = f"{models_dir}/500000.zip"

# models_dir = "models/PPO_cont-1710190129"
# models_path = f"{models_dir}/200000.zip"

#CHOOSE ENVIRONMENT

# append a new directory to sys.path
sys.path.append('Training/Envirnments')

# now you can import your module
#from Discrete_Eedward_official_environment import * # Discrete simulation
#from Real_Discrete_Eedward_official_environment_v3 import * # Discrete Real
from Continuos_Eedward_official_environment import * # Continuous Real
#from Continuos_2_0_Eedward_official_environment import * # Continuous Real
#from Continuos_3_0_Eedward_official_environment import * # Continuous Sim s=(ang_err, p_diff_inlet, p_diff_outlet)
#from Real_Continuos_Eedward_official_environment import * # Continuuos Real



env = PulleyEnv(render_mode="human")
#env = PulleyEnv()
discrete = True
try:
    env.action_space.n
    discrete = True
except:
    discrete = False


env.set_target_freq(target_frequency)

data = pd.DataFrame()

model = PPO.load(models_path, env=env)
print(model.policy)

episodes = 1

for ep in range(episodes):
    # obs, info = env.reset()
    done = False
    truncated = False
    obs, info = env.reset()
    while not done or not truncated:
        env.render()
        stats = env.get_stats()
        action, _ =  model.predict(obs)
        obs, reward, done, truncated, info = env.step(action)
        if discrete:
            new_row = {'Time (s)': stats[0], 'PAM_pressure (bar)': stats[1], 'Angle (deg)':stats[2],'Target Angle (deg)':stats[3], 'Velocity': stats[4], 'Target Velocity (deg)':stats[5], 'Acceleration (deg)': stats[6], 'Target Accel (deg)': stats[7], 'Error (deg)': stats[8], 'Action': stats[9] }
        else:
            new_row = {'Time (s)': stats[0], 'PAM_pressure (bar)': stats[1], 'Angle (deg)':stats[2],'Target Angle (deg)':stats[3], 'Velocity': stats[4], 'Target Velocity (deg)':stats[5], 'Acceleration (deg)': stats[6], 'Target Accel (deg)': stats[7], 'Error (deg)': stats[8], 'Action': action, 'Inlet Action': stats[9], 'Outlet Action': stats[10]}
            #new_row = {'Time (s)': stats[0], 'PAM_pressure (bar)': stats[1], 'Angle (deg)':stats[2],'Target Angle (deg)':stats[3], 'Velocity': stats[4], 'Target Velocity (deg)':stats[5], 'Acceleration (deg)': stats[6], 'Target Accel (deg)': stats[7], 'Error (deg)': stats[8], 'Action': action, 'Inlet Action': stats[9], 'Outlet Action': stats[10], 'Encoder': stats[11]  }
        data = pd.concat([data, pd.DataFrame([new_row])], ignore_index=True)


        if keyboard.is_pressed("esc"):
            #env.send_command(4095)
                # Key was pressed
            break
        
        if stats[0]>((2*math.pi)/target_frequency):
            break
        #     #env.send_command(4095)
        #         # Key was pressed
        #     break
        
data.to_csv("M:\Thesis\Assets\Deployment of Model\REal tetss\Real- at " + str(target_frequency) + " test_1.csv")

# data.plot(x='Time (s)', y='PAM_pressure (bar)')
ax = data.plot(x='Time (s)', y='Angle (deg)')
data.plot(x='Time (s)', y='Target Angle (deg)', ax=ax, c='Red')
data.plot(x='Time (s)', y='Error (deg)', ax=ax, c='Orange')
ax.set_title("Real - RL Agent Tracking Performance at " + str(target_frequency) + " rad/s")
ax.set_ylabel("Angle (°)")
ax.set_xlabel("Time (s)")
ax.legend(["RL Agent", "Target Trajectory", "Error"])
ax1 = data.plot.scatter(x='Error (deg)', y='Inlet Action', legend = None)
ax1.set_title("Simulation - Inlet RL Agent Action vs Error at " + str(target_frequency) +" rads/sec")
ax2 = data.plot.scatter(x='Error (deg)', y='Outlet Action', c='Red', legend = None)
ax2.set_title("Simulation - Outlet RL Agent Action vs Error " + str(target_frequency) +" at 0.2 rads/sec")
# data.plot(x='Time (s)', y='Target Angle (deg)', ax=ax, c='Red')
# #data.plot(x='Time (s)', y = "Encoder")

# data.plot(x='Time (s)', y='PAM_pressure (bar)')
# ax = data.plot(x='Time (s)', y='Angle (deg)')
# ax1 = data.plot.scatter(x='Error (deg)', y='Inlet Action')
# data.plot.scatter(x='Error (deg)', y='Outlet Action', ax=ax1, c='Red')
# data.plot(x='Time (s)', y='Target Angle (deg)', ax=ax, c='Red')
# #data.plot(x='Time (s)', y = "Encoder")



print(data['Error (deg)'].abs().max())
a = data['Error (deg)']**2
print(a.mean()**.5)
plt.show()
env.close()