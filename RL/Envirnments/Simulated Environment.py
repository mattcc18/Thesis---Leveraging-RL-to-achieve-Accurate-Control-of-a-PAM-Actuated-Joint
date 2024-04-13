from os import path
from typing import Optional
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from gymnasium.envs.classic_control import utils
from gymnasium.error import DependencyNotInstalled
import math
from PAM_model_Final import *
import random

DEFAULT_X = np.pi
DEFAULT_Y = 1.0

class PulleyEnv(gym.Env):
    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": 30,
    }

    def __init__(self, render_mode: Optional[str] = None):
        self.dt = 0.01
        self.g = 9.81
        self.m = 0.1#0.1
        self.L_arm = 0.2
        self.r_pulley = 0.0186
        self.m_pulley = 0.1
        self.L_0 = 0.150
        self.T_k = 293
        self.R = 287
        self.last_v = PAM_volume(self.L_0)
        self.time = 0
        self.k = 750
        self.I = self.m*self.L_arm*self.L_arm + self.m_pulley*self.r_pulley**2; # assume only point mass (pendulum)
        self.inlet_pressure = 6*10**5
        self.exhaust_pressure = 1.01325*10**5
        self.b = 0.3 # damping friction coefficient
        self.max_angle = 105*math.pi/180
        self.min_angle = -5*math.pi/180
        self.inlet_diameter = 14.4
        self.outlet_diameter = 17.8
        self.inlet_D = 72
        self.outlet_D = 89
        self.freq = 0.5 # how fast target is moving e.g., sin(freq*t)
        self.episode_time = 5*math.pi/self.freq
        self.current = 0
        self.action = 0
        self.last_u = 0

        self.render_mode = render_mode

        self.screen_dim = 500
        self.screen = None
        self.clock = None
        self.isopen = True

 # Spaces 
        self.action_space = spaces.Box(low=-1, high=1, dtype=np.float32) # 
        self.observation_space = spaces.Box(low = np.array([self.min_angle]), high = np.array([self.max_angle]), dtype=np.float32)

    def step(self, u):
        # Initial varibales 
        g = self.g
        m = self.m
        dt = self.dt
        L_0 = self.L_0
        r_pulley = self.r_pulley
        k = self.k
        L_arm = self.L_arm
        R = self.R
        T_k = self.T_k
        I = self.I
        self.time += dt
        self.max_current = 165 # 180mA

        #Get recent values 
        theta = self.theta
        theta_dot = self.theta_dot
        theta_dot_dot = self.theta_dot_dot
        pressure = self.pressure

        # Update PAM dimensions / state
        L = L_0-(r_pulley*theta) # PAM Length
        self.L = L
        v = PAM_volume(L)# PAM Volume
        dvdt = (v - self.last_v)/dt # change in PAM volume
        self.last_v = v


        #Convert Current to Diameter
        current = self.max_current*abs(u/1)

        #Calculate flow rate

        if u == 0: #valves closed
            dmdt = 0
        elif u > 0: # inflow
            increasing = False
            if self.last_u <= u:
                increasing = True
            p_diff = (self.inlet_pressure - pressure)/(10**5)
            d = current_to_diameter(p_diff, current, increasing, self.inlet_diameter, self.max_current)
            D = 8*(d/1.6)
            dmdt = mass_flow_rate_real(pressure, self.inlet_pressure, d, D)*dt# mass flow rate in
        elif u < 0: # outflow
            increasing = False
            if self.last_u >= u:
                increasing = True
            p_diff = (pressure- self.exhaust_pressure)/(10**5)
            d = current_to_diameter(p_diff, current, increasing, self.outlet_diameter, self.max_current)
            D = 8*(d/1.6)
            dmdt = -mass_flow_rate_exhaust_real(pressure, self.exhaust_pressure, d, D)*dt # mass flow rate out

        #Misc
        self.last_u = u  # for rendering

        # Change in Pressure 
        dpdt = ((dmdt*R*T_k)/v) - ((pressure*dvdt)/v); # Chnage in PAM Pressure
        p = pressure + dpdt*dt; # PAM Pressure 
        np.clip(p, self.exhaust_pressure, self.inlet_pressure)
        

        #Calculating Moments 
        M_arm = m*g*L_arm*math.sin(theta) # Arm Moment
        M_spring = k*r_pulley*theta*r_pulley # Spring Moment
        F_PAM = PAM_Force(p, L); # PAM Force
        M_PAM = F_PAM*r_pulley; # PAM Moment 

        T = M_PAM - M_spring - M_arm; #Torque_net
        T = T - self.b*theta_dot -self.start_Moment

        # Calculte acceleration and angle 
        theta_dot_dot = T/I # accerlation 
        newth_dot = theta_dot + dt*theta_dot_dot # velocity
        newth = theta + dt*newth_dot # angle

        #Calcualte error
        self.state_error = newth-self.target
        self.vel_error = newth_dot-self.target_omega
        self.accel_error = theta_dot_dot-self.target_alpha
        
        #Enforce Limits
        if newth > self.max_angle:
            newth = self.max_angle
            newth_dot = 0
            theta_dot_dot = 0

        if newth < self.min_angle:
            newth = self.min_angle
            newth_dot = 0
            theta_dot_dot = 0
       
        if newth < self.min_angle or newth > self.max_angle:
            terminated = True
        else: 
            terminated = False

        if self.time >= self.episode_time:
            truncated = True
        else:
            truncated = False

# set Target
        self.target = (math.pi/4)-(math.pi/4)*math.cos(self.freq*self.time)
        #self.target = math.pi/3
        self.target_omega = (self.freq*math.pi/4)*math.sin(self.freq*self.time)
        self.target_alpha = ((self.freq**2)*math.pi*math.cos(self.freq*self.time))/4
#set Reward Function
        costs = float(-(self.state_error)**2)

        #costs = -((self.state_error)**2 + 0.1*(self.accel_error)**2)

#Update Saved Values 
        self.theta = newth
        self.theta_dot = newth_dot
        self.pressure = p
        self.theta_dot_dot = theta_dot_dot
        self.action = self.last_u
        
#Return state 
        self.state = np.array([self.state_error], dtype=np.float32)

        if self.render_mode == "human":
            self.render()
        # truncation=False as the time limit is handled by the `TimeLimit` wrapper added during `make`
        return self._get_obs(), costs, terminated, truncated, {}
    
    def set_stats(self, p_inlet=6*10**5, p_exhaust=1.01325*10**5, b_damp=0, d_inlet=1.6, d_outlet=1.6, D_inlet= 8, D_outlet = 8, k=750, m_pulley=1, m_weight=0.1, L_arm=0.2, min_angle = 0, max_angle = 105):
        self.inlet_pressure = p_inlet
        self.exhaust_pressure = p_exhaust
        self.b = b_damp
        self.inlet_diameter = d_inlet
        self.outlet_diameter = d_outlet
        self.inlet_D = D_inlet
        self.outlet_D = D_outlet
        self.k = k
        self.m_pulley = m_pulley
        self.m = m_weight
        self.L_arm = L_arm
        self.min_angle = min_angle*(math.pi/180)
        self.max_angle = max_angle*(math.pi/180)
        self.pressure = self.exhaust_pressure

    def get_stats(self):
        time = np.reshape(self.time, (1,))
        angle = np.reshape(self.theta*(180/math.pi), (1,))
        vel = np.reshape(self.theta_dot*(180/math.pi), (1,))
        press_Pam = np.reshape(self.pressure/(10**5), (1,))
        accel = np.reshape(self.theta_dot_dot*(180/math.pi), (1,))
        target_theta = np.reshape(self.target*(180/math.pi), (1,))
        target_omega = np.reshape(self.target_omega*(180/math.pi), (1,))
        target_alpha = np.reshape(self.target_alpha*(180/math.pi), (1,))
        action_inlet = np.reshape(0, (1,))
        action_outlet = np.reshape(0, (1,))
        if self.action > 0:
            action_inlet = np.reshape(abs(self.action), (1,))
        elif self.action < 0:
            action_outlet = np.reshape(abs(self.action), (1,))
        error = np.reshape(self.state_error*(180/math.pi), (1,))
        stats = np.array([time, press_Pam, angle, target_theta, vel, target_omega, accel, target_alpha, error, action_inlet, action_outlet])
        stats = np.reshape(stats, (11, ))
        return stats

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        if options is None:
            high = np.array([DEFAULT_X, DEFAULT_Y])
        else:
            # Note that if you use custom reset bounds, it may lead to out-of-bound
            # state/observations.
            x = options.get("x_init") if "x_init" in options else DEFAULT_X
            y = options.get("y_init") if "y_init" in options else DEFAULT_Y
            x = utils.verify_number_and_cast(x)
            y = utils.verify_number_and_cast(y)
            high = np.array([x, y])
        self.last_u = 2
        self.state_error = 0
        start_angle = 0
        self.L = self.L_0-(self.r_pulley*start_angle) # PAM Length
        self.last_v = PAM_volume(self.L_0)
        self.state = np.array([0.0])
        self.time = 0
        self.target = -(math.pi/4)*math.cos(self.freq*self.time)+(math.pi/4)
        self.past_error = 0
        self.costs = 0.0
        self.target_alpha = 0
        self.target_omega = 0
        self.in_range = 1
        self.theta = 0
        self.theta_dot = 0
        self.pressure = self.exhaust_pressure
        self.start_Moment = PAM_Force(self.exhaust_pressure, L_0)*self.r_pulley
        self.theta_dot_dot = 0
        #self.freq = round(random.uniform(0.1, 4.0), 1)
        #self.freq = 1


        if self.render_mode == "human":
            self.render()
        return self._get_obs(), {}

    def _get_obs(self):
        error = self.state
        a = np.array([error], dtype=np.float32)
        a = np.reshape(a, (1, ))
        return a 
    
    def set_target_freq(self, freq:float):
        self.freq = freq

    def render(self):
        if self.render_mode is None:
            assert self.spec is not None
            gym.logger.warn(
                "You are calling render method without specifying any render mode. "
                "You can specify the render_mode at initialization, "
                f'e.g. gym.make("{self.spec.id}", render_mode="rgb_array")'
            )
            return

        try:
            import pygame
            from pygame import gfxdraw
        except ImportError as e:
            raise DependencyNotInstalled(
                "pygame is not installed, run `pip install gymnasium[classic-control]`"
            ) from e

        if self.screen is None:
            pygame.init()
            if self.render_mode == "human":
                pygame.display.init()
                self.screen = pygame.display.set_mode(
                    (self.screen_dim, self.screen_dim)
                )
            else:  # mode in "rgb_array"
                self.screen = pygame.Surface((self.screen_dim, self.screen_dim))
        if self.clock is None:
            self.clock = pygame.time.Clock()

        self.surf = pygame.Surface((self.screen_dim, self.screen_dim))
        self.surf.fill((255, 255, 255))

        bound = 2.2
        scale = self.screen_dim / (bound * 2)
        offset = self.screen_dim // 2

        rod_length = 1 * scale
        rod_width = 0.2 * scale
        l, r, t, b = 0, rod_length, rod_width / 2, -rod_width / 2
        coords = [(l, b), (l, t), (r, t), (r, b)]
        transformed_coords = []
        for c in coords:
            c = pygame.math.Vector2(c).rotate_rad(self.theta-(np.pi/2))
            c = (c[0] + offset, c[1] + offset)
            transformed_coords.append(c)
        gfxdraw.aapolygon(self.surf, transformed_coords, (204, 77, 77))
        gfxdraw.filled_polygon(self.surf, transformed_coords, (204, 77, 77))

        gfxdraw.aacircle(self.surf, offset, offset, int(rod_width / 2), (204, 77, 77))
        gfxdraw.filled_circle(
            self.surf, offset, offset, int(rod_width / 2), (204, 77, 77)
        )
        gfxdraw.filled_circle(self.surf, 400, 7, 2, (255, 0, 0))
        rod_end = (rod_length, 0)
        rod_end = pygame.math.Vector2(rod_end).rotate_rad(self.theta-(np.pi/2))
        rod_end = (int(rod_end[0] + offset), int(rod_end[1] + offset))

        target_end = (rod_length, 0)
        target_end  = pygame.math.Vector2(target_end).rotate_rad(self.target-(np.pi/2))
        target_end  = (int(target_end[0] + offset), int(target_end[1] + offset))

        gfxdraw.aacircle(
            self.surf, rod_end[0], rod_end[1], int(rod_width / 2), (204, 77, 77)
        )
        gfxdraw.filled_circle(
            self.surf, rod_end[0], rod_end[1], int(rod_width / 2), (204, 77, 77)
        )
        gfxdraw.filled_circle(
            self.surf, rod_end[0], rod_end[1], int(rod_width / 2), (204, 77, 77)
        )

        gfxdraw.filled_circle(
            self.surf, target_end[0], target_end[1], int(rod_width / 2), (0, 77, 77)
        )

        # fname = path.join(path.dirname(__file__), "assets/clockwise.png")
        # img = pygame.image.load(fname)
        # if self.last_u is not None:
        #     scale_img = pygame.transform.smoothscale(
        #         img,
        #         (scale * np.abs(self.last_u) / 2, scale * np.abs(self.last_u) / 2),
        #     )
        #     is_flip = bool(self.last_u > 0)
        #     scale_img = pygame.transform.flip(scale_img, is_flip, True)
        #     self.surf.blit(
        #         scale_img,
        #         (
        #             offset - scale_img.get_rect().centerx,
        #             offset - scale_img.get_rect().centery,
        #         ),
        #     )

        # drawing axle
        gfxdraw.aacircle(self.surf, offset, offset, int(0.05 * scale), (0, 0, 0))
        gfxdraw.filled_circle(self.surf, offset, offset, int(0.05 * scale), (0, 0, 0))

        self.surf = pygame.transform.flip(self.surf, False, True)
        self.screen.blit(self.surf, (0, 0))
        if self.render_mode == "human":
            pygame.event.pump()
            self.clock.tick(self.metadata["render_fps"])
            pygame.display.flip()

        else:  # mode == "rgb_array":
            return np.transpose(
                np.array(pygame.surfarray.pixels3d(self.screen)), axes=(1, 0, 2)
            )

    def close(self):
        if self.screen is not None:
            import pygame

            pygame.display.quit()
            pygame.quit()
            self.isopen = False


def angle_normalize(x):
    return ((x + np.pi) % (2 * np.pi)) - np.pi
