__credits__ = ["Carlos Luis"]

from os import path
from typing import Optional

import numpy as np

import gymnasium as gym
from gymnasium import spaces
from gymnasium.envs.classic_control import utils
from gymnasium.error import DependencyNotInstalled
import math
import serial
import time
from scipy import signal


DEFAULT_X = np.pi
DEFAULT_Y = 1.0

com_port = 'com7' #set COm PORT
baud_rate = 2000000 #set baud_rate


class PulleyEnv(gym.Env):
    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": 30,
    }

    def __init__(self, render_mode: Optional[str] = None):


        self.render_mode = render_mode

        self.screen_dim = 500
        self.screen = None
        self.clock = None
        self.isopen = True

        self.max_angle = 105*math.pi/180
        self.min_angle = -5*math.pi/180

        self.theta = 0
        self.pressure = 0
        self.time = 0
        self.theta_dot = 0
        self.theta_dot_dot = 0
        self.action = 0
        self.start_angle = 0

        #Start Serial Communications
        self.connect_arduino() 
        self.send_command(4095) # shut valves at start
        time.sleep(1)
        self.send_command(10000)
        incoming_data = ""
        while len(incoming_data) != 3:
            try:
                incoming_data = self.ser.readline().strip() 
                incoming_data = incoming_data.decode('utf-8').split(',')
            except UnicodeDecodeError:
                break

        #Parse State
        self.theta = float(incoming_data[2])*0.0015491088 - self.start_angle# in radians
        self.pressure = float(incoming_data[1])*10**5 # in Pa
        self.start_timestamp = float(incoming_data[0])*0.001 # in s
        self.time = 0

        #Target Frequency 
        self.freq = 0.1

        #Spaces 
        self.action_space = spaces.Box(low=-1, high=1, dtype=np.float32) # 
        self.observation_space = spaces.Box(low = np.array([self.min_angle]), high = np.array([self.max_angle]), dtype=np.float32)

    def step(self, u):

        # Valve Operation
        if u != self.last_u:
            if u > 0: # inflow
                command = int(4096*abs(u/1) + 4096 - 1)
                self.send_command(command)
            elif u < 0:#outflow
                command = int(4096*abs(u/1)) -1
                self.send_command(command)
            elif u == 0:#closed
                command=0
                self.send_command(command)
        
        self.last_u = u  # for rendering

        #Get current State
        self.send_command(number=10000)
        incoming_data = ""
        while len(incoming_data) != 3:
            try:
                incoming_data = self.ser.readline().strip() 
                incoming_data = incoming_data.decode('utf-8').split(',')
            except UnicodeDecodeError:
                break

        #Parse State
        newth = float(incoming_data[2])*0.0015491088  - self.start_angle# in radians
        if newth < -0.5*math.pi/180:
            newth = self.theta
        # try:
        #     if abs((newth-self.theta)/self.theta)>0.6:
        #         newth = self.theta
        # except:
        #     newth = newth
        self.ang_encoder = float(incoming_data[2])
        pressure = float(incoming_data[1])*10**5 # in Pa
        timestamp = float(incoming_data[0])*0.001 - self.start_timestamp # in s
        dt = timestamp-self.time
        if dt == 0:
            newth_dot = 0 # velocity
            theta_dot_dot = 0 #acceleration
        else:
            newth_dot = (newth - self.theta)/dt # velocity
            theta_dot_dot = (newth_dot-self.theta_dot)/dt #acceleration

        #Calculate error
        self.state_error = newth-self.target # target when action was made

        #Reward Funciton
        costs = -(self.state_error)**2

        #Termination
        truncated = False
        terminated = False
        
        #Update state values
        self.theta = newth
        self.pressure = pressure
        self.time = timestamp
        self.theta_dot = newth_dot
        self.theta_dot_dot = theta_dot_dot
        self.action = self.last_u

        # set Target in radians
        #self.target = 0.75*(signal.sawtooth((2 * np.pi * 0.1 * self.time)) + 1)
        #self.target = 0.75*(signal.square((2 * np.pi * 0.1 * self.time)) + 1)
        self.target = (math.pi/4)-(math.pi/4)*math.cos(self.freq*self.time)
        #if self.time>10:
        #    self.target = (math.pi/4)-(math.pi/4)*math.cos(self.freq*10)
        #(math.pi/4)-(math.pi/4)*math.cos(self.freq*self.time)
        #
        self.target_omega = (self.freq*math.pi/4)*math.sin(self.freq*self.time)
        self.target_alpha = ((self.freq**2)*math.pi*math.cos(self.freq*self.time))/4

        #Send State
        self.state = np.array([self.state_error])

        #Render step
        if self.render_mode == "human":
            self.render()

        # Return
        return self._get_obs(), costs, terminated, truncated, {}

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        time.sleep(1)
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
        low = -high  # We enforce symmetric limits.

        #Close Valves
        self.last_u = 0
        self.send_command(0) # shut valves at start
        time.sleep(1)

        #Get current State
        self.send_command(number=10000)
        incoming_data = ""
        while len(incoming_data) != 3:
            try:
                incoming_data = self.ser.readline().strip() 
                incoming_data = incoming_data.decode('utf-8').split(',')
            except UnicodeDecodeError:
                break

        self.send_command(number=10000)
        incoming_data = ""
        while len(incoming_data) != 3:
            try:
                incoming_data = self.ser.readline().strip() 
                incoming_data = incoming_data.decode('utf-8').split(',')
            except UnicodeDecodeError:
                break

        #Parse State
        self.ang_encoder = float(incoming_data[2])
        self.start_angle = float(incoming_data[2])*0.0015491088 # in radians
        print(self.start_angle)
        self.theta = 0
        self.pressure = float(incoming_data[1])*10**5 # in Pa
        self.start_timestamp = float(incoming_data[0])*0.001 # in s
        self.time = 0

        # set Target in radians
        self.target = (math.pi/4)-(math.pi/4)*math.cos(self.freq*self.time)
        self.target_omega = (self.freq*math.pi/4)*math.sin(self.freq*self.time)
        self.target_alpha = ((self.freq**2)*math.pi*math.cos(self.freq*self.time))/4
        
        #set Current state
        self.state_error = self.theta-self.target
        self.state = np.array([self.state_error])
        
        if self.render_mode == "human":
            self.render()
        return self._get_obs(), {}
    
    def send_command(self, number:int):
        #Send the high byte
        self.ser.write(bytes([(number >> 8) & 0xFF]))
        # Send the low byte next
        self.ser.write(bytes([number & 0xFF]))

    def _get_obs(self):
        error = self.state
        a = np.array([error], dtype=np.float32)
        a = np.reshape(a, (1, ))
        return a 
    
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
        encoder = np.reshape(self.ang_encoder, (1,)) 
        stats = np.array([time, press_Pam, angle, target_theta, vel, target_omega, accel, target_alpha, error, action_inlet, action_outlet, encoder])
        stats = np.reshape(stats, (12, ))
        return stats
    
    def set_target_freq(self, freq:float):
        self.freq = freq

    def connect_arduino(self):
        self.ser = serial.Serial(com_port,
                            baudrate=baud_rate,
                            timeout=0.1,
                            )

        # Toggle DTR to reset Arduino
        self.ser.setDTR(False)
        time.sleep(1)
        self.ser.flushInput()
        self.ser.setDTR(True)

        #Wait until connected message recieved from arduino 
        message = self.ser.readline().strip()  # Read 1 byte from Arduino
        message = message.decode('utf-8').split(',')
        while message[0] != 'Connected': 
            message = self.ser.readline().strip()  # Read 1 byte from Arduino
            message = message.decode('utf-8').split(',')

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
