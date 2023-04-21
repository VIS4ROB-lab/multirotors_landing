#!/usr/bin/env python3

import gym
import numpy as np
import logging

class EnvWrapper(gym.Env):
    """Main python wrapper for gym environments coded in C++.
    """
    def __init__(self, env: gym.Env):
        
        self.env = env
        self.env.init()
        self.num_obs = env.getObsDim()
        self.num_act = env.getActDim()

        self._observation_space = gym.spaces.Box(
            np.zeros(self.num_obs),
            np.ones(self.num_obs) * 1.,
            dtype=np.float32)

        # The actions are eventually constrained by the action space.
        self._action_space = gym.spaces.Box(
            low=np.zeros(self.num_act),
            high=np.ones(self.num_act) * 1.,
            dtype=np.float32)

        # Initialization of gym variables
        self.observation = np.zeros(self.num_obs, dtype=np.float32)
        self.reward = np.float32(0.0)
        self.done = False
        gym.Env.__init__(self)

        self._max_episode_steps = 300
        self._max_steps_per_ep = 3

        # Calculate the number of steps we do with the environment before
        # collecting the reward
        sim_dt = self.env.getSimTimeStep()
        max_t = self.env.getMaxT()
        self._num_env_steps = int((max_t / sim_dt) / self._max_steps_per_ep)

        # Message
        logging.info("Environment set up correctly")

    def seed(self, seed=None):
        self.env.setSeed(seed)

    def step(self, action):
        # FIXME
        # for _ in range(self._num_env_steps):
        #     self.reward = self.env.step(action, self.observation)
        #     # Check that we did not hit the terminal state
        #     terminal_reward = 0.0
        #     self.done = self.env.isTerminalState(terminal_reward)
        #     if self.done:
        #         break
        # return self.observation.copy(), self.reward, \
        #     self.done, [dict(reward_run=self.reward, reward_ctrl=0.0)]

        self.reward = self.env.step(action, self.observation)
        terminal_reward = 0.0
        self.done = self.env.isTerminalState(terminal_reward)
        return self.observation.copy(), self.reward, \
            self.done, [dict(reward_run=self.reward, reward_ctrl=0.0)]

    def reset(self):
        self.reward = 0.0
        self.env.reset(self.observation)
        return self.observation.copy()

    def reset_and_update_info(self):
        return self.reset()

    def obs(self):
        self.env.getObs(self.observation)
        return self.observation

    def close(self,):
        return True

    @property
    def observation_space(self):
        return self._observation_space

    @property
    def action_space(self):
        return self._action_space

    @property
    def max_episode_steps(self):
        return self._max_episode_steps
