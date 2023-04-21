#!/usr/bin/env python3

from typing import Dict, List, Tuple
import numpy as np
from gym import spaces
import logging

import gym
from stable_baselines.common.vec_env import VecEnv

from multirotors_landing_rl.common.common_defs import \
    ACTION_SET_SIZE, QUAD_FULL_STATE_SIZE


class VecQuadrotorEnvVulkan(VecEnv):
    """Vectorized version of quadrotor environment for landing using Vulkan renderer
    """
    def __init__(self, impl: gym.Env, image_size : Tuple, verbose: bool=False):
        logging.info("Setting up vectorized environment")
        
        self.wrapper = impl
        self.num_obs = self.wrapper.getObsDim()
        self.num_acts = self.wrapper.getActDim()
        self._verbose = verbose

        # Chose action space depending on action type
        self._action_space = spaces.Discrete(ACTION_SET_SIZE)

        # Observation type
        self._image_size = (int(image_size[0]), int(image_size[1]), image_size[2])

        # Observation space
        observation_size = (int(image_size[0]), int(image_size[1]), int(image_size[2]))
        self._observation_space = spaces.Box(
            low=0., high=255., shape=observation_size, dtype=np.float32)
        self._observation = np.zeros(
            (self.num_envs, self._image_size[0], self._image_size[1],
                self._image_size[2]), dtype=np.float32)

        # Other containers
        self._reward = np.zeros(self.num_envs, dtype=np.float32)
        self._done = np.zeros(self.num_envs, dtype=np.bool)
        self._extraInfoNames = self.wrapper.getExtraInfoNames()
        self._extraInfo = np.zeros([self.num_envs,
                                    len(self._extraInfoNames)],
                                   dtype=np.float32)
        self.rewards = [[] for _ in range(self.num_envs)]
        self._actions = np.zeros(self.num_envs, dtype=np.float32)

    def __del__(self):
        logging.warn("Closing RL environment!")

    def seed(self, seed: int=0) -> None:
        self.wrapper.setSeed(seed)

    def step(self, action):
        observation_size = self._image_size[0] * self._image_size[1] * \
                           self._image_size[2]
        observation_tmp = np.zeros(
            [self.num_envs, observation_size], dtype=np.float32)
        if action.dtype != np.dtype('float32'):
            action = np.array(action, dtype=np.float32)
        self.wrapper.step(action, observation_tmp,
                          self._reward, self._done, self._extraInfo)
        self.generate_observation_vector(observation_tmp)

        if len(self._extraInfoNames) is not 0:
            info = [{'extra_info': {
                self._extraInfoNames[j]: self._extraInfo[i, j] for j in
                range(0, len(self._extraInfoNames))
            }} for i in range(self.num_envs)]
        else:
            info = [{} for _ in range(self.num_envs)]

        for i in range(self.num_envs):
            self.rewards[i].append(self._reward[i])
            if self._done[i]:
                eprew = sum(self.rewards[i])
            
                if self._verbose:
                    logging.info("FINAL REWARD: {}".format(eprew))

                eplen = len(self.rewards[i])
                epinfo = {"r": eprew, "l": eplen}
                info[i]['episode'] = epinfo
                self.rewards[i].clear()

        # FIXME Check the observation is correct by displaying it
        return self._observation.copy(), self._reward.copy(), \
               self._done.copy(), info.copy()

    def sample_actions(self) -> np.ndarray:
        actions = []
        for i in range(self.num_envs):
            action = self.action_space.sample().tolist()
            actions.append(action)
        return np.asarray(actions, dtype=np.float32)

    def reset(self):
        self._reward = np.zeros(self.num_envs, dtype=np.float32)
        observation_tmp = np.zeros(
            [self.num_envs, self._image_size[0] * self._image_size[1] *
             self._image_size[2]], dtype=np.float32)
        self.wrapper.reset(observation_tmp)
        # Generate observation
        self.generate_observation_vector(observation_tmp)

        return self._observation.copy()

    def reset_and_update_info(self) -> np.ndarray:
        return self.reset(), self._update_epi_info()

    def update_model_id(self, model_id_list: List) -> None:
        model_id_vec = np.array(model_id_list, dtype=np.float32)
        self.wrapper.setModelId(model_id_vec)

    def _update_epi_info(self) -> Dict:
        info = [{} for _ in range(self.num_envs)]

        for i in range(self.num_envs):
            eprew = sum(self.rewards[i])
            eplen = len(self.rewards[i])
            epinfo = {"r": eprew, "l": eplen}
            info[i]['episode'] = epinfo
            self.rewards[i].clear()
        return info

    def render(self, mode: str='human') -> None:
        raise RuntimeError('This method is not implemented')

    def close(self) -> None:
        self.wrapper.close()

    def generate_observation_vector(self, observation_tmp: np.ndarray) -> None:
        # Cache image sizes
        h, w, channels = self._image_size[0], self._image_size[1], self._image_size[2]
        # Cache quad states and respective goals
        quad_states = np.zeros([self.num_envs, QUAD_FULL_STATE_SIZE],
                               dtype=np.float32)
        self.wrapper.getRobotsStates(quad_states)
        #
        # Now transform the observation into an image
        for e in range(self.num_envs):
            # Get the images according to the channels
            for ch in range(channels):
                self._observation[e, :, :, ch:ch + 1] = \
                    observation_tmp[e, w * h * ch: w * h * (ch + 1)].reshape(h, w, 1)

    @property
    def num_envs(self) -> int:
        return self.wrapper.getNumOfEnvs()

    @property
    def observation_space(self) -> spaces.Box:
        return self._observation_space

    @property
    def action_space(self) -> spaces.Discrete:
        return self._action_space

    @property
    def extra_info_names(self):
        return self._extraInfoNames

    def curriculum_callback(self) -> None:
        self.wrapper.curriculumUpdate()

    def step_async(self, actions: np.ndarray) -> None:
        self._actions = np.array(actions, dtype=np.float32)

    def step_wait(self):
        # raise RuntimeError('This method is not implemented')
        return self.step(self._actions)

    def goal_reached(self, env_id: int) -> bool:
        return self.wrapper.goalReached(env_id)

    def get_quad_state(self) -> np.ndarray:
        quad_states = np.zeros([self.num_envs, QUAD_FULL_STATE_SIZE],
                               dtype=np.float32)
        self.wrapper.getRobotsStates(quad_states)
        return quad_states

    def get_attr(self, attr_name, indices=None):
        """
        Return attribute from vectorized environment.
        :param attr_name: (str) The name of the attribute whose value to return
        :param indices: (list,int) Indices of envs to get attribute from
        :return: (list) List of values of 'attr_name' in all environments
        """
        raise RuntimeError('This method is not implemented')

    def set_attr(self, attr_name, value, indices=None):
        """
        Set attribute inside vectorized environments.
        :param attr_name: (str) The name of attribute to assign new value
        :param value: (obj) Value to assign to `attr_name`
        :param indices: (list,int) Indices of envs to assign value
        :return: (NoneType)
        """
        raise RuntimeError('This method is not implemented')

    def env_method(self, method_name, *method_args, indices=None,
                   **method_kwargs):
        """
        Call instance methods of vectorized environments.
        :param method_name: (str) The name of the environment method to invoke.
        :param indices: (list,int) Indices of envs whose method to call
        :param method_args: (tuple) Any positional arguments to provide in the call
        :param method_kwargs: (dict) Any keyword arguments to provide in the call
        :return: (list) List of items returned by the environment's method call
        """
        raise RuntimeError('This method is not implemented')