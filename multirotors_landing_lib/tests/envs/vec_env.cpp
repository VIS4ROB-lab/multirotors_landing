/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022,
 *  ETH Zurich - V4RL, Department of Mechanical and Process Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Luca Bartolomei
 *********************************************************************/

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <vector>

#include "multirotors_landing_lib/envs/vec_env_quadrotor.hpp"

using namespace ml;

static constexpr int SIM_STEPS_N = 20;

TEST(VecEnv, ResetEnv) {
  ml::VecQuadrotorEnvVulkan vec_env("test");
  //
  const int vec_obs_dim = vec_env.getObsDim();
  const int vec_num_env = vec_env.getNumOfEnvs();

  // reset the environment
  MatrixRowMajor<> obs;
  obs.resize(vec_num_env, vec_obs_dim);

  EXPECT_TRUE(vec_env.reset(obs));
  EXPECT_TRUE(obs.allFinite());

  // This should throw an error in the terminal ('Input matrix dimensions...')
  obs.resize(vec_num_env, vec_obs_dim + 1);
  EXPECT_FALSE(vec_env.reset(obs));
}

TEST(VecEnv, StepEnv) {
  ml::VecQuadrotorEnvVulkan vec_env("test");

  const int obs_dim = vec_env.getObsDim();
  const int act_dim = vec_env.getActDim();
  const int num_envs = vec_env.getNumOfEnvs();
  const std::vector<std::string> extra_info_names = vec_env.getExtraInfoNames();

  // reset the environment
  MatrixRowMajor<> obs, act, extra_info;
  Vector<> reward;
  BoolVector<> done;

  act.resize(num_envs, act_dim);
  obs.resize(num_envs, obs_dim);
  extra_info.resize(num_envs, extra_info_names.size());
  reward.resize(num_envs);
  done.resize(num_envs);

  EXPECT_TRUE(vec_env.reset(obs));
  EXPECT_TRUE(obs.allFinite());

  // Test step function
  for (int i = 0; i < SIM_STEPS_N; i++) {
    act.setRandom();
    act = act.cwiseMax(-1).cwiseMin(1);
    vec_env.step(act, obs, reward, done, extra_info);
  }
  EXPECT_TRUE(act.allFinite());
  EXPECT_TRUE(obs.allFinite());
  EXPECT_TRUE(reward.allFinite());
  EXPECT_TRUE(done.allFinite());

  // Test action dimension failure case
  act.resize(num_envs, act_dim - 1);
  act.setRandom();
  act = act.cwiseMax(-1).cwiseMin(1);
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));

  // Test observation dimension failure case
  act.resize(num_envs, act_dim);
  act.setRandom();
  act = act.cwiseMax(-1).cwiseMin(1);
  obs.resize(num_envs, obs_dim + 1);
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));

  // Test reward dimension failure case
  obs.resize(num_envs, obs_dim);
  reward.resize(num_envs + 1);
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));

  // Test done dimension failure case
  reward.resize(num_envs);
  done.resize(num_envs + 1);
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));

  // Test extra info failure case
  done.resize(num_envs);
  extra_info.resize(num_envs, extra_info_names.size() + 1);
  EXPECT_FALSE(vec_env.step(act, obs, reward, done, extra_info));
}
