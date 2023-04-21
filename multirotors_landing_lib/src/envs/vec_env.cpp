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

#include "multirotors_landing_lib/envs/vec_env.hpp"

#include <yaml-cpp/yaml.h>

namespace ml {

template <typename EnvBase>
VecEnv<EnvBase>::VecEnv(const std::string& env) : logger_("Vec " + env) {
  init(env);
}

template <typename EnvBase>
VecEnv<EnvBase>::~VecEnv() {
  logger_.warn("Closing!");
}

template <typename EnvBase>
bool VecEnv<EnvBase>::reset(Ref<MatrixRowMajor<>> obs) {
  // Check dimensions input w.r.t. environments
  if (obs.rows() != this->num_envs_ || obs.cols() != this->obs_dim_) {
    logger_.error(
        "Input matrix dimensions do not match with that of the environment");
    return false;
  }

  // Actual reset
  bool success = true;
#pragma omp parallel for schedule(dynamic)
  for (uint i = 0; i < num_envs_; i++) {
    envs_[i]->reset(obs.row(i));
  }
  return true;
}

template <typename EnvBase>
bool VecEnv<EnvBase>::step(Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> obs,
                           Ref<Vector<>> reward, Ref<BoolVector<>> done,
                           Ref<MatrixRowMajor<>> extra_info) {
  // Check input dimensions
  if (act.rows() != num_envs_ || act.cols() != act_dim_ ||
      obs.rows() != num_envs_ || obs.cols() != obs_dim_ ||
      reward.rows() != num_envs_ || reward.cols() != 1 ||
      done.rows() != num_envs_ || done.cols() != 1 ||
      extra_info.rows() != num_envs_ ||
      extra_info.cols() != extra_info_names_.size()) {
    logger_.error(
        "Input matrix dimensions do not match with that of the environment");
    return false;
  }

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < num_envs_; i++) {
    perAgentStep(i, act, obs, reward, done, extra_info);
  }
  return true;
}

template <typename EnvBase>
void VecEnv<EnvBase>::close() {
  for (uint i = 0; i < num_envs_; i++) {
    envs_[i]->close();
  }
}

template <typename EnvBase>
void VecEnv<EnvBase>::setSeed(const int seed) {
  int seed_inc = seed;
  for (uint i = 0; i < num_envs_; i++)
    envs_[i]->setSeed(seed_inc++);
}

template <typename EnvBase>
void VecEnv<EnvBase>::getObs(Ref<MatrixRowMajor<>> obs) const {
  for (uint i = 0; i < num_envs_; i++)
    envs_[i]->getObs(obs.row(i));
}

template <typename EnvBase>
void VecEnv<EnvBase>::isTerminalState(Ref<BoolVector<>> terminal_states,
                                      Ref<Vector<>> rewards) {
  {
#pragma omp parallel for schedule(dynamic)
    for (uint i = 0; i < num_envs_; i++) {
      terminal_states[i] = envs_[i]->isTerminalState(rewards[i]);
    }
  }
}

template <typename EnvBase>
bool VecEnv<EnvBase>::getRobotsStates(Ref<MatrixRowMajor<>> states) const {
  bool success = true;
  for (uint i = 0; i < num_envs_; i++) {
    success &= envs_[i]->getRobotState(states.row(i));
  }
  return success;
}

template <typename EnvBase>
void VecEnv<EnvBase>::init(const std::string& env) {
  // Check that environment variables have been declared
  if (getenv("ML_PATH") == nullptr) {
    throw std::invalid_argument("ML_PATH not declared");
  }

  // Get the parameters from YAML
  const auto file_name = getenv("ML_PATH") +
                         std::string("/multirotors_landing_lib/config/") +
                         env + std::string(".yaml");
  YAML::Node cfg = YAML::LoadFile(file_name);
  num_envs_ = cfg["env"]["num_envs"].as<int>();
  omp_set_num_threads(cfg["env"]["num_threads"].as<int>());

  // Set up environments
  if (num_envs_ <= 0) {
    throw std::invalid_argument("Specify a valid number of environments!");
  }

  for (int i = 0; i < num_envs_; i++) {
    envs_.push_back(std::make_unique<EnvBase>(i, env));
  }

  // Info about spaces
  obs_dim_ = envs_[0]->getObsDim();
  act_dim_ = envs_[0]->getActDim();

  // Generate rewards names: compute it once to get reward names. actual value
  // is not used
  envs_[0]->updateExtraInfo();
  for (auto& re : envs_[0]->extra_info_) {
    extra_info_names_.push_back(re.first);
  }

  // Message
  logger_.info("Set up complete");
}

template <typename EnvBase>
void VecEnv<EnvBase>::perAgentStep(int agent_id, Ref<MatrixRowMajor<>> act,
                                   Ref<MatrixRowMajor<>> obs,
                                   Ref<Vector<>> reward, Ref<BoolVector<>> done,
                                   Ref<MatrixRowMajor<>> extra_info) {
  // Take one step in the environment and collect reward
  const size_t id = static_cast<size_t>(agent_id);
  reward(agent_id) = envs_[id]->step(act.row(agent_id), obs.row(agent_id));

  // Check if environment is done
  Scalar terminal_reward = 0;
  done(agent_id) = envs_[id]->isTerminalState(terminal_reward);

  // Update infos and post-process results of step
  envs_[id]->updateExtraInfo();
  for (int j = 0; j < extra_info.cols(); j++) {
    extra_info(agent_id, j) = envs_[id]->extra_info_[extra_info_names_[j]];
  }

  if (done[agent_id]) {
    envs_[id]->reset(obs.row(agent_id));
    reward(agent_id) += terminal_reward;
  }
}

// IMPORTANT. Otherwise:
// Segmentation fault (core dumped)
template class VecEnv<QuadrotorEnv>;

}  // end namespace ml
