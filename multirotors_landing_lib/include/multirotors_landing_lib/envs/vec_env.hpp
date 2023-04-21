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

#ifndef __VEC_ENV_HPP__
#define __VEC_ENV_HPP__

#include <omp.h>

#include <memory>

#include "multirotors_landing_lib/envs/quadrotor_env.hpp"

namespace ml {

template <typename EnvBase>
class VecEnv {
  public:
  /**
   * @brief Constructor
   * @param[in] env Name of the environment
   */
  VecEnv(const std::string& env);

  /**
   * @brief Destructor
   */
  ~VecEnv();

  /**
   * @brief Resets the environments to its starting state
   * @param[out] obs Vector of Observations at reset state for all environments
   * @return True if reset is successful, False otherwise
   */
  bool reset(Ref<MatrixRowMajor<>> obs);

  /**
   * @brief Takes one step in the environment
   * @param[in] act Vector of actions for the step for all environments
   * @param[out] obs Vector of Observations for the step for all environments
   * @param[out] reward Reward values for all the environments
   * @param[out] done Vector of values for all the environments saying the environment is done
   * @param[out] extra_info Vector of infos for all the environments
   * @return True if reset is successful, False otherwise
   */
  bool step(Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> obs,
            Ref<Vector<>> reward, Ref<BoolVector<>> done,
            Ref<MatrixRowMajor<>> extra_info);

  /**
   * @brief Function to close the environment
   */
  void close();

  /**
   * @brief Function to set the seed value
   * @param[in] seed Seed value
   */
  void setSeed(const int seed);

  /**
   * @brief Function to get the current observations for all the environments
   * @param obs Vector of observations per environment
   */
  void getObs(Ref<MatrixRowMajor<>> obs) const;

  /**
   * @brief Checks if the current state if a terminal state
   * @param[in] terminal_states Vector of states to be checked
   * @param[out] rewards Terminal reward values for all the environments
   */
  void isTerminalState(Ref<BoolVector<>> terminal_states,
                       Ref<Vector<>> rewards);
  /**
   * @brief Gets the observation space dimension
   * @return Observation space dimension (# elements)
   */
  int getObsDim() const {
    return obs_dim_;
  }

  /**
   * @brief Gets the action space dimension
   * @return Action space dimension (# elements)
   */
  int getActDim() const {
    return act_dim_;
  }

  /**
   * @brief Gets the number of environments
   * @return Number of environments
   */
  int getNumOfEnvs() const {
    return num_envs_;
  }

  /**
   * @brief Gets the names of the extra infos
   * @return Extra info names as vector of strings
   */
  std::vector<std::string> getExtraInfoNames() const {
    return extra_info_names_;
  }

  /**
   * @brief Get the states of the robots
   * @param[out] states States of the robots
   * @return True if successful, False otherwise
   */
  bool getRobotsStates(Ref<MatrixRowMajor<>> states) const;

  // Exposed members
  std::vector<std::unique_ptr<EnvBase>> envs_;
  Logger logger_;

  protected:
  /**
   * @brief Function to initialize the single environments
   * @param[in] env Type of environment
   */
  void init(const std::string& env);

  /**
   * @brief Function that takes one step in one specific environment
   * @param[in] agent_id ID of the environment
   * @param[in] act Action to be taken
   * @param[out] obs Observation for the current step
   * @param[out] reward Reward from the step
   * @param[out] done True if environment is done, False otherwise
   * @param[out] extra_info Information about environment
   */
  void perAgentStep(int agent_id, Ref<MatrixRowMajor<>> act,
                    Ref<MatrixRowMajor<>> obs, Ref<Vector<>> reward,
                    Ref<BoolVector<>> done, Ref<MatrixRowMajor<>> extra_info);

  // Main components
  std::vector<std::string> extra_info_names_;

  // Auxiliary variables
  uint num_envs_, obs_dim_, act_dim_;

};  // end class VecEnv

}  // end namespace ml

#endif  // __VEC_ENV_HPP__
