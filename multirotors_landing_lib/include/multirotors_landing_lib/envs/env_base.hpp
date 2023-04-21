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

#ifndef __ENV_BASE_HPP__
#define __ENV_BASE_HPP__

#include <random>
#include <unordered_map>

#include "multirotors_landing_lib/common/logger.hpp"
#include "multirotors_landing_lib/common/typedefs.hpp"

namespace ml {

template <typename Renderer, typename RobotType>
class EnvBase {
  public:
  /**
   * @brief Constructor
   * @param[in] id ID of the environment
   */
  EnvBase(const int id);

  /**
   * @brief Destructor
   */
  virtual ~EnvBase() = 0;

  /**
   * @brief Resets the environment to its starting state
   * @param[out] obs Observation at reset state
   * @return True if reset is successful, False otherwise
   */
  virtual bool reset(Ref<Vector<>> obs) = 0;

  /**
   * @brief Takes one step in the environment
   * @param[in] act Action for the step
   * @param[out] obs Observation after having taken the step
   * @return Reward value
   */
  virtual Scalar step(const Ref<Vector<>> act, Ref<Vector<>> obs) = 0;

  /**
   * @brief Function to close the environment
   */
  virtual void close();

  /**
   * @brief Function to update information about environment
   */
  virtual void updateExtraInfo();

  /**
   * @brief Get current observation
   * @param[out] obs Current observation from the environment
   * @return True if reset is successful, False otherwise
   */
  virtual bool getObs(Ref<Vector<>> obs) = 0;

  /**
   * @brief Checks if the current state if a terminal state
   * @param[out] reward Terminal reward (0 if state is not terminal)
   * @return True if current state is terminal state, False otherwise
   */
  virtual bool isTerminalState(Scalar& reward);

  /**
   * @brief Function to set the renderer of the environment (obtained from
   *        vectorized environment)
   * @attention Input renderer is not protected
   * @param[in] renderer Pointer to renderer
   */
  void setRenderer(std::shared_ptr<Renderer>& renderer) {
    renderer_ = renderer;
  }

  /**
   * @brief Set the seed for the simulation
   * @param[in] seed Chosen seed for the simulation
   */
  inline void setSeed(const int seed) {
    std::srand(seed);
  }

  /**
   * @brief Get observation space dimension
   * @return Observation space dimension
   */
  inline int getObsDim() {
    return obs_dim_;
  }

  /**
   * @brief Get action space dimension
   * @return Action space dimension
   */
  inline int getActDim() {
    return act_dim_;
  }

  /**
   * @brief Get time step used for numerical integration
   * @return Time step value
   */
  inline Scalar getSimTimeStep() {
    return sim_dt_;
  }

  /**
   * @brief Get the dimension of the extra information from the environment
   * @return Extra information dictionary dimension
   */
  inline int getExtraInfoDim() {
    return extra_info_.size();
  }

  /**
   * @brief Get maximum allowed time for an episode
   * @return Maximum time for the episode
   */
  inline Scalar getMaxT() {
    return max_t_;
  }

  /**
   * @brief Get access to the robot container as pointer.
   * @warning This can modify the underlying structure!
   * @return Pointer to robot structure
   */
  inline RobotType* getRobotPtr() const {
    return robot_.get();
  }

  /**
   * @brief Gets the robot state
   * @param[out] state State of the robot
   * @return True if successful, False otherwise
   */
  virtual bool getRobotState(Ref<Vector<>> state) = 0;

  // Variable containing all the information necessary to the training
  // algorithm
  std::unordered_map<std::string, float> extra_info_;

  protected:
  // Renderer (template)
  std::shared_ptr<Renderer> renderer_;

  // Robot (template)
  std::shared_ptr<RobotType> robot_;

  // Observation and action dimensions(for Reinforcement learning)
  int obs_dim_;
  int act_dim_;

  // Control time step
  Scalar sim_dt_{0.02f};  //< Time integration step
  Scalar max_t_{5.0f};    //< Maximum time for episode
  uint32_t step_;         //< Current step ID

  // Environement ID
  int id_;

  // Environment limits (min_x, max_x, min_y, max_y, min_z, max_z)
  Vector<6> env_limits_;

  // Auxiliaries
  LoggerPtr logger_;

};  // end class

}  // namespace ml

#endif /* __ENV_BASE_HPP__ */
