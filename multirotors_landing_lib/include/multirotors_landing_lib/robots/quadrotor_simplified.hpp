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
 *
 *********************************************************************/

#ifndef __QUADROTOR_SIMPLIFIED_HPP__
#define __QUADROTOR_SIMPLIFIED_HPP__

#include <yaml-cpp/yaml.h>

#include <memory>

#include "multirotors_landing_lib/robots/robot_base.hpp"

namespace ml {

class QuadrotorSimplified final : public RobotBase<Vector3> {
  public:
  /**
   * @brief Empty Constructor
   */
  QuadrotorSimplified();

  /**
   * @brief Constructor using a configuration file
   * @param[in] cfg_path path to configuration file
   */
  QuadrotorSimplified(const std::string& cfg_path);

  /**
   * @brief Constructor using a YAML configuration
   * @param[in] yaml_cfg YAML configuration
   */
  QuadrotorSimplified(const YAML::Node& yaml_cfg);

  /**
   * @brief Destructor
   */
  ~QuadrotorSimplified() override;

  /**
   * @brief Resets quadrotor internal states
   * @return True if successful, False otherwise
   */
  bool reset() override;

  /**
   * @brief Runs the quadrotor dynamics for dt using desired velocity.
   * @param dt Time to run the quadrotor
   * @return True if successful, False otherwise
   */
  bool run(const Scalar dt) override;

  /**
   * @brief Apply desired action to the quadrotor
   * @param action From RL agent (select action out of 10 possibilities)
   * @param dt Simulation time during which the action is applied
   */
  void run(const int action, const Scalar dt);

  private:
  /**
   * @brief Function parsing a YAML configuration
   * @param[in] yaml_cfg YAML configuration
   */
  void readYamlConfig(const YAML::Node& yaml_cfg);

  // Desired velocity
  Vector3 vel_des_;

  // Main parameters
  Scalar vel_lateral_;
  Scalar vel_vertical_;

};  // end class QuadrotorSimplified

// Typedefs
typedef std::shared_ptr<QuadrotorSimplified> QuadrotorSimplifiedPtr;

}  // end namespace ml

#endif  // __QUADROTOR_SIMPLIFIED_HPP__
