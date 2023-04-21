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

#include "multirotors_landing_lib/robots/quadrotor_simplified.hpp"

namespace ml {

QuadrotorSimplified::QuadrotorSimplified() : vel_des_(Vector3::Zero()) {}

QuadrotorSimplified::QuadrotorSimplified(const std::string& cfg_path)
    : QuadrotorSimplified() {
  // Read configuration
  readYamlConfig(YAML::LoadFile(cfg_path));
}

QuadrotorSimplified::QuadrotorSimplified(const YAML::Node& yaml_cfg)
    : QuadrotorSimplified() {
  // Read configuration
  readYamlConfig(yaml_cfg);
}

void QuadrotorSimplified::readYamlConfig(const YAML::Node& yaml_cfg) {
  if (yaml_cfg["quadrotor"]) {
    vel_lateral_ = yaml_cfg["quadrotor"]["vel_lateral"].as<Scalar>();
    vel_vertical_ = yaml_cfg["quadrotor"]["vel_vertical"].as<Scalar>();
  } else {
    throw std::invalid_argument(
        "Not information about quadrotor found in configuration file");
  }
}

QuadrotorSimplified::~QuadrotorSimplified() {}

bool QuadrotorSimplified::reset() {
  robot_state_.setZero();
  vel_des_.setZero();
  return true;
}

bool QuadrotorSimplified::run(const Scalar dt) {
  // In the simplified quadrotor model, we assume that the robot
  // can directly jump to the desired position
  robot_state_ += vel_des_ * dt;
  return true;
}

void QuadrotorSimplified::run(const int action, const Scalar dt) {
  // Here we do the mapping RL action -> new direction
  constexpr int n_rot = 8;

  if (action == 0) {
    // Do not move
    vel_des_.setZero();
  } else if (action != 0 && action < 9) {
    Scalar angle = (action - 1) * 2 * static_cast<Scalar>(M_PI) / n_rot;
    vel_des_[0] = vel_lateral_ * std::cos(angle);
    vel_des_[1] = vel_lateral_ * std::sin(angle);
    vel_des_[2] = 0.0;
  } else if (action == 9) {
    vel_des_.setZero();
    vel_des_[2] -= vel_vertical_;
  } else {
    throw std::invalid_argument("Action out of bounds!");
  }

  // Update position
  run(dt);
}

}  // end namespace ml
