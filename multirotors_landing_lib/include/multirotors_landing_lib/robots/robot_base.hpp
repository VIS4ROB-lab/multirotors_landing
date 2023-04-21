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

#ifndef __ROBOT_BASE_HPP__
#define __ROBOT_BASE_HPP__

#include "multirotors_landing_lib/common/common_defs.hpp"

namespace ml {

template <typename State>
class RobotBase {
  public:
  /**
   * @brief Constructor
   */
  RobotBase();

  /**
   * @brief Destructor
   */
  virtual ~RobotBase();

  /**
   * @brief Resets the robot dynamics. Virtual function.
   * @return True if reset was successful, False otherwise
   */
  virtual bool reset() = 0;

  /**
   * @brief Runs the robot dynamics for 'dt' time
   * @param[in] dt Time for dynamics computation
   * @return True if reset was successful, False otherwise
   */
  virtual bool run(const Scalar dt) = 0;

  /**
   * @brief Getters
   */
  inline State getState() const {
    return robot_state_;
  }

  /**
   * @brief Setters
   */
  inline void setState(const State &state) {
    robot_state_ = state;
  }

  protected:
  // Actual state of the robot
  State robot_state_;

};  // end class RobotBase

}  // end namespace ml

#endif  // __ROBOT_BASE_HPP__
