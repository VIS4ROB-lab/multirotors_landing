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

#ifndef __COLLIDER_BASE_HPP__
#define __COLLIDER_BASE_HPP__

#include "multirotors_landing_lib/common/typedefs.hpp"

namespace ml {

namespace collider {

class ColliderBase {
  public:
  /**
   * @brief Constructor
   * @param[in] robot_dim Robot's dimension in meter (default: 0.5m)
   */
  ColliderBase(const Scalar &robot_dim = 0.5);

  /**
   * @brief Destructor
   */
  virtual ~ColliderBase();

  /**
   * @brief Function to set the robot's dumension
   * @param robot_dim Robot's dimension in meter
   */
  inline void setRobotDim(const Scalar &robot_dim) {
    robot_dim_ = robot_dim;
  }

  /**
   * @brief Function to check if a position 'p' is in collision (virtual fun)
   * @param[in] p Position to check
   * @return True if in collision, False otherwise
   */
  virtual bool isPositionInCollision(const Vector3 &p) const = 0;

  protected:
  Scalar robot_dim_;

};  // end class ColliderBase

}  // end namespace collider

}  // end namespace ml

#endif  // __COLLIDER_BASE_HPP__
