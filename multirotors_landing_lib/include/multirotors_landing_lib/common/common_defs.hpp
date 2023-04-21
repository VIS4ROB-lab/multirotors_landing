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

#ifndef __COMMON_DEFS_HPP__
#define __COMMON_DEFS_HPP__

#include "multirotors_landing_lib/common/typedefs.hpp"

namespace ml {

/**
 * @brief Reasons that can terminate an episode
 */
enum DoneReason {
  kNone = -1,
  kSemantics = 0,
  kCollision = 1,
  kMaxTime = 2,
  kBoundaries = 3,
  kTargetHeight = 4
};

/**
 * @brief Parameters used in the reward specification
 */
struct RewardParams {
  // Config
  bool use_early_stop_semantics;

  // Terminal reward
  Scalar landing_good_class;
  Scalar landing_bad_class;
  Scalar max_time;
  Scalar target_height;

  // Step reward
  Scalar step;
  Scalar sem_good;
  Scalar sem_bad = -1;

  // Penalization lateral movements
  Scalar lateral_mov;
  // Penalization hovering
  Scalar stand_still;
  // Incentivation landing movements
  Scalar down_mov;

  // Collision reward
  Scalar coll_survival;

  // Environment limits
  Scalar out_of_bounds;

  // Weights
  Scalar sem_weight;
  Scalar coll_weight;
  Scalar distance_weight;
};

/**
 * @brief Parameters for robot collisions
 */
struct CollisionParams {
  Scalar min_dist;      //< Min distance to be kept from obstacles
  Scalar robot_radius;  //< Robot dimension (modeled as a sphere)
};

}  // end namespace ml

#endif  // __COMMON_DEFS_HPP__
