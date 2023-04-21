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

#include "multirotors_landing_lib/envs/env_base.hpp"

// Forward declarations
#include "multirotors_landing_lib/renderer/renderer_vulkan.hpp"
#include "multirotors_landing_lib/robots/quadrotor_simplified.hpp"

namespace ml {

template <typename Renderer, typename RobotType>
EnvBase<Renderer, RobotType>::EnvBase(const int id)
    : obs_dim_(0), act_dim_(0), sim_dt_(0.0), step_(0), id_(id) {}

template <typename Renderer, typename RobotType>
EnvBase<Renderer, RobotType>::~EnvBase() {}

template <typename Renderer, typename RobotType>
void EnvBase<Renderer, RobotType>::close() {}

template <typename Renderer, typename RobotType>
void EnvBase<Renderer, RobotType>::updateExtraInfo() {}

template <typename Renderer, typename RobotType>
bool EnvBase<Renderer, RobotType>::isTerminalState(Scalar& reward) {
  reward = 0.f;
  return false;
}

// IMPORTANT. Otherwise:
// Segmentation fault (core dumped)
template class EnvBase<renderer::RendererBase, ml::QuadrotorSimplified>;
template class EnvBase<renderer::RendererVulkan, ml::QuadrotorSimplified>;

}  // namespace ml
