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

// pybind11
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// Our library
#include "multirotors_landing_lib/common/common_defs.hpp"
#include "multirotors_landing_lib/envs/vec_env_quadrotor.hpp"

namespace py = pybind11;
using namespace ml;

// Utilities (iterator for enumerations)
template <typename Enum>
struct iter_enum {
  struct iterator {
    using value_type = Enum;
    using difference_type = ptrdiff_t;
    using reference = const Enum&;
    using pointer = const Enum*;
    using iterator_category = std::input_iterator_tag;

    iterator(Enum value) : cur(value) {}

    reference operator*() {
      return cur;
    }
    pointer operator->() {
      return &cur;
    }
    bool operator==(const iterator& other) {
      return cur == other.cur;
    }
    bool operator!=(const iterator& other) {
      return !(*this == other);
    }
    iterator& operator++() {
      if (cur != Enum::kNone)
        cur = static_cast<Enum>(static_cast<std::underlying_type_t<Enum>>(cur) +
                                1);
      return *this;
    }
    iterator operator++(int) {
      iterator other = *this;
      ++(*this);
      return other;
    }

private:
    Enum cur;
  };

  iterator begin() {
    return iterator(static_cast<Enum>(0));
  }

  iterator end() {
    return iterator(Enum::kNone);
  }
};

// Actual definitions
PYBIND11_MODULE(mlgym, m) {
  // Environments
  py::class_<VecQuadrotorEnvVulkan>(m, "QuadrotorEnvVulkan")
      .def(py::init<const std::string&>())
      .def("reset", &VecQuadrotorEnvVulkan::reset)
      .def("step", &VecQuadrotorEnvVulkan::step)
      .def("setSeed", &VecQuadrotorEnvVulkan::setSeed)
      .def("close", &VecQuadrotorEnvVulkan::close)
      .def("isTerminalState", &VecQuadrotorEnvVulkan::isTerminalState)
      .def("getNumOfEnvs", &VecQuadrotorEnvVulkan::getNumOfEnvs)
      .def("getObsDim", &VecQuadrotorEnvVulkan::getObsDim)
      .def("getActDim", &VecQuadrotorEnvVulkan::getActDim)
      .def("getExtraInfoNames", &VecQuadrotorEnvVulkan::getExtraInfoNames)
      .def("getRobotsStates", &VecQuadrotorEnvVulkan::getRobotsStates)
      .def("setModelId", &VecQuadrotorEnvVulkan::setModelId)
      .def("__repr__", [](const VecQuadrotorEnvVulkan& /*a*/) {
        return "Quadrotor Environment for Landing using Vulkan renderer";
      });

  // Utilities
  py::enum_<DoneReason>(m, "DoneReason")
      .value("None", DoneReason::kNone)
      .value("Semantics", DoneReason::kSemantics)
      .value("Collision", DoneReason::kCollision)
      .value("MaxTime", DoneReason::kMaxTime)
      .value("Boundaries", DoneReason::kBoundaries)
      .value("TargetHeight", DoneReason::kTargetHeight)
      .export_values()
      // add other values and __str__
      .def_static("iter", []() {
        iter_enum<DoneReason> i;
        return py::make_iterator(i.begin(), i.end());
      });
}
