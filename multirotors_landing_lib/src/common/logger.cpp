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
 * Authors: Philipp Foehn, RPG
 * Modified by: Luca Bartolomei, V4RL
 *********************************************************************/

#include "multirotors_landing_lib/common/logger.hpp"

namespace ml {

Logger::Logger(const std::string& name, const bool color)
    : sink_(std::cout.rdbuf()), colored_(color) {
  name_ = "[" + name + "]";

  if (name_.size() < NAME_PADDING)
    name_ = name_ + std::string(NAME_PADDING - name_.size(), ' ');
  else
    name_ = name_ + " ";

  sink_.precision(DEFAULT_PRECISION);
}

Logger::Logger(const std::string& name, const std::string& filename)
    : Logger(name, false) {
  if (!filename.empty()) {
    std::filebuf* fbuf = new std::filebuf;
    if (fbuf->open(filename, std::ios::out))
      sink_.rdbuf(fbuf);
    else
      warn("Could not open file %s. Logging to console!", filename);
  }
  sink_.precision(DEFAULT_PRECISION);
}

Logger::~Logger() {}

inline std::streamsize Logger::precision(const std::streamsize n) {
  return sink_.precision(n);
}

inline void Logger::scientific(const bool on) {
  if (on)
    sink_ << std::scientific;
  else
    sink_ << std::fixed;
}

void Logger::info(const std::string& message) const {
  if (colored_)
    sink_ << name_ << message << std::endl;
  else
    sink_ << name_ << INFO << message << std::endl;
}

void Logger::warn(const std::string& message) const {
  if (colored_)
    sink_ << YELLOW << name_ << message << RESET << std::endl;
  else
    sink_ << name_ << WARN << message << std::endl;
}

void Logger::error(const std::string& message) const {
  if (colored_)
    sink_ << RED << name_ << message << RESET << std::endl;
  else
    sink_ << name_ << ERROR << message << std::endl;
}

}  // namespace ml
