// This is taken from Flightmare -- """credit: Philipp Foehn """

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

#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__

#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

namespace ml {

class Logger {
  public:
  Logger(const std::string& name, const bool color = true);
  Logger(const std::string& name, const std::string& filename);
  ~Logger();

  inline std::streamsize precision(const std::streamsize n);
  inline void scientific(const bool on = true);

  template <class... Args>
  void info(const std::string& message, const Args&... args) const;
  void info(const std::string& message) const;

  template <class... Args>
  void warn(const std::string& message, const Args&... args) const;
  void warn(const std::string& message) const;

  template <class... Args>
  void error(const std::string& message, const Args&... args) const;
  void error(const std::string& message) const;

  template <class... Args>
  void fatal(const std::string& message, const Args&... args) const;
  void fatal(const std::string& message) const;

  template <typename T>
  std::ostream& operator<<(const T& printable) const;

  static constexpr int MAX_CHARS = 256;

  private:
  static constexpr int DEFAULT_PRECISION = 3;
  static constexpr int NAME_PADDING = 12;
  static constexpr char RESET[] = "\033[0m";
  static constexpr char RED[] = "\033[31m";
  static constexpr char YELLOW[] = "\033[33m";
  static constexpr char INFO[] = "Info:    ";
  static constexpr char WARN[] = "Warning: ";
  static constexpr char ERROR[] = "Error:   ";
  static constexpr char FATAL[] = "Fatal:   ";
  //
  std::string name_;
  mutable std::ostream sink_;
  const bool colored_;
};

template <class... Args>
void Logger::info(const std::string& message, const Args&... args) const {
  char buf[MAX_CHARS];
  const int n = std::snprintf(buf, MAX_CHARS, message.c_str(), args...);
  if (n >= 0 && n < MAX_CHARS)
    info(buf);
  else
    error("=== Logging error ===\n");
}

template <class... Args>
void Logger::warn(const std::string& message, const Args&... args) const {
  char buf[MAX_CHARS];
  const int n = std::snprintf(buf, MAX_CHARS, message.c_str(), args...);
  if (n >= 0 && n < MAX_CHARS)
    warn(buf);
  else
    error("=== Logging error ===\n");
}

template <class... Args>
void Logger::error(const std::string& message, const Args&... args) const {
  char buf[MAX_CHARS];
  const int n = std::snprintf(buf, MAX_CHARS, message.c_str(), args...);
  if (n >= 0 && n < MAX_CHARS)
    error(buf);
  else
    error("=== Logging error ===\n");
}

template <class... Args>
void Logger::fatal(const std::string& message, const Args&... args) const {
  char buf[MAX_CHARS];
  const int n = std::snprintf(buf, MAX_CHARS, message.c_str(), args...);
  if (n >= 0 && n < MAX_CHARS)
    fatal(buf);
  else
    fatal("=== Logging error ===\n");
}

template <typename T>
std::ostream& Logger::operator<<(const T& printable) const {
  return sink_ << name_ << printable;
}

typedef std::unique_ptr<Logger> LoggerPtr;

}  // namespace ml

#endif
