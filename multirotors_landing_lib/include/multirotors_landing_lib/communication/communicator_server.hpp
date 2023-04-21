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

#ifndef __COMMUNCATOR_SERVER_HPP__
#define __COMMUNCATOR_SERVER_HPP__

#include <memory>
#include <mutex>
#include <thread>

#include "multirotors_landing_lib/common/typedefs.hpp"
#include "multirotors_landing_lib/communication/server_comm.h"

namespace ml {

class CommunicatorServer;
typedef std::shared_ptr<ml::CommunicatorServer> CommunicatorServerPtr;

class CommunicatorServer {
  public:
  CommunicatorServer(const size_t &num_sockets, const int port = 12334,
                     const cv::Size &image_size = cv::Size(),
                     const bool verbose = false);

  ~CommunicatorServer();

  inline void updateImage(const cv::Mat &image, const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    sockets_[id]->updateImage(image);
  }

  inline void updateConfidence(const cv::Mat &image, const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    sockets_[id]->updateConfidence(image);
  }

  inline void updateSemantics(const cv::Mat &image, const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    sockets_[id]->updateSemantics(image);
  }

  inline void updateState(const std::vector<Scalar> &state,
                          const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    sockets_[id]->updateState(state);
  }

  inline void setSimReset(const bool state, const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    sockets_[id]->setSimReset(state);
  }

  inline void setResetStatus(const int state, const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    sockets_[id]->setResetStatus(state);
  }

  inline void setEnvId(const int env_id, const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    sockets_[id]->setEnvId(env_id);
  }

  inline int getEnvId(const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    return sockets_[id]->getEnvId();
  }

  inline bool requestedSimReset(const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    return sockets_[id]->needSimReset();
  }

  inline bool requestedShutdownSim() {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    for (size_t i = 0; i < sockets_.size(); ++i) {
      if (sockets_[i]->shutdownSim())
        return true;
    }
    return false;
  }

  inline void clearActionVector(const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    sockets_[id]->clearActionVector();
  }

  inline void setTimeoutAction(const double timeout) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    for (auto &s : sockets_)
      s->setTimeoutAction(timeout);
  }

  inline void clearActionVectorTime(const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    sockets_[id]->clearActionVectorTime();
  }

  inline void setCollisionState(const bool collision, const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    sockets_[id]->setCollisionState(collision);
  }

  inline void setCollisionState(const bool collision,
                                const std::vector<Scalar> &normal,
                                const int semantic_class,
                                const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    sockets_[id]->setCollisionState(collision, normal, semantic_class);
  }

  inline void heartbitAlive(const bool alive, const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    sockets_[id]->heartbitAlive(alive);
  }

  inline void setCollisionGoal(const bool collision, const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    sockets_[id]->setCollisionGoal(collision);
  }

  inline std::vector<float> getActionFromServer(const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    return sockets_[id]->getActionVector();
  }

  inline std::vector<float> getGoalFromServer(const unsigned int &id) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    return sockets_[id]->getGoalVector();
  }

  inline void setVerboseSocket(const bool verbose) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    for (auto &s : sockets_)
      s->setVerbose(verbose);
  }

  inline void setVerboseSocket(const unsigned int &id, const bool verbose) {
    // std::lock_guard<std::mutex> lock(server_mutex_);
    sockets_[id]->setVerbose(verbose);
  }

  private:
  // Communication server (running on separate thread)
  std::vector<std::unique_ptr<std::thread>> server_thread_;

  std::mutex server_mutex_;
  std::vector<socket_comm::ServerPtr> sockets_;

};  // end class

}  // end namespace ml

#endif
