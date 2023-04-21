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

#include "multirotors_landing_lib/communication/communicator_server.hpp"

#include <thread>

namespace ml {

CommunicatorServer::CommunicatorServer(const size_t &num_sockets,
                                       const int port,
                                       const cv::Size &image_size,
                                       const bool verbose) {
  // Start threads associated to the server sockets
  for (size_t i = 0; i < num_sockets; ++i) {
    // Sockets
    sockets_.push_back(std::make_shared<socket_comm::Server>(
        image_size.width, image_size.height, verbose));
    // Threads
    server_thread_.push_back(std::make_unique<std::thread>(
        &socket_comm::Server::advertise, sockets_.back().get(), port + i));
    server_thread_.back()->detach();
  }
}

CommunicatorServer::~CommunicatorServer() {
  std::lock_guard<std::mutex> lock(server_mutex_);
  for (size_t i = 0; i < sockets_.size(); ++i) {
    sockets_[i]->stop();
    if (server_thread_[i] && server_thread_[i]->joinable()) {
      server_thread_[i]->join();
    }
  }
}

}  // end namespace ml
