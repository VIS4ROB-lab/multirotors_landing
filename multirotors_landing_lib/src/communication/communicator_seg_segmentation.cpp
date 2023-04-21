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

#include <thread>

#include "multirotors_landing_lib/communication/communicator_sem_segmentation.hpp"

namespace ml {

CommunicatorSemSegmentation::CommunicatorSemSegmentation(
    const size_t &num_sockets, const bool verbose) {
  for (size_t i = 0; i < num_sockets; ++i) {
    sockets_.push_back(
        std::make_shared<socket_comm::SemSegmentationClient>(verbose));
  }
}

CommunicatorSemSegmentation::~CommunicatorSemSegmentation() {}

bool CommunicatorSemSegmentation::initializeCommunication(
    const std::string ip, const int port) const {
  // Actual initialization of the sockets
  bool success = true;
  int id = 0;
  for (auto &s : sockets_) {
    success &= s->init(ip, port + id);
    ++id;
  }
  return success;
}

bool CommunicatorSemSegmentation::initializeCommunicationPortId(
    const int id) const {
  // IP and port for socket connection with server
  const std::string ip = "127.0.0.1";
  const int port = 32334;
  // Actual initialization of the socket
  std::cout
      << "\033[1;33m[Communicator Client] Initializing the evaluation socket! "
         "Assuming there is only 1 socket!\033[0m"
      << std::endl;
  return sockets_.front()->init(ip, port + id);
}

void CommunicatorSemSegmentation::sendImage(const cv::Mat &rgb,
                                            const unsigned int id) {
  sockets_[id]->send_image(rgb);
}

bool CommunicatorSemSegmentation::receiveImage(cv::Mat &result_img,
                                               const unsigned int id) {
  return sockets_[id]->get_result_image(result_img);
}

}  // end namespace ml
