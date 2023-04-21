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

#ifndef __COMMUNCATOR_DEPTH_COMPLETION_HPP__
#define __COMMUNCATOR_DEPTH_COMPLETION_HPP__

#include "multirotors_landing_lib/common/typedefs.hpp"
#include "multirotors_landing_lib/communication/depth_completion_comm.h"

namespace ml {

class CommunicatorDepthCompletion;
typedef std::shared_ptr<ml::CommunicatorDepthCompletion>
    CommunicatorDepthCompletionPtr;

class CommunicatorDepthCompletion {
  public:
  CommunicatorDepthCompletion(const size_t &num_sockets,
                              const bool verbose = false);

  ~CommunicatorDepthCompletion();

  bool initializeCommunication(const std::string ip = "127.0.0.1",
                               const int port = 22334) const;

  bool initializeCommunicationPortId(const int id) const;

  void sendImage(const cv::Mat &rgb, const unsigned int id);

  bool receiveImage(cv::Mat &result_img, const unsigned int id);

  private:
  std::vector<socket_comm::DepthCompletionClientPtr> sockets_;

};  // end class

}  // end namespace ml

#endif