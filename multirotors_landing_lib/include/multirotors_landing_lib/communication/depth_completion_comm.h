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

#ifndef __DEPTH_COMPLETION_COMM_H__
#define __DEPTH_COMPLETION_COMM_H__

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace socket_comm {
class DepthCompletionClient {
  public:
  DepthCompletionClient(const bool verbose = false) : verbose_(verbose) {}
  ~DepthCompletionClient() {
    close(client__socket_id_);
  }

  bool init(const std::string ip = "127.0.0.1", int port = 22334) {
    client__socket_id_ = socket(AF_INET, SOCK_STREAM, 0);
    if (client__socket_id_ < 0) {
      std::cout << "[Depth Completion Client]: ERROR establishing socket\n"
                << std::endl;
      return false;
    }

    bool connected = false;
    int connection_attempts = 5;

    while ((!connected) && (connection_attempts > 0)) {
      struct sockaddr_in serv_addr;
      serv_addr.sin_family = AF_INET;
      serv_addr.sin_port = htons(port);
      inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr);

      if (connect(client__socket_id_, (const struct sockaddr *)&serv_addr,
                  sizeof(serv_addr)) == 0) {
        connected = true;
        std::cout << "[Depth Completion Client]: Cpp socket client connected "
                     "(port: "
                  << port << ")" << std::endl;
      } else {
        sleep(1);
        connection_attempts -= 1;
        std::cout << "[Depth Completion Client]: Error connecting to " << ip
                  << ":" << port << std::endl;
      }
    }
    return connected;
  }

  void send_image(const cv::Mat &img) {
    int msg_size = img.total() * img.elemSize();

    std::ostringstream ss;
    ss << std::setw(size_message_length_) << std::setfill('0') << msg_size
       << "\n";

    send(client__socket_id_, "L83F", 4, 0);  // magic id
    send(client__socket_id_, ss.str().c_str(), size_message_length_, 0);
    send(client__socket_id_, (uchar *)(&img.data[0]), msg_size, 0);
  }

  bool get_result_image(cv::Mat &img) {
    int imgSize = img.total() * img.elemSize();
    uchar *iptr = img.data;
    int bytes = 0;

    if (verbose_)
      std::cout << "Image Size:" << imgSize << std::endl;

    if ((bytes = recv(client__socket_id_, iptr, imgSize, MSG_WAITALL)) == -1) {
      if (verbose_)
        std::cerr << "recv failed, received bytes = " << bytes << std::endl;
      return false;
    }

    if (verbose_)
      std::cerr << "recv success, received bytes = " << bytes << std::endl;

    return true;
  }

  private:
  int client__socket_id_;
  const int size_message_length_ = 16;  // Buffer size for the length

  bool verbose_;
};

typedef std::shared_ptr<socket_comm::DepthCompletionClient>
    DepthCompletionClientPtr;

}  // namespace socket_comm

#endif