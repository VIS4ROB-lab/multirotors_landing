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

using namespace socket_comm;
using namespace ml;

int main(int, char **) {
  // Set up client
  const size_t num_sockets = 1;
  const bool verbose = true;
  CommunicatorSemSegmentation client(num_sockets, verbose);

  const int port = 12334;
  const std::string ip = "127.0.0.1";
  client.initializeCommunication(ip, port);

  // Image to send
  const std::string base_path = getenv("ML_PATH");
  cv::Mat image = cv::imread(
      base_path +
      "/multirotors_landing_lib/tests/communication/data/irchel_rgb.png");

  // Info about image we receive
  cv::Size sem_mask_size = image.size();

  size_t iter = 0;
  const size_t iter_max = 100;
  while (iter < iter_max) {
    std::cout << "Sending image..." << std::endl;
    client.sendImage(image, 0);

    std::cout << "Asking for image back..." << std::endl;

    // Remember to initialize sem_mask with right size!
    // Otherwise we will hang in there
    cv::Mat sem_mask = cv::Mat::zeros(sem_mask_size, CV_8UC1);
    client.receiveImage(sem_mask, 0);
    cv::imshow("Client", sem_mask);
    cv::waitKey(1);

    // Sleep time
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ++iter;
  }

  // Exit
  return 0;
}
