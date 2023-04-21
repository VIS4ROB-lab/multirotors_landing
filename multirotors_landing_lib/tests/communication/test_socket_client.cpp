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

#include "multirotors_landing_lib/communication/client_comm.h"

using namespace socket_comm;

int main(int, char **) {
  // Set up client
  Client client;

  // Try to connect
  client.init();

  size_t iter = 0;
  const size_t iter_max = 1;
  while (iter < iter_max) {
    // Try to send a vector of float
    std::vector<float> vec{0.5f, 1.5f, 2.5f, 3.5f};
    std::cout << "Sending vector 1..." << std::endl;
    client.sendVectorFloat(vec);

    // Sleep time
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::vector<float> vec2{4.5f, 5.5f, 6.5f};
    std::cout << "Sending vector 2..." << std::endl;
    client.sendVectorFloat(vec2);

    // Sleep time
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::cout << "Sending reset request..." << std::endl;
    client.requestSimReset();

    // Sleep time
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::cout << "Requesting state..." << std::endl;
    std::vector<float> state;
    client.requestState(state, 4);
    std::cout << "\tReceived state = ";
    for (const auto &s : state) {
      std::cout << s << " ";
    }
    std::cout << std::endl;

    // Sleep time
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Read response image
    cv::Mat image = cv::Mat::zeros(480, 752, CV_8UC1);
    client.requestResultImage(image);
    cv::imshow("Response", image);
    cv::waitKey(2000);
    cv::destroyWindow("Response");

    ++iter;
  }

  // Exit
  return 0;
}
