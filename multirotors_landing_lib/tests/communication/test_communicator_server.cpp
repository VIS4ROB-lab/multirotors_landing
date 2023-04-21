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

#include "multirotors_landing_lib/communication/communicator_server.hpp"

using namespace ml;

int main(int, char**) {
  // Set up server communicator
  const size_t num_sockets = 2;
  const bool verbose = true;
  CommunicatorServer server(num_sockets);
  server.setVerboseSocket(verbose);

  // Read image to send to client
  const std::string base_path = getenv("ML_PATH");
  cv::Mat image = cv::imread(
      base_path +
      "/multirotors_landing_lib/tests/communication/data/irchel_rgb.png");
  cv::Mat img_gray;
  cv::cvtColor(image, img_gray, cv::COLOR_BGR2GRAY);
  server.updateImage(img_gray, 0);

  cv::Mat sem = cv::imread(
      base_path +
      "/multirotors_landing_lib/tests/communication/data/irchel_sem.png");
  cv::Mat sem_gray;
  cv::cvtColor(sem, sem_gray, cv::COLOR_BGR2GRAY);
  server.updateSemantics(sem_gray, 0);

  // Set fake quad states
  std::vector<float> state1{1, 2, 3, 4, 11, 22, 33, 44};
  server.updateState(state1, 0);

  std::vector<float> state2{5, 6, 7, 8, 55, 66, 77, 88};
  server.updateState(state2, 1);

  std::vector<float> normal{0, 0, 1};
  int sem_class = 4;
  server.setCollisionState(true, normal, sem_class, 0);

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // Exit
  return 0;
}
