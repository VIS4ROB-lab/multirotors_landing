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

#include <random>
#include <thread>

#include "multirotors_landing_lib/communication/communicator_client.hpp"

using namespace ml;

int main(int argc, char **argv) {
  // Set up communicator
  const int num_sockets = 2;
  const bool verbose = true;
  CommunicatorClient client(num_sockets, verbose);

  const int port = 12334;
  std::string ip;
  if (argc < 2) {
    std::cout << "\033[93mInput IP not specified - using 127.0.0.1\n\033[0m";
    ip = "127.0.0.1";
    std::this_thread::sleep_for(std::chrono::seconds(1));
  } else {
    ip = argv[1];
  }
  client.initializeCommunication(ip, port);

  // random generation for testing
  std::random_device rd;
  std::mt19937 random_gen{rd()};

  size_t iter = 0;
  const size_t iter_max = 1;
  while (iter < iter_max) {
    // Socket number 1
    std::vector<float> v1{0.5f, 1.5f, 2.5f, 3.5f};
    std::cout << "Sending vector 1..." << std::endl;
    client.communicateAction(v1, 0);

    // Sleep time
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Socket number 2
    std::vector<float> v2{4.5f, 5.5f, 6.5f, 7.5f};
    std::cout << "Sending vector 2..." << std::endl;
    client.communicateAction(v2, 1);

    // Sleep time
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Socket number 1
    std::vector<float> state1;
    std::cout << "Requesting state 1..." << std::endl;
    client.receiveState(state1, 8, 0);

    std::cout << "Received state: ";
    for (const auto &s : state1)
      std::cout << s << " ";
    std::cout << std::endl;

    // Sleep time
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Socket number 2
    std::vector<float> state2;
    std::cout << "Requesting state 2..." << std::endl;
    client.receiveState(state2, 8, 1);

    std::cout << "Received state: ";
    for (const auto &s : state2)
      std::cout << s << " ";
    std::cout << std::endl;

    // Socket number 1
    std::vector<float> goal1{11., 12., 13.};
    std::cout << "Sending goal 1..." << std::endl;
    client.communicateGoal(goal1, 0);

    // Sleep time
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Socket number 2
    std::vector<float> goal2{14., 15., 16.};
    std::cout << "Sending goal 2..." << std::endl;
    client.communicateGoal(goal2, 1);

    // Sleep time
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Socket number 1
    std::vector<float> normal_1;
    int sem_class_1;
    bool collision_1 = client.requestCollisionState(normal_1, sem_class_1, 0);
    std::cout << "Request collision 1..." << std::endl;
    std::cout << "Collision state 1: " << collision_1;
    std::cout << " (Normal: " << normal_1[0] << ", " << normal_1[1] << ", "
              << normal_1[2];
    std::cout << " - Sem Class: " << sem_class_1 << ")" << std::endl;

    // Sleep time
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Socket number 2
    std::vector<float> normal_2;
    int sem_class_2;
    bool collision_2 = client.requestCollisionState(normal_2, sem_class_2, 1);
    std::cout << "Request collision 2..." << std::endl;
    std::cout << "Collision state 2: " << collision_2;
    std::cout << " (Normal: " << normal_2[0] << ", " << normal_2[1] << ", "
              << normal_2[2];
    std::cout << " - Sem Class: " << sem_class_2 << ")" << std::endl;

    // Sleep time
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Socket number 1
    std::cout << "Request shutdown 1..." << std::endl;
    client.requestShutdownSim(0);

    // Sleep time
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Socket number 2
    std::cout << "Request shutdown 2..." << std::endl;
    client.requestShutdownSim(1);

    // Read response image
    cv::Mat image = cv::Mat::zeros(480, 752, CV_8UC1);
    client.receiveImage(image, 0);
    cv::imshow("Response", image);
    cv::waitKey(1000);
    cv::destroyWindow("Response");

    // Read semantics image
    cv::Mat semantics = cv::Mat::zeros(480, 752, CV_8UC1);
    client.receiveSemantics(semantics, 0);
    cv::imshow("Semantics", semantics);
    cv::waitKey(1000);
    cv::destroyWindow("Semantics");

    // Send environment ID
    std::uniform_int_distribution<int> uniform_int_dist_resets(0, 5);
    const auto sample = uniform_int_dist_resets(random_gen);
    client.communicateEnvId(sample, 0);
    std::cout << "Communicating new environment id: " << sample << std::endl;

    ++iter;
  }

  // Exit
  return 0;
}
