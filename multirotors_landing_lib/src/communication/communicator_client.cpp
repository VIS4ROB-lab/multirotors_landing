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

#include "multirotors_landing_lib/communication/communicator_client.hpp"

#include <thread>

namespace ml {

CommunicatorClient::CommunicatorClient(const size_t &num_sockets,
                                       const bool verbose) {
  for (size_t i = 0; i < num_sockets; ++i) {
    sockets_.push_back(std::make_shared<socket_comm::Client>(verbose));
  }
}

CommunicatorClient::~CommunicatorClient() {}

bool CommunicatorClient::initializeCommunication(const std::string ip,
                                                 const int port) const {
  // IP and port for socket connection with server
  // Actual initialization of the sockets
  bool success = true;
  int id = 0;
  for (auto &s : sockets_) {
    success &= s->init(ip, port + id);
    ++id;
  }
  return success;
}

bool CommunicatorClient::initializeCommunicationPortId(const int id) const {
  // IP and port for socket connection with server
  const std::string ip = "127.0.0.1";
  const int port = 12334;
  // Actual initialization of the socket
  std::cout
      << "\033[1;33m[Communicator Client] Initializing the evaluation socket! "
         "Assuming there is only 1 socket!\033[0m"
      << std::endl;
  return sockets_.front()->init(ip, port + id);
}

void CommunicatorClient::communicateAction(const Ref<Vector<>> &action,
                                           const unsigned int &id) {
  std::vector<Scalar> action_vec(action.rows());
  for (size_t i = 0; i < action.rows(); ++i) {
    action_vec[i] = action(i);
  }
  communicateAction(action_vec, id);
}

void CommunicatorClient::communicateAction(const std::vector<Scalar> &action,
                                           const unsigned int &id) {
  // Send the action to the simulator
  sockets_[id]->sendVectorFloat(action);
}

void CommunicatorClient::communicateGoal(const Vector<3> &goal,
                                         const unsigned int &id) {
  std::vector<Scalar> goal_vec(3);
  for (size_t i = 0; i < 3; ++i) {
    goal_vec[i] = goal[i];
  }
  communicateGoal(goal_vec, id);
}

void CommunicatorClient::communicateGoal(const std::vector<Scalar> &goal,
                                         const unsigned int &id) {
  if (goal.size() != 3) {
    std::cout << "[Communicator Client]: Trying to send goal with size "
              << goal.size() << " - Abort!" << std::endl;
    return;
  }

  sockets_[id]->sendVectorFloat(goal, "GOAL");
}

bool CommunicatorClient::receiveImage(cv::Mat &image, const unsigned int &id) {
  return sockets_[id]->requestResultImage(image);
}

bool CommunicatorClient::receiveSemantics(cv::Mat &image,
                                          const unsigned int &id) {
  return sockets_[id]->requestSemanticsImage(image);
}

bool CommunicatorClient::receiveConfidence(cv::Mat &image,
                                           const unsigned int &id) {
  return sockets_[id]->requestConfidenceImage(image);
}

bool CommunicatorClient::requestReset(const unsigned int &id) const {
  return sockets_[id]->requestSimReset();
}

int CommunicatorClient::responseReset(const unsigned int &id) const {
  return sockets_[id]->responseSimReset();
}

bool CommunicatorClient::requestShutdownSim(const unsigned int &id) const {
  return sockets_[id]->requestShutdownSim();
}

void CommunicatorClient::communicateEnvId(const int env_id,
                                          const unsigned int &id) {
  sockets_[id]->sendEnvId(env_id);
}

bool CommunicatorClient::requestCollisionState(std::vector<float> &normal,
                                               int &sem_class,
                                               const unsigned int &id) const {
  return sockets_[id]->requestCollisionState(normal, sem_class);
}

bool CommunicatorClient::requestCollisionState(const unsigned int &id) const {
  return sockets_[id]->requestCollisionState();
}

bool CommunicatorClient::receiveState(std::vector<float> &state,
                                      const int &state_size,
                                      const unsigned int &id) const {
  return sockets_[id]->requestState(state, state_size);
}

bool CommunicatorClient::receiveHeartbit(const unsigned int &id) const {
  return sockets_[id]->receiveHeartbit();
}

}  // end namespace ml
