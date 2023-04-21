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

#ifndef __SERVER_COMM_H__
#define __SERVER_COMM_H__

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

struct CollisionState {
  CollisionState() : collision(false), normal({0, 0, 0}), sem_class(-1) {}

  bool collision;
  std::vector<float> normal;
  int sem_class;
};

namespace socket_comm {
class Server {
  public:
  Server(const int img_width = 480, const int img_height = 752,
         const bool verbose = false)
      : image_data_(cv::Mat::zeros(img_width, img_height, CV_8UC1)),
        confidence_data_(cv::Mat::zeros(img_width, img_height, CV_8UC1)),
        semantic_data_(cv::Mat::zeros(img_width, img_height, CV_8UC1)),
        timeout_action_(100.0),  //[ms]
        stop_(false),
        shutdown_sim_(false),
        reset_sim_(false),
        reset_status_(0),
        heartbit_(false),
        goal_collision_(false),
        verbose_(verbose) {}
  ~Server() {
    std::cout << "[Server]: Closing the server!" << std::endl;
    close(server__socket_id_);
  }

  inline void updateImage(const cv::Mat& image) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    image_data_ = image;
  }

  inline void updateConfidence(const cv::Mat& image) {
    std::lock_guard<std::mutex> lock(confidence_mutex_);
    confidence_data_ = image;
  }

  inline void updateSemantics(const cv::Mat& semantics) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    semantic_data_ = semantics;
  }

  inline std::vector<float> getActionVector() {
    std::lock_guard<std::mutex> lock(action_mutex_);
    return action_vec_;
  }

  inline void clearActionVector() {
    std::lock_guard<std::mutex> lock(action_mutex_);
    action_vec_.clear();
  }

  inline void clearActionVectorTime() {
    auto time = std::chrono::high_resolution_clock::now();
    std::lock_guard<std::mutex> lock(action_mutex_);
    if (std::chrono::duration_cast<std::chrono::milliseconds>(time -
                                                              age_action_)
            .count() > timeout_action_) {
      action_vec_.clear();
    }
  }

  inline void setTimeoutAction(const double timeout) {
    std::lock_guard<std::mutex> lock(action_mutex_);
    timeout_action_ = timeout;
  }

  inline std::vector<float> getGoalVector() {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    return goal_vec_;
  }

  inline void stop() {
    std::lock_guard<std::mutex> lock(params_mutex_);
    stop_ = true;
  }

  inline void start() {
    std::lock_guard<std::mutex> lock(params_mutex_);
    stop_ = false;
  }

  inline void setVerbose(const bool verbose) {
    std::lock_guard<std::mutex> lock(params_mutex_);
    verbose_ = verbose;
  }

  inline bool needSimReset() {
    std::lock_guard<std::mutex> lock(params_mutex_);
    return reset_sim_;
  }

  inline bool shutdownSim() {
    std::lock_guard<std::mutex> lock(params_mutex_);
    return shutdown_sim_;
  }

  inline void setSimReset(const bool state) {
    std::lock_guard<std::mutex> lock(params_mutex_);
    reset_sim_ = state;
  }

  inline void setResetStatus(const int state) {
    std::lock_guard<std::mutex> lock(params_mutex_);
    reset_status_ = state;
  }

  inline void heartbitAlive(const bool alive) {
    std::lock_guard<std::mutex> lock(params_mutex_);
    heartbit_ = alive;
  }

  inline void setCollisionState(const bool collision) {
    std::lock_guard<std::mutex> lock(params_mutex_);
    collision_state_.collision = collision;
    collision_state_.normal = std::vector<float>{0, 0, 0};  // fake
    collision_state_.sem_class = -1;                        // unknown
  }

  inline void setCollisionState(const bool collision,
                                const std::vector<float>& normal,
                                const int semantic_class) {
    std::lock_guard<std::mutex> lock(params_mutex_);
    collision_state_.collision = collision;
    collision_state_.normal = normal;
    collision_state_.sem_class = semantic_class;
  }

  inline void setCollisionGoal(const bool collision) {
    std::lock_guard<std::mutex> lock(params_mutex_);
    goal_collision_ = collision;
  }

  inline void updateState(const std::vector<float>& state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_vec_ = state;
  }

  inline void setEnvId(const int env_id) {
    std::lock_guard<std::mutex> lock(env_id_mutex_);
    env_id_ = env_id;
  }

  inline int getEnvId() {
    std::lock_guard<std::mutex> lock(env_id_mutex_);
    return env_id_;
  }

  void advertise(int port = 12334) {
    server__socket_id_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server__socket_id_ < 0) {
      std::cout << "[Server]: ERROR establishing socket\n" << std::endl;
      return;
    }

    struct sockaddr_in serv_addr, cli_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port);

    if (bind(server__socket_id_, (struct sockaddr*)&serv_addr,
             sizeof(serv_addr)) < 0) {
      std::cout << "[Server]: ERROR binding" << std::endl;
      return;
    }

    std::vector<std::unique_ptr<std::thread>> tids;
    listen(server__socket_id_, 5);
    std::vector<int> server__socket_connected;
    while (!stop_) {
      socklen_t clilen = sizeof(cli_addr);
      server__socket_connected.push_back(
          accept(server__socket_id_, (struct sockaddr*)&cli_addr, &clilen));
      if (server__socket_connected.back() < 0) {
        std::cout << "[Server]: ERROR new socket connection" << std::endl;
        break;
      }

      std::cout << "[Server]: New connection with port " << port << std::endl;
      tids.push_back(
          std::make_unique<std::thread>(&Server::callbackClient, this,
                                        server__socket_connected.back(), port));
      tids.back()->detach();
    }
    // Close socket
    for (const auto& s : server__socket_connected)
      close(s);

    std::cout << "[Server]: Shutting down" << std::endl;
    return;
  }

  private:
  void callbackClient(const int& socket_s, const int& port) {
    std::cout << "[Server]: Create new socket (ID " << port << ")" << std::endl;
    while (!stop_) {
      // Receive encoding
      char encoding[4];
      if (recv(socket_s, &(encoding[0]), 4, 0) <= 0) {
        // std::cout << "[Server]: Error reading encoding!" << std::endl;
        // std::cout << "[Server]: Read encoding: " << encoding << std::endl;
        continue;
      }
      if (verbose_)
        std::cout << "[Server " << port
                  << "]: Read encoding: " << std::string(encoding) << std::endl;

      if (strcmp(encoding, "STRI") == 0) {
        if (verbose_)
          std::cout << "[Server " << port << "]: Received string" << std::endl;
        std::string message;
        if (!readString(socket_s, message))
          break;
      } else if (strcmp(encoding, "ARRA") == 0) {
        if (verbose_)
          std::cout << "[Server " << port << "]: Received vector" << std::endl;
        if (!readVectorFloat(socket_s))
          break;

        if (verbose_) {
          std::cout << "[Server " << port << "]: Vector ";
          for (const auto& v : action_vec_) {
            std::cout << v << " ";
          }
          std::cout << std::endl;
        }
      } else if (strcmp(encoding, "REQU") == 0) {
        if (verbose_)
          std::cout << "[Server " << port << "]: Received image request"
                    << std::endl;
        // Send result image as response
        sendImage(socket_s);
      } else if (strcmp(encoding, "SEMA") == 0) {
        if (verbose_)
          std::cout << "[Server " << port << "]: Received sem image request"
                    << std::endl;
        // Send result image as response
        sendSemantics(socket_s);
      } else if (strcmp(encoding, "CONF") == 0) {
        if (verbose_)
          std::cout << "[Server " << port << "]: Received conf image request"
                    << std::endl;
        // Send result image as response
        sendConfidence(socket_s);
      } else if (strcmp(encoding, "STAT") == 0) {
        if (verbose_)
          std::cout << "[Server " << port << "]: Received state request"
                    << std::endl;
        sendState(socket_s);
      } else if (strcmp(encoding, "GOAL") == 0) {
        if (verbose_)
          std::cout << "[Server " << port << "]: Received goal vector"
                    << std::endl;
        if (!readVectorGoalFloat(socket_s))
          break;

        if (verbose_) {
          std::cout << "[Server " << port << "]: Vector ";
          for (const auto& g : goal_vec_) {
            std::cout << g << " ";
          }
          std::cout << std::endl;
        }
      } else if (strcmp(encoding, "RESE") == 0) {
        if (verbose_)
          std::cout << "[Server " << port
                    << "]: Received reset request for the simulation"
                    << std::endl;
        std::lock_guard<std::mutex> lock(params_mutex_);
        reset_sim_ = true;
      } else if (strcmp(encoding, "REST") == 0) {
        if (verbose_)
          std::cout << "[Server " << port << "]: Received reset state request"
                    << std::endl;
        sendResetStatus(socket_s);
      } else if (strcmp(encoding, "ENVI") == 0) {
        if (verbose_)
          std::cout << "[Server " << port
                    << "]: Received request to change environment" << std::endl;
        if (!readEnvId(socket_s))
          break;

        if (verbose_) {
          std::lock_guard<std::mutex> lock(env_id_mutex_);
          std::cout << "[Server " << port << "]: Env id " << env_id_
                    << std::endl;
        }

      } else if (strcmp(encoding, "SHUT") == 0) {
        if (verbose_)
          std::cout << "[Server " << port
                    << "]: Received shutdown request for the simulation"
                    << std::endl;
        std::lock_guard<std::mutex> lock(params_mutex_);
        shutdown_sim_ = true;
      } else if (strcmp(encoding, "COLL") == 0) {
        if (verbose_)
          std::cout << "[Server " << port
                    << "]: Received reset request for collision checks"
                    << std::endl;
        sendCollisionStatus(socket_s);
      } else if (strcmp(encoding, "HEAR") == 0) {
        if (verbose_)
          std::cout << "[Server " << port
                    << "]: Received reset request for heartbit" << std::endl;
        sendHeartbit(socket_s);
      }
    }

    std::cout << "[Server " << port << "]: Close thread socket" << std::endl;
    close(socket_s);
  }

  bool readString(const int& socket, std::string& message) {
    char message_lenght[16];
    if (recv(socket, &message_lenght, 16, 0) <= 0) {
      if (verbose_)
        std::cout << "[Server]: Error reading message length!" << std::endl;
      return false;
    }

    char message_socket[256];
    int message_lenght_int;
    try {
      message_lenght_int = std::stoi(message_lenght);
    } catch (const std::invalid_argument& ex) {
      std::cout << "[Server]: Failed to read message dimension: " << ex.what()
                << std::endl;
      return false;
    }
    if (recv(socket, &message_socket, message_lenght_int, 0) > 0) {
      if (verbose_)
        std::cout << "[Server]: Received message: " << message_socket
                  << std::endl;
      message = message_socket;
      return true;
    }

    return false;
  }

  bool readVectorFloat(const int& socket) {
    char message_lenght[16];
    if (recv(socket, &message_lenght, 16, 0) <= 0) {
      if (verbose_)
        std::cout << "[Server]: Error reading message length!" << std::endl;
      return false;
    }
    int message_lenght_int;
    try {
      message_lenght_int = std::stoi(message_lenght);
    } catch (const std::invalid_argument& ex) {
      std::cout << "[Server]: Failed to read message dimension: " << ex.what()
                << std::endl;
      return false;
    }
    if (verbose_)
      std::cout << "[Server]: Message lenght: " << message_lenght_int
                << std::endl;

    // Read array values
    std::array<char, 1024> msgbuf;
    float val;
    char data[sizeof(float)];
    int carryover = 0, bytes = 0;

    std::lock_guard<std::mutex> lock(action_mutex_);
    action_vec_.clear();
    do {
      int b =
          recv(socket, &msgbuf[carryover], message_lenght_int - carryover, 0);
      if (b <= 0) {
        if (verbose_)
          std::cout << "[Server]: Error reading vector message!" << std::endl;
        return false;
      }
      bytes += b;
      const char* mp = &msgbuf[0];
      while (b >= sizeof(float)) {
        char* bp = data;
        for (int i = 0; i < sizeof(float); ++i) {
          *bp++ = *mp++;
        }
        std::memcpy(&val, data, sizeof(float));
        action_vec_.push_back(val);
        b -= sizeof(float);
      }
      carryover = b % sizeof(float);
      for (int j = 0; j < carryover; ++j) {
        msgbuf[j] = *mp++;
      }
    } while (bytes < message_lenght_int);

    // Log the time this action was created
    age_action_ = std::chrono::high_resolution_clock::now();
    return true;
  }

  bool readVectorGoalFloat(const int& socket) {
    char message_lenght[16];
    if (recv(socket, &message_lenght, 16, 0) <= 0) {
      if (verbose_)
        std::cout << "[Server]: Error reading message length!" << std::endl;
      return false;
    }
    int message_lenght_int;
    try {
      message_lenght_int = std::stoi(message_lenght);
    } catch (const std::invalid_argument& ex) {
      std::cout << "[Server]: Failed to read message dimension: " << ex.what()
                << std::endl;
      return false;
    }
    if (verbose_)
      std::cout << "[Server]: Message lenght: " << message_lenght_int
                << std::endl;

    // Read array values
    std::array<char, 1024> msgbuf;
    float val;
    char data[sizeof(float)];
    int carryover = 0, bytes = 0;

    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal_vec_.clear();
    do {
      int b =
          recv(socket, &msgbuf[carryover], message_lenght_int - carryover, 0);
      if (b <= 0) {
        if (verbose_)
          std::cout << "[Server]: Error reading vector message!" << std::endl;
        return false;
      }
      bytes += b;
      const char* mp = &msgbuf[0];
      while (b >= sizeof(float)) {
        char* bp = data;
        for (int i = 0; i < sizeof(float); ++i) {
          *bp++ = *mp++;
        }
        std::memcpy(&val, data, sizeof(float));
        goal_vec_.push_back(val);
        b -= sizeof(float);
      }
      carryover = b % sizeof(float);
      for (int j = 0; j < carryover; ++j) {
        msgbuf[j] = *mp++;
      }
    } while (bytes < message_lenght_int);

    return true;
  }

  bool readEnvId(const int& socket) {
    char message_lenght[16];
    if (recv(socket, &message_lenght, 16, 0) <= 0) {
      if (verbose_)
        std::cout << "[Server]: Error reading message length!" << std::endl;
      return false;
    }
    int message_lenght_int;
    try {
      message_lenght_int = std::stoi(message_lenght);
    } catch (const std::invalid_argument& ex) {
      std::cout << "[Server]: Failed to read message dimension: " << ex.what()
                << std::endl;
      return false;
    }
    if (verbose_)
      std::cout << "[Server]: Message lenght: " << message_lenght_int
                << std::endl;

    // Read array values
    std::array<char, 1024> msgbuf;
    int val;
    char data[sizeof(int)];
    int carryover = 0, bytes = 0;

    std::lock_guard<std::mutex> lock(env_id_mutex_);
    do {
      int b =
          recv(socket, &msgbuf[carryover], message_lenght_int - carryover, 0);
      if (b <= 0) {
        if (verbose_)
          std::cout << "[Server]: Error reading vector message!" << std::endl;
        return false;
      }
      bytes += b;
      const char* mp = &msgbuf[0];
      while (b >= sizeof(int)) {
        char* bp = data;
        for (int i = 0; i < sizeof(int); ++i) {
          *bp++ = *mp++;
        }
        std::memcpy(&val, data, sizeof(int));
        env_id_ = val;
        b -= sizeof(int);
      }
      carryover = b % sizeof(int);
      for (int j = 0; j < carryover; ++j) {
        msgbuf[j] = *mp++;
      }
    } while (bytes < message_lenght_int);

    return true;
  }

  void sendImage(const int& socket) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    int msg_size = image_data_.total() * image_data_.elemSize();

    // Send message size
    std::ostringstream ss;
    ss << std::setw(size_message_length_) << std::setfill('0') << msg_size
       << "\n";
    send(socket, ss.str().c_str(), size_message_length_, 0);  // header

    // Send image
    send(socket, (uchar*)(&image_data_.data[0]), msg_size, 0);

    if (verbose_ || msg_size == 0)
      std::cout << "[Server]: Sent " << msg_size << " bytes for image"
                << std::endl;
  }

  void sendConfidence(const int& socket) {
    std::lock_guard<std::mutex> lock(confidence_mutex_);
    int msg_size = confidence_data_.total() * confidence_data_.elemSize();

    // Send message size
    std::ostringstream ss;
    ss << std::setw(size_message_length_) << std::setfill('0') << msg_size
       << "\n";
    send(socket, ss.str().c_str(), size_message_length_, 0);  // header

    // Send image
    send(socket, (uchar*)(&confidence_data_.data[0]), msg_size, 0);

    if (verbose_ || msg_size == 0)
      std::cout << "[Server]: Sent " << msg_size << " bytes for conf image"
                << std::endl;
  }

  void sendSemantics(const int& socket) {
    std::lock_guard<std::mutex> lock(semantic_mutex_);
    int msg_size = semantic_data_.total() * semantic_data_.elemSize();

    // Send message size
    std::ostringstream ss;
    ss << std::setw(size_message_length_) << std::setfill('0') << msg_size
       << "\n";
    send(socket, ss.str().c_str(), size_message_length_, 0);  // header

    // Send semantic image
    send(socket, (uchar*)(&semantic_data_.data[0]), msg_size, 0);

    if (verbose_ || msg_size == 0)
      std::cout << "[Server]: Sent " << msg_size << " bytes for semantics"
                << std::endl;
  }

  void sendState(const int& socket) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // for debugging
    if (state_vec_.empty()) {
      std::cout << "[Server]: State is empty!" << std::endl;
      return;
    }

    int msg_size = sizeof(state_vec_[0]) * state_vec_.size();
    send(socket, state_vec_.data(), msg_size, 0);
  }

  void sendResetStatus(const int& socket) {
    std::lock_guard<std::mutex> lock(params_mutex_);
    std::vector<float> status_reset;
    if (reset_status_)
      status_reset.push_back(1);
    else
      status_reset.push_back(0);
    int msg_size = sizeof(status_reset[0]) * status_reset.size();
    send(socket, status_reset.data(), msg_size, 0);
  }

  void sendCollisionStatus(const int& socket) {
    std::lock_guard<std::mutex> lock(params_mutex_);

    // FIXME Find better way to do this
    // Fill a fake vector
    std::vector<float> collision_vec;
    collision_vec.push_back(collision_state_.collision ? 1 : 0);
    collision_vec.push_back(collision_state_.normal[0]);
    collision_vec.push_back(collision_state_.normal[1]);
    collision_vec.push_back(collision_state_.normal[2]);
    collision_vec.push_back(static_cast<int>(collision_state_.sem_class));

    int msg_size = sizeof(collision_vec[0]) * collision_vec.size();
    send(socket, collision_vec.data(), msg_size, 0);
  }

  void sendGoalCollisionStatus(const int& socket) {
    std::lock_guard<std::mutex> lock(params_mutex_);

    // FIXME Find better way to do this
    // Fill a fake vector
    std::vector<int> goal_collision_vec;
    goal_collision_vec.push_back(goal_collision_ ? 1 : 0);

    int msg_size = sizeof(goal_collision_vec[0]) * goal_collision_vec.size();
    send(socket, goal_collision_vec.data(), msg_size, 0);
  }

  void sendHeartbit(const int& socket) {
    std::lock_guard<std::mutex> lock(params_mutex_);

    std::vector<float> heartbit;
    heartbit.push_back(heartbit_ ? 1 : 0);
    heartbit_ = false;

    int msg_size = sizeof(heartbit[0]) * heartbit.size();
    send(socket, heartbit.data(), msg_size, 0);
  }

  int server__socket_id_;
  const int size_message_length_ = 16;  // Buffer size for the length

  std::mutex image_mutex_;
  cv::Mat image_data_;

  std::mutex confidence_mutex_;
  cv::Mat confidence_data_;

  std::mutex semantic_mutex_;
  cv::Mat semantic_data_;

  std::mutex action_mutex_;
  std::vector<float> action_vec_;
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      age_action_;
  double timeout_action_;

  std::mutex state_mutex_;
  std::vector<float> state_vec_;

  std::mutex goal_mutex_;
  std::vector<float> goal_vec_;

  std::mutex env_id_mutex_;
  int env_id_;

  std::mutex params_mutex_;
  bool stop_;
  bool shutdown_sim_;
  bool reset_sim_;
  int reset_status_;  // 0: fail, 1: changing, 2: ready
  bool heartbit_;     // 0: dead, 1: alive

  CollisionState collision_state_;
  bool goal_collision_;

  bool verbose_{false};
};

// Useful typedef
typedef std::shared_ptr<Server> ServerPtr;

}  // namespace socket_comm

#endif