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

#ifndef __CLIENT_COMM_H__
#define __CLIENT_COMM_H__

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace socket_comm {
class Client {
  public:
  Client(const bool verbose = false) : verbose_(verbose) {}
  ~Client() {
    close(client__socket_id_);
  }

  bool init(const std::string ip = "127.0.0.1", int port = 12334) {
    client__socket_id_ = socket(AF_INET, SOCK_STREAM, 0);
    if (client__socket_id_ < 0) {
      std::cout << "[Client]: ERROR establishing socket\n" << std::endl;
      return false;
    }

    // Add timeout interval
    struct timeval tv;
    const double timeout_in_sec = 1;
    tv.tv_sec = timeout_in_sec;
    tv.tv_usec = 0;
    setsockopt(client__socket_id_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,
               sizeof tv);

    bool connected = false;
    int connection_attempts = 5;

    while ((!connected) && (connection_attempts > 0)) {
      struct sockaddr_in serv_addr;
      serv_addr.sin_family = AF_INET;
      serv_addr.sin_port = htons(port);
      inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr);

      if (connect(client__socket_id_, (const struct sockaddr*)&serv_addr,
                  sizeof(serv_addr)) == 0) {
        connected = true;
        std::cout << "[Client]: Socket client connected (port: " << port << ")."
                  << std::endl;
      } else {
        sleep(1);
        connection_attempts -= 1;
        std::cout << "[Client]: Error connecting to " << ip << ":" << port
                  << std::endl;
      }
    }
    return connected;
  }

  void sendImage(const cv::Mat& img) {
    int msg_size = img.total() * img.elemSize();

    std::ostringstream ss;
    ss << std::setw(size_message_length_) << std::setfill('0') << msg_size
       << "\n";

    send(client__socket_id_, "L83F", 4, 0);  // header
    send(client__socket_id_, ss.str().c_str(), size_message_length_, 0);
    send(client__socket_id_, (uchar*)(&img.data[0]), msg_size, 0);
  }

  void sendVectorFloat(const std::vector<float>& vec,
                       const std::string& header = "ARRA") {
    int msg_size = sizeof(vec[0]) * vec.size();

    std::ostringstream ss;
    ss << std::setw(size_message_length_) << std::setfill('0') << msg_size
       << "\n";

    send(client__socket_id_, header.c_str(), 4, 0);  // header
    send(client__socket_id_, ss.str().c_str(), size_message_length_, 0);
    send(client__socket_id_, vec.data(), msg_size, 0);
  }

  void sendEnvId(const int env_id, const std::string& header = "ENVI") {
    // Convert to std vector of int (1 int)
    std::vector<int> env_id_vec{env_id};

    // Send request
    int msg_size = sizeof(env_id_vec[0]) * env_id_vec.size();

    std::ostringstream ss;
    ss << std::setw(size_message_length_) << std::setfill('0') << msg_size
       << "\n";

    send(client__socket_id_, header.c_str(), 4, 0);  // header
    send(client__socket_id_, ss.str().c_str(), size_message_length_, 0);
    send(client__socket_id_, env_id_vec.data(), msg_size, 0);
  }

  void sendString(const std::string& string) {
    int msg_size = sizeof(string[0]) * string.size();

    std::ostringstream ss;
    ss << std::setw(size_message_length_) << std::setfill('0') << msg_size
       << "\n";

    send(client__socket_id_, "STRI", 4, 0);  // header
    send(client__socket_id_, ss.str().c_str(), size_message_length_, 0);
    send(client__socket_id_, string.data(), msg_size, 0);
  }

  bool requestResultImage(cv::Mat& img) {
    // Send request first
    std::ostringstream ss;
    ss << std::setw(size_message_length_) << std::setfill('0') << 4 << "\n";
    send(client__socket_id_, "REQU", 4, 0);  // header

    // Receive response
    char message_lenght[size_message_length_];
    int msg_bytes_size =
        recv(client__socket_id_, &message_lenght, size_message_length_, 0);
    int message_lenght_int;
    try {
      message_lenght_int = std::stoi(message_lenght);
    } catch (const std::invalid_argument& ex) {
      std::cout << "[Client]: Failed to read message dimension (result image): "
                << ex.what() << std::endl;
      return false;
    }
    if (message_lenght_int == 0) {
      std::cerr << "Trying to read an empty image, skipping it!" << std::endl;
      return false;
    }

    // Receive response
    int imgSize = img.total() * img.elemSize();
    uchar* iptr = img.data;
    int bytes = 0;
    if (verbose_)
      std::cout << "Image Size:" << imgSize << std::endl;
    if ((bytes = recv(client__socket_id_, iptr, imgSize, MSG_PEEK)) < 0) {
      std::cout << "Peek failed!" << std::endl;
      return false;
    } else if (verbose_) {
      std::cout << "Peek saw " << bytes << " bytes" << std::endl;
    }

    if ((bytes = recv(client__socket_id_, iptr, imgSize, MSG_WAITALL)) == -1) {
      if (verbose_)
        std::cerr << "recv failed, received bytes = " << bytes << std::endl;
      return false;
    } else if (verbose_) {
      std::cerr << "recv success, received bytes = " << bytes << std::endl;
    }
    return true;
  }

  bool requestSemanticsImage(cv::Mat& img) {
    // Send request first
    std::ostringstream ss;
    ss << std::setw(size_message_length_) << std::setfill('0') << 4 << "\n";
    send(client__socket_id_, "SEMA", 4, 0);  // header

    // Receive response
    char message_lenght[size_message_length_];
    int msg_bytes_size =
        recv(client__socket_id_, &message_lenght, size_message_length_, 0);

    int message_lenght_int;
    try {
      message_lenght_int = std::stoi(message_lenght);
    } catch (const std::invalid_argument& ex) {
      std::cout
          << "[Client]: Failed to read message dimension (semantic image): "
          << ex.what() << std::endl;
      return false;
    }
    if (message_lenght_int == 0) {
      std::cerr << "Trying to read an empty image, skipping it!" << std::endl;
      return false;
    }

    // Image
    int imgSize = img.total() * img.elemSize();
    uchar* iptr = img.data;
    int bytes = 0;
    if (verbose_)
      std::cout << "Image Size:" << imgSize << std::endl;
    if ((bytes = recv(client__socket_id_, iptr, imgSize, MSG_PEEK)) < 0) {
      std::cout << "Peek failed!" << std::endl;
      return false;
    } else if (verbose_) {
      std::cout << "Peek saw " << bytes << " bytes" << std::endl;
    }
    if ((bytes = recv(client__socket_id_, iptr, imgSize, MSG_WAITALL)) == -1) {
      if (verbose_)
        std::cerr << "recv failed, received bytes = " << bytes << std::endl;
      return false;
    } else if (verbose_) {
      std::cerr << "recv success, received bytes = " << bytes << std::endl;
    }
    return true;
  }

  bool requestConfidenceImage(cv::Mat& img) {
    // Send request first
    std::ostringstream ss;
    ss << std::setw(size_message_length_) << std::setfill('0') << 4 << "\n";
    send(client__socket_id_, "CONF", 4, 0);  // header

    // Receive response
    char message_lenght[size_message_length_];
    int msg_bytes_size =
        recv(client__socket_id_, &message_lenght, size_message_length_, 0);
    int message_lenght_int;
    try {
      message_lenght_int = std::stoi(message_lenght);
    } catch (const std::invalid_argument& ex) {
      std::cout
          << "[Client]: Failed to read message dimension (confidence image): "
          << ex.what() << std::endl;
      return false;
    }
    if (message_lenght_int == 0) {
      std::cerr << "Trying to read an empty image, skipping it!" << std::endl;
      return false;
    }

    // Receive response
    int imgSize = img.total() * img.elemSize();
    uchar* iptr = img.data;
    int bytes = 0;
    if (verbose_)
      std::cout << "Image Size:" << imgSize << std::endl;
    if ((bytes = recv(client__socket_id_, iptr, imgSize, MSG_PEEK)) < 0) {
      std::cout << "Peek failed!" << std::endl;
      return false;
    } else if (verbose_) {
      std::cout << "Peek saw " << bytes << " bytes" << std::endl;
    }

    if ((bytes = recv(client__socket_id_, iptr, imgSize, MSG_WAITALL)) == -1) {
      if (verbose_)
        std::cerr << "recv failed, received bytes = " << bytes << std::endl;
      return false;
    } else if (verbose_) {
      std::cerr << "recv success, received bytes = " << bytes << std::endl;
    }
    return true;
  }

  bool requestSimReset() {
    send(client__socket_id_, "RESE", 4, 0);  // header
    return true;
  }

  int responseSimReset() {
    // Send request first
    send(client__socket_id_, "REST", 4, 0);  // header (REset STate)

    // Receive response
    const int message_size = 1;
    int vecSize = message_size *
                  sizeof(float);  // we know we'll receive a bunch of floats
    float reset_state;
    // Read array values
    std::array<char, 1024> msgbuf;
    char data[sizeof(float)];
    int carryover = 0, bytes = 0;
    do {
      int b =
          recv(client__socket_id_, &msgbuf[carryover], vecSize - carryover, 0);
      if (b <= 0) {
        if (verbose_)
          std::cout << "[Client]: Error reading reset state message!"
                    << std::endl;
        return false;
      }
      bytes += b;
      const char* mp = &msgbuf[0];
      while (b >= sizeof(float)) {
        char* bp = data;
        for (int i = 0; i < sizeof(float); ++i) {
          *bp++ = *mp++;
        }
        std::memcpy(&reset_state, data, sizeof(float));
        b -= sizeof(float);
      }
      carryover = b % sizeof(float);
      for (int j = 0; j < carryover; ++j) {
        msgbuf[j] = *mp++;
      }
    } while (bytes < vecSize);

    // Post-process val
    if (sizeof(reset_state) > vecSize) {
      std::cerr << "[Client]: Weird reset state value received! Size: "
                << sizeof(reset_state) << " bytes" << std::endl;
      return false;
    }

    // Here we just check the reset state
    return static_cast<int>(reset_state);  // if 1, then valid reset state
  }

  bool requestShutdownSim() {
    send(client__socket_id_, "SHUT", 4, 0);  // header
    return true;
  }

  bool requestCollisionState(std::vector<float>& normal, int& sem_class) {
    // Send request first
    send(client__socket_id_, "COLL", 4, 0);  // header

    // Receive response
    const int message_size = 5;
    int vecSize = message_size *
                  sizeof(float);  // we know we'll receive a bunch of floats
    std::vector<float> collision;
    // Read array values
    std::array<char, 1024> msgbuf;
    float val;
    char data[sizeof(float)];
    int carryover = 0, bytes = 0;
    do {
      int b =
          recv(client__socket_id_, &msgbuf[carryover], vecSize - carryover, 0);
      if (b <= 0) {
        if (verbose_)
          std::cout << "[Client]: Error reading collision message!"
                    << std::endl;
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
        collision.push_back(val);
        b -= sizeof(float);
      }
      carryover = b % sizeof(float);
      for (int j = 0; j < carryover; ++j) {
        msgbuf[j] = *mp++;
      }
    } while (bytes < vecSize);

    // Post-process val
    if (collision.size() > message_size) {
      std::cerr << "[Client]: Weird collision value received!" << std::endl;
      return false;
    }

    // Store output
    normal.resize(3);
    normal[0] = collision[1];
    normal[1] = collision[2];
    normal[2] = collision[3];
    sem_class = static_cast<int>(collision.back());

    // Here we just check if we are in collision
    return static_cast<int>(collision[0]) == 1;  // if 1, then we are colliding
  }

  bool requestCollisionState() {
    std::vector<float> dummy_normal;
    int dummy_sem_class;
    return requestCollisionState(dummy_normal, dummy_sem_class);
  }

  bool requestState(std::vector<float>& state, const int& state_size) {
    // Send request first
    send(client__socket_id_, "STAT", 4, 0);  // header

    // Receive response
    int vecSize = sizeof(float) * state_size;

    // Read array values
    std::array<char, 1024> msgbuf;
    float val;
    char data[sizeof(float)];
    int carryover = 0, bytes = 0;
    do {
      int b =
          recv(client__socket_id_, &msgbuf[carryover], vecSize - carryover, 0);
      if (b <= 0) {
        if (verbose_)
          std::cout << "[Client]: Error reading vector message!" << std::endl;
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
        state.push_back(val);
        b -= sizeof(float);
      }
      carryover = b % sizeof(float);
      for (int j = 0; j < carryover; ++j) {
        msgbuf[j] = *mp++;
      }
    } while (bytes < vecSize);

    return true;
  }

  bool receiveHeartbit() {
    // Send request first
    send(client__socket_id_, "HEAR", 4, 0);  // header
    // Receive response
    const int message_size = 1;
    int vecSize = message_size *
                  sizeof(float);  // we know we'll receive a bunch of floats
    std::vector<float> heartbit;
    // Read array values
    std::array<char, 128> msgbuf;
    float val;
    char data[sizeof(float)];
    int carryover = 0, bytes = 0;
    do {
      int b =
          recv(client__socket_id_, &msgbuf[carryover], vecSize - carryover, 0);
      if (b <= 0) {
        if (verbose_)
          std::cout << "[Client]: Error reading heartbit message!" << std::endl;
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
        heartbit.push_back(val);
        b -= sizeof(float);
      }
      carryover = b % sizeof(float);
      for (int j = 0; j < carryover; ++j) {
        msgbuf[j] = *mp++;
      }
    } while (bytes < vecSize);

    // Post-process val
    if (heartbit.size() > message_size) {
      std::cerr << "[Client]: Weird heartbit value received!" << std::endl;
      return false;
    }
    // Here we just check if we are alive
    return static_cast<int>(heartbit[0]) == 1;  // if 1, then we are alive
  }

  private:
  int client__socket_id_;
  const int size_message_length_ = 16;  // Buffer size for the length
  const bool verbose_{false};
};

// Useful typedef
typedef std::shared_ptr<Client> ClientPtr;

}  // namespace socket_comm

#endif