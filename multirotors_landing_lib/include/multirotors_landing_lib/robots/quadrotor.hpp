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
 *
 * This implementation is mostly taken from Flightmare.
 *
 *********************************************************************/

#ifndef __QUADROTOR_HPP__
#define __QUADROTOR_HPP__

#include <stdlib.h>

#include "multirotors_landing_lib/dynamics/dynamics_quadrotor.hpp"
#include "multirotors_landing_lib/robots/robot_base.hpp"

#define QUAD_STATE_SIZE 10

namespace ml {

/**
 * @brief Disturbance that can act on the quadrotor
 */
struct Disturbance {
  Vector<3> force;
  Vector<3> momentum;
};

/**
 * @brief Commands going into the quadrotor
 */
struct Command {
  Scalar force[3];
  Scalar qx, qy, qz, qw;
  Scalar kR[3];
  Scalar kOm[3];
  Scalar corrections[3];
  Scalar current_yaw;
  bool use_external_yaw;
};

class Quadrotor final : public RobotBase<QuadState> {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief Empty constructor
   */
  Quadrotor();

  /**
   * @brief Constructor using a configuration file
   * @param[in] cfg_path path to configuration file
   */
  Quadrotor(const std::string& cfg_path);

  /**
   * @brief Destructor
   */
  ~Quadrotor() override;

  /**
   * @brief Resets functions
   * @return True if reset was successful, False otherwise
   */
  bool reset() override;
  bool reset(const QuadState& state);

  /**
   * @brief Initialization function
   */
  void init();

  /**
   * @brief Runs the quadrotor for dt time
   * @param[in] dt Time to run the quadrotor
   * @return True if successful, False otherwise
   */
  bool run(const Scalar dt) override;

  /**
   * @brief Runs the quadrotor for dt time given a command
   * @param[in] cmd Input command (speed of the rotors)
   * @param[in] dt Time to run the quadrotor
   * @return True if successful, False otherwise
   */
  bool run(const Eigen::Array4f& cmd, const Scalar dt);

  /**
   * @brief Getters
   */
  QuadState getState() const;
  bool isResetState() const;

  Scalar getCurrentYaw() const;
  const Vector<3>& getAcc() const;

  /**
   * @brief Setters
   */
  void setState(const QuadState& state);
  void setStatePos(const Vector<3>& pos);
  bool setResetState(const bool is_reset);

  // constrain world box
  bool setWorldBox(const Ref<Matrix<3, 2>> box);

  //
  inline Scalar getMass() {
    return dynamics_.getMass();
  }
  inline void setCollision(const bool collision) {
    collision_ = collision;
  }
  inline const Vector<3>& getKx() const {
    return kx_;
  }
  inline const Vector<3>& getKv() const {
    return kv_;
  }
  inline const Eigen::Array3f getKr() const {
    return Eigen::Array3f(kR_[0], kR_[1], kR_[2]);
  }
  inline const Eigen::Array3f getKom() const {
    return Eigen::Array3f(kOm_[0], kOm_[1], kOm_[2]);
  }
  inline const Eigen::Array3f getCorrections() const {
    return Eigen::Array3f(corrections_[0], corrections_[1], corrections_[2]);
  }

  // Dynamics-related
  void setInput(const Scalar input[4]);
  void setInput(const Scalar u0, const Scalar u1, const Scalar u2,
                const Scalar u3);

  inline const Vector<3>& getExternalForce() const {
    return dynamics_.getExternalForce();
  }
  inline void setExternalForce(const Vector<3>& force) {
    dynamics_.setExternalForce(force);
  }

  inline const Vector<3>& getExternalMoment() const {
    return dynamics_.getExternalMoment();
  }
  inline void setExternalMoment(const Vector<3>& moment) {
    dynamics_.setExternalMoment(moment);
  }

  inline Scalar getPropellerThrustCoefficient() const {
    return dynamics_.getPropellerThrustCoefficient();
  }

  inline Scalar getPropellerMomentCoefficient() const {
    return dynamics_.getPropellerMomentCoefficient();
  }

  inline Scalar getArmLength() const {
    return dynamics_.getArmLength();
  }

  inline const Matrix<3, 3>& getInertia() const {
    return dynamics_.getInertia();
  }

  inline bool inCollision() const {
    return collision_;
  }

  /**
   * @brief Integrates the dynamics for dt time
   * @param[in] dt Integration time
   */
  void step(Scalar dt) {
    dynamics_.step(dt);

    // Update state
    robot_state_ = dynamics_.getState();
  }

  private:
  // Get the control to the motors (RPM) from command
  void getControlFromCommand(const Command& cmd);

  Command getCommandFromDesiredPose(const QuadState& des_state,
                                    const Vector<3>& des_acc);

  // quadrotor dynamics, integrators
  DynamicsQuadrotor dynamics_;
  Vector<3> kx_, kv_;

  // quad state
  bool collision_;
  bool is_reset_state_;

  // Parameters
  Scalar corrections_[3];  //<! correction to angle fed to controller
  Scalar kR_[3];           //<! gain on rotation
  Scalar kOm_[3];          //<! gain on angle

  // auxiliary variables
  Matrix<3, 2> world_box_;
};

// Typedefs
typedef std::shared_ptr<Quadrotor> QuadrotorPtr;

}  // end namespace ml

#endif  // __QUADROTOR_HPP__
