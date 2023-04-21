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
 * This implementation is taken from Flightmare.
 *
 *********************************************************************/

#ifndef __DYNAMICS_QUADROTOR_HPP__
#define __DYNAMICS_QUADROTOR_HPP__

#include <yaml-cpp/yaml.h>

#include <boost/array.hpp>

#include "multirotors_landing_lib/dynamics/dynamics_base.hpp"

namespace ml {

struct QuadState {
  Vector<3> x;
  Vector<3> v;
  Matrix<3, 3> R;
  Vector<3> omega;
  Eigen::Array4f motor_rpm;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  QuadState()
      : x(Vector<3>::Zero()),
        v(Vector<3>::Zero()),
        R(Matrix<3, 3>::Identity()),
        omega(Vector<3>::Zero()),
        motor_rpm(Eigen::Array4f::Zero()) {}
};

class DynamicsQuadrotor : DynamicsBase {
  public:
  DynamicsQuadrotor();

  ~DynamicsQuadrotor();

  void reset();

  void setDynamics(const YAML::Node& cfg);

  bool validState() const;

  const QuadState& getState() const;

  void setState(const QuadState& state);

  void setStatePos(const Vector<3>& Pos);

  Scalar getMass(void) const;
  void setMass(Scalar mass);

  Scalar getGravity(void) const;
  void setGravity(Scalar g);

  const Matrix<3, 3>& getInertia() const;
  void setInertia(const Matrix<3, 3>& inertia);

  Scalar getArmLength(void) const;
  void setArmLength(Scalar d);

  Scalar getPropRadius(void) const;
  void setPropRadius(Scalar r);

  Scalar getPropellerThrustCoefficient(void) const;
  void setPropellerThrustCoefficient(Scalar kf);

  Scalar getPropellerMomentCoefficient(void) const;
  void setPropellerMomentCoefficient(Scalar km);

  Scalar getMotorTimeConstant(void) const;
  void setMotorTimeConstant(Scalar k);

  const Vector<3>& getExternalForce(void) const;
  void setExternalForce(const Vector<3>& force);

  const Vector<3>& getExternalMoment(void) const;
  void setExternalMoment(const Vector<3>& moment);

  Scalar getMaxRPM(void) const;
  void setMaxRPM(Scalar max_rpm);

  Scalar getMinRPM(void) const;
  void setMinRPM(Scalar min_rpm);

  // Inputs are desired RPM for the motors
  // Rotor numbering is:
  //   *1*    Front
  // 3     4
  //    2
  // with 1 and 2 clockwise and 3 and 4 counter-clockwise (looking from top)
  void setInput(Scalar u1, Scalar u2, Scalar u3, Scalar u4);
  void setInput(const Scalar control[4]);

  // Runs the actual dynamics simulation with a time step of dt
  void step(Scalar dt);

  // For internal use, but needs to be public for odeint
  typedef boost::array<Scalar, 22> InternalState;
  void operator()(const DynamicsQuadrotor::InternalState& x,
                  DynamicsQuadrotor::InternalState& dxdt, const Scalar /* t */);

  const Vector<3>& getAcc() const;

  private:
  void updateInternalState();

  Scalar alpha0;  // AOA
  Scalar mass_;
  Matrix<3, 3> J_;  // Inertia
  Scalar kf_;
  Scalar km_;
  Scalar prop_radius_;
  Scalar arm_length_;
  Scalar motor_time_constant_;  // unit: sec
  Scalar max_rpm_;
  Scalar min_rpm_;

  QuadState state_;

  Vector<3> acc_;

  Eigen::Array4f cmd_;
  Vector<3> external_force_;
  Vector<3> external_moment_;

  InternalState internal_state_;
};

}  // end namespace ml

#endif  // __DYNAMICS_QUADROTOR_HPP__
