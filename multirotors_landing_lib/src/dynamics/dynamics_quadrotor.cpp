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

#include "multirotors_landing_lib/dynamics/dynamics_quadrotor.hpp"

#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <iostream>

// ODE integrators
#include "multirotors_landing_lib/dynamics/ode/boost/numeric/odeint.hpp"

namespace odeint = boost::numeric::odeint;

namespace ml {

DynamicsQuadrotor::DynamicsQuadrotor() {
  alpha0 = 48;    // degree
  mass_ = 0.98f;  // 0.5;
  Scalar Ixx = 2.64e-3f, Iyy = 2.64e-3f, Izz = 4.96e-3f;
  prop_radius_ = 0.062f;
  J_ = Vector<3>(Ixx, Iyy, Izz).asDiagonal();

  kf_ = 8.98132e-9f;
  // km_ = 2.5e-9; // from Nate
  // km = (Cq/Ct)*Dia*kf
  // Cq/Ct for 8 inch props from UIUC prop db ~ 0.07
  km_ = 0.07f * (3 * prop_radius_) * kf_;

  arm_length_ = 0.26f;
  motor_time_constant_ = 1.0f / 30;
  min_rpm_ = 1200;
  max_rpm_ = 35000;

  state_.x = Vector<3>::Zero();
  state_.v = Vector<3>::Zero();
  state_.R = Matrix<3, 3>::Identity();
  state_.omega = Vector<3>::Zero();
  state_.motor_rpm = Eigen::Array4f::Zero();

  external_force_.setZero();

  updateInternalState();

  cmd_ = Eigen::Array4f::Zero();
}

DynamicsQuadrotor::~DynamicsQuadrotor() {}

void DynamicsQuadrotor::reset() {
  state_.x.setZero();
  state_.v.setZero();
  state_.R.setIdentity();
  state_.omega.setZero();
  state_.motor_rpm.setZero();
  cmd_.setZero();
  external_force_.setZero();
  external_moment_.setZero();
}

void DynamicsQuadrotor::setDynamics(const YAML::Node& cfg) {}

bool DynamicsQuadrotor::validState() const {
  bool x_nan = ((state_.x.array() != state_.x.array())).all();
  bool x_infinite =
      ((state_.x - state_.x).array() != (state_.x - state_.x).array()).all();

  bool v_nan = ((state_.v.array() != state_.v.array())).all();
  bool v_infinite =
      ((state_.v - state_.v).array() != (state_.v - state_.v).array()).all();

  bool omega_nan = ((state_.omega.array() != state_.omega.array())).all();
  bool omega_infinite = ((state_.omega - state_.omega).array() !=
                         (state_.omega - state_.omega).array())
                            .all();

  bool motor_rpm_nan =
      ((state_.motor_rpm.array() != state_.motor_rpm.array())).all();
  bool motor_rpm_infinite = ((state_.motor_rpm - state_.motor_rpm).array() !=
                             (state_.motor_rpm - state_.motor_rpm).array())
                                .all();

  bool invalid_state = x_nan || x_infinite || v_nan || v_infinite ||
                       omega_nan || omega_infinite || motor_rpm_nan ||
                       motor_rpm_infinite;
  return !invalid_state;
}

void DynamicsQuadrotor::step(Scalar dt) {
  auto save = internal_state_;
  odeint::integrate(boost::ref(*this), internal_state_, 0.0,
                    static_cast<double>(dt), static_cast<double>(dt));

  for (unsigned int i = 0; i < 22; ++i) {
    if (std::isnan(internal_state_[i])) {
      std::cout << "dump " << i << " << pos ";
      for (unsigned int j = 0; j < 22; ++j) {
        std::cout << save[j] << " ";
      }
      std::cout << std::endl;
      internal_state_ = save;
      break;
    }
  }

  for (unsigned int i = 0u; i < 3u; i++) {
    state_.x(i) = internal_state_[0 + i];
    state_.v(i) = internal_state_[3 + i];
    state_.R(i, 0) = internal_state_[6 + i];
    state_.R(i, 1) = internal_state_[9 + i];
    state_.R(i, 2) = internal_state_[12 + i];
    state_.omega(i) = internal_state_[15 + i];
  }
  state_.motor_rpm(0) = internal_state_[18];
  state_.motor_rpm(1) = internal_state_[19];
  state_.motor_rpm(2) = internal_state_[20];
  state_.motor_rpm(3) = internal_state_[21];

  // Re-orthonormalize R (polar decomposition)
  Eigen::LLT<Matrix<3, 3>> llt(state_.R.transpose() * state_.R);
  Matrix<3, 3> P = llt.matrixL();
  Matrix<3, 3> R = state_.R * P.inverse();
  state_.R = R;

  // Don't go below zero, simulate floor
  if (state_.x(2) < 0.f && state_.v(2) < 0.f) {
    state_.x(2) = 0;
    state_.v(2) = 0;
  }
  updateInternalState();
}

void DynamicsQuadrotor::operator()(const DynamicsQuadrotor::InternalState& x,
                                   DynamicsQuadrotor::InternalState& dxdt,
                                   const Scalar /* t */) {
  QuadState cur_state;
  for (unsigned int i = 0; i < 3; i++) {
    cur_state.x(i) = x[0 + i];
    cur_state.v(i) = x[3 + i];
    cur_state.R(i, 0) = x[6 + i];
    cur_state.R(i, 1) = x[9 + i];
    cur_state.R(i, 2) = x[12 + i];
    cur_state.omega(i) = x[15 + i];
  }
  for (unsigned int i = 0; i < 4; i++) {
    cur_state.motor_rpm(i) = x[18 + i];
  }

  // Re-orthonormalize R (polar decomposition)
  Eigen::LLT<Matrix<3, 3>> llt(cur_state.R.transpose() * cur_state.R);
  Matrix<3, 3> P = llt.matrixL();
  Matrix<3, 3> R = cur_state.R * P.inverse();

  Vector<3> x_dot, v_dot, omega_dot;
  Matrix<3, 3> R_dot;
  Eigen::Array4f motor_rpm_dot;
  Vector<3> vnorm;
  Eigen::Array4f motor_rpm_sq;
  Matrix<3, 3> omega_vee(Matrix<3, 3>::Zero());

  omega_vee(2, 1) = cur_state.omega(0);
  omega_vee(1, 2) = -cur_state.omega(0);
  omega_vee(0, 2) = cur_state.omega(1);
  omega_vee(2, 0) = -cur_state.omega(1);
  omega_vee(1, 0) = cur_state.omega(2);
  omega_vee(0, 1) = -cur_state.omega(2);

  motor_rpm_sq = cur_state.motor_rpm.array().square();

  //! @todo implement
  Eigen::Array4f blade_linear_velocity;
  Eigen::Array4f motor_linear_velocity;
  Eigen::Array4f AOA;
  blade_linear_velocity = 0.104719755  // rpm to rad/s
                          * cur_state.motor_rpm.array() * prop_radius_;
  for (int i = 0; i < 4; ++i)
    AOA[i] =
        alpha0 - atan2(motor_linear_velocity[i], blade_linear_velocity[i]) *  //
                     180.f / M_PI;
  //! @todo end

  // Scalar totalF = kf_ * motor_rpm_sq.sum();
  Scalar thrust = kf_ * motor_rpm_sq.sum();

  Vector<3> moments;
  moments(0) = kf_ * (motor_rpm_sq(2) - motor_rpm_sq(3)) * arm_length_;
  moments(1) = kf_ * (motor_rpm_sq(1) - motor_rpm_sq(0)) * arm_length_;
  moments(2) = km_ * (motor_rpm_sq(0) + motor_rpm_sq(1) - motor_rpm_sq(2) -
                      motor_rpm_sq(3));

  Scalar resistance = 0.1f *                                  // C
                      M_PI * (arm_length_) * (arm_length_) *  // S
                      cur_state.v.norm() * cur_state.v.norm();

  //  ROS_INFO("resistance: %lf, Thrust: %lf%% ", resistance,
  //           motor_rpm_sq.sum() / (4 * max_rpm_ * max_rpm_) * 100.0);

  vnorm = cur_state.v;
  if (vnorm.norm() != 0) {
    vnorm.normalize();
  }
  x_dot = cur_state.v;
  v_dot = -Vector<3>(0, 0, GRAVITY) + thrust * R.col(2) / mass_ +
          external_force_ / mass_ /*; //*/ - resistance * vnorm / mass_;

  acc_ = v_dot;
  //  acc_[2] = -acc_[2]; // to NED

  R_dot = R * omega_vee;
  omega_dot =
      J_.inverse() * (moments - cur_state.omega.cross(J_ * cur_state.omega) +
                      external_moment_);
  motor_rpm_dot = (cmd_ - cur_state.motor_rpm) / motor_time_constant_;

  for (unsigned int i = 0; i < 3; i++) {
    dxdt[0 + i] = x_dot(i);
    dxdt[3 + i] = v_dot(i);
    dxdt[6 + i] = R_dot(i, 0);
    dxdt[9 + i] = R_dot(i, 1);
    dxdt[12 + i] = R_dot(i, 2);
    dxdt[15 + i] = omega_dot(i);
  }
  for (unsigned int i = 0; i < 4; i++) {
    dxdt[18 + i] = motor_rpm_dot(i);
  }
  for (unsigned int i = 0; i < 22; ++i) {
    if (std::isnan(dxdt[i])) {
      dxdt[i] = 0;
    }
  }
}

void DynamicsQuadrotor::setInput(Scalar u1, Scalar u2, Scalar u3, Scalar u4) {
  cmd_(0) = u1;
  cmd_(1) = u2;
  cmd_(2) = u3;
  cmd_(3) = u4;
  for (int i = 0; i < 4; i++) {
    if (std::isnan(cmd_(i))) {
      cmd_(i) = (max_rpm_ + min_rpm_) / 2;
      std::cout << "NAN input ";
    }
    if (cmd_(i) > max_rpm_)
      cmd_(i) = max_rpm_;
    else if (cmd_(i) < min_rpm_)
      cmd_(i) = min_rpm_;
  }
}

void DynamicsQuadrotor::setInput(const Scalar control[4]) {
  setInput(control[0], control[1], control[2], control[3]);
}

const QuadState& DynamicsQuadrotor::getState() const {
  return state_;
}
void DynamicsQuadrotor::setState(const QuadState& state) {
  state_.x = state.x;
  state_.v = state.v;
  state_.R = state.R;
  state_.omega = state.omega;
  state_.motor_rpm = state.motor_rpm;

  updateInternalState();
}

void DynamicsQuadrotor::setStatePos(const Vector<3>& Pos) {
  state_.x = Pos;

  updateInternalState();
}

Scalar DynamicsQuadrotor::getMass(void) const {
  return mass_;
}
void DynamicsQuadrotor::setMass(Scalar mass) {
  mass_ = mass;
}

Scalar DynamicsQuadrotor::getGravity(void) const {
  return GRAVITY;
}

const Matrix<3, 3>& DynamicsQuadrotor::getInertia(void) const {
  return J_;
}
void DynamicsQuadrotor::setInertia(const Matrix<3, 3>& inertia) {
  if (inertia != inertia.transpose()) {
    std::cerr << "Inertia matrix not symmetric, not setting" << std::endl;
    return;
  }
  J_ = inertia;
}

Scalar DynamicsQuadrotor::getArmLength(void) const {
  return arm_length_;
}
void DynamicsQuadrotor::setArmLength(Scalar d) {
  if (d <= 0) {
    std::cerr << "Arm length <= 0, not setting" << std::endl;
    return;
  }

  arm_length_ = d;
}

Scalar DynamicsQuadrotor::getPropRadius(void) const {
  return prop_radius_;
}
void DynamicsQuadrotor::setPropRadius(Scalar r) {
  if (r <= 0) {
    std::cerr << "Prop radius <= 0, not setting" << std::endl;
    return;
  }
  prop_radius_ = r;
}

Scalar DynamicsQuadrotor::getPropellerThrustCoefficient(void) const {
  return kf_;
}
void DynamicsQuadrotor::setPropellerThrustCoefficient(Scalar kf) {
  if (kf <= 0) {
    std::cerr << "Thrust coefficient <= 0, not setting" << std::endl;
    return;
  }

  kf_ = kf;
}

Scalar DynamicsQuadrotor::getPropellerMomentCoefficient(void) const {
  return km_;
}
void DynamicsQuadrotor::setPropellerMomentCoefficient(Scalar km) {
  if (km <= 0) {
    std::cerr << "Moment coefficient <= 0, not setting" << std::endl;
    return;
  }

  km_ = km;
}

Scalar DynamicsQuadrotor::getMotorTimeConstant(void) const {
  return motor_time_constant_;
}
void DynamicsQuadrotor::setMotorTimeConstant(Scalar k) {
  if (k <= 0) {
    std::cerr << "Motor time constant <= 0, not setting" << std::endl;
    return;
  }

  motor_time_constant_ = k;
}

const Vector<3>& DynamicsQuadrotor::getExternalForce(void) const {
  return external_force_;
}
void DynamicsQuadrotor::setExternalForce(const Vector<3>& force) {
  external_force_ = force;
}

const Vector<3>& DynamicsQuadrotor::getExternalMoment(void) const {
  return external_moment_;
}
void DynamicsQuadrotor::setExternalMoment(const Vector<3>& moment) {
  external_moment_ = moment;
}

Scalar DynamicsQuadrotor::getMaxRPM(void) const {
  return max_rpm_;
}
void DynamicsQuadrotor::setMaxRPM(Scalar max_rpm) {
  if (max_rpm <= 0) {
    std::cerr << "Max rpm <= 0, not setting" << std::endl;
    return;
  }
  max_rpm_ = max_rpm;
}

Scalar DynamicsQuadrotor::getMinRPM(void) const {
  return min_rpm_;
}
void DynamicsQuadrotor::setMinRPM(Scalar min_rpm) {
  if (min_rpm < 0) {
    std::cerr << "Min rpm < 0, not setting" << std::endl;
    return;
  }
  min_rpm_ = min_rpm;
}

void DynamicsQuadrotor::updateInternalState(void) {
  for (unsigned int i = 0; i < 3; i++) {
    internal_state_[0 + i] = state_.x(i);
    internal_state_[3 + i] = state_.v(i);
    internal_state_[6 + i] = state_.R(i, 0);
    internal_state_[9 + i] = state_.R(i, 1);
    internal_state_[12 + i] = state_.R(i, 2);
    internal_state_[15 + i] = state_.omega(i);
  }
  internal_state_[18] = state_.motor_rpm(0);
  internal_state_[19] = state_.motor_rpm(1);
  internal_state_[20] = state_.motor_rpm(2);
  internal_state_[21] = state_.motor_rpm(3);
}

const Vector<3>& DynamicsQuadrotor::getAcc() const {
  return acc_;
}

}  // end namespace ml
