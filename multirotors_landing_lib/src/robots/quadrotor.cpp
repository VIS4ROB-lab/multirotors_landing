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

#include "multirotors_landing_lib/robots/quadrotor.hpp"

#include "multirotors_landing_lib/common/geometry_utils.hpp"

namespace ml {

Quadrotor::Quadrotor()
    : collision_(false),
      is_reset_state_(true),
      world_box_(
          (Matrix<3, 2>() << -100, 100, -100, 100, -100, 100).finished()) {
  kx_ = Vector<3>(5.7f, 5.7f, 6.2f);
  kv_ = Vector<3>(3.4f, 3.4f, 4.0f);

  kR_[0] = 1.5f;
  kR_[1] = 1.5f;
  kR_[2] = 1.0f;

  kOm_[0] = 0.13f;
  kOm_[1] = 0.13f;
  kOm_[2] = 0.1f;

  corrections_[0] = 0.f;
  corrections_[1] = 0.f;
  corrections_[2] = 0.f;

  init();
}

Quadrotor::Quadrotor(const std::string &cfg_path)
    : collision_(false),
      is_reset_state_(true),
      world_box_(
          (Matrix<3, 2>() << -100, 100, -100, 100, -100, 100).finished()) {
  // Read parameters - TODO
  YAML::Node cfg = YAML::LoadFile(cfg_path);
  kx_ = Vector<3>(5.7f, 5.7f, 6.2f);
  kv_ = Vector<3>(3.4f, 3.4f, 4.0f);

  kR_[0] = 1.5f;
  kR_[1] = 1.5f;
  kR_[2] = 1.0f;

  kOm_[0] = 0.13f;
  kOm_[1] = 0.13f;
  kOm_[2] = 0.1f;

  corrections_[0] = 0.f;
  corrections_[1] = 0.f;
  corrections_[2] = 0.f;

  init();
}

Quadrotor::~Quadrotor() {}

bool Quadrotor::run(const Eigen::Array4f & /*cmd*/, const Scalar ctl_dt) {
  // TODO
  return run(ctl_dt);
}

bool Quadrotor::run(const Scalar /*ctl_dt*/) {
  // TODO
  if (!dynamics_.validState())
    return false;
  //
  return true;
}

void Quadrotor::init() {
  // reset
  reset();
}

bool Quadrotor::reset() {
  dynamics_.reset();
  return true;
}

bool Quadrotor::reset(const QuadState &state) {
  if (!dynamics_.validState())
    return false;
  dynamics_.setState(state);
  return true;
}

void Quadrotor::setState(const QuadState &state) {
  dynamics_.setState(state);
}

void Quadrotor::setStatePos(const Vector<3> &pos) {
  dynamics_.setStatePos(pos);
}

bool Quadrotor::setResetState(const bool is_reset) {
  is_reset_state_ = is_reset;
  return true;
}

bool Quadrotor::setWorldBox(const Ref<Matrix<3, 2>> box) {
  if (box(0, 0) >= box(0, 1) || box(1, 0) >= box(1, 1) ||
      box(2, 0) >= box(2, 1)) {
    return false;
  }
  world_box_ = box;
  return true;
}

void Quadrotor::setInput(const Scalar input[4]) {
  dynamics_.setInput(input);
}

void Quadrotor::setInput(const Scalar u0, const Scalar u1, const Scalar u2,
                         const Scalar u3) {
  dynamics_.setInput(u0, u1, u2, u3);
}

QuadState Quadrotor::getState() const {
  return dynamics_.getState();
}

bool Quadrotor::isResetState() const {
  return is_reset_state_;
}

Scalar Quadrotor::getCurrentYaw() const {
  return ml::R_to_ypr(dynamics_.getState().R)[0];
}

const Vector<3> &Quadrotor::getAcc() const {
  return dynamics_.getAcc();
}

void Quadrotor::getControlFromCommand(const Command &cmd) {
  const Scalar _kf = dynamics_.getPropellerThrustCoefficient();
  const Scalar _km = dynamics_.getPropellerMomentCoefficient();
  const Scalar kf = _kf - cmd.corrections[0];
  const Scalar km = _km / _kf * kf;

  const Scalar d = dynamics_.getArmLength();
  const Matrix<3, 3> J = dynamics_.getInertia().cast<Scalar>();
  const Scalar I[3][3] = {{J(0, 0), J(0, 1), J(0, 2)},
                          {J(1, 0), J(1, 1), J(1, 2)},
                          {J(2, 0), J(2, 1), J(2, 2)}};
  const ml::QuadState state = dynamics_.getState();

  // Rotation, may use external yaw
  Vector<3> _ypr = R_to_ypr(state.R);
  Vector<3> ypr = _ypr;
  if (cmd.use_external_yaw)
    ypr[0] = cmd.current_yaw;
  Eigen::Matrix3f R;
  R = Eigen::AngleAxisf(ypr[0], Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(ypr[1], Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(ypr[2], Eigen::Vector3f::UnitX());
  Scalar R11 = R(0, 0);
  Scalar R12 = R(0, 1);
  Scalar R13 = R(0, 2);
  Scalar R21 = R(1, 0);
  Scalar R22 = R(1, 1);
  Scalar R23 = R(1, 2);
  Scalar R31 = R(2, 0);
  Scalar R32 = R(2, 1);
  Scalar R33 = R(2, 2);

  Scalar Om1 = state.omega(0);
  Scalar Om2 = state.omega(1);
  Scalar Om3 = state.omega(2);

  Scalar Rd11 =
      cmd.qw * cmd.qw + cmd.qx * cmd.qx - cmd.qy * cmd.qy - cmd.qz * cmd.qz;
  Scalar Rd12 = 2 * (cmd.qx * cmd.qy - cmd.qw * cmd.qz);
  Scalar Rd13 = 2 * (cmd.qx * cmd.qz + cmd.qw * cmd.qy);
  Scalar Rd21 = 2 * (cmd.qx * cmd.qy + cmd.qw * cmd.qz);
  Scalar Rd22 =
      cmd.qw * cmd.qw - cmd.qx * cmd.qx + cmd.qy * cmd.qy - cmd.qz * cmd.qz;
  Scalar Rd23 = 2 * (cmd.qy * cmd.qz - cmd.qw * cmd.qx);
  Scalar Rd31 = 2 * (cmd.qx * cmd.qz - cmd.qw * cmd.qy);
  Scalar Rd32 = 2 * (cmd.qy * cmd.qz + cmd.qw * cmd.qx);
  Scalar Rd33 =
      cmd.qw * cmd.qw - cmd.qx * cmd.qx - cmd.qy * cmd.qy + cmd.qz * cmd.qz;

  Scalar Psi = 0.5f * (3.0f - (Rd11 * R11 + Rd21 * R21 + Rd31 * R31 +
                               Rd12 * R12 + Rd22 * R22 + Rd32 * R32 +
                               Rd13 * R13 + Rd23 * R23 + Rd33 * R33));

  Scalar force = 0;
  if (Psi < 1.0f)  // Position control stability guaranteed only when Psi < 1
    force = cmd.force[0] * R13 + cmd.force[1] * R23 + cmd.force[2] * R33;

  Scalar eR1 = 0.5f * (R12 * Rd13 - R13 * Rd12 + R22 * Rd23 - R23 * Rd22 +
                       R32 * Rd33 - R33 * Rd32);
  Scalar eR2 = 0.5f * (R13 * Rd11 - R11 * Rd13 - R21 * Rd23 + R23 * Rd21 -
                       R31 * Rd33 + R33 * Rd31);
  Scalar eR3 = 0.5f * (R11 * Rd12 - R12 * Rd11 + R21 * Rd22 - R22 * Rd21 +
                       R31 * Rd32 - R32 * Rd31);

  Scalar eOm1 = Om1;
  Scalar eOm2 = Om2;
  Scalar eOm3 = Om3;

  Scalar in1 = Om2 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3) -
               Om3 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3);
  Scalar in2 = Om3 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3) -
               Om1 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3);
  Scalar in3 = Om1 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3) -
               Om2 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3);

  Scalar M1 = -cmd.kR[0] * eR1 - cmd.kOm[0] * eOm1 + in1;  // - I[0][0]*muR1;
  Scalar M2 = -cmd.kR[1] * eR2 - cmd.kOm[1] * eOm2 + in2;  // - I[1][1]*muR2;
  Scalar M3 = -cmd.kR[2] * eR3 - cmd.kOm[2] * eOm3 + in3;  // - I[2][2]*muR3;

  Scalar w_sq[4];
  w_sq[0] = force / (4 * kf) - M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[1] = force / (4 * kf) + M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[2] = force / (4 * kf) + M1 / (2 * d * kf) - M3 / (4 * km);
  w_sq[3] = force / (4 * kf) - M1 / (2 * d * kf) - M3 / (4 * km);

  // Compute the control
  Scalar control[4];
  for (int i = 0; i < 4; i++) {
    if (w_sq[i] < 0)
      w_sq[i] = 0;
    control[i] = sqrtf(w_sq[i]);
  }
  dynamics_.setInput(control);
}

Command Quadrotor::getCommandFromDesiredPose(const QuadState &des_state,
                                             const Vector<3> &des_acc) {
  // Compute the command from current state
  const QuadState curr_state = dynamics_.getState();
  const Vector<3> curr_acc = dynamics_.getAcc();
  const Scalar mass = dynamics_.getMass();

  // Get the yaw from the desired state
  const Scalar des_yaw = R_to_ypr(des_state.R)[0];

  Vector<3> totalError = (des_state.x - curr_state.x) +
                         (des_state.v - curr_state.v) + (des_acc - curr_acc);

  Vector<3> ka(fabsf(totalError[0]) > 3 ? 0 : (fabsf(totalError[0]) * 0.2f),
               fabsf(totalError[1]) > 3 ? 0 : (fabsf(totalError[1]) * 0.2f),
               fabsf(totalError[2]) > 3 ? 0 : (fabsf(totalError[2]) * 0.2f));

  Vector<3> force;
  force.noalias() = kx_.asDiagonal() * (des_state.x - curr_state.x) +
                    kv_.asDiagonal() * (des_state.v - curr_state.v) +
                    mass * (des_acc) +
                    mass * ka.asDiagonal() * (des_acc - curr_acc) +
                    mass * GRAVITY * Vector<3>(0, 0, 1);

  // Limit control angle to 45 degree
  Scalar c = cosf(M_PI / 2.f);
  Vector<3> f;
  f.noalias() = kx_.asDiagonal() * (des_state.x - curr_state.x) +
                kv_.asDiagonal() * (des_state.v - curr_state.v) +
                mass * des_acc + mass * ka.asDiagonal() * (des_acc - curr_acc);
  if (Vector<3>(0, 0, 1).dot(force / force.norm()) < c) {
    Scalar nf = f.norm();
    Scalar A = c * c * nf * nf - f(2) * f(2);
    Scalar B = 2 * (c * c - 1) * f(2) * mass * GRAVITY;
    Scalar C = (c * c - 1) * mass * mass * GRAVITY * GRAVITY;
    Scalar s = (-B + sqrtf(B * B - 4 * A * C)) / (2.f * A);
    force.noalias() = s * f + mass * GRAVITY * Vector<3>(0, 0, 1);
  }
  // Limit control angle to 45 degree

  Vector<3> b1c, b2c, b3c;
  Vector<3> b1d(cosf(des_yaw), sinf(des_yaw), 0);

  if (force.norm() > 1e-6f)
    b3c.noalias() = force.normalized();
  else
    b3c.noalias() = Vector<3>(0, 0, 1);

  b2c.noalias() = b3c.cross(b1d).normalized();
  b1c.noalias() = b2c.cross(b3c).normalized();

  Matrix<3, 3> R;
  R << b1c, b2c, b3c;

  // Fill command
  Command so3_command;
  for (int i = 0; i < 3; i++) {
    so3_command.force[i] = force[i];
    so3_command.kR[i] = kR_[i];
    so3_command.kOm[i] = kOm_[i];
  }

  Eigen::Quaternionf q(R);
  so3_command.qx = q.x();
  so3_command.qy = q.y();
  so3_command.qz = q.z();
  so3_command.qw = q.w();

  so3_command.current_yaw = R_to_ypr(curr_state.R)[0];
  so3_command.corrections[0] = corrections_[0];
  so3_command.corrections[1] = corrections_[1];
  so3_command.corrections[2] = corrections_[2];

  so3_command.use_external_yaw = true;  // FIXME

  return so3_command;
}

}  // namespace ml
