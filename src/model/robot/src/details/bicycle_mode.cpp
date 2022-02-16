/* 
 * bicycle_mode.cpp
 * Created on: Feb 16, 2022 21:08
 * Description: 
 * 
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */ 

#include "robot/details/bicycle_model.hpp"

#include <cmath>

#include "helper/helper.hpp"

namespace pllee4 {
BicycleMotion::BicycleMotion() : initial_state_(RobotState(0, 0, 0, 0)) {
  Reset();
}

BicycleMotion::BicycleMotion(double x0, double y0, double psi0, double V0)
    : initial_state_(RobotState(x0, y0, psi0, V0)) {
  Reset();
}

void BicycleMotion::Reset() {
  current_state_ = initial_state_;
  steering_command_ = initial_state_.steering;
  velocity_command_ = initial_state_.v;
}

void BicycleMotion::Reset(RobotState state) {
  initial_state_ = state;
  Reset();
}

void BicycleMotion::Update(double dt) {
  double cosPsi = cos(current_state_.psi);
  double sinPsi = sin(current_state_.psi);
  double x = current_state_.x + current_state_.v * cosPsi * dt;
  double y = current_state_.y + current_state_.v * sinPsi * dt;

  double accel = velocity_command_ - current_state_.v;
  if (accel > max_acceleration_) {
    accel = max_acceleration_;
  }
  if (accel < -max_acceleration_) {
    accel = -max_acceleration_;
  }

  double steer = steering_command_;
  if (steer > max_steering_) {
    steer = max_steering_;
  }
  if (steer < -max_steering_) {
    steer = -max_steering_;
  }

  double vel = current_state_.v + accel * dt;
  if (vel > max_velocity_) {
    vel = max_velocity_;
  }
  if (vel < -max_velocity_) {
    vel = -max_velocity_;
  }

  double psi_dot = current_state_.v * steer / wheel_base_;
  double psi = WrapAngle(current_state_.psi + psi_dot * dt);
  current_state_ = RobotState(x, y, psi, vel, psi_dot, steer);
}
}  // namespace pllee4