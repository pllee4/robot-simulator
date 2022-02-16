/* 
 * bicycle_model.hpp
 * Created on: Feb 16, 2022 21:08
 * Description: 
 * 
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */ 

#ifndef BICYCLE_MODEL_HPP
#define BICYCLE_MODEL_HPP

#include "robot/details/robot_state.hpp"

namespace pllee4 {

class BicycleMotion {
 public:
  BicycleMotion();
  BicycleMotion(double x0, double y0, double psi0, double V0);
  ~BicycleMotion() = default;

  void Reset();
  void Reset(RobotState state);

  void Update(double dt);
  void SetSteeringCmd(double steer) { steering_command_ = steer; }
  void SetVelocityCmd(double accel) { velocity_command_ = accel; }
  RobotState GetRobotState() const { return current_state_; }

 private:
  RobotState current_state_;
  RobotState initial_state_;

  double steering_command_;
  double velocity_command_;

  double wheel_base_{4.0};
  double max_velocity_{28.0};
  double max_acceleration_{2.0};
  double max_steering_{0.8};
};
}  // namespace pllee4
#endif /* BICYCLE_MODEL_HPP */
