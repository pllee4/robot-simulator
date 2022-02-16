/*
 * motion_cmd_base.hpp
 * Created on: Feb 15, 2022 22:22
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef MOTION_CMD_BASE_HPP
#define MOTION_CMD_BASE_HPP

#include "robot/details/robot_state.hpp"

namespace pllee4 {
class MotionCommandBase {
 public:
  MotionCommandBase() = default;
  virtual ~MotionCommandBase() = default;

  virtual void StartCommand(double time, RobotState state) {
    start_time_ = time;
    start_state_ = state;
  }

  virtual void EndCommand(double time, double dt, RobotState state) {}

  virtual bool Update(double time, double dt, RobotState state) {
    return false;
  }

  virtual double GetVelocityCommand() const { return velocity_command_; }
  virtual double GetSteeringCommand() const { return steering_command_; }

 protected:
  double velocity_command_{0.0}, steering_command_{0.0}, start_time_;
  RobotState start_state_;
};
}  // namespace pllee4
#endif /* MOTION_CMD_BASE_HPP */
