/* 
 * motion_cmd.hpp
 * Created on: Feb 15, 2022 22:23
 * Description: 
 * 
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */ 

#ifndef MOTION_CMD_HPP
#define MOTION_CMD_HPP

#include "robot/details/motion_cmd_base.hpp"
#include "helper/helper.hpp"

namespace pllee4 {
class MotionCommandStraight : public MotionCommandBase {
 public:
  MotionCommandStraight(double command_time, double command_velocity)
      : command_time_(command_time), command_velocity(command_velocity) {}

  bool Update(double time, double dt, RobotState state) {
    velocity_command_ = command_velocity;
    steering_command_ = 0.0;
    return time > (start_time_ + command_time_);
  }

 private:
  double command_time_, command_velocity;
};

class MotionCommandTurnTo : public MotionCommandBase {
 public:
  MotionCommandTurnTo(double command_heading, double command_velocity)
      : command_heading_(command_heading), command_velocity(command_velocity) {}

  bool Update(double time, double dt, RobotState state) {
    velocity_command_ = command_velocity;
    double angle_error = WrapAngle(command_heading_ - state.psi);
    steering_command_ = angle_error * (std::signbit(state.v) ? -1.0 : 1.0);
    return std::fabs(angle_error) < 0.001;
  }

 private:
  double command_heading_, command_velocity;
};

class MotionCommandMoveTo : public MotionCommandBase {
 public:
  MotionCommandMoveTo(double command_x, double command_y,
                      double command_velocity)
      : command_x_(command_x),
        command_y_(command_y),
        command_velocity(command_velocity) {}

  bool Update(double time, double dt, RobotState state) {
    velocity_command_ = command_velocity;
    double delta_x = command_x_ - state.x;
    double delta_y = command_y_ - state.y;
    double range = sqrt(delta_x * delta_x + delta_y * delta_y);
    double angle_command = atan2(delta_y, delta_x);
    double psi = WrapAngle(state.psi - (std::signbit(state.v) ? M_PI : 0.0));
    double angle_error = WrapAngle(angle_command - psi);
    steering_command_ = angle_error * (std::signbit(state.v) ? -1.0 : 1.0);
    return (range < 5.0);
  }

 private:
  double command_x_, command_y_, command_velocity;
};

}  // namespace pllee4

#endif /* MOTION_CMD_HPP */
