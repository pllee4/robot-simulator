/*
 * robot_state.hpp
 * Created on: Feb 15, 2022 22:22
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP

namespace pllee4 {
struct RobotState {
  double x{}, y{}, psi{}, v{};
  double yaw_rate{}, steering{};

  RobotState() = default;

  RobotState(double set_x, double set_y, double set_psi, double set_v)
      : x(set_x), y(set_y), psi(set_psi), v(set_v) {}

  RobotState(double set_x, double set_y, double set_psi, double set_v,
             double set_psi_dot, double set_steering)
      : x(set_x),
        y(set_y),
        psi(set_psi),
        v(set_v),
        yaw_rate(set_psi_dot),
        steering(set_steering) {}
};
}  // namespace pllee4
#endif /* ROBOT_STATE_HPP */
