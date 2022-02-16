/* 
 * robot.hpp
 * Created on: Feb 16, 2022 21:09
 * Description: 
 * 
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */ 

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <queue>

#include "helper/helper.hpp"
#include "robot/details/motion_cmd_base.hpp"
#include "robot/details/bicycle_model.hpp"
#include "types/types.hpp"

namespace pllee4 {
class Display;

class Robot {
 public:
  Robot();
  ~Robot();

  void Reset(double x0, double y0, double psi0, double V0);
  void AddVehicleCommand(MotionCommandBase* cmd);
  RobotState GetRobotState() const { return robot_model_.GetRobotState(); }

  bool Update(double time, double dt);
  void Render(Display& disp);

 private:
  BicycleMotion robot_model_;
  MotionCommandBase* current_command_;
  std::queue<MotionCommandBase*> robot_commands_;

  std::vector<Vector2> robot_lines_body_;
  std::vector<Vector2> wheel_lines_;
  std::vector<std::vector<Vector2>> marker_lines_;
  Vector2 wheel_fl_offset_, wheel_fr_offset_, wheel_rl_offset_,
      wheel_rr_offset_;
};

}  // namespace pllee4

#endif /* ROBOT_HPP */
