/* 
 * robot.cpp
 * Created on: Feb 16, 2022 21:09
 * Description: 
 * 
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */ 

#include "robot/robot.hpp"

#include "display/display.hpp"
#include "helper/details/transformation.hpp"
#include "helper/helper.hpp"

namespace pllee4 {

Robot::Robot() : robot_model_(), current_command_(nullptr) {
  // Create Display Geometry
  robot_lines_body_ = {{2, -1}, {2, 1}, {-2, 1}, {-2, -1}, {2, -1}};
  marker_lines_ = {{{0.5, 0.5}, {-0.5, -0.5}},
                   {{0.5, -0.5}, {-0.5, 0.5}},
                   {{0, 0}, {3.5, 0}}};
  wheel_lines_ = {
      {-0.6, 0.3}, {0.6, 0.3}, {0.6, -0.3}, {-0.6, -0.3}, {-0.6, 0.3}};
  wheel_fl_offset_ = Vector2(2, -1.6);
  wheel_fr_offset_ = Vector2(2, 1.6);
  wheel_rl_offset_ = Vector2(-2, -1.6);
  wheel_rr_offset_ = Vector2(-2, 1.6);
}

Robot::~Robot() = default;

void Robot::Reset(double x0, double y0, double psi0, double V0) {
  robot_model_.Reset(RobotState(x0, y0, psi0, V0));
  while (!robot_commands_.empty()) {
    robot_commands_.pop();
  }
  current_command_ = nullptr;
}

void Robot::AddVehicleCommand(MotionCommandBase* cmd) {
  if (cmd != nullptr) {
    robot_commands_.push(cmd);
  }
}

bool Robot::Update(double time, double dt) {
  // Update Command
  if (current_command_ == nullptr && !robot_commands_.empty()) {
    current_command_ = robot_commands_.front();
    robot_commands_.pop();
    current_command_->StartCommand(time, robot_model_.GetRobotState());
  }

  // Run Command
  if (current_command_ != nullptr) {
    bool cmd_complete =
        current_command_->Update(time, dt, robot_model_.GetRobotState());
    robot_model_.SetSteeringCmd(current_command_->GetSteeringCommand());
    robot_model_.SetVelocityCmd(current_command_->GetVelocityCommand());
    if (cmd_complete) {
      current_command_ = nullptr;
    }
  } else {
    robot_model_.SetSteeringCmd(0.0);
    robot_model_.SetVelocityCmd(0.0);
  }

  // Update Vehicle
  robot_model_.Update(dt);

  return true;
}

void Robot::Render(Display& disp) {
  double steering_psi = robot_model_.GetRobotState().steering;
  double robot_psi_offset = robot_model_.GetRobotState().psi;
  Vector2 robot_pos_offset =
      Vector2(robot_model_.GetRobotState().x, robot_model_.GetRobotState().y);

  disp.SetDrawColour(0, 255, 0);
  disp.DrawLines(
      TransformPoints(robot_lines_body_, robot_pos_offset, robot_psi_offset));
  disp.DrawLines(
      TransformPoints(marker_lines_, robot_pos_offset, robot_psi_offset));

  disp.SetDrawColour(0, 201, 0);
  disp.DrawLines(TransformPoints(
      TransformPoints(wheel_lines_, wheel_fl_offset_, steering_psi),
      robot_pos_offset, robot_psi_offset));
  disp.DrawLines(TransformPoints(
      TransformPoints(wheel_lines_, wheel_fr_offset_, steering_psi),
      robot_pos_offset, robot_psi_offset));
  disp.DrawLines(TransformPoints(OffsetPoints(wheel_lines_, wheel_rl_offset_),
                                 robot_pos_offset, robot_psi_offset));
  disp.DrawLines(TransformPoints(OffsetPoints(wheel_lines_, wheel_rr_offset_),
                                 robot_pos_offset, robot_psi_offset));
}
}  // namespace pllee4
