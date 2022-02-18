/*
 * simulation_profile.cpp
 *
 * Created on: Feb 18, 2022 22:53
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "simulation/profile/simulation_profile.hpp"

#include "robot/details/motion_cmd.hpp"

namespace pllee4 {
SimulationParams LoadSimulation1Parameters() {
  SimulationParams sim_params;
  sim_params.profile_name =
      "1 - Constant Velocity + GPS + GYRO + Zero Initial Conditions";
  sim_params.robot_initial_velocity = 5;
  sim_params.robot_initial_psi = M_PI / 180.0 * 45.0;
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(500, 500, 5));
  return sim_params;
}

SimulationParams LoadSimulation2Parameters() {
  SimulationParams sim_params;
  sim_params.profile_name =
      "2 - Constant Velocity + GPS + GYRO + Non-zero Initial Conditions";
  sim_params.robot_initial_x = 500;
  sim_params.robot_initial_y = 500;
  sim_params.robot_initial_velocity = 5;
  sim_params.robot_initial_psi = M_PI / 180.0 * -135.0;
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(0, 0, 5));
  return sim_params;
}

SimulationParams LoadSimulation3Parameters() {
  SimulationParams sim_params;
  sim_params.profile_name = "3 - Constant Speed Profile + GPS + GYRO";
  sim_params.robot_initial_velocity = 5;
  sim_params.robot_initial_psi = M_PI / 180.0 * 45.0;
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(100, 100, 5));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(100, -100, 5));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(0, 100, 5));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(0, 0, 5));
  return sim_params;
}

SimulationParams LoadSimulation4Parameters() {
  SimulationParams sim_params;
  sim_params.profile_name = "4 - Variable Speed Profile + GPS + GYRO";
  sim_params.end_time = 200;
  sim_params.robot_initial_velocity = 0;
  sim_params.robot_initial_psi = M_PI / 180.0 * 45.0;
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(100, 100, 2));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(100, -100, 5));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(0, 100, 7));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(0, 0, 2));
  return sim_params;
}

SimulationParams LoadSimulation5Parameters() {
  SimulationParams sim_params = LoadSimulation1Parameters();
  sim_params.profile_name =
      "5 - Constant Velocity + GPS + GYRO + LIDAR+ Zero Initial Conditions";
  sim_params.lidar_enabled = true;
  return sim_params;
}

SimulationParams LoadSimulation6Parameters() {
  SimulationParams sim_params = LoadSimulation2Parameters();
  sim_params.profile_name =
      "6 - Constant Velocity + GPS + GYRO + LIDAR + Non-zero Initial "
      "Conditions";
  sim_params.lidar_enabled = true;
  return sim_params;
}

SimulationParams LoadSimulation7Parameters() {
  SimulationParams sim_params = LoadSimulation3Parameters();
  sim_params.profile_name = "7 - Constant Speed Profile + GPS + GYRO + LIDAR";
  sim_params.lidar_enabled = true;
  return sim_params;
}

SimulationParams LoadSimulation8Parameters() {
  SimulationParams sim_params = LoadSimulation4Parameters();
  sim_params.profile_name = "8 - Variable Speed Profile + GPS + GYRO + LIDAR";
  sim_params.lidar_enabled = true;
  return sim_params;
}

SimulationParams LoadSimulation9Parameters() {
  SimulationParams sim_params;
  sim_params.profile_name = "9 - CAPSTONE";
  sim_params.gyro_enabled = true;
  sim_params.lidar_enabled = true;
  sim_params.end_time = 500;
  sim_params.robot_initial_x = 400;
  sim_params.robot_initial_y = -400;
  sim_params.robot_initial_velocity = 0;
  sim_params.robot_initial_psi = M_PI / 180.0 * -90.0;
  sim_params.gps_error_probability = 0.05;
  sim_params.gps_denied_x = 250.0;
  sim_params.gps_denied_y = -250.0;
  sim_params.gps_denied_range = 100.0;
  sim_params.gyro_bias = -3.1 / 180.0 * M_PI;
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandStraight>(3, -2));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandTurnTo>(M_PI / 180.0 * 90.0, -2));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(400, -300, 5));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(350, -300, 2));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(300, -250, 7));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(300, -300, 5));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(250, -250, 5));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(250, -300, 5));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(200, -250, 5));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(200, -300, 5));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(200, -150, 2));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(100, -100, -2));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(200, 0, 7));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(300, -100, 5));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(300, -300, 7));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(400, -300, 3));
  sim_params.robot_commands.emplace_back(
      std::make_shared<MotionCommandMoveTo>(400, -400, 1));
  return sim_params;
}

SimulationParams LoadSimulation0Parameters() {
  SimulationParams sim_params = LoadSimulation9Parameters();
  sim_params.profile_name =
      "0 - CAPSTONE BONUS (with No Lidar Data Association)";
  sim_params.lidar_id_enabled = false;
  return sim_params;
}
}  // namespace pllee4