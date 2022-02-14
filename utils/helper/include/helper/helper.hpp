/* 
 * helper.hpp
 * Created on: Feb 13, 2022 17:56
 * Description: 
 * 
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */ 

#ifndef HELPER_HPP
#define HELPER_HPP

#include <vector>

#include "types/types.hpp"

double WrapAngle(double angle);
double CalculateMean(const std::vector<double>& dataset);
double CalculateRMSE(const std::vector<double>& dataset);

std::vector<Vector2> GenerateEllipse(double x, double y, double sigma_xx,
                                     double sigma_yy, double sigma_xy,
                                     int num_points = 50);
std::vector<Vector2> GenerateCircle(double x, double y, double radius,
                                    int num_points = 50);

#endif /* HELPER_HPP */
