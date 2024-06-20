/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include <map>
#include "../estimator/feature_manager.h"


class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame(const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>& _points, double _t):t{_t},is_key_frame{false}
        {
            points = _points;
        };
        std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>> > > points;
        double t;
        Eigen::Matrix3d R;
        Eigen::Vector3d T;
        IntegrationBase *pre_integration;
        bool is_key_frame;
};
void solveGyroscopeBias(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs);
bool VisualIMUAlignment(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, Eigen::Vector3d &g, Eigen::VectorXd &x);