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

#include <opencv2/opencv.hpp>
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

bool inBorder(const cv::Point2f &pt);
void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status);
void reduceVector(std::vector<int> &v, std::vector<uchar> status);

class FeatureTracker
{
public:
    FeatureTracker();
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void setMask();
    void readIntrinsicParameter(const std::vector<std::string> &calib_file);
    void showUndistortion(const std::string &name);
    void rejectWithF();
    void undistortedPoints();
    std::vector<cv::Point2f> undistortedPts(std::vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    std::vector<cv::Point2f> ptsVelocity(std::vector<int> &ids, std::vector<cv::Point2f> &pts,
                                    std::map<int, cv::Point2f> &cur_id_pts, std::map<int, cv::Point2f> &prev_id_pts);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   std::vector<int> &curLeftIds,
                                   std::vector<cv::Point2f> &curLeftPts,
                                   std::vector<cv::Point2f> &curRightPts,
                                   std::map<int, cv::Point2f> &prevLeftPtsMap);
    void setPrediction(std::map<int, Eigen::Vector3d> &predictPts);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    void removeOutliers(std::set<int> &removePtsIds);
    cv::Mat getTrackImage();
    bool inBorder(const cv::Point2f &pt);

    int row, col;
    cv::Mat imTrack;
    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img;
    std::vector<cv::Point2f> n_pts;
    std::vector<cv::Point2f> predict_pts;
    std::vector<cv::Point2f> predict_pts_debug;
    std::vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
    std::vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
    std::vector<cv::Point2f> pts_velocity, right_pts_velocity;
    std::vector<int> ids, ids_right;
    std::vector<int> track_cnt;
    std::map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
    std::map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    std::map<int, cv::Point2f> prevLeftPtsMap;
    std::vector<camodocal::CameraPtr> m_camera;
    double cur_time;
    double prev_time;
    bool stereo_cam;
    int n_id;
    bool hasPrediction;
};
