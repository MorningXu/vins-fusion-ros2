/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <thread>
#include <mutex>
#include <ceres/ceres.h>
#include <queue>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "parameters.h"
#include "feature_manager.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/marginalization_factor.h"
#include "../featureTracker/feature_tracker.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"


class Estimator {
public:
    Estimator();

    ~Estimator();

    void setParameter();

    // interface
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);

    void inputIMU(double t, const Eigen::Vector3d &linearAcceleration, const Eigen::Vector3d &angularVelocity);

    void inputFeature(
        double t, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);

    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());

    void processIMU(double t, double dt, const Eigen::Vector3d &linear_acceleration,
                    const Eigen::Vector3d &angular_velocity);

    void processImage(const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                      const double header);

    void processMeasurements();

    void changeSensorType(int use_imu, int use_stereo);

    // internal
    void clearState();

    bool initialStructure();

    bool visualInitialAlign();

    bool relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l);

    void slideWindow();

    void slideWindowNew();

    void slideWindowOld();

    void optimization();

    void vector2double();

    void double2vector();

    bool failureDetection();

    bool getIMUInterval(double t0, double t1, std::vector<std::pair<double, Eigen::Vector3d>> &accVector,
                        std::vector<std::pair<double, Eigen::Vector3d>> &gyrVector);

    void getPoseInWorldFrame(Eigen::Matrix4d &T);

    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);

    void predictPtsInNextFrame();

    void outliersRejection(std::set<int> &removeIndex);

    double reprojectionError(Eigen::Matrix3d &Ri, Eigen::Vector3d &Pi, Eigen::Matrix3d &rici, Eigen::Vector3d &tici,
                             Eigen::Matrix3d &Rj, Eigen::Vector3d &Pj, Eigen::Matrix3d &ricj, Eigen::Vector3d &ticj,
                             double depth, Eigen::Vector3d &uvi, Eigen::Vector3d &uvj);

    void updateLatestStates();

    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);

    bool IMUAvailable(double t);

    void initFirstIMUPose(std::vector<std::pair<double, Eigen::Vector3d>> &accVector);

    enum SolverFlag {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    std::mutex mProcess;
    std::mutex mBuf;
    std::mutex mPropagate;
    std::queue<std::pair<double, Eigen::Vector3d>> accBuf;
    std::queue<std::pair<double, Eigen::Vector3d>> gyrBuf;
    std::queue<std::pair<double, std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>>> featureBuf;
    double prevTime, curTime;
    bool openExEstimation;

    std::thread trackThread;
    std::thread processThread;

    FeatureTracker featureTracker;

    SolverFlag solver_flag;
    MarginalizationFlag marginalization_flag;
    Eigen::Vector3d g;

    Eigen::Matrix3d ric[2];
    Eigen::Vector3d tic[2];

    Eigen::Vector3d Ps[(WINDOW_SIZE + 1)];
    Eigen::Vector3d Vs[(WINDOW_SIZE + 1)];
    Eigen::Matrix3d Rs[(WINDOW_SIZE + 1)];
    Eigen::Vector3d Bas[(WINDOW_SIZE + 1)];
    Eigen::Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td;

    Eigen::Matrix3d back_R0, last_R, last_R0;
    Eigen::Vector3d back_P0, last_P, last_P0;
    double Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Eigen::Vector3d acc_0, gyr_0;

    std::vector<double> dt_buf[(WINDOW_SIZE + 1)];
    std::vector<Eigen::Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    std::vector<Eigen::Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
    int inputImageCnt;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    std::vector<Eigen::Vector3d> point_cloud;
    std::vector<Eigen::Vector3d> margin_cloud;
    std::vector<Eigen::Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[2][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    std::vector<double *> last_marginalization_parameter_blocks;

    std::map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;

    double latest_time;
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    Eigen::Quaterniond latest_Q;

    bool initFirstPoseFlag;
    bool initThreadFlag;
};
