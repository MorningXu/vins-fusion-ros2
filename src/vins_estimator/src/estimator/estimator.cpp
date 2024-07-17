/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"

#include <pcl/common/copy_point.h>

#include "../utility/visualization.h"

Estimator::Estimator(): f_manager{Rs} {
    RCLCPP_INFO(rclcpp::get_logger("vins_estimator"), "init begins");
    initThreadFlag = false;
    clearState();
}

Estimator::~Estimator() {
    if (MULTIPLE_THREAD) {
        processThread.join();
        printf("join thread \n");
    }
}

void Estimator::clearState() {
    mProcess.lock();
    while (!accBuf.empty())
        accBuf.pop();
    while (!gyrBuf.empty())
        gyrBuf.pop();
    while (!featureBuf.empty())
        featureBuf.pop();

    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr) {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = Eigen::Vector3d::Zero();
        ric[i] = Eigen::Matrix3d::Identity();
    }

    first_imu = false,
            sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;

    mProcess.unlock();
}

void Estimator::setParameter() {
    mProcess.lock();
    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        std::cout << " exitrinsic cam " << i << std::endl << ric[i] << std::endl << tic[i].transpose() << std::endl;
    }
    f_manager.setRic(ric);
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
    td = TD;
    g = G;
    std::cout << "set g " << g.transpose() << std::endl;
    featureTracker.readIntrinsicParameter(CAM_NAMES);

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !initThreadFlag) {
        initThreadFlag = true;
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
    mProcess.unlock();
}

void Estimator::changeSensorType(int use_imu, int use_stereo) {
    bool restart = false;
    mProcess.lock();
    if (!use_imu && !use_stereo)
        printf("at least use two sensors! \n");
    else {
        if (USE_IMU != use_imu) {
            USE_IMU = use_imu;
            if (USE_IMU) {
                // reuse imu; restart system
                restart = true;
            } else {
                if (last_marginalization_info != nullptr)
                    delete last_marginalization_info;

                tmp_pre_integration = nullptr;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
        }

        STEREO = use_stereo;
        printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
    }
    mProcess.unlock();
    if (restart) {
        clearState();
        setParameter();
    }
}

void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1) {
    inputImageCnt++;
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    TicToc featureTrackerTime;

    if (_img1.empty())
        featureFrame = featureTracker.trackImage(t, _img);
    else
        featureFrame = featureTracker.trackImage(t, _img, _img1);
    //printf("featureTracker time: %f\n", featureTrackerTime.toc());

    if (SHOW_TRACK) {
        cv::Mat imgTrack = featureTracker.getTrackImage();
        pubTrackImage(imgTrack, t);
    }

    if (MULTIPLE_THREAD) {
        if (inputImageCnt % 2 == 0) {
            mBuf.lock();
            featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();
        }
    } else {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));
        mBuf.unlock();
        TicToc processTime;
        processMeasurements();
        printf("process time: %f\n", processTime.toc());
    }
}

void Estimator::inputIMU(double t, const Eigen::Vector3d &linearAcceleration, const Eigen::Vector3d &angularVelocity) {
    mBuf.lock();
    accBuf.push(std::make_pair(t, linearAcceleration));
    gyrBuf.push(std::make_pair(t, angularVelocity));
    //printf("input imu with time %f \n", t);
    mBuf.unlock();

    if (solver_flag == NON_LINEAR) {
        mPropagate.lock();
        fastPredictIMU(t, linearAcceleration, angularVelocity);
        pubLatestOdometry(latest_P, latest_Q, latest_V, t);
        mPropagate.unlock();
    }
}

void Estimator::inputFeature(
    double t, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame) {
    mBuf.lock();
    featureBuf.push(std::make_pair(t, featureFrame));
    mBuf.unlock();

    if (!MULTIPLE_THREAD)
        processMeasurements();
}


bool Estimator::getIMUInterval(double t0, double t1, std::vector<std::pair<double, Eigen::Vector3d>> &accVector,
                               std::vector<std::pair<double, Eigen::Vector3d>> &gyrVector) {
    if (accBuf.empty()) {
        printf("not receive imu\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if (t1 <= accBuf.back().first) {
        while (accBuf.front().first <= t0) {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1) {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    } else {
        printf("wait for imu\n");
        return false;
    }
    return true;
}

bool Estimator::IMUAvailable(double t) {
    if (!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}

void Estimator::processMeasurements() {
    while (1) {
        //printf("process measurments\n");
        std::pair<double, std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>> feature;
        std::vector<std::pair<double, Eigen::Vector3d>> accVector, gyrVector;
        if (!featureBuf.empty()) {
            feature = featureBuf.front();
            curTime = feature.first + td;
            while (1) {
                if ((!USE_IMU || IMUAvailable(feature.first + td)))
                    break;
                else {
                    printf("wait for imu ... \n");
                    if (!MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            mBuf.lock();
            if (USE_IMU)
                getIMUInterval(prevTime, curTime, accVector, gyrVector);

            featureBuf.pop();
            mBuf.unlock();

            if (USE_IMU) {
                if (!initFirstPoseFlag)
                    initFirstIMUPose(accVector);
                for (size_t i = 0; i < accVector.size(); i++) {
                    double dt;
                    if (i == 0)
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                }
            }
            mProcess.lock();
            processImage(feature.second, feature.first);
            prevTime = curTime;

            printStatistics(*this, 0);

            std_msgs::msg::Header header;
            header.frame_id = "world";
            long secs = feature.first;
            long nanosecs = (feature.first - secs)* 1e9;
            header.stamp.sec = secs;
            header.stamp.nanosec = nanosecs;

            pubOdometry(*this, header);
            pubKeyPoses(*this, header);
            pubCameraPose(*this, header);
            pubPointCloud(*this, header);
            pubKeyframe(*this);
            pubTF(*this, header);
            mProcess.unlock();
        }

        if (!MULTIPLE_THREAD)
            break;

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void Estimator::initFirstIMUPose(std::vector<std::pair<double, Eigen::Vector3d>> &accVector) {
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    //return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for (size_t i = 0; i < accVector.size(); i++) {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Eigen::Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    std::cout << "init R0 " << std::endl << Rs[0] << std::endl;
    //Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r) {
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}


void Estimator::processIMU(double t, double dt, const Eigen::Vector3d &linear_acceleration,
                           const Eigen::Vector3d &angular_velocity) {
    if (!first_imu) {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count]) {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0) {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
        Eigen::Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Eigen::Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processImage(const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                             const double header) {
    RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "new image coming ------------------------------------------");
    RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "Adding feature points %lu", image.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image, td)) {
        marginalization_flag = MARGIN_OLD;
        //printf("keyframe\n");
    } else {
        marginalization_flag = MARGIN_SECOND_NEW;
        //printf("non-keyframe\n");
    }

    RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "Solving %d", frame_count);
    RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(std::make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if (ESTIMATE_EXTRINSIC == 2) {
        RCLCPP_INFO(rclcpp::get_logger("vins_estimator"), "calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0) {
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres = f_manager.getCorresponding(
                frame_count - 1, frame_count);
            Eigen::Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric)) {
                RCLCPP_WARN(rclcpp::get_logger("vins_estimator"), "initial extrinsic rotation calib success");
                RCLCPP_WARN_STREAM(rclcpp::get_logger("vins_estimator"),
                                   "initial extrinsic rotation: " << std::endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL) {
        // monocular + IMU initilization
        if (!STEREO && USE_IMU) {
            if (frame_count == WINDOW_SIZE) {
                bool result = false;
                if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1) {
                    result = initialStructure();
                    initial_timestamp = header;
                }
                if (result) {
                    optimization();
                    updateLatestStates();
                    solver_flag = NON_LINEAR;
                    slideWindow();
                    RCLCPP_INFO(rclcpp::get_logger("vins_estimator"), "Initialization finish!");
                } else
                    slideWindow();
            }
        }

        // stereo + IMU initilization
        if (STEREO && USE_IMU) {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            if (frame_count == WINDOW_SIZE) {
                std::map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++) {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                solveGyroscopeBias(all_image_frame, Bgs);
                for (int i = 0; i <= WINDOW_SIZE; i++) {
                    pre_integrations[i]->repropagate(Eigen::Vector3d::Zero(), Bgs[i]);
                }
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                RCLCPP_INFO(rclcpp::get_logger("vins_estimator"), "Initialization finish!");
            }
        }

        // stereo only initilization
        if (STEREO && !USE_IMU) {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            optimization();

            if (frame_count == WINDOW_SIZE) {
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                RCLCPP_INFO(rclcpp::get_logger("vins_estimator"), "Initialization finish!");
            }
        }

        if (frame_count < WINDOW_SIZE) {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }
    } else {
        TicToc t_solve;
        if (!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        optimization();
        std::set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
        if (!MULTIPLE_THREAD) {
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }

        RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "solver costs: %fms", t_solve.toc());

        if (failureDetection()) {
            RCLCPP_WARN(rclcpp::get_logger("vins_estimator"), "failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "system reboot!");
            return;
        }

        slideWindow();
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
    }
}

bool Estimator::initialStructure() {
    TicToc t_sfm;
    //check imu observibility
    {
        std::map<double, ImageFrame>::iterator frame_it;
        Eigen::Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++) {
            double dt = frame_it->second.pre_integration->sum_dt;
            Eigen::Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Eigen::Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++) {
            double dt = frame_it->second.pre_integration->sum_dt;
            Eigen::Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if (var < 0.25) {
            RCLCPP_INFO(rclcpp::get_logger("vins_estimator"), "IMU excitation not enouth!");
            //return false;
        }
    }
    // global sfm
    Eigen::Quaterniond Q[frame_count + 1];
    Eigen::Vector3d T[frame_count + 1];
    std::map<int, Eigen::Vector3d> sfm_tracked_points;
    std::vector<SFMFeature> sfm_f;
    for (auto &it_per_id: f_manager.feature) {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame: it_per_id.feature_per_frame) {
            imu_j++;
            Eigen::Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(std::make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    }
    Eigen::Matrix3d relative_R;
    Eigen::Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l)) {
        RCLCPP_INFO(rclcpp::get_logger("vins_estimator"), "Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if (!sfm.construct(frame_count + 1, Q, T, l,
                       relative_R, relative_T,
                       sfm_f, sfm_tracked_points)) {
        RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    std::map<double, ImageFrame>::iterator frame_it;
    std::map<int, Eigen::Vector3d>::iterator it;
    frame_it = all_image_frame.begin();
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++) {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if ((frame_it->first) == Headers[i]) {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if ((frame_it->first) > Headers[i]) {
            i++;
        }
        Eigen::Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Eigen::Vector3d P_inital = -R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        std::vector<cv::Point3f> pts_3_vector;
        std::vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts: frame_it->second.points) {
            int feature_id = id_pts.first;
            for (auto &i_p: id_pts.second) {
                it = sfm_tracked_points.find(feature_id);
                if (it != sfm_tracked_points.end()) {
                    Eigen::Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Eigen::Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if (pts_3_vector.size() < 6) {
            std::cout << "pts_3_vector size " << pts_3_vector.size() << std::endl;
            RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "Not enough points for solve pnp !");
            return false;
        }
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1)) {
            RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        Eigen::MatrixXd R_pnp, tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        Eigen::MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else {
        RCLCPP_INFO(rclcpp::get_logger("vins_estimator"), "misalign visual structure with IMU");
        return false;
    }
}

bool Estimator::visualInitialAlign() {
    TicToc t_g;
    Eigen::VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if (!result) {
        RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++) {
        Eigen::Matrix3d Ri = all_image_frame[Headers[i]].R;
        Eigen::Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        pre_integrations[i]->repropagate(Eigen::Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    std::map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++) {
        if (frame_i->second.is_key_frame) {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    Eigen::Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Eigen::Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++) {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("vins_estimator"), "g0     " << g.transpose());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("vins_estimator"), "my R0  " << Utility::R2ypr(Rs[0]).transpose());

    f_manager.clearDepth();
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

    return true;
}

bool Estimator::relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l) {
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++) {
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20) {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++) {
                Eigen::Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Eigen::Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T)) {
                l = i;
                RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"),
                             "average_parallax %f choose l %d and newest frame to triangulate the whole structure",
                             average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::vector2double() {
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Eigen::Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if (USE_IMU) {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++) {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Eigen::Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }


    Eigen::VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
}

void Estimator::double2vector() {
    Eigen::Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Eigen::Vector3d origin_P0 = Ps[0];

    if (failure_occur) {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    if (USE_IMU) {
        Eigen::Vector3d origin_R00 = Utility::R2ypr(Eigen::Quaterniond(para_Pose[0][6],
                                                                       para_Pose[0][3],
                                                                       para_Pose[0][4],
                                                                       para_Pose[0][5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        //TODO
        Eigen::Matrix3d rot_diff = Utility::ypr2R(Eigen::Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0) {
            RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "euler singular point!");
            rot_diff = Rs[0] * Eigen::Quaterniond(para_Pose[0][6],
                                                  para_Pose[0][3],
                                                  para_Pose[0][4],
                                                  para_Pose[0][5]).toRotationMatrix().transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++) {
            Rs[i] = rot_diff * Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).
                    normalized().toRotationMatrix();

            Ps[i] = rot_diff * Eigen::Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                               para_Pose[i][1] - para_Pose[0][1],
                                               para_Pose[i][2] - para_Pose[0][2]) + origin_P0;


            Vs[i] = rot_diff * Eigen::Vector3d(para_SpeedBias[i][0],
                                               para_SpeedBias[i][1],
                                               para_SpeedBias[i][2]);

            Bas[i] = Eigen::Vector3d(para_SpeedBias[i][3],
                                     para_SpeedBias[i][4],
                                     para_SpeedBias[i][5]);

            Bgs[i] = Eigen::Vector3d(para_SpeedBias[i][6],
                                     para_SpeedBias[i][7],
                                     para_SpeedBias[i][8]);
        }
    } else {
        for (int i = 0; i <= WINDOW_SIZE; i++) {
            Rs[i] = Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().
                    toRotationMatrix();

            Ps[i] = Eigen::Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if (USE_IMU) {
        for (int i = 0; i < NUM_OF_CAM; i++) {
            tic[i] = Eigen::Vector3d(para_Ex_Pose[i][0],
                                     para_Ex_Pose[i][1],
                                     para_Ex_Pose[i][2]);
            ric[i] = Eigen::Quaterniond(para_Ex_Pose[i][6],
                                        para_Ex_Pose[i][3],
                                        para_Ex_Pose[i][4],
                                        para_Ex_Pose[i][5]).normalized().toRotationMatrix();
        }
    }

    Eigen::VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if (USE_IMU)
        td = para_Td[0][0];
}

bool Estimator::failureDetection() {
    return false;
    if (f_manager.last_track_num < 2) {
        RCLCPP_INFO(rclcpp::get_logger("vins_estimator"), " little feature %d", f_manager.last_track_num);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5) {
        RCLCPP_INFO(rclcpp::get_logger("vins_estimator"), " big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0) {
        RCLCPP_INFO(rclcpp::get_logger("vins_estimator"), " big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Eigen::Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5) {
        //ROS_INFO(" big translation");
        //return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1) {
        //ROS_INFO(" big z translation");
        //return true;
    }
    Eigen::Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Eigen::Matrix3d delta_R = tmp_R.transpose() * last_R;
    Eigen::Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50) {
        RCLCPP_INFO(rclcpp::get_logger("vins_estimator"), " big delta_angle ");
        //return true;
    }
    return false;
}

void Estimator::optimization() {
    TicToc t_whole, t_prepare;
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++) {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if (USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if (!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    for (int i = 0; i < NUM_OF_CAM; i++) {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation) {
            //ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        } else {
            //ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    if (last_marginalization_info && last_marginalization_info->valid) {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    if (USE_IMU) {
        for (int i = 0; i < frame_count; i++) {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j],
                                     para_SpeedBias[j]);
        }
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id: f_manager.feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame: it_per_id.feature_per_frame) {
            imu_j++;
            if (imu_i != imu_j) {
                Eigen::Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(
                    pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                    it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0],
                                         para_Feature[feature_index], para_Td[0]);
            }

            if (STEREO && it_per_frame.is_stereo) {
                Eigen::Vector3d pts_j_right = it_per_frame.pointRight;
                if (imu_i != imu_j) {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(
                        pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                        it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0],
                                             para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                } else {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(
                        pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                        it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1],
                                             para_Feature[feature_index], para_Td[0]);
                }
            }
            f_m_cnt++;
        }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "visual measurement count: %d", f_m_cnt);
    //printf("prepare for ceres: %f \n", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "Iterations : %d", static_cast<int>(summary.iterations.size()));
    //printf("solver costs: %f \n", t_solver.toc());

    double2vector();
    //printf("frame_count: %d \n", frame_count);

    if (frame_count < WINDOW_SIZE)
        return;

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD) {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid) {
            std::vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if (USE_IMU) {
            if (pre_integrations[1]->sum_dt < 10.0) {
                IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                               std::vector<double *>{
                                                                                   para_Pose[0], para_SpeedBias[0],
                                                                                   para_Pose[1], para_SpeedBias[1]
                                                                               },
                                                                               std::vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        } {
            int feature_index = -1;
            for (auto &it_per_id: f_manager.feature) {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame: it_per_id.feature_per_frame) {
                    imu_j++;
                    if (imu_i != imu_j) {
                        Eigen::Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(
                            pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                            std::vector<double *>{
                                para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index],
                                para_Td[0]
                            },
                            std::vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if (STEREO && it_per_frame.is_stereo) {
                        Eigen::Vector3d pts_j_right = it_per_frame.pointRight;
                        if (imu_i != imu_j) {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(
                                pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                std::vector<double *>{
                                    para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1],
                                    para_Feature[feature_index], para_Td[0]
                                },
                                std::vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        } else {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(
                                pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                std::vector<double *>{
                                    para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]
                                },
                                std::vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++) {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if (USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        std::vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    } else {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks),
                       std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1])) {
            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid) {
                std::vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
                    assert(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "begin marginalization");
            marginalization_info->preMarginalize();
            RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "begin marginalization");
            marginalization_info->marginalize();
            RCLCPP_DEBUG(rclcpp::get_logger("vins_estimator"), "end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++) {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE) {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if (USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                } else {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if (USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];


            std::vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
    //printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    //printf("whole time for ceres: %f \n", t_whole.toc());
}

void Estimator::slideWindow() {
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD) {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE) {
            for (int i = 0; i < WINDOW_SIZE; i++) {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
                if (USE_IMU) {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if (USE_IMU) {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL) {
                std::map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            slideWindowOld();
        }
    } else {
        if (frame_count == WINDOW_SIZE) {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if (USE_IMU) {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++) {
                    double tmp_dt = dt_buf[frame_count][i];
                    Eigen::Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Eigen::Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew() {
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld() {
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth) {
        Eigen::Matrix3d R0, R1;
        Eigen::Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    } else
        f_manager.removeBack();
}


void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T) {
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T) {
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame() {
    //printf("predict pts in next frame\n");
    if (frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    std::map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id: f_manager.feature) {
        if (it_per_id.estimated_depth > 0) {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if ((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count) {
                double depth = it_per_id.estimated_depth;
                Eigen::Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Eigen::Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Eigen::Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Eigen::Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    featureTracker.setPrediction(predictPts);
    //printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Eigen::Matrix3d &Ri, Eigen::Vector3d &Pi, Eigen::Matrix3d &rici,
                                    Eigen::Vector3d &tici,
                                    Eigen::Matrix3d &Rj, Eigen::Vector3d &Pj, Eigen::Matrix3d &ricj,
                                    Eigen::Vector3d &ticj,
                                    double depth, Eigen::Vector3d &uvi, Eigen::Vector3d &uvj) {
    Eigen::Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Eigen::Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Eigen::Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(std::set<int> &removeIndex) {
    //return;
    int feature_index = -1;
    for (auto &it_per_id: f_manager.feature) {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame: it_per_id.feature_per_frame) {
            imu_j++;
            if (imu_i != imu_j) {
                Eigen::Vector3d pts_j = it_per_frame.point;
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                     Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                     depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if (STEREO && it_per_frame.is_stereo) {
                Eigen::Vector3d pts_j_right = it_per_frame.pointRight;
                if (imu_i != imu_j) {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                         Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                         depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                } else {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                         Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                         depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
            }
        }
        double ave_err = err / errCnt;
        if (ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);
    }
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity) {
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

void Estimator::updateLatestStates() {
    mPropagate.lock();
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf.lock();
    std::queue<std::pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    std::queue<std::pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    mBuf.unlock();
    while (!tmp_accBuf.empty()) {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mPropagate.unlock();
}
