/*
 * This file is part of the FGO-RIO-T package.
 *
 * Copyright (c) 2025, Vlaho-Josip Štironja,
 * University of Zagreb Faculty of Electrical Engineering and Computing
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ---
 * This file includes code adapted from the RIO project:
 * https://github.com/ethz-asl/rio
 * Copyright (c) 2024 ETH Zurich, Autonomous Systems Lab, Rik Girod
 * Licensed under the BSD 3-Clause License.
 */

#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include "fgo_rio_t/time_offset.h"
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <tf2_eigen/tf2_eigen.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <thread>

/**
 * @brief Structure containing noise parameters for state priors
 *
 * This struct holds the standard deviations for each component of the
 * factor graph prior factors, used to initialize the state estimation.
 */
struct PriorNoise
{
    gtsam::Vector3 W_p_IW; ///< Position prior noise (standard deviation in x,y,z) [m]
    gtsam::Vector3 R_IW;   ///< Orientation prior noise (standard deviation in roll,pitch,yaw) [rad]
    gtsam::Vector3 W_v;    ///< Velocity prior noise (standard deviation in x,y,z) [m/s]
    gtsam::Vector3 b_g;    ///< Gyroscope bias prior noise (standard deviation in x,y,z) [rad/s]
    gtsam::Vector3 b_a;    ///< Accelerometer bias prior noise (standard deviation in x,y,z) [m/s²]
    double t;              ///< Temporal calibration prior noise (standard deviation) [s]
};

/**
 * @brief Structure containing initial state values
 *
 * This struct holds the initial values for each component of the state
 * vector used to initialize the factor graph optimization.
 */
struct InitialState
{
    gtsam::Vector3 W_P_IW; ///< Initial position of IMU in world frame [m]
    gtsam::Rot3 R_IW;      ///< Initial orientation from world to IMU frame
    gtsam::Vector3 W_v;    ///< Initial velocity in world frame [m/s]
    gtsam::Vector3 b_g;    ///< Initial gyroscope bias estimate [rad/s]
    gtsam::Vector3 b_a;    ///< Initial accelerometer bias estimate [m/s²]
    gtsam::Vector3 b_0_w;  ///< Initial angular velocity estimate [rad/s]
    double t;              ///< Initial temporal calibration estimate [s]
};

/**
 * @brief Structure containing IMU noise parameters
 *
 * This struct holds the noise parameters for the IMU preintegration,
 * which models sensor noise and random walk biases.
 */
struct IMUParam
{
    double bias_acc_sigma;       ///< Accelerometer bias random walk noise [m/s³*sqrt(Hz)]
    double bias_omega_sigma;     ///< Gyroscope bias random walk noise [rad/s²*sqrt(Hz)]
    double bias_acc_int_sigma;   ///< Initial accelerometer bias uncertainty [m/s²]
    double bias_omega_int_sigma; ///< Initial gyroscope bias uncertainty [rad/s]
    double acc_sigma;            ///< Accelerometer measurement noise [m/s²]
    double integration_sigma;    ///< Integration noise (accounts for discretization error) [m/s²]
    double gyro_sigma;           ///< Gyroscope measurement noise [rad/s]
};

/**
 * @brief Load a parameter from the ROS parameter server
 *
 * @tparam T The parameter data type
 * @param nh The ROS node handle
 * @param name The parameter name
 * @param value Pointer to store the parameter value
 * @param default_value Default value if parameter is not found
 * @return true if parameter was loaded successfully
 * @return false if loading the parameter failed
 */
template <typename T>
bool LoadParam(const ros::NodeHandle &nh, const std::string &name,
               T *value, const T &default_value = T())
{
    if (!nh.getParam(name, *value))
    {
        ROS_WARN("Parameter '%s' not found, using default value", name.c_str());
        *value = default_value;
        return true;
    }
    return true;
}

/**
 * @brief Load a 3D vector parameter from the ROS parameter server
 *
 * @param nh The ROS node handle
 * @param name The parameter name
 * @param value Pointer to store the vector
 * @param default_value Default vector if parameter is not found
 * @return true if vector was loaded successfully
 * @return false if loading the vector failed
 */
inline bool LoadParamVector(const ros::NodeHandle &nh, const std::string &name,
                            gtsam::Vector3 *value, const gtsam::Vector3 &default_value = gtsam::Vector3())
{
    std::vector<double> temp;
    if (!nh.getParam(name, temp))
    {
        ROS_WARN("Parameter '%s' not found, using default value", name.c_str());
        *value = default_value;
        return true;
    }
    *value = gtsam::Vector::Map(temp.data(), temp.size());
    return true;
}

/**
 * @brief Load a 4D vector parameter from the ROS parameter server
 *
 * Typically used for quaternion orientation parameters.
 *
 * @param nh The ROS node handle
 * @param name The parameter name
 * @param value Pointer to store the vector
 * @param default_value Default vector if parameter is not found
 * @return true if vector was loaded successfully
 * @return false if loading the vector failed
 */
inline bool LoadParamVector4(const ros::NodeHandle &nh, const std::string &name,
                             gtsam::Vector4 *value, const gtsam::Vector4 &default_value = gtsam::Vector4())
{
    std::vector<double> temp;
    if (!nh.getParam(name, temp))
    {
        ROS_WARN("Parameter '%s' not found, using default value", name.c_str());
        *value = default_value;
        return true;
    }
    *value = gtsam::Vector::Map(temp.data(), temp.size());
    return true;
}

/**
 * @brief Initialize the IMU preintegration parameters
 *
 * Sets up the noise models and initial biases for IMU preintegration.
 *
 * @param nh The ROS node handle
 * @param imu Pointer to the preintegrated IMU measurements object
 * @param acc_bias Initial accelerometer bias
 * @param gyro_bias Initial gyroscope bias
 * @param imu_param IMU parameters struct containing noise values
 * @return true if initialization was successful
 * @return false if initialization failed
 */
inline bool InitPreintegratedIMU(const ros::NodeHandle &nh, gtsam::PreintegratedCombinedMeasurements *imu, const gtsam::Vector3 &acc_bias, const gtsam::Vector3 &gyro_bias, const IMUParam &imu_param)
{

    auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
    imu_params->biasAccCovariance = Eigen::Matrix3d::Identity() * std::pow(imu_param.bias_acc_sigma, 2);
    imu_params->biasOmegaCovariance = Eigen::Matrix3d::Identity() * std::pow(imu_param.bias_omega_sigma, 2);
    imu_params->biasAccOmegaInt.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity() * std::pow(imu_param.bias_acc_int_sigma, 2);
    imu_params->biasAccOmegaInt.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity() * std::pow(imu_param.bias_omega_int_sigma, 2);

    imu_params->accelerometerCovariance = Eigen::Matrix3d::Identity() * std::pow(imu_param.acc_sigma, 2);
    imu_params->integrationCovariance = Eigen::Matrix3d::Identity() * std::pow(imu_param.integration_sigma, 2);
    imu_params->gyroscopeCovariance = Eigen::Matrix3d::Identity() * std::pow(imu_param.gyro_sigma, 2);

    *imu = gtsam::PreintegratedCombinedMeasurements(imu_params, {acc_bias, gyro_bias});
    imu->print("Initial preintegration parameters:");

    std::cout << "Printin all parameters : " << std::endl;
    std::cout << "biasAccCovariance: " << imu_param.bias_acc_sigma << std::endl;
    std::cout << "biasOmegaCovariance: " << imu_param.bias_omega_sigma << std::endl;
    std::cout << "biasAccOmegaInt: " << imu_param.bias_acc_int_sigma << std::endl;
    std::cout << "biasAccOmegaInt: " << imu_param.bias_omega_int_sigma << std::endl;
    std::cout << "accelerometerCovariance: " << imu_param.acc_sigma << std::endl;
    std::cout << "integrationCovariance: " << imu_param.integration_sigma << std::endl;
    std::cout << "gyroscopeCovariance: " << imu_param.gyro_sigma << std::endl;
    std::cout << "acc_bias: " << acc_bias.transpose() << std::endl;
    std::cout << "gyro_bias: " << gyro_bias.transpose() << std::endl;

    return true;
}
