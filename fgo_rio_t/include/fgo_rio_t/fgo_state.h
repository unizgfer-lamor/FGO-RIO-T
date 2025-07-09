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
#include "fgo_rio_t/helper.h"

/**
 * @brief State representation for radar-inertial odometry
 *
 * This struct encapsulates the complete state of the system at a given time point,
 * including position, orientation, velocity, IMU data, and temporal offset.
 * It provides methods to access state components and convert to ROS messages.
 */
struct State
{
    /**
     * @brief Constructor for State with temporal delay
     *
     * Creates a state object that includes temporal offset between sensors.
     *
     * @param odom_frame_id Frame ID for the odometry
     * @param W_p_IW Position of IMU in world frame
     * @param R_IW Rotation from world to IMU frame
     * @param W_v Velocity in world frame
     * @param imu Pointer to the IMU message
     * @param integrator IMU preintegration object
     * @param time_delay Temporal delay between sensors
     */
    State(const std::string &odom_frame_id, const gtsam::Point3 &W_p_IW,
          const gtsam::Rot3 &R_IW, const gtsam::Vector3 &W_v,
          const sensor_msgs::ImuConstPtr &imu,
          const gtsam::PreintegratedCombinedMeasurements &integrator,
          const double &time_delay);

    /**
     * @brief Constructor for State without temporal delay
     *
     * Creates a state object without temporal offset.
     *
     * @param odom_frame_id Frame ID for the odometry
     * @param W_p_IW Position of IMU in world frame
     * @param R_IW Rotation from world to IMU frame
     * @param W_v Velocity in world frame
     * @param imu Pointer to the IMU message
     * @param integrator IMU preintegration object
     */
    State(const std::string &odom_frame_id, const gtsam::Point3 &W_p_IW,
          const gtsam::Rot3 &R_IW, const gtsam::Vector3 &W_v,
          const sensor_msgs::ImuConstPtr &imu,
          const gtsam::PreintegratedCombinedMeasurements &integrator);

    std::string odom_frame_id;                           ///< Frame ID for the odometry
    gtsam::Point3 W_p_IW;                                ///< Position of IMU in world frame [m]
    gtsam::Rot3 R_IW;                                    ///< Rotation from world to IMU frame
    gtsam::Vector3 W_v;                                  ///< Velocity in world frame [m/s]
    sensor_msgs::ImuConstPtr imu;                        ///< Pointer to the IMU message
    gtsam::PreintegratedCombinedMeasurements integrator; ///< IMU preintegration object
    double time_delay;                                   ///< Temporal delay between sensors [s]

    /**
     * @brief Get the combined pose (position and orientation)
     *
     * @return gtsam::Pose3 The pose in the world frame
     */
    gtsam::Pose3 getPose() const;

    /**
     * @brief Get the velocity vector
     *
     * @return gtsam::Velocity3 The velocity in the world frame [m/s]
     */
    gtsam::Velocity3 getVelocity() const;

    /**
     * @brief Get the IMU bias estimates
     *
     * @return gtsam::imuBias::ConstantBias The accelerometer and gyroscope biases
     */
    gtsam::imuBias::ConstantBias getBias() const;

    /**
     * @brief Get the temporal delay between sensors
     *
     * @return double The estimated time offset [s]
     */
    double getTemporalDelay() const;

    /**
     * @brief Get the navigation state (pose and velocity)
     *
     * @return gtsam::NavState Combined pose and velocity state
     */
    gtsam::NavState getNavState() const;

    /**
     * @brief Convert the state to ROS odometry message
     *
     * Creates an odometry message with position, orientation,
     * linear and angular velocities for publishing.
     *
     * @return nav_msgs::Odometry The odometry message for publishing
     */
    nav_msgs::Odometry getOdometry() const;
};