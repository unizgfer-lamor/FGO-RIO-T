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
#include "fgo_rio_t/fgo_state.h"
#include "fgo_rio_t/fgo_optimization.h"

/**
 * @brief Main odometry class for radar-inertial state estimation
 *
 * The Odometry class implements a factor graph-based fusion of IMU and
 * radar measurements to provide accurate state estimation. It handles
 * initialization of sensor biases, temporal calibration between sensors,
 * and state propagation with periodic optimization.
 */
class Odometry
{

public:
    /**
     * @brief Constructor for the Odometry class
     *
     * @param nh Global ROS node handle
     * @param nh_private Private ROS node handle for parameter loading
     */
    Odometry(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    /**
     * @brief Initialize the odometry system
     *
     * Sets up subscribers, publishers, loads parameters, and initializes
     * the optimizer with appropriate configuration.
     *
     * @return true if initialization was successful
     * @return false if any part of initialization failed
     */
    bool Initialization();

private:
    ros::NodeHandle nh_;         ///< Global ROS node handle
    ros::NodeHandle nh_private_; ///< Private ROS node handle for parameters

    ros::Subscriber imu_sub_;   ///< Subscriber for IMU measurements
    ros::Subscriber radar_sub_; ///< Subscriber for radar velocity measurements

    ros::Publisher odom_pub_;     ///< Publisher for navigation state odometry
    ros::Publisher odom_opt_pub_; ///< Publisher for optimized odometry
    ros::Publisher dt_pub_;       ///< Publisher for temporal calibration results

    /**
     * @brief Initialize ROS subscribers for sensors
     *
     * Sets up subscribers for the IMU and radar sensors based on parameters
     * loaded from the parameter server.
     *
     * @return true if all subscribers were successfully initialized
     * @return false if initialization failed
     */
    bool InitializeSubscribers();

    /**
     * @brief Initialize ROS publishers for outputs
     *
     * Sets up publishers for odometry messages and calibration results
     * based on parameters loaded from the parameter server.
     *
     * @return true if all publishers were successfully initialized
     * @return false if initialization failed
     */
    bool InitializePublishers();

    /**
     * @brief Initialize the factor graph optimizer
     *
     * Configures the ISAM2 optimizer with parameters for the fixed-lag smoother
     * and numerical optimization settings.
     *
     * @return true if optimizer was successfully initialized
     * @return false if initialization failed
     */
    bool InitializeOptimizator();

    /**
     * @brief Load parameters from the ROS parameter server
     *
     * Loads all configuration parameters including sensor noise models,
     * prior values for state estimation, IMU parameters, calibration settings,
     * and initial state values.
     *
     * @return true if all required parameters were successfully loaded
     * @return false if loading any required parameter failed
     */
    bool LoadParameters();

    /**
     * @brief Callback function for IMU data
     *
     * Processes incoming IMU measurements, handles initial bias estimation,
     * and triggers radar measurement processing when available.
     *
     * @param msg Pointer to the received IMU message
     */
    void ImuCallback(const sensor_msgs::ImuConstPtr &msg);

    /**
     * @brief Callback function for radar velocity data
     *
     * Queues incoming radar velocity measurements for processing.
     *
     * @param msg The radar velocity message with covariance
     */
    void RadarCallback(const geometry_msgs::TwistWithCovarianceStamped &msg);

    /**
     * @brief Process queued radar velocity measurements
     *
     * Integrates IMU measurements up to the radar timestamp, splits
     * the state trajectory, adds factors to the optimization graph,
     * and triggers graph optimization.
     */
    void ProcessVelocities();

    /**
     * @brief Process IMU measurements for state propagation
     *
     * Integrates IMU measurements to propagate the state forward in time,
     * publishes the propagated odometry, and updates the state after optimization.
     *
     * @param msg Pointer to the IMU message to process
     */
    void ProcessIMUMeasurements(const sensor_msgs::ImuConstPtr &msg);

    std::string odom_frame_id_; ///< Frame ID for published odometry

    bool imu_bias_initialized_{false};                    ///< Flag indicating if IMU biases are initialized
    bool initialize_imu_{true};                           ///< Flag to enable IMU initialization
    std::deque<sensor_msgs::ImuConstPtr> imu_init_queue_; ///< Queue for IMU messages during initialization
    double init_dt_{10.0};                                ///< Duration in seconds for IMU bias initialization

    bool online_temporal_{true}; ///< Flag to enable online temporal calibration

    gtsam::PreintegratedCombinedMeasurements integrator_; ///< IMU preintegration object
    InitialState prior_;                                  ///< Prior values for the initial state
    PriorNoise prior_noise_;                              ///< Noise models for prior factors
    IMUParam params_;                                     ///< IMU parameters for preintegration

    /**
     * @brief Initial state with temporal calibration
     *
     * Used when online temporal calibration is enabled
     */
    std::shared_ptr<State> initial_state_t_{std::make_shared<State>(
        "odom", gtsam::Z_3x1, gtsam::Rot3::Identity(), gtsam::Z_3x1, nullptr,
        gtsam::PreintegratedCombinedMeasurements(), 0.0)};

    /**
     * @brief Initial state without temporal calibration
     *
     * Used when online temporal calibration is disabled
     */
    std::shared_ptr<State> initial_state_{std::make_shared<State>(
        "odom", gtsam::Z_3x1, gtsam::Rot3::Identity(), gtsam::Z_3x1, nullptr,
        gtsam::PreintegratedCombinedMeasurements())};

    std::deque<sensor_msgs::ImuConstPtr> imu_queue_;                    ///< Queue of IMU messages waiting for processing
    std::deque<geometry_msgs::TwistWithCovarianceStamped> radar_queue_; ///< Queue of radar messages waiting for processing

    uint64_t idx_{0}; ///< Index counter for states in the factor graph
    FGO optimizer_;   ///< Factor graph optimization engine

    std::deque<Propagation> propagation_; ///< Queue of state propagations

    /**
     * @brief Split a propagation at the specified timestamp
     *
     * Divides the state trajectory at a specific time point, typically
     * when a radar measurement arrives to enable adding factors at the
     * correct measurement time.
     *
     * @param t Timestamp at which to split the propagation
     * @return Iterator to the propagation at the split point
     */
    std::deque<Propagation>::iterator SplitPropagation(const ros::Time &t);

    gtsam::Pose3 extrinsic_; ///< Extrinsic calibration between IMU and radar sensors

    double delta_t; ///< Noise parameter for temporal delay estimation
    double huber_k; ///< Huber loss parameter for robustness against outliers
};
