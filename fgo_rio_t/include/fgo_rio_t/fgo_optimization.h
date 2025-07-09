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
 
 class Propagation
 {
 public:
     /**
      * @brief Default constructor for Propagation class
      */
     inline Propagation() {};
 
     /**
      * @brief Constructor for Propagation using an initial state
      * 
      * @param initial_state Initial state object
      * @param first_state_idx Index of the first state
      * @param last_state_idx Optional index of the last state
      */
     Propagation(const State &initial_state, const uint64_t first_state_idx,
                 const std::optional<uint64_t> &last_state_idx = std::nullopt);
 
     /**
      * @brief Constructor for Propagation using a shared pointer to initial state
      * 
      * @param initial_state Shared pointer to initial state
      * @param first_state_idx Index of the first state
      * @param last_state_idx Optional index of the last state
      */
     Propagation(const std::shared_ptr<State> &initial_state,
                 const uint64_t first_state_idx,
                 const std::optional<uint64_t> &last_state_idx = std::nullopt);
 
     /**
      * @brief Constructor for Propagation using a vector of shared pointers to states
      * 
      * @param initial_states Vector of shared pointers to states
      * @param first_state_idx Index of the first state
      * @param last_state_idx Optional index of the last state
      */
     Propagation(const std::vector<std::shared_ptr<State>> &initial_states,
                 const uint64_t first_state_idx,
                 const std::optional<uint64_t> &last_state_idx = std::nullopt);
 
     /**
      * @brief Get the index of the first state
      * 
      * @return uint64_t Index of the first state
      */
     inline uint64_t getFirstStateIdx() const { return first_state_idx_; }
 
     /**
      * @brief Get the index of the last state, or calculate it if not explicitly set
      * 
      * @return std::optional<uint64_t> Index of the last state
      */
     inline std::optional<uint64_t> getLastStateIdx() const {
         if (last_state_idx_) {
             return last_state_idx_;
         } else if (!states_.empty()) {
             return first_state_idx_ + states_.size() - 1;
         }
         return std::nullopt;
     }
 
     /**
      * @brief Get the first state in the propagation
      * 
      * @return std::shared_ptr<State> Shared pointer to the first state
      */
     inline std::shared_ptr<State> getFirstState() const {
         if (states_.empty()) {
             return nullptr;
         }
         return states_.front();
     }
 
     /**
      * @brief Get the last state in the propagation
      * 
      * @return std::shared_ptr<State> Shared pointer to the last state
      */
     inline std::shared_ptr<State> getLastState() const {
         if (states_.empty()) {
             return nullptr;
         }
         return states_.back();
     }
 
     /**
      * @brief Get a state by its index
      * 
      * @param idx Index of the state to retrieve
      * @return std::shared_ptr<State> Shared pointer to the requested state
      * @throws std::out_of_range If the requested index is outside the valid range
      */
     inline std::shared_ptr<State> getState(const uint64_t idx) const {
         if (idx < first_state_idx_ || idx >= first_state_idx_ + states_.size()) {
             throw std::out_of_range("State index out of range");
         }
         return states_[idx - first_state_idx_];
     }
 
     /**
      * @brief Split the propagation at the specified timestamp
      * 
      * This method splits the current propagation into two parts at the given timestamp:
      * one before the timestamp and one after. This is useful for adding measurements
      * that arrived at a specific time.
      * 
      * @param t Timestamp at which to split the propagation
      * @param split_idx Pointer to the split index (will be incremented)
      * @param propagation_to_t Output parameter for the propagation up to time t
      * @param propagation_from_t Output parameter for the propagation after time t
      * @param online_temporal Flag indicating if online temporal calibration is enabled
      * @return true if the split was successful
      * @return false if the split failed (e.g., timestamp out of range)
      */
     bool Split(const ros::Time &t, uint64_t *split_idx,
                Propagation *propagation_to_t,
                Propagation *propagation_from_t, 
                const bool &online_temporal = true) const;
 
     /**
      * @brief Repropagate the states from an initial state
      * 
      * Recomputes all states in the propagation starting from the given initial state.
      * Used after optimization to update the state estimates.
      * 
      * @param initial_state The initial state to start repropagation from
      * @param online_temporal Flag indicating if online temporal calibration is enabled
      * @return true if repropagation was successful
      * @return false if repropagation failed
      */
     bool Repropagate(const State &initial_state, 
                     const bool &online_temporal = true);
 
     /**
      * @brief Add an IMU measurement to the propagation
      * 
      * Integrates a new IMU measurement and creates a new state.
      * 
      * @param msg Shared pointer to the IMU message
      * @param online_temporal Flag indicating if online temporal calibration is enabled
      * @return true if the measurement was successfully added
      * @return false if adding the measurement failed (e.g., negative or zero dt)
      */
     bool AddImuMeasurement(const sensor_msgs::ImuConstPtr &msg, 
                           const bool &online_temporal = true);
 
     /**
      * @brief Add an IMU measurement to the propagation
      * 
      * Overload that takes an IMU message by value.
      * 
      * @param msg The IMU message
      * @param online_temporal Flag indicating if online temporal calibration is enabled
      * @return true if the measurement was successfully added
      * @return false if adding the measurement failed
      */
     bool AddImuMeasurement(const sensor_msgs::Imu &msg, 
                           const bool &online_temporal = true);
 
     std::optional<gtsam::Pose3> extrinsics_;
     std::optional<gtsam::Velocity3> velocity_;
     std::optional<gtsam::Matrix3> covariance_;
     std::vector<std::shared_ptr<State>> states_;
 
 private:
     uint64_t first_state_idx_{0};
     std::optional<uint64_t> last_state_idx_;
 };
 
 class FGO
 {
 public:
     /**
      * @brief Constructor for the FGO (Factor Graph Optimization) class
      * 
      * @param smoother_lag Time in seconds to keep factors (default: 1.0 second)
      * @param parameters Parameters for the ISAM2 optimizer
      */
     FGO(double smoother_lag = 1.0, const gtsam::ISAM2Params &parameters = gtsam::ISAM2Params());
 
     /**
      * @brief Add prior factors to the graph for a propagation
      * 
      * This adds prior factors for position, orientation, velocity, IMU biases,
      * and optionally temporal delay to constrain the initial state.
      * 
      * @param propagation The propagation containing the state to constrain
      * @param noise_ Prior noise models for each state component
      */
     void AddPriorFactor(const Propagation &propagation, const PriorNoise &noise_);
 
     /**
      * @brief Add factors connecting radar measurements to the graph
      * 
      * Adds IMU factors between propagations and velocity factors for radar measurements.
      * 
      * @param propagation_to_radar Propagation up to the radar measurement time
      * @param propagation_from_radar Propagation after the radar measurement time
      * @param huber_k Huber loss parameter for robust optimization
      * @param noise_model_delta_t Noise model for temporal delay
      * @param timestamp Timestamp of the radar measurement
      */
     void AddFactors(
         const Propagation &propagation_to_radar,
         const Propagation &propagation_from_radar,
         double &huber_k,
         const gtsam::SharedNoiseModel &noise_model_delta_t,
         const ros::Time &timestamp);
 
     /**
      * @brief Solve the factor graph optimization problem
      * 
      * Launches a thread to solve the optimization problem with the current
      * graph, values, and timestamps.
      * 
      * @param propagations Collection of propagations to optimize
      * @return true if the optimization was successfully started
      * @return false if another optimization is already running
      */
     bool Solve(const std::deque<Propagation> &propagations);
 
     /**
      * @brief Retrieve the optimization results and update propagations
      * 
      * Updates the provided propagations with the optimized state estimates.
      * 
      * @param propagation Pointer to deque of propagations to update
      * @return true if results were successfully retrieved and propagations updated
      * @return false if no results are available or an error occurred
      */
     bool GetResult(std::deque<Propagation> *propagation);
 
     /**
      * @brief Set whether to use online temporal calibration
      * 
      * @param online_temporal Whether to estimate sensor time offsets online
      */
     void SetOnlineTemporalCalibration(bool online_temporal)
     {
         online_temporal_ = online_temporal;
     }
 
     private:
     /**
      * @brief Add a prior factor for pose
      * 
      * Adds a prior factor for the pose (position and orientation) of the
      * first state in the propagation.
      * 
      * @param propagation The propagation containing the state to constrain
      * @param noise_model Noise model for the prior
      */
     void AddPriorPoseFactor(
         const Propagation &propagation,
         const gtsam::SharedNoiseModel &noise_model);
 
     /**
      * @brief Add a prior factor for velocity
      * 
      * Adds a prior factor for the velocity of the first state in the propagation.
      * 
      * @param propagation The propagation containing the state to constrain
      * @param noise_model Noise model for the prior
      */
     void AddPriorVelocityFactor(
         const Propagation &propagation,
         const gtsam::SharedNoiseModel &noise_model);
 
     /**
      * @brief Add a prior factor for IMU biases
      * 
      * Adds a prior factor for the accelerometer and gyroscope biases
      * of the first state in the propagation.
      * 
      * @param propagation The propagation containing the state to constrain
      * @param noise_model Noise model for the prior
      */
     void AddPriorImuBiasFactor(
         const Propagation &propagation,
         const gtsam::SharedNoiseModel &noise_model);
 
     /**
      * @brief Add a prior factor for temporal delay
      * 
      * Adds a prior factor for the temporal delay between sensors
      * for the first state in the propagation.
      * 
      * @param propagation The propagation containing the state to constrain
      * @param noise_model Noise model for the prior
      */
     void AddPriorDeltaTFactor(
         const Propagation &propagation,
         const gtsam::SharedNoiseModel &noise_model);
 
     /**
      * @brief Add IMU preintegration factor between states
      * 
      * Adds a CombinedImuFactor that constrains consecutive states using
      * preintegrated IMU measurements.
      * 
      * @param propagation The propagation containing the states to connect
      */
     void AddCombinedImuFactor(
         const Propagation &propagation);
 
     /**
      * @brief Add a temporal consistency factor
      * 
      * Adds a factor that enforces the temporal delay to remain consistent
      * between states. Used for online temporal calibration.
      * 
      * @param propagation The propagation containing the states to constrain
      * @param noise_model Noise model for the constraint
      */
     void AddConsTFactor(
         const Propagation &propagation, 
         const gtsam::SharedNoiseModel &noise_model);
 
     /**
      * @brief Add a radar velocity factor with online temporal calibration
      * 
      * Adds a factor that constrains the vehicle velocity based on radar
      * measurements, accounting for the temporal offset between sensors.
      * 
      * @param propagation The propagation containing the state to constrain
      * @param huber_k Huber loss parameter for robust estimation
      * @param timestamp Timestamp of the radar measurement
      */
     void AddVelocityFactorOnline(
         const Propagation &propagation,
         double &huber_k,
         const ros::Time &timestamp);
 
     /**
      * @brief Add a radar velocity factor without temporal calibration
      * 
      * Adds a factor that constrains the vehicle velocity based on radar
      * measurements, assuming perfect time synchronization.
      * 
      * @param propagation The propagation containing the state to constrain
      * @param huber_k Huber loss parameter for robust estimation
      * @param timestamp Timestamp of the radar measurement
      */
     void AddVelocityFactor(
         const Propagation &propagation,
         double &huber_k,
         const ros::Time &timestamp);
 
     /**
      * @brief Add state values to the optimization variables
      * 
      * Inserts pose, velocity, biases, and (if enabled) temporal delay
      * values for the states in the propagation.
      * 
      * @param propagation_to_radar The propagation containing the states to add
      */
     void AddValues(const Propagation &propagation_to_radar);
 
     /**
      * @brief Thread function for solving the optimization problem
      * 
      * Runs the optimization in a separate thread to avoid blocking the
      * main process, then updates propagations with optimized values.
      * 
      * @param graph The factor graph to optimize
      * @param values Initial values for optimization variables
      * @param stamps Timestamps associated with each optimization key
      * @param propagations Collection of propagations to update after optimization
      */
     void SolveThreaded(const gtsam::NonlinearFactorGraph graph,
                        const gtsam::Values values,
                        const gtsam::FixedLagSmoother::KeyTimestampMap stamps,
                        std::deque<Propagation> propagations);
 
     /**
      * @brief Type alias for IMU bias expression in the factor graph
      */
     typedef gtsam::Expression<gtsam::imuBias::ConstantBias> ConstantBias_;
 
     /**
      * @brief Apply gyroscope bias correction to a measurement
      * 
      * Creates an expression that represents a gyroscope measurement
      * corrected by its estimated bias.
      * 
      * @param bias Expression for the gyroscope bias
      * @param measurement Expression for the raw gyroscope measurement
      * @return Expression for the corrected gyroscope measurement
      */
     inline gtsam::Vector3_ correctGyroscope_(const ConstantBias_ &bias,
                                             const gtsam::Vector3_ &measurement)
     {
         return gtsam::Vector3_(bias, &gtsam::imuBias::ConstantBias::correctGyroscope,
                               measurement);
     }
 
     /**
      * @brief Multiply a scalar and a vector in expression form
      * 
      * Creates an expression that represents the multiplication of
      * a scalar and a vector, with proper Jacobians for optimization.
      * 
      * @param s Scalar expression
      * @param v Vector expression
      * @return Expression for the scaled vector
      */
     inline gtsam::Vector3_ multiplyexp(const gtsam::Double_ &s, const gtsam::Vector3_ &v)
     {
         auto multiply = [](const double &a, const gtsam::Vector3 &b, gtsam::OptionalJacobian<3, 1> H1, gtsam::OptionalJacobian<3, 3> H2)
         {
             if (H1)
                 *H1 = b;
             if (H2)
                 *H2 = a * gtsam::I_3x3;
             return a * b;
         };
         return gtsam::Vector3_(multiply, s, v);
     }
 
     gtsam::IncrementalFixedLagSmoother smoother_; ///< Fixed-lag smoother for efficient optimization
     std::deque<Propagation> propagations_;         ///< Collection of propagations with optimization results
     gtsam::NonlinearFactorGraph graph_;            ///< Factor graph containing all constraints
     gtsam::Values values_;                         ///< Values of all optimization variables
     gtsam::FixedLagSmoother::KeyTimestampMap timestamps_; ///< Timestamps for each optimization key
     std::atomic<bool> running_{false};             ///< Flag indicating if optimization thread is running
     std::thread thread_;                           ///< Thread for running the optimization
     bool new_result_{false};                       ///< Flag indicating if new optimization results are available
     std::mutex mutex_;                             ///< Mutex for thread-safe access to optimization results
     bool online_temporal_{true};                   ///< Flag to enable/disable online temporal calibration
 };