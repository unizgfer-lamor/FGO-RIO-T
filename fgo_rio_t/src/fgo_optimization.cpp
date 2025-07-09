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

#include "fgo_rio_t/fgo_optimization.h"

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::T;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

Propagation::Propagation(const State &initial_state,
                         const uint64_t first_state_idx,
                         const std::optional<uint64_t> &last_state_idx)
    : Propagation(std::make_shared<State>(initial_state), first_state_idx,
                  last_state_idx) {}

Propagation::Propagation(const std::shared_ptr<State> &initial_state,
                         const uint64_t first_state_idx,
                         const std::optional<uint64_t> &last_state_idx)
    : Propagation(std::vector<std::shared_ptr<State>>({initial_state}),
                  first_state_idx, last_state_idx) {}

Propagation::Propagation(const std::vector<std::shared_ptr<State>> &initial_states,
                         const uint64_t first_state_idx,
                         const std::optional<uint64_t> &last_state_idx)
    : states_(initial_states),
      first_state_idx_(first_state_idx),
      last_state_idx_(last_state_idx) {}

bool Propagation::Repropagate(const State &initial_state, const bool &online_temporal)
{
  if (states_.empty())
  {
    return false;
  }
  auto first_state = initial_state;
  first_state.integrator.resetIntegration();

  Propagation propagation(first_state, first_state_idx_, last_state_idx_);
  for (auto it = std::next(states_.begin()); it != states_.end(); ++it)
  {
    if (!(propagation.AddImuMeasurement(((*it)->imu), online_temporal)))
    {
      return false;
    }
  }

  *this = propagation;

  return true;
}

bool Propagation::Split(const ros::Time &t, uint64_t *split_idx,
                        Propagation *propagation_to_t,
                        Propagation *propagation_from_t, const bool &online_temporal) const
{

  if (states_.empty())
  {
    std::cout << "States_ is empty, skipping split." << std::endl;
    return false;
  }
  if (states_.front() == nullptr)
  {
    std::cout << "First state_ is null, skipping split." << std::endl;
    return false;
  }
  if (t < states_.front()->imu->header.stamp)
  {
    std::cout << "t is before first IMU measurement, skipping split." << std::endl;
    return false;
  }
  if (t > states_.back()->imu->header.stamp)
  {
    return false;
  }
  auto state_1 =
      std::lower_bound(states_.begin(), states_.end(), t,
                       [](const std::shared_ptr<State> &state, const ros::Time &t)
                       {
                         return state->imu->header.stamp < t;
                       });
  if (state_1 == states_.begin())
  {
    std::cout << "t is before first IMU measurement, skipping split." << std::endl;
    return false;
  }
  if (state_1 == states_.end())
  {
    return false;
  }
  auto state_0 = std::prev(state_1);

  sensor_msgs::Imu imu;
  imu.header.stamp = t;
  imu.linear_acceleration = (*state_0)->imu->linear_acceleration;
  imu.angular_velocity = (*state_0)->imu->angular_velocity;

  *propagation_to_t =
      Propagation(std::vector<std::shared_ptr<State>>(states_.begin(), state_1),
                  first_state_idx_, (*split_idx));
  if (t > (*state_0)->imu->header.stamp)
    propagation_to_t->AddImuMeasurement(imu, online_temporal);
  else
  {
    std::cout << "t is before or exactly at measurement time, skipping split." << std::endl;
  }

  if (t < (*state_1)->imu->header.stamp)
  {
    if (online_temporal)
    {
      State initial_state = {
          propagation_to_t->getLastState()->odom_frame_id,
          propagation_to_t->getLastState()->W_p_IW,
          propagation_to_t->getLastState()->R_IW,
          propagation_to_t->getLastState()->W_v,
          propagation_to_t->getLastState()->imu,
          propagation_to_t->getLastState()->integrator,
          propagation_to_t->getLastState()->getTemporalDelay()};

      initial_state.integrator.resetIntegrationAndSetBias(
          propagation_to_t->getLastState()->integrator.biasHat());

      *propagation_from_t =
          Propagation(initial_state, (*split_idx), last_state_idx_);

      for (auto it = state_1; it != states_.end(); ++it)
      {
        propagation_from_t->AddImuMeasurement((*it)->imu, online_temporal);
      }
    }
    else
    {
      State initial_state = {
          propagation_to_t->getLastState()->odom_frame_id,
          propagation_to_t->getLastState()->W_p_IW,
          propagation_to_t->getLastState()->R_IW,
          propagation_to_t->getLastState()->W_v,
          propagation_to_t->getLastState()->imu,
          propagation_to_t->getLastState()->integrator};

      initial_state.integrator.resetIntegrationAndSetBias(
          propagation_to_t->getLastState()->integrator.biasHat());

      *propagation_from_t =
          Propagation(initial_state, (*split_idx), last_state_idx_);

      for (auto it = state_1; it != states_.end(); ++it)
      {
        propagation_from_t->AddImuMeasurement((*it)->imu, online_temporal);
      }
    }
  }
  else
  {
    *propagation_from_t =
        Propagation(std::vector<std::shared_ptr<State>>(state_1, states_.end()),
                    (*split_idx), last_state_idx_);
    std::cout << "Split after of exactly at measurement time." << std::endl;
  }
  (*split_idx)++;

  return true;
}

bool Propagation::AddImuMeasurement(const sensor_msgs::ImuConstPtr &msg, const bool &online_temporal)
{

  auto dt = (msg->header.stamp - states_.back()->imu->header.stamp).toSec();
  if (dt < 0)
  {
    std::cout << "Negative dt, skipping IMU integration." << std::endl;
    return false;
  }
  if (dt == 0)
  {
    std::cout << "Zero dt, skipping IMU integration." << std::endl;
    return false;
  }
  gtsam::Vector3 acc = gtsam::Vector3::Zero(), gyro = gtsam::Vector3::Zero();

  tf2::fromMsg(msg->linear_acceleration, acc);
  tf2::fromMsg(msg->angular_velocity, gyro);
  auto integrator = states_.back()->integrator;
  integrator.integrateMeasurement(acc, gyro, dt);
  auto prediction =
      integrator.predict(states_.front()->getNavState(), integrator.biasHat());
  if (online_temporal)
  {
    states_.push_back(std::make_shared<State>(
        states_.back()->odom_frame_id, prediction.pose().translation(),
        prediction.pose().rotation(), prediction.velocity(), msg, integrator, states_.back()->getTemporalDelay()));
  }
  else
  {
    states_.push_back(std::make_shared<State>(
        states_.back()->odom_frame_id, prediction.pose().translation(),
        prediction.pose().rotation(), prediction.velocity(), msg, integrator));
  }
  return true;
}

bool Propagation::AddImuMeasurement(const sensor_msgs::Imu &msg, const bool &online_temporal)
{
  return AddImuMeasurement(sensor_msgs::ImuConstPtr(new sensor_msgs::Imu(msg)), online_temporal);
}

FGO::FGO(double smoother_lag, const gtsam::ISAM2Params &parameters)
{
  smoother_ = gtsam::IncrementalFixedLagSmoother(smoother_lag, parameters);
}

void FGO::AddPriorFactor(const Propagation &propagation,
                         const PriorNoise &noise_)
{
  auto noise_W_p_IW =
      gtsam::noiseModel::Diagonal::Sigmas(noise_.W_p_IW);
  auto noise_R_IW =
      gtsam::noiseModel::Diagonal::Sigmas(noise_.R_IW);
  auto noise_pose = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector6() << noise_.R_IW, noise_.W_p_IW).finished());
  auto noise_W_v =
      gtsam::noiseModel::Diagonal::Sigmas(noise_.W_v);
  auto noise_biases = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector6() << noise_.b_a, noise_.b_g).finished());
  auto noise_t =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(noise_.t));

  AddPriorPoseFactor(propagation, noise_pose);
  AddPriorVelocityFactor(propagation, noise_W_v);
  AddPriorImuBiasFactor(propagation, noise_biases);
  if (online_temporal_)
  {
    AddPriorDeltaTFactor(propagation, noise_t);
  }
}

void FGO::AddConsTFactor(const Propagation &propagation, const gtsam::SharedNoiseModel &noise_model)
{

  auto last_idx = propagation.getLastStateIdx().value();
  auto last_state = propagation.getLastState();
  auto first_idx = propagation.getFirstStateIdx();
  auto first_state = propagation.getFirstState();

  if (!propagation.getLastStateIdx().has_value())
  {
    std::cout << "Propagation has no last state index, skipping adding Doppler factor." << std::endl;
    return;
  }
  else
  {
    auto h_last = gtsam::Expression<double>(T(last_idx));
    auto h_first = gtsam::Expression<double>(T(first_idx));

    auto factor_between = gtsam::BetweenFactor<double>(T(first_idx), T(last_idx), 0.0, noise_model);
    graph_.add(factor_between);
  }
}

void FGO::AddVelocityFactorOnline(
    const Propagation &propagation,
    double &huber_k,
    const ros::Time &timestamp)
{

  if (!propagation.getLastStateIdx().has_value())
  {
    std::cout << "Propagation has no last state index, skipping adding Doppler factor." << std::endl;
    return;
  }
  if (!propagation.velocity_.has_value())
  {
    std::cout << "Propagation has no velocity value, skipping adding Doppler factor." << std::endl;
    return;
  }
  if (!propagation.extrinsics_.has_value())
  {
    std::cout << "Propagation has no extrinsics value, skipping adding Doppler factor." << std::endl;
    return;
  }

  auto idx = propagation.getLastStateIdx().value();
  auto state = propagation.getLastState();
  gtsam::Vector3 B_omega;
  tf2::fromMsg(state->imu->angular_velocity, B_omega);

  auto T_WI = gtsam::Pose3_(X(idx));
  auto T_IR = gtsam::Pose3_(gtsam::Pose3(propagation.extrinsics_.value()));

  auto R_v = unrotate(
      rotation((T_WI)*T_IR),
      gtsam::Vector3_(V(idx)) +
          rotate(rotation(T_WI),
                 cross(correctGyroscope_(ConstantBias_(B(idx)), B_omega),
                       translation(T_IR))));

  double target_time = state->imu->header.stamp.toSec() - propagation.getLastState()->getTemporalDelay();

  std::shared_ptr<State> closest_state = nullptr;
  double min_time_diff = std::numeric_limits<double>::max();

  int iter = 0;
  for (const auto &state1 : propagation.states_)
  {
    iter++;
    double time_diff = std::abs(state1->imu->header.stamp.toSec() - target_time);
    if (time_diff < min_time_diff)
    {
      min_time_diff = time_diff;
      closest_state = state;
    }
  }

  if (!closest_state)
  {
    std::cout << "Could not find closest IMU measurement to radar time" << std::endl;
    return;
  }

  gtsam::Vector3 I_a;
  tf2::fromMsg(closest_state->imu->linear_acceleration, I_a);

  auto acc_c_frame_state0 = (rotate(rotation(T_WI), gtsam::Vector3_(I_a))) - gtsam::Vector3_(gtsam::Vector3(0, 0, 9.81));
  auto acc_c_frame_state = unrotate(rotation(T_WI), gtsam::Vector3_(acc_c_frame_state0)) - gtsam::Vector3_(state->getBias().accelerometer());

  auto a_test = unrotate(
      rotation(T_IR),
      gtsam::Vector3_(acc_c_frame_state));

  auto scaled_delta_velocity = gtsam::Vector3_(multiplyexp(gtsam::Double_(T(idx)), gtsam::Vector3_(a_test)));

  auto R_v_updated = R_v - scaled_delta_velocity;

  auto h = R_v_updated;

  auto z = propagation.velocity_.value();

  auto noise_model_gaussian = gtsam::noiseModel::Gaussian::Covariance(propagation.covariance_.value());

  auto noise_model_huber = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(huber_k), noise_model_gaussian);

  auto factor = gtsam::ExpressionFactor<gtsam::Vector3>(noise_model_huber, z, h);
  graph_.add(factor);
}

void FGO::AddVelocityFactor(
    const Propagation &propagation,
    double &huber_k,
    const ros::Time &timestamp)
{

  if (!propagation.getLastStateIdx().has_value())
  {
    std::cout << "Propagation has no last state index, skipping adding Doppler factor." << std::endl;
    return;
  }
  if (!propagation.velocity_.has_value())
  {
    std::cout << "Propagation has no velocity value, skipping adding Doppler factor." << std::endl;
    return;
  }
  if (!propagation.extrinsics_.has_value())
  {
    std::cout << "Propagation has no extrinsics value, skipping adding Doppler factor." << std::endl;
    return;
  }

  auto idx = propagation.getLastStateIdx().value();
  auto state = propagation.getLastState();
  gtsam::Vector3 B_omega;
  tf2::fromMsg(state->imu->angular_velocity, B_omega);
  
  auto T_WI = gtsam::Pose3_(X(idx));
  auto T_IR = gtsam::Pose3_(gtsam::Pose3(propagation.extrinsics_.value()));

  auto R_v = unrotate(
      rotation(T_WI*T_IR),
      gtsam::Vector3_(V(idx)) +
          rotate(rotation(T_WI),
                 cross(correctGyroscope_(ConstantBias_(B(idx)), B_omega),
                       translation(T_IR))));

  auto h = R_v;

  auto z = propagation.velocity_.value();

  auto noise_model_gaussian = gtsam::noiseModel::Gaussian::Covariance(propagation.covariance_.value());

  auto noise_model_huber = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(huber_k), noise_model_gaussian);

  auto factor = gtsam::ExpressionFactor<gtsam::Vector3>(noise_model_huber, z, h);
  graph_.add(factor);
}

void FGO::AddValues(const Propagation &propagation_to_radar)
{
  if (propagation_to_radar.getLastStateIdx().has_value())
  {
    auto idx = propagation_to_radar.getLastStateIdx().value();
    auto state = propagation_to_radar.getLastState();
    values_.insert(X(idx), state->getPose());
    timestamps_[X(idx)] = state->imu->header.stamp.toSec();

    values_.insert(V(idx), state->getVelocity());
    timestamps_[V(idx)] = state->imu->header.stamp.toSec();

    values_.insert(B(idx), state->getBias());
    timestamps_[B(idx)] = state->imu->header.stamp.toSec();

    if (online_temporal_)
    {
      values_.insert(T(idx), state->getTemporalDelay());
      timestamps_[T(idx)] = state->imu->header.stamp.toSec();
    }
  }
}

bool FGO::Solve(const std::deque<Propagation> &propagations)
{

  if (running_.load())
  {
    std::cout << "Optimization thread still running." << std::endl;
    return false;
  }

  if (thread_.joinable())
  {
    std::cout << "Optimization thread is joinable." << std::endl;
    return false;
  }

  running_.store(true);

  thread_ = std::thread(&FGO::SolveThreaded, this, graph_,
                        values_, timestamps_, propagations);

  graph_.resize(0);
  values_.clear();
  timestamps_.clear();
  return true;
}

void FGO::SolveThreaded(
    const gtsam::NonlinearFactorGraph graph, const gtsam::Values values,
    const gtsam::FixedLagSmoother::KeyTimestampMap stamps,
    std::deque<Propagation> propagations)
{

  if (values.empty())
  {
    std::cout << "Values are empty, skipping optimization." << std::endl;
    running_.store(false);
    return;
  }

  try
  {
    smoother_.update(graph, values, stamps);

    std::cout << "Optimization done." << std::endl;
  }
  catch (const std::exception &e)
  {
    std::cout << "Exception in optimization: " << e.what() << std::endl;
    running_.store(false);
    return;
  }

  gtsam::Values new_values;
  try
  {
    new_values = smoother_.calculateEstimate();
  }
  catch (const std::exception &e)
  {
    std::cout << "Exception in calculateEstimate: " << e.what() << std::endl;
    running_.store(false);
    return;
  }

  auto smallest_time = std::min_element(
      smoother_.timestamps().begin(), smoother_.timestamps().end(),
      [](const auto &a, const auto &b)
      { return a.second < b.second; });
  while (!propagations.empty() &&
         propagations.front().getFirstState()->imu->header.stamp.toSec() <
             smallest_time->second)
  {
    propagations.pop_front();
  }

  for (auto &propagation : propagations)
  {
    if (online_temporal_)
    {
      try
      {
        State initial_state(
            propagation.getFirstState()->odom_frame_id,
            gtsam::Point3(new_values.at<gtsam::Pose3>(X(propagation.getFirstStateIdx())).translation()),
            gtsam::Rot3(new_values.at<gtsam::Pose3>(X(propagation.getFirstStateIdx())).rotation()),
            new_values.at<gtsam::Velocity3>(V(propagation.getFirstStateIdx())),
            propagation.getFirstState()->imu,
            propagation.getFirstState()->integrator,
            new_values.at<double>(T(propagation.getFirstStateIdx())));

        initial_state.integrator.resetIntegrationAndSetBias(
            new_values.at<gtsam::imuBias::ConstantBias>(
                B(propagation.getFirstStateIdx())));

        if (!propagation.Repropagate(initial_state, online_temporal_))
        {
          std::cout << "Failed to repropagate." << std::endl;
          running_.store(false);
          return;
        }
      }
      catch (const std::exception &e)
      {
        std::cout << "Exception in repropagation: " << e.what() << std::endl;
        running_.store(false);
        return;
      }
    }

    else
    {
      try
      {
        State initial_state(
            propagation.getFirstState()->odom_frame_id,
            gtsam::Point3(new_values.at<gtsam::Pose3>(X(propagation.getFirstStateIdx())).translation()),
            gtsam::Rot3(new_values.at<gtsam::Pose3>(X(propagation.getFirstStateIdx())).rotation()),
            new_values.at<gtsam::Velocity3>(V(propagation.getFirstStateIdx())),
            propagation.getFirstState()->imu,
            propagation.getFirstState()->integrator);

        initial_state.integrator.resetIntegrationAndSetBias(
            new_values.at<gtsam::imuBias::ConstantBias>(
                B(propagation.getFirstStateIdx())));

        if (!propagation.Repropagate(initial_state, online_temporal_))
        {
          std::cout << "Failed to repropagate." << std::endl;
          running_.store(false);
          return;
        }
      }
      catch (const std::exception &e)
      {
        std::cout << "Exception in repropagation: " << e.what() << std::endl;
        running_.store(false);
        return;
      }
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  propagations_ = propagations;
  new_result_ = true;

  running_.store(false);
}

void FGO::AddFactors(
    const Propagation &propagation_to_radar,
    const Propagation &propagation_from_radar,
    double &huber_k,
    const gtsam::SharedNoiseModel &noise_model_delta_t,
    const ros::Time &timestamp)
{



  if (propagation_to_radar.getLastStateIdx().has_value())
  {
    AddValues(propagation_to_radar);
  }

  AddCombinedImuFactor(propagation_to_radar);
  // AddCombinedImuFactor(propagation_from_radar);
  if (online_temporal_)
  {

    AddConsTFactor(propagation_to_radar, noise_model_delta_t);
    AddVelocityFactorOnline(propagation_to_radar, huber_k, timestamp);
  }
  else
  {
    AddVelocityFactor(propagation_to_radar, huber_k, timestamp);
  }
}

void FGO::AddCombinedImuFactor(
    const Propagation &propagation)
{
  auto first_id = propagation.getFirstStateIdx();
  auto first_state = propagation.getFirstState();
  auto second_id = propagation.getLastStateIdx();
  auto second_state = propagation.getLastState();

  if (!second_id.has_value())
  {
    std::cout << "Second state index is not available, skipping." << std::endl;
    return;
  }
  else
  {

    graph_.add(gtsam::CombinedImuFactor(
        X(first_id), V(first_id), X(second_id.value()), V(second_id.value()),
        B(first_id), B(second_id.value()), second_state->integrator));
  }
}

void FGO::AddPriorPoseFactor(
    const Propagation &propagation,
    const gtsam::SharedNoiseModel &noise_model)
{
  auto idx = propagation.getFirstStateIdx();
  auto state = propagation.getFirstState();
  values_.insert(X(idx), state->getPose());
  timestamps_[X(idx)] = state->imu->header.stamp.toSec();
  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx), state->getPose(), noise_model));
}

void FGO::AddPriorVelocityFactor(const Propagation &propagation, const gtsam::SharedNoiseModel &noise_model)
{
  auto idx = propagation.getFirstStateIdx();
  auto state = propagation.getFirstState();
  values_.insert(V(idx), state->getVelocity());
  timestamps_[V(idx)] = state->imu->header.stamp.toSec();
  graph_.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), state->getVelocity(), noise_model));
}

void FGO::AddPriorImuBiasFactor(
    const Propagation &propagation,
    const gtsam::SharedNoiseModel &noise_model)
{
  auto idx = propagation.getFirstStateIdx();
  auto state = propagation.getFirstState();
  values_.insert(B(idx), state->getBias());
  timestamps_[B(idx)] = state->imu->header.stamp.toSec();
  graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(idx), state->getBias(),
                                                              noise_model));
}

void FGO::AddPriorDeltaTFactor(
    const Propagation &propagation,
    const gtsam::SharedNoiseModel &noise_model)
{
  auto idx = propagation.getFirstStateIdx();
  auto state = propagation.getFirstState();
  values_.insert(T(idx), state->getTemporalDelay());
  timestamps_[T(idx)] = state->imu->header.stamp.toSec();
  graph_.add(gtsam::PriorFactor<double>(T(idx), state->getTemporalDelay(), noise_model));
}

bool FGO::GetResult(std::deque<Propagation> *propagation)
{
  if (running_.load())
  {
    std::cout << "Optimization thread still running." << std::endl;
    return false;
  }
  if (thread_.joinable())
  {
    thread_.join();
  }
  else
  {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!new_result_)
  {
    std::cout << "No new result available." << std::endl;
    return false;
  }
  new_result_ = false;

  while (!propagation->empty() &&
         propagation->front().getFirstStateIdx() !=
             propagations_.front().getFirstStateIdx())
  {
    propagation->pop_front();
  }

  std::set<std::deque<Propagation>::iterator> updated;
  for (auto it = propagation->begin(); it != propagation->end(); ++it)
  {
    auto result_it = std::find_if(
        propagations_.begin(), propagations_.end(), [it](const auto &p)
        { return p.getFirstStateIdx() == it->getFirstStateIdx() &&
                 p.getLastStateIdx().has_value() &&
                 it->getLastStateIdx().has_value() &&
                 p.getLastStateIdx().value() == it->getLastStateIdx().value(); });
    if (result_it != propagations_.end())
    {
      *it = *result_it;
      updated.insert(it);
      if (result_it == propagations_.begin())
        propagations_.pop_front();
    }
  }

  for (auto it = propagation->begin(); it != propagation->end(); ++it)
  {
    if (updated.count(it) > 0)
      continue;
    if (it == propagation->begin())
    {
      std::cout << "First propagation not updated, skipping." << std::endl;
      continue;
    }
    if (!it->Repropagate((*(std::prev(it)->getLastState())), online_temporal_))
    {
      std::cout << "Failed to repropagate." << std::endl;
      continue;
    }
  }

  return true;
}
