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

#include "fgo_rio_t/odometry.h"

Odometry::Odometry(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
}

bool Odometry::Initialization()
{
   if (InitializeSubscribers() && InitializePublishers() && LoadParameters() && InitializeOptimizator())
   {
      return true;
   }
   return false;
}

bool Odometry::InitializeSubscribers()
{
   int queue_size = 1;
   if (!LoadParam<int>(nh_private_, "ROS/queue_size", &queue_size, queue_size))
      return false;

   std::string imu_topic;
   if (!LoadParam<std::string>(nh_private_, "ROS/imu_topic", &imu_topic, "/imu"))
      return false;
   imu_sub_ = nh_.subscribe(imu_topic, queue_size, &Odometry::ImuCallback, this);

   std::string radar_topic;
   if (!LoadParam<std::string>(nh_private_, "ROS/radar_topic", &radar_topic, "/radar"))
      return false;
   radar_sub_ = nh_.subscribe(radar_topic, queue_size, &Odometry::RadarCallback, this);

   return true;
}

bool Odometry::InitializePublishers()
{
   int queue_size = 1;
   if (!LoadParam<int>(nh_private_, "ROS/queue_size", &queue_size, queue_size))
   {
      std::cout << "Failed to load queue size parameter." << std::endl;
      return false;
   }

   std::string odometry_topic;
   if (!LoadParam<std::string>(nh_private_, "ROS/odometry_topic", &odometry_topic, "/odometry"))
   {
      std::cout << "Failed to load odometry topic parameter." << std::endl;
      return false;
   }
   odom_pub_ = nh_private_.advertise<nav_msgs::Odometry>(odometry_topic, queue_size);

   std::string odometry_optimization_topic;
   if (!LoadParam<std::string>(nh_private_, "ROS/odometry_optimization_topic", &odometry_optimization_topic, "/odometry_optimization"))
   {
      std::cout << "Failed to load odometry optimization topic parameter." << std::endl;
      return false;
   }
   odom_opt_pub_ = nh_private_.advertise<nav_msgs::Odometry>(odometry_optimization_topic, queue_size);

   std::string time_offset_topic;
   if (!LoadParam<std::string>(nh_private_, "ROS/time_offset_topic", &time_offset_topic, "/dt"))
   {
      std::cout << "Failed to load time offset topic parameter." << std::endl;
      return false;
   }
   dt_pub_ = nh_private_.advertise<fgo_rio_t::time_offset>(time_offset_topic, queue_size);

   return true;
}

bool Odometry::InitializeOptimizator()
{
   gtsam::ISAM2Params parameters;

   double relinearize_threshold;
   if (!LoadParam<double>(nh_private_, "iSAM2/relinearize_threshold", &relinearize_threshold, double(0.1)))
   {
      std::cout << "Failed to load iSAM2 relinearize threshold parameter." << std::endl;
      return false;
   }
   parameters.relinearizeThreshold = relinearize_threshold;

   if (!LoadParam<int>(nh_private_, "iSAM2/relinearize_skip", &parameters.relinearizeSkip, int(10)))
   {
      std::cout << "Failed to load iSAM2 relinearize threshold parameter." << std::endl;
      return false;
   }

   double smoother_lag;
   if (!LoadParam<double>(nh_private_, "iSAM2/smoother_lag", &smoother_lag, double(2.0)))
   {
      std::cout << "Failed to load iSAM2 smoother lag parameter." << std::endl;
      return false;
   }

   optimizer_.SetOnlineTemporalCalibration(online_temporal_);

   return true;
}

bool Odometry::LoadParameters()
{

   if (!LoadParamVector(nh_private_, "PriorNoise/R_IW", &prior_noise_.R_IW, gtsam::Vector3(0.0, 0.0, 0.0)))
   {
      std::cout << "Failed to load prior noise R_IW parameter." << std::endl;
      return false;
   }

   if (!LoadParamVector(nh_private_, "PriorNoise/W_p_IW", &prior_noise_.W_p_IW, gtsam::Vector3(0.0, 0.0, 0.0)))
   {
      std::cout << "Failed to load prior noise W_p_IW parameter." << std::endl;
      return false;
   }
   if (!LoadParamVector(nh_private_, "PriorNoise/W_v", &prior_noise_.W_v, gtsam::Vector3(0.0, 0.0, 0.0)))
   {
      std::cout << "Failed to load prior noise W_v parameter." << std::endl;
      return false;
   }
   if (!LoadParamVector(nh_private_, "PriorNoise/b_a", &prior_noise_.b_a, gtsam::Vector3(0.0, 0.0, 0.0)))
   {
      std::cout << "Failed to load prior noise b_a parameter." << std::endl;
      return false;
   }
   if (!LoadParamVector(nh_private_, "PriorNoise/b_g", &prior_noise_.b_g, gtsam::Vector3(0.0, 0.0, 0.0)))
   {
      std::cout << "Failed to load prior noise b_g parameter." << std::endl;
      return false;
   }

   if (!LoadParam<double>(nh_private_, "PriorNoise/delta_t", &prior_noise_.t, double(0.1)))
   {
      std::cout << "Failed to load prior noise delta_t parameter." << std::endl;
      return false;
   }

   if (!LoadParam<double>(nh_private_, "IMU/b_a_sigma", &params_.bias_acc_sigma))
   {
      std::cout << "Failed to initialize parameter: IMU/bias_acc_sigma" << std::endl;
      return false;
   }
   if (!LoadParam<double>(nh_private_, "IMU/b_g_sigma", &params_.bias_omega_sigma))
   {
      std::cout << "Failed to initialize parameter: IMU/bias_omega_sigma" << std::endl;
      return false;
   }
   if (!LoadParam<double>(nh_private_, "IMU/b_a_int_sigma", &params_.bias_acc_int_sigma))
   {
      std::cout << "Failed to initialize parameter: IMU/bias_acc_int_sigma" << std::endl;
      return false;
   }
   if (!LoadParam<double>(nh_private_, "IMU/b_g_int_sigma", &params_.bias_omega_int_sigma))
   {
      std::cout << "Failed to initialize parameter: IMU/bias_omega_int_sigma" << std::endl;
      return false;
   }
   if (!LoadParam<double>(nh_private_, "IMU/a_sigma", &params_.acc_sigma))
   {
      std::cout << "Failed to initialize parameter: IMU/acc_sigma" << std::endl;
      return false;
   }

   if (!LoadParam<double>(nh_private_, "IMU/g_sigma", &params_.gyro_sigma))
   {
      std::cout << "Failed to initialize parameter: IMU/gyro_sigma" << std::endl;
      return false;
   }
   if (!LoadParam<double>(nh_private_, "IMU/int_sigma", &params_.integration_sigma))
   {
      std::cout << "Failed to initialize parameter: IMU/integration_sigma" << std::endl;
      return false;
   }

   if (!LoadParam<std::string>(nh_private_, "ROS/odometry_frame_id", &odom_frame_id_, "odom"))
   {
      std::cout << "Failed to load odometry frame id parameter." << std::endl;
      return false;
   }

   if (!LoadParam<double>(nh_private_, "IMU/duration", &init_dt_, double(10.0)))
   {
      std::cout << "Failed to load IMU bias initalization duration parameter." << std::endl;
      return false;
   }

   if (!LoadParam<bool>(nh_private_, "IMU/initialize", &initialize_imu_, bool(true)))
   {
      std::cout << "Failed to load IMU initialization parameter." << std::endl;
      return false;
   }

   if (!LoadParam<bool>(nh_private_, "OnlineCalibration/temporal", &online_temporal_, bool(true)))
   {
      std::cout << "Failed to load online temporal calibration parameter." << std::endl;
      return false;
   }
   std::cout << "Online temporal calibration: " << online_temporal_ << std::endl;

   if (!LoadParamVector(nh_private_, "Initial_state/b_0_w_deg", &prior_.b_0_w, gtsam::Vector3(0.0, 0.0, 0.0)))
   {
      std::cout << "Failed to load IMU initial gyro bias parameters." << std::endl;
      return false;
   }

   if (!LoadParamVector(nh_private_, "Initial_state/b_a", &prior_.b_a, gtsam::Vector3(0.0, 0.0, 0.0)))
   {
      std::cout << "Failed to load IMU initial bias acceleration parameter." << std::endl;
      return false;
   }
   if (!LoadParamVector(nh_private_, "Initial_state/b_g", &prior_.b_g, gtsam::Vector3(0.0, 0.0, 0.0)))
   {
      std::cout << "Failed to load IMU initial bias gyroscope parameter." << std::endl;
      return false;
   }

   if (!LoadParamVector(nh_private_, "Initial_state/W_p_IW", &prior_.W_P_IW, gtsam::Vector3(0.0, 0.0, 0.0)))
   {
      std::cout << "Failed to load initial position parameter." << std::endl;
      return false;
   }

   gtsam::Vector4 q_IW;
   if (!LoadParamVector4(nh_private_, "Initial_state/R_IW", &q_IW, gtsam::Vector4(1.0, 0.0, 0.0, 0.0)))
   {
      std::cout << "Failed to load initial orientation parameter." << std::endl;
      return false;
   }
   prior_.R_IW = gtsam::Rot3(q_IW[0], q_IW[1], q_IW[2], q_IW[3]);

   if (!LoadParamVector(nh_private_, "Initial_state/W_v", &prior_.W_v, gtsam::Vector3(0.0, 0.0, 0.0)))
   {
      std::cout << "Failed to load initial velocity parameter." << std::endl;
      return false;
   }

   if (!LoadParam<double>(nh_private_, "Initial_state/delta_t", &prior_.t, double(0.0)))
   {
      std::cout << "Failed to load initialization dt parameter." << std::endl;
      return false;
   }

   if (!LoadParamVector4(nh_private_, "ExtrinsicCalibration/R_IR", &q_IW, gtsam::Vector4(1.0, 0.0, 0.0, 0.0)))
   {
      std::cout << "Failed to load extrinsic orientation parameter." << std::endl;
      return false;
   }
   gtsam::Vector3 p_IR;
   if (!LoadParamVector(nh_private_, "ExtrinsicCalibration/p_IR", &p_IR, gtsam::Vector3(0.0, 0.0, 0.0)))
   {
      std::cout << "Failed to load extrinsic translation parameter." << std::endl;
      return false;
   }
   extrinsic_ = gtsam::Pose3(gtsam::Rot3(q_IW[0], q_IW[1], q_IW[2], q_IW[3]), gtsam::Point3(p_IR[0], p_IR[1], p_IR[2]));

   if (!LoadParam<double>(nh_private_, "Noise/Radar/huber_k", &huber_k, double(0.1)))
   {
      std::cout << "Failed to load radar noise huber_k parameter." << std::endl;
      return false;
   }
   if (!LoadParam<double>(nh_private_, "Noise/delta_t", &delta_t, double(1.0e-7)))
   {
      std::cout << "Failed to load delta_t parameter." << std::endl;
      return false;
   }

   std::cout << "Printing initial state: " << std::endl;
   std::cout << "W_p_IW: " << prior_.W_P_IW.transpose() << std::endl;
   std::cout << "R_IW: " << prior_.R_IW << std::endl;
   std::cout << "W_v: " << prior_.W_v.transpose() << std::endl;
   std::cout << "Time delay: " << prior_.t << std::endl;
   std::cout << "b_a: " << prior_.b_a.transpose() << std::endl;
   std::cout << "b_g: " << prior_.b_g.transpose() << std::endl;

   return true;
}

void Odometry::ImuCallback(const sensor_msgs::ImuConstPtr &msg)
{

   if (!imu_bias_initialized_ && initialize_imu_)
   {
      imu_init_queue_.push_back(msg);

      if ((double(imu_init_queue_.back()->header.stamp.toSec()) - double(imu_init_queue_.front()->header.stamp.toSec())) > init_dt_)
      {
         imu_bias_initialized_ = true;
         ROS_INFO("IMU bias initialized");

         gtsam::Vector3 gyro_bias;
         gtsam::Vector3 acc_bias;
         gtsam::Vector3 gyro_bias_sum = gtsam::Vector3::Zero();
         gtsam::Vector3 acc_bias_sum = gtsam::Vector3::Zero();
         for (const auto &imu_msg : imu_init_queue_)
         {
            gyro_bias_sum += gtsam::Vector3(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
            acc_bias_sum += gtsam::Vector3(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
         }
         gyro_bias = gyro_bias_sum / imu_init_queue_.size() + prior_.b_0_w * M_PI / 180.0;
         acc_bias = acc_bias_sum / imu_init_queue_.size();

         double roll = -1 * atan2(acc_bias.y(), -acc_bias.z());
         double pitch = atan2(acc_bias.x(), sqrt(acc_bias.y() * acc_bias.y() + acc_bias.z() * acc_bias.z()));

         std::cout << "Roll: " << roll << std::endl;
         std::cout << "Pitch: " << pitch << std::endl;

         gtsam::Rot3 R_IW = gtsam::Rot3::RzRyRx(0.0, pitch, roll);

         if (!LoadParamVector(nh_private_, "Initial_state/b_a", &acc_bias, gtsam::Vector3(0.0, 0.0, 0.0)))
         {
            std::cout << "Failed to load IMU initial bias acceleration parameter." << std::endl;
            return;
         }

         ROS_INFO("Gyro bias: [%f, %f, %f]", gyro_bias.x(), gyro_bias.y(), gyro_bias.z());
         ROS_INFO("Acc bias: [%f, %f, %f]", acc_bias.x(), acc_bias.y(), acc_bias.z());

         InitPreintegratedIMU(nh_private_, &integrator_, acc_bias, gyro_bias, params_);

         if (online_temporal_)
         {
            initial_state_t_ = std::make_shared<State>(
                initial_state_t_->odom_frame_id, prior_.W_P_IW, prior_.R_IW,
                prior_.W_v, msg, integrator_, prior_.t);

            std::cout << "Printing initial state: " << std::endl;
            std::cout << "W_p_IW: " << initial_state_t_->W_p_IW.transpose() << std::endl;
            std::cout << "R_IW: " << initial_state_t_->R_IW << std::endl;
            std::cout << "W_v: " << initial_state_t_->W_v.transpose() << std::endl;
            std::cout << "Time delay: " << initial_state_t_->time_delay << std::endl;
         }
         else
         {
            initial_state_ = std::make_shared<State>(
                initial_state_->odom_frame_id, prior_.W_P_IW, prior_.R_IW,
                prior_.W_v, msg, integrator_);
            std::cout << "Printing initial state: " << std::endl;
            std::cout << "W_p_IW: " << initial_state_->W_p_IW.transpose() << std::endl;
            std::cout << "R_IW: " << initial_state_->R_IW << std::endl;
            std::cout << "W_v: " << initial_state_->W_v.transpose() << std::endl;
         }

         imu_init_queue_.clear();
      }
   }
   else if (!initialize_imu_)
   {

      ROS_INFO("Initial IMU bias");
      ROS_INFO("Gyro bias: [%f, %f, %f]", prior_.b_g.x(), prior_.b_g.y(), prior_.b_g.z());
      ROS_INFO("Acc bias: [%f, %f, %f]", prior_.b_a.x(), prior_.b_a.y(), prior_.b_a.z());
      prior_.b_g = prior_.b_g + prior_.b_0_w * M_PI / 180.0;

      InitPreintegratedIMU(nh_private_, &integrator_, prior_.b_a, prior_.b_g, params_);

      initialize_imu_ = true;
      imu_bias_initialized_ = true;

      if (online_temporal_)
      {
         initial_state_t_ = std::make_shared<State>(
             initial_state_t_->odom_frame_id, prior_.W_P_IW, prior_.R_IW,
             prior_.W_v, msg, integrator_, prior_.t);

         std::cout << "Printing initial state: " << std::endl;
         std::cout << "W_p_IW: " << initial_state_t_->W_p_IW.transpose() << std::endl;
         std::cout << "R_IW: " << initial_state_t_->R_IW << std::endl;
         std::cout << "W_v: " << initial_state_t_->W_v.transpose() << std::endl;
         std::cout << "Time delay: " << initial_state_t_->time_delay << std::endl;
      }
      else
      {
         initial_state_ = std::make_shared<State>(
             initial_state_->odom_frame_id, prior_.W_P_IW, prior_.R_IW,
             prior_.W_v, msg, integrator_);
         std::cout << "Printing initial state: " << std::endl;
         std::cout << "W_p_IW: " << initial_state_->W_p_IW.transpose() << std::endl;
         std::cout << "R_IW: " << initial_state_->R_IW << std::endl;
         std::cout << "W_v: " << initial_state_->W_v.transpose() << std::endl;
      }
   }
   else
   {
      if ((initial_state_t_->imu == nullptr) && (initial_state_->imu == nullptr))
      {
         std::cout << "Initial state is null" << std::endl;
         return;
      }

      else if (propagation_.empty())
      {
         ROS_INFO("Initializing states with initial state.");
         if (online_temporal_)
         {
            propagation_.emplace_back(initial_state_t_, idx_++);
         }
         else
         {
            propagation_.emplace_back(initial_state_, idx_++);
         }
         optimizer_.AddPriorFactor(propagation_.back(), prior_noise_);
         return;
      }

      imu_queue_.push_back(msg);
      if (!radar_queue_.empty())
      {
         ProcessVelocities();
      }
   }
}

void Odometry::ProcessVelocities()
{
   geometry_msgs::TwistWithCovarianceStamped radar_msg = radar_queue_.front();
   if (radar_queue_.empty() ||
       imu_queue_.empty())
   {
      std::cout << "Radar or IMU queue is empty" << std::endl;
      return;
   }
   if (propagation_.empty())
   {
      std::cout << "Propagation queue is empty" << std::endl;
      return;
   }
   if (imu_queue_.back()->header.stamp < radar_msg.header.stamp)
   {
      return;
   }
   radar_queue_.pop_front();

   while (!imu_queue_.empty())
   {
      ProcessIMUMeasurements(imu_queue_.front());
      imu_queue_.pop_front();
   }

   auto split_iter = SplitPropagation(radar_msg.header.stamp);
   if (split_iter == propagation_.end())
   {
      std::cout << "Split iterator is end" << std::endl;
      imu_queue_.clear();
      return;
   }
   split_iter->extrinsics_ = extrinsic_;
   split_iter->velocity_ = gtsam::Velocity3(radar_msg.twist.twist.linear.x,
                                            radar_msg.twist.twist.linear.y,
                                            radar_msg.twist.twist.linear.z);
   Eigen::Matrix3d covariance;
   covariance << radar_msg.twist.covariance[0], radar_msg.twist.covariance[1], radar_msg.twist.covariance[2],
       radar_msg.twist.covariance[6], radar_msg.twist.covariance[7], radar_msg.twist.covariance[8],
       radar_msg.twist.covariance[12], radar_msg.twist.covariance[13], radar_msg.twist.covariance[14];
   split_iter->covariance_ = covariance;

   gtsam::SharedNoiseModel noise_model_delta_t_ = gtsam::noiseModel::Gaussian::Covariance((gtsam::Vector1() << delta_t).finished().asDiagonal());
   optimizer_.AddFactors(*split_iter, *std::next(split_iter),
                         huber_k, noise_model_delta_t_, radar_msg.header.stamp);

   optimizer_.Solve(propagation_);

   imu_queue_.clear();
}

void Odometry::ProcessIMUMeasurements(const sensor_msgs::ImuConstPtr &msg)
{
   auto new_result = optimizer_.GetResult(&propagation_);

   if (!propagation_.back().AddImuMeasurement(msg, online_temporal_))
   {
      std::cout << "Failed to add IMU measurement" << std::endl;
      return;
   }
   auto new_odometry = propagation_.back().getLastState()->getOdometry();
   odom_pub_.publish(new_odometry);

   if (new_result)
   {

      odom_opt_pub_.publish(new_odometry);

      if (online_temporal_)
      {
         double dt = propagation_.back().getLastState()->getTemporalDelay();
         std::cout << "dt: " << dt << std::endl;
         fgo_rio_t::time_offset dt_msg;
         dt_msg.dt = dt;
         dt_msg.header = propagation_.back().getLastState()->imu->header;
         dt_pub_.publish(dt_msg);
      }
   }
}

void Odometry::RadarCallback(const geometry_msgs::TwistWithCovarianceStamped &msg)
{
   if (imu_bias_initialized_ && initialize_imu_)
   {
      radar_queue_.push_back(msg);
   }

   else
   {
      ROS_INFO("IMU bias not initialized or IMU not initialized");
   }
}

std::deque<Propagation>::iterator Odometry::SplitPropagation(const ros::Time &t)
{
   auto iterator = propagation_.begin();
   for (; iterator != propagation_.end(); ++iterator)
   {

      Propagation propagation_to_t, propagation_from_t;
      if (iterator->Split(t, &idx_, &propagation_to_t, &propagation_from_t, online_temporal_))
      {
         *iterator = propagation_to_t;
         auto distance = std::distance(propagation_.begin(), iterator);

         propagation_.insert(std::next(iterator), propagation_from_t);
         return std::next(propagation_.begin(), distance);
      }
   }

   return iterator;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "fgo_rio_t_node");
   ros::NodeHandle nh;
   ros::NodeHandle nh_private("~");

   Odometry odometry(nh, nh_private);

   if (!odometry.Initialization())
   {
      ROS_ERROR("Failed to initialize odometry.");
      return 1;
   }

   ros::spin();
   return 0;
}