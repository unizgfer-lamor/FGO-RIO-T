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

#include "fgo_rio_t/fgo_state.h"

State::State(const std::string &odom_frame_id, const gtsam::Point3 &W_p_IW,
             const gtsam::Rot3 &R_IW, const gtsam::Vector3 &W_v,
             const sensor_msgs::ImuConstPtr &imu,
             const gtsam::PreintegratedCombinedMeasurements &integrator,
             const double &time_delay)
{
    this->odom_frame_id = odom_frame_id;
    this->W_p_IW = W_p_IW;
    this->R_IW = R_IW;
    this->W_v = W_v;
    this->imu = imu;
    this->integrator = integrator;
    this->time_delay = time_delay;
}

State::State(const std::string &odom_frame_id, const gtsam::Point3 &W_p_IW,
             const gtsam::Rot3 &R_IW, const gtsam::Vector3 &W_v,
             const sensor_msgs::ImuConstPtr &imu,
             const gtsam::PreintegratedCombinedMeasurements &integrator)
{
    this->odom_frame_id = odom_frame_id;
    this->W_p_IW = W_p_IW;
    this->R_IW = R_IW;
    this->W_v = W_v;
    this->imu = imu;
    this->integrator = integrator;
}

gtsam::Pose3 State::getPose() const { return gtsam::Pose3(R_IW, W_p_IW); }

gtsam::Velocity3 State::getVelocity() const { return gtsam::Velocity3(W_v); }

gtsam::imuBias::ConstantBias State::getBias() const
{
    return integrator.biasHat();
}

double State::getTemporalDelay() const { return time_delay; }

gtsam::NavState State::getNavState() const
{
    return gtsam::NavState(R_IW, W_p_IW, W_v);
}

nav_msgs::Odometry State::getOdometry() const
{
    nav_msgs::Odometry odom;
    odom.header.stamp = imu->header.stamp;
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = imu->header.frame_id;
    odom.pose.pose.orientation = tf2::toMsg(R_IW.toQuaternion());
    odom.pose.pose.position = tf2::toMsg(W_p_IW);
    tf2::toMsg(R_IW.unrotate(W_v), odom.twist.twist.linear);
    odom.twist.twist.angular = imu->angular_velocity;
    return odom;
}