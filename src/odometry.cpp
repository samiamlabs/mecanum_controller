/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, PAL Robotics, S.L.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the PAL Robotics nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/*
* Author: Luca Marchionni
* Author: Bence Magyar
* Author: Enrique Fernández
* Author: Paul Mathieu
*/

#include <mecanum_controller/odometry.h>
#include <boost/bind.hpp>

namespace mecanum_controller
{
namespace bacc = boost::accumulators;

Odometry::Odometry(size_t velocity_rolling_window_size)
: last_update_timestamp_(0.0)
, x_(0.0)
, y_(0.0)
, heading_(0.0)
, linear_(0.0)
, linear_x_(0.0)
, linear_y_(0.0)
, angular_(0.0)
, track_(0.0)
, wheel_base_(0.0)
, wheel_radius_(0.0)
, velocity_rolling_window_size_(velocity_rolling_window_size)
, linear_accel_acc_(RollingWindow::window_size = velocity_rolling_window_size)
, linear_jerk_acc_(RollingWindow::window_size = velocity_rolling_window_size)
, angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
{
}

void Odometry::init(const ros::Time& time)
{
  // Reset accumulators and timestamp:
  resetAccumulators();
  last_update_timestamp_ = time;
}

bool Odometry::update(const double &front_left_vel, const double &front_right_vel,
                      const double &rear_left_vel, const double &rear_right_vel,
                      const ros::Time &time)
{
  linear_x_ = (front_left_vel + front_right_vel + rear_left_vel + rear_right_vel)*(wheel_radius_/4);
  linear_y_ = (-front_left_vel + front_right_vel + rear_left_vel - rear_right_vel)*(wheel_radius_/4);

  angular_ = (-front_left_vel + front_right_vel - rear_left_vel + rear_right_vel)*(wheel_radius_/(4*(wheel_base_ + track_)));

  const double dt = (time - last_update_timestamp_).toSec();
  if (dt < 0.0001)
    return false; // Interval too small to integrate with

  last_update_timestamp_ = time;

  integrateXY(linear_x_*dt, linear_y_*dt, angular_*dt);

  return true;
}

void Odometry::updateOpenLoop(double linear_x, double linear_y, double angular, const ros::Time &time)
{
  /// Save last linear and angular velocity:
  linear_x_ = linear_x;
  linear_y_ = linear_y;
  angular_ = angular;

  /// Integrate odometry:
  const double dt = (time - last_update_timestamp_).toSec();
  last_update_timestamp_ = time;

  integrateXY(linear_x*dt, linear_y*dt, angular_*dt);
}

void Odometry::setWheelParams(double track, double wheel_base, double wheel_radius)
{
  track_ = track;
  wheel_base_ = wheel_base;
  wheel_radius_ = wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

void Odometry::integrateXY(double linear_x, double linear_y, double angular)
{
  const double delta_x = linear_x*cos(heading_) - linear_y*sin(heading_);
  const double delta_y = linear_x*sin(heading_) + linear_y*cos(heading_);

  x_ += delta_x;
  y_ += delta_y;
  heading_ += angular;
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_       += linear * cos(direction);
  y_       += linear * sin(direction);
  heading_ += angular;
}

/**
 * \brief Other possible integration method provided by the class
 * \param linear
 * \param angular
 */
void Odometry::integrateExact(double linear, double angular)
{
  if (fabs(angular) < 1e-6)
    integrateRungeKutta2(linear, angular);
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear/angular;
    heading_ += angular;
    x_       +=  r * (sin(heading_) - sin(heading_old));
    y_       += -r * (cos(heading_) - cos(heading_old));
  }
}

void Odometry::resetAccumulators()
{
  linear_accel_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  linear_jerk_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
}

} // namespace mecanum_controller
