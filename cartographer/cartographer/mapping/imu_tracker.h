/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
#define CARTOGRAPHER_MAPPING_IMU_TRACKER_H_

#include "Eigen/Geometry"
#include "cartographer/common/time.h"

namespace cartographer {
namespace mapping {
/*
ImuTracker
作用：使用来自imu的角速度+加速度用于跟踪pose的orientation

构造函数指定重力加速度g和最新测量时间。
数据成员包含
1,加速度g
2,测量时间
3,方向,四元数
4,三个加速度3d
5,imu角速度
*/
// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
class ImuTracker {
 public:
  ImuTracker(double imu_gravity_time_constant, common::Time time);

  // Advances to the given 'time' and updates the orientation to reflect this.
  // 系统时间增加t，更新方向角
  void Advance(common::Time time);

  // Updates from an IMU reading (in the IMU frame).
  void AddImuLinearAccelerationObservation(
      const Eigen::Vector3d& imu_linear_acceleration);
  void AddImuAngularVelocityObservation(
      const Eigen::Vector3d& imu_angular_velocity);

  // Query the current time.
  common::Time time() const { return time_; }

  // Query the current orientation estimate.返回目前估计pose的方向角。
  Eigen::Quaterniond orientation() const { return orientation_; }

 private:
  const double imu_gravity_time_constant_;//重力常量
  common::Time time_;//最新一次测量时间
  common::Time last_linear_acceleration_time_;//加速度测量时间
  Eigen::Quaterniond orientation_;//pose的方向角
  Eigen::Vector3d gravity_vector_;//加速度测量的方向
  Eigen::Vector3d imu_angular_velocity_;//角速度
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
