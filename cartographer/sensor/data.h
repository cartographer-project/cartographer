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

#ifndef CARTOGRAPHER_MAPPING_DATA_H_
#define CARTOGRAPHER_MAPPING_DATA_H_

#include "cartographer/common/time.h"
#include "cartographer/mapping/global_trajectory_builder_interface.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace sensor {

class Data {
 public:
  virtual ~Data() {}

  virtual common::Time GetTime() const = 0;
  virtual void AddToTrajectoryBuilder(
      mapping::GlobalTrajectoryBuilderInterface* trajectory_builder) = 0;
};

class DispatchableImuData : public Data {
 public:
  DispatchableImuData(const ImuData& imu_data) : imu_data_(imu_data) {}

  common::Time GetTime() const override { return imu_data_.time; }
  void AddToTrajectoryBuilder(mapping::GlobalTrajectoryBuilderInterface* const
                                  trajectory_builder) override {
    trajectory_builder->AddImuData(imu_data_);
  }

 private:
  const ImuData imu_data_;
};

class DispatchableRangefinderData : public Data {
 public:
  DispatchableRangefinderData(const common::Time time,
                              const Eigen::Vector3f& origin,
                              const PointCloud& ranges)
      : time_(time), origin_(origin), ranges_(ranges) {}

  common::Time GetTime() const override { return time_; }
  void AddToTrajectoryBuilder(mapping::GlobalTrajectoryBuilderInterface* const
                                  trajectory_builder) override {
    trajectory_builder->AddRangefinderData(time_, origin_, ranges_);
  }

 private:
  const common::Time time_;
  const Eigen::Vector3f origin_;
  const PointCloud ranges_;
};

class DispatchableOdometerData : public Data {
 public:
  DispatchableOdometerData(const common::Time time,
                           const transform::Rigid3d& odometer_pose)
      : time_(time), odometer_pose_(odometer_pose) {}

  common::Time GetTime() const override { return time_; }
  void AddToTrajectoryBuilder(mapping::GlobalTrajectoryBuilderInterface* const
                                  trajectory_builder) override {
    trajectory_builder->AddOdometerData(time_, odometer_pose_);
  }

 private:
  const common::Time time_;
  const transform::Rigid3d odometer_pose_;
};

class DispatchableFixedFramePoseData : public Data {
 public:
  DispatchableFixedFramePoseData(const common::Time time,
                                 const transform::Rigid3d& fixed_frame_pose)
      : fixed_frame_pose_data_{time, fixed_frame_pose} {}

  common::Time GetTime() const override { return fixed_frame_pose_data_.time; }
  void AddToTrajectoryBuilder(mapping::GlobalTrajectoryBuilderInterface* const
                                  trajectory_builder) override {
    trajectory_builder->AddFixedFramePoseData(fixed_frame_pose_data_);
  }

 private:
  const FixedFramePoseData fixed_frame_pose_data_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_DATA_H_
