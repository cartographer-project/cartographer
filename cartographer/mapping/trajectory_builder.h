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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_H_

#include <functional>
#include <memory>
#include <string>

#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/sensor/data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

// This interface is used for both 2D and 3D SLAM.
class TrajectoryBuilder {
 public:
  // Represents a newly computed pose. 'pose' is the end-user visualization of
  // orientation and 'point_cloud' is the point cloud, in the local map frame.
  struct PoseEstimate {
    PoseEstimate() = default;
    PoseEstimate(common::Time time, const transform::Rigid3d& pose,
                 const sensor::PointCloud& point_cloud)
        : time(time), pose(pose), point_cloud(point_cloud) {}

    common::Time time = common::Time::min();
    transform::Rigid3d pose = transform::Rigid3d::Identity();
    sensor::PointCloud point_cloud;
  };

  TrajectoryBuilder() {}
  virtual ~TrajectoryBuilder() {}

  TrajectoryBuilder(const TrajectoryBuilder&) = delete;
  TrajectoryBuilder& operator=(const TrajectoryBuilder&) = delete;

  virtual const Submaps* submaps() const = 0;
  virtual const PoseEstimate& pose_estimate() const = 0;

  virtual void AddSensorData(const string& sensor_id,
                             std::unique_ptr<sensor::Data> data) = 0;

  void AddRangefinderData(const string& sensor_id, common::Time time,
                          const Eigen::Vector3f& origin,
                          const sensor::PointCloud& ranges) {
    AddSensorData(sensor_id,
                  common::make_unique<sensor::Data>(
                      time, sensor::Data::Rangefinder{origin, ranges}));
  }

  void AddImuData(const string& sensor_id, common::Time time,
                  const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity) {
    AddSensorData(sensor_id, common::make_unique<sensor::Data>(
                                 time, sensor::Data::Imu{linear_acceleration,
                                                         angular_velocity}));
  }

  void AddOdometerData(const string& sensor_id, common::Time time,
                       const transform::Rigid3d& odometer_pose) {
    AddSensorData(sensor_id,
                  common::make_unique<sensor::Data>(time, odometer_pose));
  }
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_H_
