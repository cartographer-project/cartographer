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

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_estimate.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/sensor/data.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping {

proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

// This interface is used for both 2D and 3D SLAM.
class TrajectoryBuilder {
 public:
  TrajectoryBuilder() {}
  virtual ~TrajectoryBuilder() {}

  TrajectoryBuilder(const TrajectoryBuilder&) = delete;
  TrajectoryBuilder& operator=(const TrajectoryBuilder&) = delete;

  virtual void AddRangefinderData(const std::string& sensor_id,
                                  common::Time time,
                                  const Eigen::Vector3f& origin,
                                  const sensor::TimedPointCloud& ranges) = 0;

  virtual void AddImuData(const std::string& sensor_id, common::Time time,
                          const Eigen::Vector3d& linear_acceleration,
                          const Eigen::Vector3d& angular_velocity) = 0;

  virtual void AddOdometerData(const std::string& sensor_id, common::Time time,
                               const transform::Rigid3d& odometer_pose) = 0;

  virtual void AddFixedFramePoseData(
      const std::string& sensor_id, common::Time time,
      const transform::Rigid3d& fixed_frame_pose) = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_H_
