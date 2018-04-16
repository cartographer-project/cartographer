/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_LOCAL_SLAM_RESULT_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_LOCAL_SLAM_RESULT_3D_H_

#include "cartographer/mapping/internal/submap_controller.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

namespace cartographer {
namespace mapping {

class LocalSlamResult3D : public LocalSlamResultData {
 public:
  LocalSlamResult3D(
      const std::string& sensor_id,
      const mapping::proto::LocalSlamResultData local_slam_result_data,
      SubmapController<mapping::Submap3D>* submap_controller)
      : LocalSlamResultData(sensor_id, common::FromUniversal(
                                           local_slam_result_data.timestamp())),
        sensor_id_(sensor_id),
        local_slam_result_data_(local_slam_result_data),
        submap_controller_(submap_controller) {}

  void AddToTrajectoryBuilder(
      TrajectoryBuilderInterface* const trajectory_builder) override;
  void AddToPoseGraph(int trajectory_id, PoseGraph* pose_graph) const override;

 private:
  const std::string sensor_id_;
  const mapping::proto::LocalSlamResultData local_slam_result_data_;
  SubmapController<mapping::Submap3D>* submap_controller_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_LOCAL_SLAM_RESULT_3D_H_
