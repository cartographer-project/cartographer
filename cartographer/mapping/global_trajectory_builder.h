/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_

#include "cartographer/mapping/global_trajectory_builder_interface.h"

#include "glog/logging.h"

namespace cartographer {
namespace mapping {

template <typename LocalTrajectoryBuilder,
          typename LocalTrajectoryBuilderOptions, typename SparsePoseGraph>
class GlobalTrajectoryBuilder
    : public mapping::GlobalTrajectoryBuilderInterface {
 public:
  GlobalTrajectoryBuilder(const LocalTrajectoryBuilderOptions& options,
                          const int trajectory_id,
                          SparsePoseGraph* const sparse_pose_graph)
      : trajectory_id_(trajectory_id),
        sparse_pose_graph_(sparse_pose_graph),
        local_trajectory_builder_(options) {}
  ~GlobalTrajectoryBuilder() override {}

  GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
  GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;

  const mapping::PoseEstimate& pose_estimate() const override {
    return local_trajectory_builder_.pose_estimate();
  }

  void AddRangefinderData(const common::Time time,
                          const Eigen::Vector3f& origin,
                          const sensor::PointCloud& ranges) override {
    std::unique_ptr<typename LocalTrajectoryBuilder::InsertionResult>
        insertion_result = local_trajectory_builder_.AddRangeData(
            time, sensor::RangeData{origin, ranges, {}});
    if (insertion_result == nullptr) {
      return;
    }
    sparse_pose_graph_->AddScan(insertion_result->constant_data, trajectory_id_,
                                insertion_result->insertion_submaps);
  }

  void AddSensorData(const sensor::ImuData& imu_data) override {
    local_trajectory_builder_.AddImuData(imu_data);
    sparse_pose_graph_->AddImuData(trajectory_id_, imu_data);
  }

  void AddSensorData(const sensor::OdometryData& odometry_data) override {
    CHECK(odometry_data.pose.IsValid()) << odometry_data.pose;
    local_trajectory_builder_.AddOdometerData(odometry_data);
    sparse_pose_graph_->AddOdometerData(trajectory_id_, odometry_data);
  }

  void AddSensorData(
      const sensor::FixedFramePoseData& fixed_frame_pose) override {
    CHECK(fixed_frame_pose.pose.IsValid()) << fixed_frame_pose.pose;
    sparse_pose_graph_->AddFixedFramePoseData(trajectory_id_, fixed_frame_pose);
  }

 private:
  const int trajectory_id_;
  SparsePoseGraph* const sparse_pose_graph_;
  LocalTrajectoryBuilder local_trajectory_builder_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_
