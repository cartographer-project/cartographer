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
          typename LocalTrajectoryBuilderOptions, typename PoseGraph>
class GlobalTrajectoryBuilder
    : public mapping::GlobalTrajectoryBuilderInterface {
 public:
  GlobalTrajectoryBuilder(
      const LocalTrajectoryBuilderOptions& options, const int trajectory_id,
      PoseGraph* const pose_graph,
      const LocalSlamResultCallback& local_slam_result_callback)
      : trajectory_id_(trajectory_id),
        pose_graph_(pose_graph),
        local_trajectory_builder_(options),
        local_slam_result_callback_(local_slam_result_callback) {}
  ~GlobalTrajectoryBuilder() override {}

  GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
  GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;

  void AddRangefinderData(const common::Time time,
                          const Eigen::Vector3f& origin,
                          const sensor::TimedPointCloud& ranges) override {
    std::unique_ptr<typename LocalTrajectoryBuilder::MatchingResult>
        matching_result = local_trajectory_builder_.AddRangeData(
            time, sensor::TimedRangeData{origin, ranges, {}});
    if (matching_result == nullptr) {
      // The range data has not been fully accumulated yet.
      return;
    }
    std::unique_ptr<mapping::NodeId> node_id;
    if (matching_result->insertion_result != nullptr) {
      node_id = ::cartographer::common::make_unique<mapping::NodeId>(
          pose_graph_->AddNode(
              matching_result->insertion_result->constant_data, trajectory_id_,
              matching_result->insertion_result->insertion_submaps));
      CHECK_EQ(node_id->trajectory_id, trajectory_id_);
    }
    if (local_slam_result_callback_) {
      local_slam_result_callback_(
          trajectory_id_, matching_result->time, matching_result->local_pose,
          std::move(matching_result->range_data_in_local), std::move(node_id));
    }
  }

  void AddSensorData(const sensor::ImuData& imu_data) override {
    local_trajectory_builder_.AddImuData(imu_data);
    pose_graph_->AddImuData(trajectory_id_, imu_data);
  }

  void AddSensorData(const sensor::OdometryData& odometry_data) override {
    CHECK(odometry_data.pose.IsValid()) << odometry_data.pose;
    local_trajectory_builder_.AddOdometryData(odometry_data);
    pose_graph_->AddOdometryData(trajectory_id_, odometry_data);
  }

  void AddSensorData(
      const sensor::FixedFramePoseData& fixed_frame_pose) override {
    CHECK(fixed_frame_pose.pose.IsValid()) << fixed_frame_pose.pose;
    pose_graph_->AddFixedFramePoseData(trajectory_id_, fixed_frame_pose);
  }

 private:
  const int trajectory_id_;
  PoseGraph* const pose_graph_;
  LocalTrajectoryBuilder local_trajectory_builder_;
  LocalSlamResultCallback local_slam_result_callback_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_
