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

#ifndef CARTOGRAPHER_INTERNAL_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_INTERNAL_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_

#include "cartographer/mapping/trajectory_builder_interface.h"

#include "glog/logging.h"

namespace cartographer {
namespace mapping {

template <typename LocalTrajectoryBuilder,
          typename LocalTrajectoryBuilderOptions, typename PoseGraph>
class GlobalTrajectoryBuilder : public mapping::TrajectoryBuilderInterface {
 public:
  // Passing a 'nullptr' for 'local_trajectory_builder' is acceptable, but no
  // 'TimedPointCloudData' may be added in that case.
  GlobalTrajectoryBuilder(
      std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder,
      const int trajectory_id, PoseGraph* const pose_graph,
      const LocalSlamResultCallback& local_slam_result_callback)
      : trajectory_id_(trajectory_id),
        pose_graph_(pose_graph),
        local_trajectory_builder_(std::move(local_trajectory_builder)),
        local_slam_result_callback_(local_slam_result_callback) {}
  ~GlobalTrajectoryBuilder() override {}

  GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
  GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;

  void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    CHECK(local_trajectory_builder_)
        << "Cannot add TimedPointCloudData without a LocalTrajectoryBuilder.";
    std::unique_ptr<typename LocalTrajectoryBuilder::MatchingResult>
        matching_result = local_trajectory_builder_->AddRangeData(
            timed_point_cloud_data.time,
            sensor::TimedRangeData{timed_point_cloud_data.origin,
                                   timed_point_cloud_data.ranges,
                                   {}});
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

  void AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data) override {
    if (local_trajectory_builder_) {
      local_trajectory_builder_->AddImuData(imu_data);
    }
    pose_graph_->AddImuData(trajectory_id_, imu_data);
  }

  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data) override {
    CHECK(odometry_data.pose.IsValid()) << odometry_data.pose;
    if (local_trajectory_builder_) {
      local_trajectory_builder_->AddOdometryData(odometry_data);
    }
    pose_graph_->AddOdometryData(trajectory_id_, odometry_data);
  }

  void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose) override {
    if (fixed_frame_pose.pose.has_value()) {
      CHECK(fixed_frame_pose.pose.value().IsValid())
          << fixed_frame_pose.pose.value();
    }
    pose_graph_->AddFixedFramePoseData(trajectory_id_, fixed_frame_pose);
  }

 private:
  const int trajectory_id_;
  PoseGraph* const pose_graph_;
  std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder_;
  LocalSlamResultCallback local_slam_result_callback_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_INTERNAL_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_
