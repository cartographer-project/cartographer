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

#include "cartographer/mapping/internal/testing/test_helpers.h"

#include "absl/memory/memory.h"
#include "cartographer/common/config.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {
namespace testing {

std::unique_ptr<::cartographer::common::LuaParameterDictionary>
ResolveLuaParameters(const std::string& lua_code) {
  auto file_resolver =
      absl::make_unique<::cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{
              std::string(::cartographer::common::kSourceDirectory) +
              "/configuration_files"});
  return absl::make_unique<::cartographer::common::LuaParameterDictionary>(
      lua_code, std::move(file_resolver));
}

std::vector<cartographer::sensor::TimedPointCloudData>
GenerateFakeRangeMeasurements(double travel_distance, double duration,
                              double time_step) {
  const Eigen::Vector3f kDirection = Eigen::Vector3f(2., 1., 0.).normalized();
  return GenerateFakeRangeMeasurements(kDirection * travel_distance, duration,
                                       time_step,
                                       transform::Rigid3f::Identity());
}

std::vector<cartographer::sensor::TimedPointCloudData>
GenerateFakeRangeMeasurements(const Eigen::Vector3f& translation,
                              double duration, double time_step,
                              const transform::Rigid3f& local_to_global) {
  std::vector<cartographer::sensor::TimedPointCloudData> measurements;
  cartographer::sensor::TimedPointCloud point_cloud;
  for (double angle = 0.; angle < M_PI; angle += 0.01) {
    for (double height : {-0.4, -0.2, 0.0, 0.2, 0.4}) {
      constexpr double kRadius = 5;
      point_cloud.push_back({Eigen::Vector3d{kRadius * std::cos(angle),
                                             kRadius * std::sin(angle), height}
                                 .cast<float>(),
                             0.});
    }
  }
  const Eigen::Vector3f kVelocity = translation / duration;
  for (double elapsed_time = 0.; elapsed_time < duration;
       elapsed_time += time_step) {
    cartographer::common::Time time =
        cartographer::common::FromUniversal(123) +
        cartographer::common::FromSeconds(elapsed_time);
    cartographer::transform::Rigid3f global_pose =
        local_to_global *
        cartographer::transform::Rigid3f::Translation(elapsed_time * kVelocity);
    cartographer::sensor::TimedPointCloud ranges =
        cartographer::sensor::TransformTimedPointCloud(point_cloud,
                                                       global_pose.inverse());
    measurements.emplace_back(cartographer::sensor::TimedPointCloudData{
        time, Eigen::Vector3f::Zero(), ranges});
  }
  return measurements;
}

proto::Submap CreateFakeSubmap3D(int trajectory_id, int submap_index,
                                 bool finished) {
  proto::Submap proto;
  proto.mutable_submap_id()->set_trajectory_id(trajectory_id);
  proto.mutable_submap_id()->set_submap_index(submap_index);
  proto.mutable_submap_3d()->set_num_range_data(1);
  *proto.mutable_submap_3d()->mutable_local_pose() =
      transform::ToProto(transform::Rigid3d::Identity());
  proto.mutable_submap_3d()->set_finished(finished);
  return proto;
}

proto::Node CreateFakeNode(int trajectory_id, int node_index) {
  proto::Node proto;
  proto.mutable_node_id()->set_trajectory_id(trajectory_id);
  proto.mutable_node_id()->set_node_index(node_index);
  proto.mutable_node_data()->set_timestamp(42);
  *proto.mutable_node_data()->mutable_local_pose() =
      transform::ToProto(transform::Rigid3d::Identity());
  return proto;
}

proto::PoseGraph::Constraint CreateFakeConstraint(const proto::Node& node,
                                                  const proto::Submap& submap) {
  proto::PoseGraph::Constraint proto;
  proto.mutable_submap_id()->set_submap_index(
      submap.submap_id().submap_index());
  proto.mutable_submap_id()->set_trajectory_id(
      submap.submap_id().trajectory_id());
  proto.mutable_node_id()->set_node_index(node.node_id().node_index());
  proto.mutable_node_id()->set_trajectory_id(node.node_id().trajectory_id());
  transform::Rigid3d pose(
      Eigen::Vector3d(2., 3., 4.),
      Eigen::AngleAxisd(M_PI / 8., Eigen::Vector3d::UnitX()));
  *proto.mutable_relative_pose() = transform::ToProto(pose);
  proto.set_translation_weight(0.2f);
  proto.set_rotation_weight(0.1f);
  proto.set_tag(proto::PoseGraph::Constraint::INTER_SUBMAP);
  return proto;
}

proto::Trajectory* CreateTrajectoryIfNeeded(int trajectory_id,
                                            proto::PoseGraph* pose_graph) {
  for (int i = 0; i < pose_graph->trajectory_size(); ++i) {
    proto::Trajectory* trajectory = pose_graph->mutable_trajectory(i);
    if (trajectory->trajectory_id() == trajectory_id) {
      return trajectory;
    }
  }
  proto::Trajectory* trajectory = pose_graph->add_trajectory();
  trajectory->set_trajectory_id(trajectory_id);
  return trajectory;
}

proto::PoseGraph::LandmarkPose CreateFakeLandmark(
    const std::string& landmark_id, const transform::Rigid3d& global_pose) {
  proto::PoseGraph::LandmarkPose landmark;
  landmark.set_landmark_id(landmark_id);
  *landmark.mutable_global_pose() = transform::ToProto(global_pose);
  return landmark;
}

void AddToProtoGraph(const proto::Node& node_data,
                     proto::PoseGraph* pose_graph) {
  auto* trajectory =
      CreateTrajectoryIfNeeded(node_data.node_id().trajectory_id(), pose_graph);
  auto* node = trajectory->add_node();
  node->set_timestamp(node_data.node_data().timestamp());
  node->set_node_index(node_data.node_id().node_index());
  *node->mutable_pose() = node_data.node_data().local_pose();
}

void AddToProtoGraph(const proto::Submap& submap_data,
                     proto::PoseGraph* pose_graph) {
  auto* trajectory = CreateTrajectoryIfNeeded(
      submap_data.submap_id().trajectory_id(), pose_graph);
  auto* submap = trajectory->add_submap();
  submap->set_submap_index(submap_data.submap_id().submap_index());
  if (submap_data.has_submap_2d()) {
    *submap->mutable_pose() = submap_data.submap_2d().local_pose();
  } else {
    *submap->mutable_pose() = submap_data.submap_3d().local_pose();
  }
}

void AddToProtoGraph(const proto::PoseGraph::Constraint& constraint,
                     proto::PoseGraph* pose_graph) {
  *pose_graph->add_constraint() = constraint;
}

void AddToProtoGraph(const proto::PoseGraph::LandmarkPose& landmark,
                     proto::PoseGraph* pose_graph) {
  *pose_graph->add_landmark_poses() = landmark;
}

}  // namespace testing
}  // namespace mapping
}  // namespace cartographer
