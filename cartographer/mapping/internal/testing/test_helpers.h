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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_TESTING_TEST_HELPERS_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_TESTING_TEST_HELPERS_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace mapping {
namespace testing {

std::unique_ptr<::cartographer::common::LuaParameterDictionary>
ResolveLuaParameters(const std::string& lua_code);

std::vector<cartographer::sensor::TimedPointCloudData>
GenerateFakeRangeMeasurements(double travel_distance, double duration,
                              double time_step);

std::vector<cartographer::sensor::TimedPointCloudData>
GenerateFakeRangeMeasurements(const Eigen::Vector3f& translation,
                              double duration, double time_step,
                              const transform::Rigid3f& local_to_global);

proto::Submap CreateFakeSubmap3D(int trajectory_id = 1, int submap_index = 1,
                                 bool finished = true);

proto::Node CreateFakeNode(int trajectory_id = 1, int node_index = 1);

proto::PoseGraph::Constraint CreateFakeConstraint(const proto::Node& node,
                                                  const proto::Submap& submap);

proto::Trajectory* CreateTrajectoryIfNeeded(int trajectory_id,
                                            proto::PoseGraph* pose_graph);
proto::PoseGraph::LandmarkPose CreateFakeLandmark(
    const std::string& landmark_id, const transform::Rigid3d& global_pose);

void AddToProtoGraph(const proto::Node& node_data,
                     proto::PoseGraph* pose_graph);

void AddToProtoGraph(const proto::Submap& submap_data,
                     proto::PoseGraph* pose_graph);

void AddToProtoGraph(const proto::PoseGraph::Constraint& constraint,
                     proto::PoseGraph* pose_graph);

void AddToProtoGraph(const proto::PoseGraph::LandmarkPose& landmark_node,
                     proto::PoseGraph* pose_graph);

}  // namespace testing
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_TESTING_TEST_HELPERS_H_
