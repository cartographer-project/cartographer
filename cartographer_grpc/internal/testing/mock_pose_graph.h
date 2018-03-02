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

#ifndef CARTOGRAPHER_GRPC_INTERNAL_TESTING_MOCK_POSE_GRAPH_H
#define CARTOGRAPHER_GRPC_INTERNAL_TESTING_MOCK_POSE_GRAPH_H

#include "cartographer/mapping/pose_graph_interface.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer_grpc {
namespace testing {

class MockPoseGraph : public cartographer::mapping::PoseGraphInterface {
 public:
  MockPoseGraph() = default;
  ~MockPoseGraph() override = default;

  MOCK_METHOD0(RunFinalOptimization, void());
  MOCK_METHOD0(GetAllSubmapData,
               cartographer::mapping::MapById<cartographer::mapping::SubmapId,
                                              SubmapData>());
  MOCK_METHOD0(GetAllSubmapPoses,
               cartographer::mapping::MapById<cartographer::mapping::SubmapId,
                                              SubmapPose>());
  MOCK_METHOD1(GetLocalToGlobalTransform,
               cartographer::transform::Rigid3d(int));
  MOCK_METHOD0(
      GetTrajectoryNodes,
      cartographer::mapping::MapById<cartographer::mapping::NodeId,
                                     cartographer::mapping::TrajectoryNode>());
  MOCK_METHOD0(GetTrajectoryNodePoses,
               cartographer::mapping::MapById<
                   cartographer::mapping::NodeId,
                   cartographer::mapping::TrajectoryNodePose>());
  MOCK_METHOD0(GetLandmarkPoses,
               std::map<std::string, cartographer::transform::Rigid3d>());
  MOCK_METHOD1(IsTrajectoryFinished, bool(int));
  MOCK_METHOD0(
      GetTrajectoryData,
      std::map<int,
               cartographer::mapping::PoseGraphInterface::TrajectoryData>());
  MOCK_METHOD0(constraints, std::vector<Constraint>());
  MOCK_METHOD0(ToProto, cartographer::mapping::proto::PoseGraph());
};

}  // namespace testing
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_INTERNAL_TESTING_MOCK_POSE_GRAPH_H
