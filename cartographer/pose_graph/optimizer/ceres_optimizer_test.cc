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

#include "cartographer/pose_graph/optimizer/ceres_optimizer.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/pose_graph/constraint/relative_pose_constraint_2d.h"
#include "cartographer/pose_graph/internal/testing/test_helpers.h"

namespace cartographer {
namespace pose_graph {
namespace {

using testing::ParseProto;

// TODO(pifon): Use the factory function, when the factory is done.
Pose2D GetPose2D(const proto::Node& proto) {
  return {NodeId(proto.id()), proto.constant(),
          transform::ToEigen(proto.parameters().pose_2d().translation()),
          proto.parameters().pose_2d().rotation()};
}

constexpr char kStartNode[] = R"PROTO(
  id { object_id: "start_node" timestamp: 1 }
  constant: false
  parameters {
    pose_2d {
      translation { x: 1 y: 2 }
      rotation: 5
    }
  }
)PROTO";

constexpr char kEndNode[] = R"PROTO(
  id { object_id: "end_node" timestamp: 1 }
  constant: false
  parameters {
    pose_2d {
      translation { x: 1 y: 2 }
      rotation: 5
    }
  }
)PROTO";

constexpr char kRelativePose2D[] = R"PROTO(
  first { object_id: "start_node" timestamp: 1 }
  second { object_id: "end_node" timestamp: 1 }
  parameters {
    first_t_second {
      translation { x: 1 y: 1 }
      rotation: 0
    }
    translation_weight: 1
    rotation_weight: 1
  }
)PROTO";

TEST(CeresOptimizerTest, SmokeTest) {
  PoseGraphData data;
  data.nodes.pose_2d_nodes.emplace(
      NodeId{"start_node", common::FromUniversal(1)},
      GetPose2D(ParseProto<proto::Node>(kStartNode)));
  data.nodes.pose_2d_nodes.emplace(
      NodeId{"end_node", common::FromUniversal(1)},
      GetPose2D(ParseProto<proto::Node>(kEndNode)));
  data.constraints.emplace_back(common::make_unique<RelativePoseConstraint2D>(
      "constraint_1", ParseProto<proto::RelativePose2D>(kRelativePose2D)));

  CeresOptimizer optimizer(ceres::Solver::Options{});
  EXPECT_EQ(optimizer.Solve(&data), Optimizer::SolverStatus::CONVERGENCE);
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
