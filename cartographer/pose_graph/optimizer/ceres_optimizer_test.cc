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

#include "absl/memory/memory.h"
#include "cartographer/pose_graph/constraint/relative_pose_constraint_2d.h"
#include "cartographer/testing/test_helpers.h"

namespace cartographer {
namespace pose_graph {
namespace {

using testing::ParseProto;

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
  id: "constraint_1"
  loss_function { quadratic_loss: {} }
  cost_function {
    relative_pose_2d {
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
    }
  }
)PROTO";

TEST(CeresOptimizerTest, SmokeTest) {
  PoseGraphData data;
  AddNodeToPoseGraphData(ParseProto<proto::Node>(kStartNode), &data);
  AddNodeToPoseGraphData(ParseProto<proto::Node>(kEndNode), &data);
  AddConstraintToPoseGraphData(ParseProto<proto::Constraint>(kRelativePose2D),
                               &data);
  CeresOptimizer optimizer(ceres::Solver::Options{});
  EXPECT_EQ(optimizer.Solve(&data), Optimizer::SolverStatus::CONVERGENCE);
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
