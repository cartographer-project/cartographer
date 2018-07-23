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

#include "cartographer/pose_graph/constraint/rotation_constraint_3d.h"

#include "cartographer/testing/test_helpers.h"

namespace cartographer {
namespace pose_graph {
namespace {

using testing::EqualsProto;
using testing::ParseProto;

constexpr char kConstraint[] = R"PROTO(
  id: "narf"
  cost_function {
    rotation_3d {
      first { object_id: "node0" }
      second { object_id: "node1" }
      imu_calibration { object_id: "imu_node" }
      parameters {
        delta_rotation_imu_frame { x: 0 y: 0.1 z: 0.2 w: 0.3 }
        scaling_factor: 0.4
      }
    }
  }
  loss_function { quadratic_loss {} }
)PROTO";

TEST(RotationConstraint3DTest, SerializesCorrectly) {
  const auto proto = ParseProto<proto::Constraint>(kConstraint);
  RotationContraint3D constraint(proto.id(), proto.loss_function(),
                                 proto.cost_function().rotation_3d());
  const auto actual_proto = constraint.ToProto();
  EXPECT_THAT(actual_proto, EqualsProto(kConstraint));
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
