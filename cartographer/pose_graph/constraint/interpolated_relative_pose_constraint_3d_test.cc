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

#include "cartographer/pose_graph/constraint/interpolated_relative_pose_constraint_3d.h"

#include "cartographer/testing/test_helpers.h"

namespace cartographer {
namespace pose_graph {
namespace {

using ::testing::ElementsAre;
using testing::EqualsProto;
using testing::Near;
using testing::ParseProto;

using PositionType = std::array<double, 3>;
using RotationType = std::array<double, 4>;
using ResidualType = std::array<double, 3>;

constexpr char kConstraint[] = R"PROTO(
  id: "narf"
  cost_function {
    interpolated_relative_pose_3d {
      first_start { object_id: "node0_start" }
      first_end { object_id: "node0_end" }
      second { object_id: "node1" }
      parameters {
        first_t_second {
          translation: { x: 1 y: 2 z: 3 }
          rotation: { x: 0 y: 0.3 z: 0.1 w: 0.2 }
        }
        translation_weight: 0.5
        rotation_weight: 1.0
        interpolation_factor: 0.3
      }
    }
  }

  loss_function { quadratic_loss {} }
)PROTO";

TEST(InterpolatedRelativePostConstraint3DTest, SerializesCorrectly) {
  const auto proto = ParseProto<proto::Constraint>(kConstraint);
  InterpolatedRelativePoseConstraint3D constraint(
      proto.id(), proto.loss_function(),
      proto.cost_function().interpolated_relative_pose_3d());
  const auto actual_proto = constraint.ToProto();
  EXPECT_THAT(actual_proto, EqualsProto(kConstraint));
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
