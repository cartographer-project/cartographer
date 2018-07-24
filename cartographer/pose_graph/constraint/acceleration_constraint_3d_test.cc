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

#include "cartographer/pose_graph/constraint/acceleration_constraint_3d.h"

#include "cartographer/testing/test_helpers.h"

namespace cartographer {
namespace pose_graph {
namespace {

using testing::EqualsProto;
using testing::ParseProto;

constexpr char kConstraint[] = R"PROTO(
  id: "hal_acceleration"
  cost_function {
    acceleration_3d {
      first { object_id: "hal9000" timestamp: 100 }
      second { object_id: "hal9000" timestamp: 200 }
      third { object_id: "hal9000" timestamp: 300 }
      imu_calibration { object_id: "hal_imu" }
      parameters {
        delta_velocity_imu_frame { x: 1 y: 1 z: 1 }
        first_to_second_delta_time_seconds: 10.0
        second_to_third_delta_time_seconds: 20.0
        scaling_factor: 2.0
      }
    }
  }
  loss_function { quadratic_loss {} }
)PROTO";

TEST(AccelerationConstraint3DTest, SerializesCorrectly) {
  const auto proto = ParseProto<proto::Constraint>(kConstraint);
  AccelerationConstraint3D constraint(proto.id(), proto.loss_function(),
                                      proto.cost_function().acceleration_3d());
  const auto actual_proto = constraint.ToProto();
  EXPECT_THAT(actual_proto, EqualsProto(kConstraint));
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
