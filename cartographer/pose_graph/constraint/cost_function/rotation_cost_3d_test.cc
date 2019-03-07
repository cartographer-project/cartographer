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

#include "cartographer/pose_graph/constraint/cost_function/rotation_cost_3d.h"

#include <array>

#include "cartographer/testing/test_helpers.h"

namespace cartographer {
namespace pose_graph {
namespace {

using ::testing::ElementsAre;
using testing::EqualsProto;
using testing::Near;
using testing::ParseProto;

using RotationType = std::array<double, 4>;
using ResidualType = std::array<double, 3>;

constexpr char kParameters[] = R"PROTO(
  delta_rotation_imu_frame { x: 0 y: 0.1 z: 0.2 w: 0.3 }
  scaling_factor: 0.4
)PROTO";

TEST(RotationCost3DTest, SerializesCorrectly) {
  const auto parameters =
      ParseProto<proto::Rotation3D::Parameters>(kParameters);
  RotationCost3D rotation_cost_3d(parameters);
  const auto actual_proto = rotation_cost_3d.ToProto();
  EXPECT_THAT(actual_proto, EqualsProto(kParameters));
}

TEST(RotationCost3DTest, EvaluatesCorrectly) {
  const auto parameters =
      ParseProto<proto::Rotation3D::Parameters>(kParameters);
  RotationCost3D rotation_cost_3d(parameters);

  const RotationType kStartRotation{{1., 1., 1., 1.}};
  const RotationType kEndRotation{{1.1, 1.2, 1.3, 1.4}};
  const RotationType kImuCalibration{{0.1, 0.1, 0.1, 0.1}};
  ResidualType residuals;
  rotation_cost_3d(kStartRotation.data(), kEndRotation.data(),
                   kImuCalibration.data(), residuals.data());

  EXPECT_THAT(residuals,
              ElementsAre(Near(0.01536), Near(-0.00256), Near(0.00832)));
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
