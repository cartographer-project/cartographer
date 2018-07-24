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

#include "cartographer/pose_graph/constraint/cost_function/acceleration_cost_3d.h"

#include "cartographer/testing/test_helpers.h"

namespace cartographer {
namespace pose_graph {
namespace {

using ::google::protobuf::TextFormat;
using ::testing::ElementsAre;
using testing::EqualsProto;
using testing::Near;
using testing::ParseProto;

using PositionType = std::array<double, 3>;
using RotationType = std::array<double, 4>;
using ResidualType = std::array<double, 3>;

constexpr char kParameters[] = R"PROTO(
  delta_velocity_imu_frame { x: 1 y: 1 z: 1 }
  first_to_second_delta_time_seconds: 10.0
  second_to_third_delta_time_seconds: 20.0
  scaling_factor: 2.0
)PROTO";

TEST(AccelerationCost3DTest, SerializesCorrectly) {
  AccelerationCost3D acceleration_cost_3d(
      ParseProto<proto::Acceleration3D::Parameters>(kParameters));
  EXPECT_THAT(acceleration_cost_3d.ToProto(), EqualsProto(kParameters));
}

TEST(AccelerationCost3DTest, EvaluatesCorrectly) {
  AccelerationCost3D acceleration_cost_3d(
      ParseProto<proto::Acceleration3D::Parameters>(kParameters));
  const PositionType kStartPosition{{1., 1., 1.}};

  const PositionType kMiddlePosition{{2., 2., 2.}};
  const RotationType kMiddleRotation{{0., 0., 0., 1.}};

  const PositionType kEndPosition{{3., 3., 3.}};

  const double kGravityConstant{10.};
  const RotationType kImuCalibration{{0., 0., 0., 1.0}};

  ResidualType residuals;
  EXPECT_TRUE(acceleration_cost_3d(
      kMiddleRotation.data(), kStartPosition.data(), kMiddlePosition.data(),
      kEndPosition.data(), &kGravityConstant, kImuCalibration.data(),
      residuals.data()));
  EXPECT_THAT(residuals, ElementsAre(Near(2.1), Near(2.1), Near(-297.9)));
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
