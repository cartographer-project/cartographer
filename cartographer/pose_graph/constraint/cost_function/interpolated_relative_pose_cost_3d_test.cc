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

#include "cartographer/pose_graph/constraint/cost_function/interpolated_relative_pose_cost_3d.h"

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
using ResidualType = std::array<double, 6>;

constexpr char kParameters[] = R"PROTO(
  first_t_second {
    translation: { x: 1 y: 2 z: 3 }
    rotation: { x: 0 y: 0.3 z: 0.1 w: 0.2 }
  }
  translation_weight: 1
  rotation_weight: 10
  interpolation_factor: 0.3
)PROTO";

TEST(InterpolatedRelativePoseCost3DTest, SerializesCorrectly) {
  const auto parameters =
      ParseProto<proto::InterpolatedRelativePose3D::Parameters>(kParameters);
  InterpolatedRelativePoseCost3D interpolated_relative_pose_cost_3d(parameters);
  const auto actual_proto = interpolated_relative_pose_cost_3d.ToProto();
  EXPECT_THAT(actual_proto, EqualsProto(kParameters));
}

TEST(InterpolatedRelativePoseCost3DTest, EvaluatesCorrectly) {
  const auto parameters =
      ParseProto<proto::InterpolatedRelativePose3D::Parameters>(kParameters);
  InterpolatedRelativePoseCost3D interpolated_relative_pose_cost_3d(parameters);

  const PositionType kFirstStartPosition{{1., 1., 1.}};
  const RotationType kFirstStartRotation{{1., 1., 1., 1.}};
  const PositionType kFirstEndPosition{{2., 3., 4.}};
  const RotationType kFirstEndRotation{{1.1, 1.2, 1.3, 1.4}};
  const PositionType kSecondPosition{{0., -1., -2.}};
  const RotationType kSecondRotation{{.1, .2, .3, .4}};

  ResidualType residuals;
  EXPECT_TRUE(interpolated_relative_pose_cost_3d(
      kFirstStartPosition.data(), kFirstStartRotation.data(),
      kFirstEndPosition.data(), kFirstEndRotation.data(),
      kSecondPosition.data(), kSecondRotation.data(), residuals.data()));
  EXPECT_THAT(residuals,
              ElementsAre(Near(8.4594), Near(10.27735), Near(-4.45472),
                          Near(0.968852), Near(11.96531), Near(3.34254)));
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
