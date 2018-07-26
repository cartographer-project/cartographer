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

#include "cartographer/pose_graph/constraint/cost_function/interpolated_relative_pose_cost_2d.h"

#include "cartographer/testing/test_helpers.h"

namespace cartographer {
namespace pose_graph {
namespace {

using ::testing::ElementsAre;
using testing::EqualsProto;
using testing::Near;
using testing::ParseProto;

using Position2DType = std::array<double, 3>;
using TranslationType = std::array<double, 3>;
using RotationType = std::array<double, 4>;
using ResidualType = std::array<double, 6>;

constexpr char kParameters[] = R"PROTO(
  first_t_second {
    translation: { x: 1 y: 2 z: 3 }
    rotation: { x: 0 y: 0.3 z: 0.1 w: 0.2 }
  }
  translation_weight: 1
  rotation_weight: 2
  gravity_alignment_first_start { x: 0.4 y: 0.1 z: 0.3 w: 0.2 }
  gravity_alignment_first_end { x: 0.3 y: 0.4 z: 0.2 w: 0.1 }
  interpolation_factor: 0.3
)PROTO";

TEST(InterpolatedRelativePoseCost2DTest, SerializesCorrectly) {
  const auto parameters =
      ParseProto<proto::InterpolatedRelativePose2D::Parameters>(kParameters);
  InterpolatedRelativePoseCost2D interpolated_relative_pose_cost_2d(parameters);
  const auto actual_proto = interpolated_relative_pose_cost_2d.ToProto();
  EXPECT_THAT(actual_proto, EqualsProto(kParameters));
}

TEST(InterpolatedRelativePoseCost2DTest, EvaluatesCorrectly) {
  const auto parameters =
      ParseProto<proto::InterpolatedRelativePose2D::Parameters>(kParameters);
  InterpolatedRelativePoseCost2D interpolated_relative_pose_cost_2d(parameters);

  const Position2DType kFirstStartPose{{1., 1., 1.}};
  const Position2DType kFirstEndPose{{2., 3., 4.}};
  const TranslationType kSecondTranslation{{0., -1., -2.}};
  const RotationType kSecondRotation{{.4, .2, .3, .1}};

  ResidualType residuals;
  EXPECT_TRUE(interpolated_relative_pose_cost_2d(
      kFirstStartPose.data(), kFirstEndPose.data(), kSecondTranslation.data(),
      kSecondRotation.data(), residuals.data()));
  EXPECT_THAT(residuals,
              ElementsAre(Near(4.49044), Near(1.8527), Near(3.49511),
                          Near(-1.93746), Near(3.54854), Near(4.50243)));
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
