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

#include "cartographer/pose_graph/constraint/cost_function/relative_pose_cost_3d.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/internal/optimization/cost_functions/cost_helpers.h"
#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"

namespace cartographer {
namespace pose_graph {
namespace {

using ::google::protobuf::TextFormat;
using ::testing::ElementsAre;

using PositionType = std::array<double, 3>;
using RotationType = std::array<double, 4>;
using ResidualType = std::array<double, 6>;
using ParameterBlockType = std::array<const double*, 4>;

::testing::Matcher<double> Near(double expected) {
  constexpr double kPrecision = 1e-05;
  return ::testing::DoubleNear(expected, kPrecision);
}

TEST(RelativePoseCost3DTest, EvaluateRelativePoseCost3D) {
  const PositionType position1{{1., 1., 1.}};
  const RotationType rotation1{{1., 1., 1., 1.}};
  const PositionType position2{{1., 1., 1.}};
  const RotationType rotation2{{1., 1., 1., 1.}};
  const ParameterBlockType parameter_blocks{
      {position1.data(), rotation1.data(), position2.data(), rotation2.data()}};

  constexpr char kParameters[] = R"PROTO(
    first_t_second {
      translation: { x: 1 y: 2 z: 3 }
      rotation: { x: 0 y: 0.3 z: 0.1 w: 0.2 }
    }
    translation_weight: 1
    rotation_weight: 10
  )PROTO";

  proto::RelativePose3D::Parameters relative_pose_parameters;
  ASSERT_TRUE(
      TextFormat::ParseFromString(kParameters, &relative_pose_parameters));

  RelativePoseCost3D relative_pose_cost_3d(relative_pose_parameters);
  ResidualType residuals;
  relative_pose_cost_3d.GetCeresFunction()->Evaluate(
      parameter_blocks.data(), residuals.data(), /*jacobians=*/nullptr);

  EXPECT_THAT(residuals, ElementsAre(Near(1), Near(2), Near(3), Near(0),
                                     Near(19.1037), Near(6.3679)));
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
