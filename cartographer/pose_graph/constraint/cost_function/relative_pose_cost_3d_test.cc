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
)PROTO";

TEST(RelativePoseCost3DTest, SerializesCorrectly) {
  auto relative_pose_parameters =
      ParseProto<proto::RelativePose3D::Parameters>(kParameters);
  RelativePoseCost3D relative_pose_cost_3d(relative_pose_parameters);
  const auto actual_proto = relative_pose_cost_3d.ToProto();
  EXPECT_THAT(actual_proto, EqualsProto(kParameters));
}

TEST(RelativePoseCost3DTest, EvaluatesCorrectly) {
  auto relative_pose_parameters =
      ParseProto<proto::RelativePose3D::Parameters>(kParameters);
  RelativePoseCost3D relative_pose_cost_3d(relative_pose_parameters);

  const PositionType kPosition1{{1., 1., 1.}};
  const RotationType kRotation1{{1., 1., 1., 1.}};
  ResidualType residuals;
  EXPECT_TRUE(relative_pose_cost_3d(kPosition1.data(), kRotation1.data(),
                                    kPosition1.data(), kRotation1.data(),
                                    residuals.data()));
  EXPECT_THAT(residuals, ElementsAre(Near(1), Near(2), Near(3), Near(0),
                                     Near(19.1037), Near(6.3679)));

  const PositionType kPosition2{{0., -1., -2.}};
  const RotationType kRotation2{{.1, .2, .3, .4}};
  EXPECT_TRUE(relative_pose_cost_3d(kPosition1.data(), kRotation1.data(),
                                    kPosition2.data(), kRotation2.data(),
                                    residuals.data()));
  EXPECT_THAT(residuals, ElementsAre(Near(6), Near(8), Near(-2), Near(1.03544),
                                     Near(11.38984), Near(3.10632)));
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
