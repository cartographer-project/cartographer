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

#include "cartographer/pose_graph/constraint/cost_function/relative_pose_cost_2d.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/testing/test_helpers.h"
#include "ceres/gradient_checker.h"

namespace cartographer {
namespace pose_graph {
namespace {

constexpr int kPoseDimension = 3;
constexpr int kResidualsCount = 3;
constexpr int kParameterBlocksCount = 2;
constexpr int kJacobianColDimension = kResidualsCount * kPoseDimension;

using ::testing::ElementsAre;
using testing::EqualsProto;
using testing::Near;
using testing::ParseProto;

using ResidualType = std::array<double, kResidualsCount>;
using JacobianType = std::array<std::array<double, kJacobianColDimension>,
                                kParameterBlocksCount>;

constexpr char kParameters[] = R"PROTO(
  first_t_second {
    translation: { x: 1 y: 1 }
    rotation: -2.214297
  }
  translation_weight: 1
  rotation_weight: 10
)PROTO";

class RelativePoseCost2DTest : public ::testing::Test {
 public:
  RelativePoseCost2DTest()
      : relative_pose_cost_2d_(common::make_unique<RelativePoseCost2D>(
            ParseProto<proto::RelativePose2D::Parameters>(kParameters))) {
    for (int i = 0; i < kParameterBlocksCount; ++i) {
      jacobian_ptrs_[i] = jacobian_[i].data();
    }
  }

  std::pair<const ResidualType&, const JacobianType&>
  EvaluateRelativePoseCost2D(
      const std::array<const double*, 2>& parameter_blocks) {
    relative_pose_cost_2d_->Evaluate(parameter_blocks.data(), residuals_.data(),
                                     jacobian_ptrs_.data());
    return std::make_pair(std::cref(residuals_), std::cref(jacobian_));
  }

 protected:
  ResidualType residuals_;
  JacobianType jacobian_;
  std::array<double*, kParameterBlocksCount> jacobian_ptrs_;
  std::unique_ptr<RelativePoseCost2D> relative_pose_cost_2d_;
};

TEST_F(RelativePoseCost2DTest, SerializesCorrectly) {
  EXPECT_THAT(relative_pose_cost_2d_->ToProto(), EqualsProto(kParameters));
}

TEST_F(RelativePoseCost2DTest, CheckGradient) {
  std::array<double, kPoseDimension> start_pose{{1., 1., 1.}};
  std::array<double, kPoseDimension> end_pose{{10., 1., 100.}};
  std::array<const double*, kParameterBlocksCount> parameter_blocks{
      {start_pose.data(), end_pose.data()}};

  using ::ceres::GradientChecker;
  GradientChecker gradient_checker(relative_pose_cost_2d_.get(),
                                   {} /* local parameterizations */,
                                   ceres::NumericDiffOptions{});

  GradientChecker::ProbeResults probe_results;
  gradient_checker.Probe(parameter_blocks.data(),
                         1e-08 /* relative precision */, &probe_results);
  EXPECT_TRUE(probe_results.return_value);
}

TEST_F(RelativePoseCost2DTest, EvaluateRelativePoseCost2D) {
  std::array<double, kPoseDimension> start_pose{{1., 1., 1.}};
  std::array<double, kPoseDimension> end_pose{{10., 1., 100.}};
  std::array<const double*, kParameterBlocksCount> parameter_blocks{
      {start_pose.data(), end_pose.data()}};

  auto residuals_and_jacobian = EvaluateRelativePoseCost2D(parameter_blocks);
  EXPECT_THAT(residuals_and_jacobian.first,
              ElementsAre(Near(-3.86272), Near(8.57324), Near(-6.83333)));
  EXPECT_THAT(
      residuals_and_jacobian.second,
      ElementsAre(
          ElementsAre(Near(0.540302), Near(0.841471), Near(7.57324),
                      Near(-0.841471), Near(0.540302), Near(4.86272), Near(0),
                      Near(0), Near(10)),
          ElementsAre(Near(-0.540302), Near(-0.841471), Near(0), Near(0.841471),
                      Near(-0.540302), Near(0), Near(0), Near(0), Near(-10))));
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
