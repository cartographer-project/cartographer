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
#include "cartographer/mapping/internal/optimization/cost_functions/cost_helpers.h"
#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"

namespace cartographer {
namespace pose_graph {
namespace {

constexpr int kPoseDimension = 3;
constexpr int kResidualsCount = 3;
constexpr int kParameterBlocksCount = 2;
constexpr int kJacobianColDimension = kResidualsCount * kPoseDimension;

using ::google::protobuf::TextFormat;
using ::testing::ElementsAre;
using ResidualType = std::array<double, kResidualsCount>;
using JacobianType = std::array<std::array<double, kJacobianColDimension>,
                                kParameterBlocksCount>;

// This is the autodiff version of the RelativePoseCost2D.
//
// TODO(pifon): Use the gradient_checker from Ceres.
class AutoDiffRelativePoseCost {
 public:
  explicit AutoDiffRelativePoseCost(
      const proto::RelativePose2D::Parameters& parameters)
      : translation_weight_(parameters.translation_weight()),
        rotation_weight_(parameters.rotation_weight()),
        first_T_second_(transform::ToRigid2(parameters.first_t_second())) {}

  template <typename T>
  bool operator()(const T* const start_pose, const T* const end_pose,
                  T* e) const {
    const std::array<T, 3> error = mapping::optimization::ScaleError(
        mapping::optimization::ComputeUnscaledError(first_T_second_, start_pose,
                                                    end_pose),
        translation_weight_, rotation_weight_);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  const double translation_weight_;
  const double rotation_weight_;
  const transform::Rigid2d first_T_second_;
};

class RelativePoseCost2DTest : public ::testing::Test {
 public:
  RelativePoseCost2DTest() {
    proto::RelativePose2D::Parameters parameters;
    constexpr char kParameters[] = R"PROTO(
      first_t_second {
        translation: { x: 1 y: 1 }
        rotation: -2.214297
      }
      translation_weight: 1
      rotation_weight: 10
    )PROTO";
    EXPECT_TRUE(TextFormat::ParseFromString(kParameters, &parameters));

    auto_diff_cost_ = common::make_unique<RelativePoseCost2D>(parameters);
    analytical_cost_ = common::make_unique<
        ceres::AutoDiffCostFunction<AutoDiffRelativePoseCost, kResidualsCount,
                                    kPoseDimension, kPoseDimension>>(
        new AutoDiffRelativePoseCost(parameters));
    for (int i = 0; i < kParameterBlocksCount; ++i) {
      jacobian_ptrs_[i] = jacobian_[i].data();
    }
  }

  std::pair<const ResidualType&, const JacobianType&>
  EvaluateRelativePoseCost2D(
      const std::array<const double*, 2>& parameter_blocks) {
    return Evaluate(parameter_blocks, analytical_cost_);
  }

  std::pair<const ResidualType&, const JacobianType&> EvaluateAutoDiffCost(
      const std::array<const double*, 2>& parameter_blocks) {
    return Evaluate(parameter_blocks, auto_diff_cost_);
  }

 private:
  std::pair<const ResidualType&, const JacobianType&> Evaluate(
      const std::array<const double*, 2>& parameter_blocks,
      const std::unique_ptr<ceres::CostFunction>& cost_function) {
    cost_function->Evaluate(parameter_blocks.data(), residuals_.data(),
                            jacobian_ptrs_.data());
    return std::make_pair(std::cref(residuals_), std::cref(jacobian_));
  }

  ResidualType residuals_;
  JacobianType jacobian_;
  std::array<double*, kParameterBlocksCount> jacobian_ptrs_;
  std::unique_ptr<ceres::CostFunction> auto_diff_cost_;
  std::unique_ptr<ceres::CostFunction> analytical_cost_;
};

::testing::Matcher<double> Near(double expected) {
  constexpr double kPrecision = 1e-05;
  return ::testing::DoubleNear(expected, kPrecision);
}

TEST_F(RelativePoseCost2DTest, CompareAutoDiffAndAnalytical) {
  std::array<double, kPoseDimension> start_pose{{1., 1., 1.}};
  std::array<double, kPoseDimension> end_pose{{10., 1., 100.}};
  std::array<const double*, kParameterBlocksCount> parameter_blocks{
      {start_pose.data(), end_pose.data()}};

  ResidualType auto_diff_residual, analytical_residual;
  JacobianType auto_diff_jacobian, analytical_jacobian;
  std::tie(auto_diff_residual, auto_diff_jacobian) =
      EvaluateAutoDiffCost(parameter_blocks);
  std::tie(analytical_residual, analytical_jacobian) =
      EvaluateRelativePoseCost2D(parameter_blocks);

  for (int i = 0; i < kResidualsCount; ++i) {
    EXPECT_THAT(auto_diff_residual[i], Near(analytical_residual[i]));
  }
  for (int i = 0; i < kParameterBlocksCount; ++i) {
    for (int j = 0; j < kJacobianColDimension; ++j) {
      EXPECT_THAT(auto_diff_jacobian[i][j], Near(analytical_jacobian[i][j]));
    }
  }
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
