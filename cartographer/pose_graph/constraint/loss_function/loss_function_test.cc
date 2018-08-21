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

#include "cartographer/pose_graph/constraint/loss_function/loss_function.h"

#include "cartographer/testing/test_helpers.h"

namespace cartographer {
namespace pose_graph {
namespace {

using testing::ParseProto;

TEST(LossFunctionTest, ConstructQuadraticLoss) {
  LossFunction quadratic_loss(
      ParseProto<proto::LossFunction>(R"(quadratic_loss: {})"));
  EXPECT_EQ(nullptr, quadratic_loss.ceres_loss());
}

TEST(LossFunctionTest, ConstructHuberLoss) {
  LossFunction huber_loss(
      ParseProto<proto::LossFunction>(R"(huber_loss: { scale: 0.5 })"));
  EXPECT_NE(nullptr, dynamic_cast<ceres::HuberLoss*>(huber_loss.ceres_loss()));
}

TEST(LossFunctionDeathTest, FailToConstructUnspecifiedLoss) {
  EXPECT_DEATH(LossFunction(proto::LossFunction{}), "");
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
