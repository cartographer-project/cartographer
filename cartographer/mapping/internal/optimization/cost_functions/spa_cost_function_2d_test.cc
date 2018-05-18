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

#include "cartographer/mapping/internal/optimization/cost_functions/spa_cost_function_2d.h"

#include <chrono>
#include <memory>
#include <random>

#include "cartographer/transform/rigid_transform.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

using ::testing::DoubleEq;
using ::testing::ElementsAre;

TEST(Benchmark, SmokeTest) {
  PoseGraphInterface::Constraint::Pose constraint{
      transform::Rigid3d(Eigen::Vector3d(1., 3., 1.),
                         Eigen::Quaterniond(1., 1., -1., -1.)),
      3, 5};

  std::unique_ptr<ceres::CostFunction> auto_cost(
      CreateAutoDiffSpaCostFunction(constraint));
  std::unique_ptr<ceres::CostFunction> real_cost(
      CreateAnalyticSpaCostFunction(constraint));

  int test_count = 20000000;

  // First create an instance of an engine.
  std::random_device rnd_device;
  // Specify the engine and distribution.
  std::mt19937 mersenne_engine(rnd_device());
  std::uniform_real_distribution<double> dist(10, 52);

  std::vector<std::array<double, 3>> start_poses(test_count);
  std::vector<std::array<double, 3>> end_poses(test_count);
  for (int i = 0; i < test_count; ++i) {
    for (int j = 0; j < 3; ++j) {
      start_poses[i][j] = dist(rnd_device);
      end_poses[i][j] = dist(rnd_device);
    }
  }

  std::array<double, 3> residuals, real_res;
  std::array<std::array<double, 9>, 2> jacobians, real_jacobians;
  std::array<double*, 2> jacobians_ptrs, real_jacobians_ptrs;
  for (int i = 0; i < 2; ++i) {
    jacobians_ptrs[i] = jacobians[i].data();
    real_jacobians_ptrs[i] = real_jacobians[i].data();
  }

  std::array<const double*, 2> parameter_blocks;

  auto real_start = std::chrono::steady_clock::now();
  for (int i = 0; i < test_count; ++i) {
    parameter_blocks[0] = start_poses[i].data();
    parameter_blocks[1] = end_poses[i].data();
    real_cost->Evaluate(parameter_blocks.data(), real_res.data(),
                        real_jacobians_ptrs.data());
  }
  auto real_end = std::chrono::steady_clock::now();
  std::cout << "Real diff = "
            << std::chrono::duration<double, std::milli>(real_end - real_start)
                   .count()
            << std::endl;

  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < test_count; ++i) {
    parameter_blocks[0] = start_poses[i].data();
    parameter_blocks[1] = end_poses[i].data();
    auto_cost->Evaluate(parameter_blocks.data(), residuals.data(),
                        jacobians_ptrs.data());
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "Auto diff = "
            << std::chrono::duration<double, std::milli>(end - start).count()
            << std::endl;

  for (int i = 0; i < 2; ++i) {
    std::cout << "auto res " << i << " = " << residuals[i]
              << " real_res = " << real_res[i] << std::endl;
    for (int j = 0; j < 9; ++j) {
      std::cout << "auto jac " << i << "," << j << " = " << jacobians[i][j]
                << " real_jac = " << real_jacobians[i][j] << std::endl;
    }
  }
}

}  // namespace
}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
