/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer/mapping_3d/scan_matching/precomputation_grid.h"

#include <random>
#include <tuple>
#include <vector>

#include "cartographer/mapping_3d/hybrid_grid.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {
namespace {

TEST(PrecomputedGridGeneratorTest, TestAgainstNaiveAlgorithm) {
  HybridGrid hybrid_grid(2.f);

  std::mt19937 rng(23847);
  std::uniform_int_distribution<int> coordinate_distribution(-50, 49);
  std::uniform_real_distribution<float> value_distribution(
      mapping::kMinProbability, mapping::kMaxProbability);
  for (int i = 0; i < 1000; ++i) {
    const auto x = coordinate_distribution(rng);
    const auto y = coordinate_distribution(rng);
    const auto z = coordinate_distribution(rng);
    const Eigen::Array3i cell_index(x, y, z);
    hybrid_grid.SetProbability(cell_index, value_distribution(rng));
  }

  std::vector<PrecomputationGrid> precomputed_grids;
  for (int depth = 0; depth <= 3; ++depth) {
    if (depth == 0) {
      precomputed_grids.push_back(ConvertToPrecomputationGrid(hybrid_grid));
    } else {
      precomputed_grids.push_back(
          PrecomputeGrid(precomputed_grids.back(), false,
                         (1 << (depth - 1)) * Eigen::Array3i::Ones()));
    }
    const int width = 1 << depth;
    for (int i = 0; i < 100; ++i) {
      const auto x = coordinate_distribution(rng);
      const auto y = coordinate_distribution(rng);
      const auto z = coordinate_distribution(rng);
      float max_probability = 0.;
      for (int dx = 0; dx < width; ++dx) {
        for (int dy = 0; dy < width; ++dy) {
          for (int dz = 0; dz < width; ++dz) {
            max_probability = std::max(
                max_probability, hybrid_grid.GetProbability(
                                     Eigen::Array3i(x + dx, y + dy, z + dz)));
          }
        }
      }

      EXPECT_NEAR(max_probability,
                  PrecomputationGrid::ToProbability(
                      precomputed_grids.back().value(Eigen::Array3i(x, y, z))),
                  1e-2);
    }
  }
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer
