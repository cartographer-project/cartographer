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

#include "cartographer/mapping_3d/hybrid_grid.h"

#include <random>
#include <tuple>

#include "gmock/gmock.h"

namespace cartographer {
namespace mapping_3d {
namespace {

TEST(HybridGridTest, ApplyOdds) {
  HybridGrid hybrid_grid(1.f, Eigen::Vector3f(-0.5f, -0.5f, -0.5f));

  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(0, 0, 0)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(0, 1, 0)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(1, 0, 0)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(1, 1, 0)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(0, 0, 1)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(0, 1, 1)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(1, 0, 1)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(1, 1, 1)));

  hybrid_grid.SetProbability(Eigen::Array3i(1, 0, 1), 0.5f);

  hybrid_grid.StartUpdate();
  hybrid_grid.ApplyLookupTable(
      Eigen::Array3i(1, 0, 1),
      mapping::ComputeLookupTableToApplyOdds(mapping::Odds(0.9f)));
  EXPECT_GT(hybrid_grid.GetProbability(Eigen::Array3i(1, 0, 1)), 0.5f);

  hybrid_grid.SetProbability(Eigen::Array3i(0, 1, 0), 0.5f);

  hybrid_grid.StartUpdate();
  hybrid_grid.ApplyLookupTable(
      Eigen::Array3i(0, 1, 0),
      mapping::ComputeLookupTableToApplyOdds(mapping::Odds(0.1f)));
  EXPECT_LT(hybrid_grid.GetProbability(Eigen::Array3i(0, 1, 0)), 0.5f);

  // Tests adding odds to an unknown cell.
  hybrid_grid.StartUpdate();
  hybrid_grid.ApplyLookupTable(
      Eigen::Array3i(1, 1, 1),
      mapping::ComputeLookupTableToApplyOdds(mapping::Odds(0.42f)));
  EXPECT_NEAR(hybrid_grid.GetProbability(Eigen::Array3i(1, 1, 1)), 0.42f, 1e-4);

  // Tests that further updates are ignored if StartUpdate() isn't called.
  hybrid_grid.ApplyLookupTable(
      Eigen::Array3i(1, 1, 1),
      mapping::ComputeLookupTableToApplyOdds(mapping::Odds(0.9f)));
  EXPECT_NEAR(hybrid_grid.GetProbability(Eigen::Array3i(1, 1, 1)), 0.42f, 1e-4);
  hybrid_grid.StartUpdate();
  hybrid_grid.ApplyLookupTable(
      Eigen::Array3i(1, 1, 1),
      mapping::ComputeLookupTableToApplyOdds(mapping::Odds(0.9f)));
  EXPECT_GT(hybrid_grid.GetProbability(Eigen::Array3i(1, 1, 1)), 0.42f);
}

TEST(HybridGridTest, GetProbability) {
  HybridGrid hybrid_grid(1.f, Eigen::Vector3f(-0.5f, -0.5f, -0.5f));

  hybrid_grid.SetProbability(
      hybrid_grid.GetCellIndex(Eigen::Vector3f(-0.5f, 0.5f, 0.5f)),
      mapping::kMaxProbability);
  EXPECT_NEAR(hybrid_grid.GetProbability(
                  hybrid_grid.GetCellIndex(Eigen::Vector3f(-0.5f, 0.5f, 0.5f))),
              mapping::kMaxProbability, 1e-6);
  for (const Eigen::Array3i& index :
       {hybrid_grid.GetCellIndex(Eigen::Vector3f(-0.5f, 1.5, 0.5f)),
        hybrid_grid.GetCellIndex(Eigen::Vector3f(.5f, 0.5, 0.5f)),
        hybrid_grid.GetCellIndex(Eigen::Vector3f(0.5f, 1.5, 0.5f))}) {
    EXPECT_FALSE(hybrid_grid.IsKnown(index));
  }
}

MATCHER_P(AllCwiseEqual, index, "") { return (arg == index).all(); }

TEST(HybridGridTest, GetCellIndex) {
  HybridGrid hybrid_grid(2.f, Eigen::Vector3f(-7.f, -13.f, -2.f));

  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(-7.f, -13.f, -2.f)),
              AllCwiseEqual(Eigen::Array3i(0, 0, 0)));
  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(-7.f, 13.f, 8.f)),
              AllCwiseEqual(Eigen::Array3i(0, 13, 5)));
  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(7.f, -13.f, 8.f)),
              AllCwiseEqual(Eigen::Array3i(7, 0, 5)));
  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(7.f, 13.f, -2.f)),
              AllCwiseEqual(Eigen::Array3i(7, 13, 0)));

  // Check around the origin.
  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(1.5f, -1.5f, -1.5f)),
              AllCwiseEqual(Eigen::Array3i(4, 6, 0)));
  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(0.5f, -0.5f, -0.5f)),
              AllCwiseEqual(Eigen::Array3i(4, 6, 1)));
  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(-0.5f, 1.5f, 0.5f)),
              AllCwiseEqual(Eigen::Array3i(3, 7, 1)));
  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(-1.5f, 0.5f, 1.5f)),
              AllCwiseEqual(Eigen::Array3i(3, 7, 2)));
}

TEST(HybridGridTest, GetCenterOfCell) {
  HybridGrid hybrid_grid(2.f, Eigen::Vector3f(-7.f, -13.f, -2.f));

  const Eigen::Array3i index(3, 2, 1);
  const Eigen::Vector3f center = hybrid_grid.GetCenterOfCell(index);
  EXPECT_NEAR(-1.f, center.x(), 1e-6);
  EXPECT_NEAR(-9.f, center.y(), 1e-6);
  EXPECT_NEAR(0.f, center.z(), 1e-6);
  EXPECT_THAT(hybrid_grid.GetCellIndex(center), AllCwiseEqual(index));
}

TEST(HybridGridTest, TestIteration) {
  HybridGrid hybrid_grid(2.f, Eigen::Vector3f(-7.f, -12.f, 0.f));

  std::map<std::tuple<int, int, int>, float> values;
  std::mt19937 rng(1285120005);
  std::uniform_real_distribution<float> value_distribution(
      mapping::kMinProbability, mapping::kMaxProbability);
  std::uniform_int_distribution<int> xyz_distribution(-3000, 2999);
  for (int i = 0; i < 10000; ++i) {
    const auto x = xyz_distribution(rng);
    const auto y = xyz_distribution(rng);
    const auto z = xyz_distribution(rng);
    values.emplace(std::make_tuple(x, y, z), value_distribution(rng));
  }

  for (const auto& pair : values) {
    const Eigen::Array3i cell_index(std::get<0>(pair.first),
                                    std::get<1>(pair.first),
                                    std::get<2>(pair.first));
    hybrid_grid.SetProbability(cell_index, pair.second);
  }

  for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {
    const Eigen::Array3i cell_index = it.GetCellIndex();
    const float iterator_probability =
        mapping::ValueToProbability(it.GetValue());
    EXPECT_EQ(iterator_probability, hybrid_grid.GetProbability(cell_index));
    const std::tuple<int, int, int> key =
        std::make_tuple(cell_index[0], cell_index[1], cell_index[2]);
    EXPECT_TRUE(values.count(key));
    EXPECT_NEAR(values[key], iterator_probability, 1e-4);
    values.erase(key);
  }

  // Test that range based loop is equivalent to using the iterator.
  auto it = HybridGrid::Iterator(hybrid_grid);
  for (const auto& cell : hybrid_grid) {
    ASSERT_FALSE(it.Done());
    EXPECT_THAT(cell.first, AllCwiseEqual(it.GetCellIndex()));
    EXPECT_EQ(cell.second, it.GetValue());
    it.Next();
  }

  // Now 'values' must not contain values.
  for (const auto& pair : values) {
    const Eigen::Array3i cell_index(std::get<0>(pair.first),
                                    std::get<1>(pair.first),
                                    std::get<2>(pair.first));
    ADD_FAILURE() << cell_index << " Probability: " << pair.second;
  }
}

}  // namespace
}  // namespace mapping_3d
}  // namespace cartographer
