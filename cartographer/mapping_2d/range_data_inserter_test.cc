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

#include "cartographer/mapping_2d/range_data_inserter.h"

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping_2d {
namespace {

class RangeDataInserterTest : public ::testing::Test {
 protected:
  RangeDataInserterTest()
      : probability_grid_(
            MapLimits(1., Eigen::Vector2d(1., 5.), CellLimits(5, 5))) {
    auto parameter_dictionary = common::MakeDictionary(
        "return { "
        "insert_free_space = true, "
        "hit_probability = 0.7, "
        "miss_probability = 0.4, "
        "}");
    options_ = CreateRangeDataInserterOptions(parameter_dictionary.get());
    range_data_inserter_ = common::make_unique<RangeDataInserter>(options_);
  }

  void InsertPointCloud() {
    sensor::RangeData range_data;
    range_data.returns.emplace_back(-3.5f, 0.5f, 0.f);
    range_data.returns.emplace_back(-2.5f, 1.5f, 0.f);
    range_data.returns.emplace_back(-1.5f, 2.5f, 0.f);
    range_data.returns.emplace_back(-0.5f, 3.5f, 0.f);
    range_data.origin.x() = -0.5f;
    range_data.origin.y() = 0.5f;
    range_data_inserter_->Insert(range_data, &probability_grid_);
    probability_grid_.FinishUpdate();
  }

  ProbabilityGrid probability_grid_;
  std::unique_ptr<RangeDataInserter> range_data_inserter_;
  proto::RangeDataInserterOptions options_;
};

TEST_F(RangeDataInserterTest, InsertPointCloud) {
  InsertPointCloud();

  EXPECT_NEAR(1., probability_grid_.limits().max().x(), 1e-9);
  EXPECT_NEAR(5., probability_grid_.limits().max().y(), 1e-9);

  const CellLimits& cell_limits = probability_grid_.limits().cell_limits();
  EXPECT_EQ(5, cell_limits.num_x_cells);
  EXPECT_EQ(5, cell_limits.num_y_cells);

  enum class State { UNKNOWN, MISS, HIT };
  State expected_states[5][5] = {
      {State::UNKNOWN, State::UNKNOWN, State::UNKNOWN, State::UNKNOWN,
       State::UNKNOWN},
      {State::UNKNOWN, State::HIT, State::MISS, State::MISS, State::MISS},
      {State::UNKNOWN, State::UNKNOWN, State::HIT, State::MISS, State::MISS},
      {State::UNKNOWN, State::UNKNOWN, State::UNKNOWN, State::HIT, State::MISS},
      {State::UNKNOWN, State::UNKNOWN, State::UNKNOWN, State::UNKNOWN,
       State::HIT}};
  for (int row = 0; row != 5; ++row) {
    for (int column = 0; column != 5; ++column) {
      Eigen::Array2i cell_index(row, column);
      EXPECT_TRUE(probability_grid_.limits().Contains(cell_index));
      switch (expected_states[column][row]) {
        case State::UNKNOWN:
          EXPECT_FALSE(probability_grid_.IsKnown(cell_index));
          break;
        case State::MISS:
          EXPECT_NEAR(options_.miss_probability(),
                      probability_grid_.GetProbability(cell_index), 1e-4);
          break;
        case State::HIT:
          EXPECT_NEAR(options_.hit_probability(),
                      probability_grid_.GetProbability(cell_index), 1e-4);
          break;
      }
    }
  }
}

TEST_F(RangeDataInserterTest, ProbabilityProgression) {
  InsertPointCloud();
  EXPECT_NEAR(
      options_.hit_probability(),
      probability_grid_.GetProbability(probability_grid_.limits().GetCellIndex(
          Eigen::Vector2f(-3.5f, 0.5f))),
      1e-4);
  EXPECT_NEAR(
      options_.miss_probability(),
      probability_grid_.GetProbability(probability_grid_.limits().GetCellIndex(
          Eigen::Vector2f(-2.5f, 0.5f))),
      1e-4);

  for (int i = 0; i < 1000; ++i) {
    InsertPointCloud();
  }
  EXPECT_NEAR(
      mapping::kMaxProbability,
      probability_grid_.GetProbability(probability_grid_.limits().GetCellIndex(
          Eigen::Vector2f(-3.5f, 0.5f))),
      1e-3);
  EXPECT_NEAR(
      mapping::kMinProbability,
      probability_grid_.GetProbability(probability_grid_.limits().GetCellIndex(
          Eigen::Vector2f(-2.5f, 0.5f))),
      1e-3);
}

}  // namespace
}  // namespace mapping_2d
}  // namespace cartographer
