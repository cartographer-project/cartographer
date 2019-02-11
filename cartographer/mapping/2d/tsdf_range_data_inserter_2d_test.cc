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

#include "cartographer/mapping/2d/tsdf_range_data_inserter_2d.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace {

class RangeDataInserterTest2DTSDF : public ::testing::Test {
 protected:
  RangeDataInserterTest2DTSDF()
      : tsdf_(MapLimits(1., Eigen::Vector2d(0., 7.), CellLimits(8, 1)), 2.0,
              10.0, &conversion_tables_) {
    auto parameter_dictionary = common::MakeDictionary(
        "return { "
        "truncation_distance = 2.0,"
        "maximum_weight = 10.,"
        "update_free_space = false,"
        "normal_estimation_options = {"
        "num_normal_samples = 2,"
        "sample_radius = 10.,"
        "},"
        "project_sdf_distance_to_scan_normal = false,"
        "update_weight_range_exponent = 0,"
        "update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0,"
        "update_weight_distance_cell_to_hit_kernel_bandwidth = 0,"
        "}");
    options_ = CreateTSDFRangeDataInserterOptions2D(parameter_dictionary.get());
    range_data_inserter_ = absl::make_unique<TSDFRangeDataInserter2D>(options_);
  }

  void InsertPoint() {
    auto range_data = sensor::RangeData();
    range_data.returns.push_back({Eigen::Vector3f{-0.5f, 3.5f, 0.f}});
    range_data.origin.x() = -0.5f;
    range_data.origin.y() = -0.5f;
    range_data_inserter_->Insert(range_data, &tsdf_);
    tsdf_.FinishUpdate();
  }

  ValueConversionTables conversion_tables_;
  proto::TSDFRangeDataInserterOptions2D options_;
  TSDF2D tsdf_;
  std::unique_ptr<TSDFRangeDataInserter2D> range_data_inserter_;
};

class MockCellProperties {
 public:
  MockCellProperties(const Eigen::Array2i& cell_index, const TSDF2D& tsdf)
      : is_known_(tsdf.IsKnown(cell_index)),
        tsd_(tsdf.GetTSD(cell_index)),
        weight_(tsdf.GetWeight(cell_index)){};

  bool is_known_;
  float tsd_;
  float weight_;
};

::std::ostream& operator<<(::std::ostream& os, const MockCellProperties& bar) {
  return os << std::to_string(bar.is_known_) + "\t" + std::to_string(bar.tsd_) +
                   "\t" + std::to_string(bar.weight_);
}

MATCHER_P3(EqualCellProperties, expected_is_known, expected_tsd,
           expected_weight,
           std::string("Expected ") + std::to_string(expected_is_known) + "\t" +
               std::to_string(expected_tsd) + "\t" +
               std::to_string(expected_weight)) {
  bool result = expected_is_known == arg.is_known_;
  result = result && (std::abs(expected_tsd - arg.tsd_) < 1e-4);
  result = result && (std::abs(expected_weight - arg.weight_) < 1e-2);
  return result;
}

TEST_F(RangeDataInserterTest2DTSDF, InsertPoint) {
  InsertPoint();
  const float truncation_distance =
      static_cast<float>(options_.truncation_distance());
  const float maximum_weight = static_cast<float>(options_.maximum_weight());

  for (float y = 1.5; y < 6.; ++y) {
    // Cell intersects with ray.
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, truncation_distance), -truncation_distance);
    float expected_weight = 1.f;
    EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
                EqualCellProperties(true, expected_tsdf, expected_weight));
    // Cells don't intersect with ray.
    x = 0.5f;
    cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    expected_tsdf = -truncation_distance;
    expected_weight = 0.;
    EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
                EqualCellProperties(false, expected_tsdf, expected_weight));
    x = 1.5f;
    cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
                EqualCellProperties(false, expected_tsdf, expected_weight));
  }

  // Cells don't intersect with ray.
  Eigen::Array2i cell_index =
      tsdf_.limits().GetCellIndex(Eigen::Vector2f(0.5, 6.5));
  EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
              EqualCellProperties(false, -truncation_distance, 0.));
  cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(-0.5, -1.5));
  EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
              EqualCellProperties(false, -truncation_distance, 0.));
  for (int i = 0; i < 1000; ++i) {
    InsertPoint();
  }
  for (float y = 1.5; y < 6.; ++y) {
    // Cell intersects with ray.
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, truncation_distance), -truncation_distance);
    EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
                EqualCellProperties(true, expected_tsdf, maximum_weight));
  }
}

TEST_F(RangeDataInserterTest2DTSDF, InsertPointWithFreeSpaceUpdate) {
  options_.set_update_free_space(true);
  range_data_inserter_ = absl::make_unique<TSDFRangeDataInserter2D>(options_);
  InsertPoint();
  const float truncation_distance =
      static_cast<float>(options_.truncation_distance());
  const float maximum_weight = static_cast<float>(options_.maximum_weight());

  for (float y = -0.5; y < 6.; ++y) {
    // Cells intersect with ray.
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, truncation_distance), -truncation_distance);
    float expected_weight = 1.f;
    EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
                EqualCellProperties(true, expected_tsdf, expected_weight));
    // Cells don't intersect with ray.
    x = 0.5f;
    cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    expected_tsdf = -truncation_distance;
    expected_weight = 0.;
    EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
                EqualCellProperties(false, expected_tsdf, expected_weight));
    x = 1.5f;
    cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
                EqualCellProperties(false, expected_tsdf, expected_weight));
  }

  // Cells don't intersect with the ray.
  Eigen::Array2i cell_index =
      tsdf_.limits().GetCellIndex(Eigen::Vector2f(-0.5, 6.5));
  EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
              EqualCellProperties(false, -truncation_distance, 0.));
  cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(-0.5, -1.5));
  EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
              EqualCellProperties(false, -truncation_distance, 0.));
  for (int i = 0; i < 1000; ++i) {
    InsertPoint();
  }
  for (float y = -0.5; y < 6.; ++y) {
    // Cell intersects with ray.
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, truncation_distance), -truncation_distance);
    EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
                EqualCellProperties(true, expected_tsdf, maximum_weight));
  }
}

TEST_F(RangeDataInserterTest2DTSDF, InsertPointLinearWeight) {
  options_.set_update_weight_range_exponent(1);
  range_data_inserter_ = absl::make_unique<TSDFRangeDataInserter2D>(options_);
  InsertPoint();
  const float truncation_distance =
      static_cast<float>(options_.truncation_distance());
  for (float y = 1.5; y < 6.; ++y) {
    // Cell intersects with ray.
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, truncation_distance), -truncation_distance);
    float expected_weight = 1.f / 4.f;
    EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
                EqualCellProperties(true, expected_tsdf, expected_weight));
  }
}

TEST_F(RangeDataInserterTest2DTSDF, InsertPointQuadraticWeight) {
  options_.set_update_weight_range_exponent(2);
  range_data_inserter_ = absl::make_unique<TSDFRangeDataInserter2D>(options_);
  InsertPoint();
  const float truncation_distance =
      static_cast<float>(options_.truncation_distance());
  for (float y = 1.5; y < 6.; ++y) {
    // Cell intersects with ray.
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, truncation_distance), -truncation_distance);
    float expected_weight = 1.f / std::pow(4.f, 2);
    EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
                EqualCellProperties(true, expected_tsdf, expected_weight));
  }
}

TEST_F(RangeDataInserterTest2DTSDF,
       InsertSmallAnglePointWithoutNormalProjection) {
  auto range_data = sensor::RangeData();
  range_data.returns.push_back({Eigen::Vector3f{-0.5f, 3.5f, 0.f}});
  range_data.returns.push_back({Eigen::Vector3f{5.5f, 3.5f, 0.f}});
  range_data.returns.push_back({Eigen::Vector3f{10.5f, 3.5f, 0.f}});
  range_data.origin.x() = -0.5f;
  range_data.origin.y() = -0.5f;
  range_data_inserter_->Insert(range_data, &tsdf_);
  tsdf_.FinishUpdate();
  float x = 4.5f;
  float y = 2.5f;
  Eigen::Array2i cell_index =
      tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
  Eigen::Vector2f ray =
      Eigen::Vector2f(-0.5f, -0.5f) - Eigen::Vector2f(5.5f, 3.5f);
  float ray_length = ray.norm();
  Eigen::Vector2f origin_to_cell =
      Eigen::Vector2f(x, y) - Eigen::Vector2f(-0.5f, -0.5f);
  float distance_origin_to_cell = origin_to_cell.norm();
  float expected_tsdf = ray_length - distance_origin_to_cell;
  float expected_weight = 1.f;
  EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
              EqualCellProperties(true, expected_tsdf, expected_weight));
}

TEST_F(RangeDataInserterTest2DTSDF, InsertSmallAnglePointWitNormalProjection) {
  options_.set_project_sdf_distance_to_scan_normal(true);
  range_data_inserter_ = absl::make_unique<TSDFRangeDataInserter2D>(options_);
  auto range_data = sensor::RangeData();
  range_data.returns.push_back({Eigen::Vector3f{-0.5f, 3.5f, 0.f}});
  range_data.returns.push_back({Eigen::Vector3f{5.5f, 3.5f, 0.f}});
  range_data.origin.x() = -0.5f;
  range_data.origin.y() = -0.5f;
  range_data_inserter_->Insert(range_data, &tsdf_);
  tsdf_.FinishUpdate();
  float x = 4.5f;
  float y = 2.5f;
  Eigen::Array2i cell_index =
      tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
  float expected_tsdf = 1.f;
  float expected_weight = 1.f;
  EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
              EqualCellProperties(true, expected_tsdf, expected_weight));
  x = 6.5f;
  y = 4.5f;
  cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
  expected_tsdf = -1.f;
  expected_weight = 1.f;
  EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
              EqualCellProperties(true, expected_tsdf, expected_weight));
}

TEST_F(RangeDataInserterTest2DTSDF,
       InsertPointsWithAngleScanNormalToRayWeight) {
  float bandwidth = 10.f;
  options_.set_update_weight_angle_scan_normal_to_ray_kernel_bandwidth(
      bandwidth);
  range_data_inserter_ = absl::make_unique<TSDFRangeDataInserter2D>(options_);
  auto range_data = sensor::RangeData();
  range_data.returns.push_back({Eigen::Vector3f{-0.5f, 3.5f, 0.f}});
  range_data.returns.push_back({Eigen::Vector3f{5.5f, 3.5f, 0.f}});
  range_data.origin.x() = -0.5f;
  range_data.origin.y() = -0.5f;
  range_data_inserter_->Insert(range_data, &tsdf_);
  tsdf_.FinishUpdate();
  float x = -0.5f;
  float y = 3.5f;
  // Ray is perpendicular to surface.
  Eigen::Array2i cell_index =
      tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
  float expected_weight = 1.f / (std::sqrt(2 * M_PI) * bandwidth);
  EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
  x = 6.5f;
  y = 4.5f;
  cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
  EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
  // Ray is inclined relative to surface.
  cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
  float angle = std::atan(7.f / 5.f);
  expected_weight = 1.f / (std::sqrt(2 * M_PI) * bandwidth) *
                    std::exp(angle * angle / (2 * std::pow(bandwidth, 2)));
  EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
  x = 6.5f;
  y = 4.5f;
  cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
  EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
}

TEST_F(RangeDataInserterTest2DTSDF, InsertPointsWithDistanceCellToHit) {
  float bandwidth = 10.f;
  options_.set_update_weight_distance_cell_to_hit_kernel_bandwidth(bandwidth);
  range_data_inserter_ = absl::make_unique<TSDFRangeDataInserter2D>(options_);
  InsertPoint();
  const float truncation_distance =
      static_cast<float>(options_.truncation_distance());
  for (float y = 1.5; y < 6.; ++y) {
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, truncation_distance), -truncation_distance);
    float expected_weight =
        1.f / (std::sqrt(2 * M_PI) * bandwidth) *
        std::exp(std::pow(expected_tsdf, 2) / (2 * std::pow(bandwidth, 2)));
    EXPECT_THAT(MockCellProperties(cell_index, tsdf_),
                EqualCellProperties(true, expected_tsdf, expected_weight));
  }
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer