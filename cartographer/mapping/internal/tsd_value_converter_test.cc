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

#include "cartographer/mapping/internal/tsd_value_converter.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

class TSDValueConverterTest : public ::testing::Test {
 protected:
  TSDValueConverterTest()
      : truncation_distance_(0.1f),
        max_weight_(10.0f),
        tsdf_value_converter_(truncation_distance_, max_weight_) {}
  float truncation_distance_;
  float max_weight_;
  TSDValueConverter tsdf_value_converter_;
};

TEST_F(TSDValueConverterTest, DefaultValues) {
  EXPECT_EQ(tsdf_value_converter_.getUnknownWeightValue(), 0);
  EXPECT_EQ(tsdf_value_converter_.getUnknownTSDValue(), 0);
  EXPECT_EQ(tsdf_value_converter_.getMinTSDF(), -truncation_distance_);
  EXPECT_EQ(tsdf_value_converter_.getMaxTSDF(), truncation_distance_);
  EXPECT_EQ(tsdf_value_converter_.getMinWeight(), 0.f);
  EXPECT_EQ(tsdf_value_converter_.getMaxWeight(), max_weight_);
}

TEST_F(TSDValueConverterTest, ValueToTSDFConversions) {
  for (uint16 i = 1; i < 32768; ++i) {
    EXPECT_EQ(
        tsdf_value_converter_.TSDFToValue(tsdf_value_converter_.ValueToTSD(i)),
        i);
  }
}

TEST_F(TSDValueConverterTest, ValueToTSDFConversionsWithUpdateMarker) {
  for (uint16 i = 1; i < 32768; ++i) {
    EXPECT_EQ(
        tsdf_value_converter_.TSDFToValue(tsdf_value_converter_.ValueToTSD(
            i + tsdf_value_converter_.getUpdateMarker())),
        i);
  }
}

TEST_F(TSDValueConverterTest, ValueToWeightConversions) {
  for (uint16 i = 1; i < 32768; ++i) {
    EXPECT_EQ(tsdf_value_converter_.WeightToValue(
                  tsdf_value_converter_.ValueToWeight(i)),
              i);
  }
}

TEST_F(TSDValueConverterTest, ValueToWeightConversionsWithUpdateMarker) {
  for (uint16 i = 1; i < 32768; ++i) {
    EXPECT_EQ(
        tsdf_value_converter_.WeightToValue(tsdf_value_converter_.ValueToWeight(
            i + tsdf_value_converter_.getUpdateMarker())),
        i);
  }
}

TEST_F(TSDValueConverterTest, TSDFToValueConversions) {
  uint16 num_samples = 1000;
  float tolerance = truncation_distance_ * 2.f / 32767.f;
  for (uint16 i = 0; i < num_samples; ++i) {
    float sdf_sample =
        -truncation_distance_ + i * 2.f * truncation_distance_ / num_samples;
    EXPECT_NEAR(tsdf_value_converter_.ValueToTSD(
                    tsdf_value_converter_.TSDFToValue(sdf_sample)),
                sdf_sample, tolerance);
  }
}

TEST_F(TSDValueConverterTest, WeightToValueConversions) {
  uint16 num_samples = 1000;
  float tolerance = max_weight_ / 32767.f;
  for (uint16 i = 0; i < num_samples; ++i) {
    float weight_sample = i * max_weight_ / num_samples;
    EXPECT_NEAR(tsdf_value_converter_.ValueToWeight(
                    tsdf_value_converter_.WeightToValue(weight_sample)),
                weight_sample, tolerance);
  }
}

TEST_F(TSDValueConverterTest, WeightToValueOutOfRangeConversions) {
  float tolerance = max_weight_ / 32767.f;
  EXPECT_NEAR(tsdf_value_converter_.ValueToWeight(
                  tsdf_value_converter_.WeightToValue(2.f * max_weight_)),
              max_weight_, tolerance);
  EXPECT_NEAR(tsdf_value_converter_.ValueToWeight(
                  tsdf_value_converter_.WeightToValue(-max_weight_)),
              0.f, tolerance);
}

TEST_F(TSDValueConverterTest, TSDFToValueOutOfRangeConversions) {
  float tolerance = truncation_distance_ * 2.f / 32767.f;
  EXPECT_NEAR(
      tsdf_value_converter_.ValueToTSD(
          tsdf_value_converter_.TSDFToValue(2.f * truncation_distance_)),
      truncation_distance_, tolerance);
  EXPECT_NEAR(
      tsdf_value_converter_.ValueToTSD(
          tsdf_value_converter_.TSDFToValue(-2.f * truncation_distance_)),
      -truncation_distance_, tolerance);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
