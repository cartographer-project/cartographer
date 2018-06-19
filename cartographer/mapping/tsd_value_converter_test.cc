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

#include "cartographer/mapping/tsd_value_converter.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

class TSDValueConverterTest : public ::testing::Test {
 protected:
  TSDValueConverterTest()
      : truncation_distance(0.1f),
        max_weight(10.0f),
        tsdf_values(truncation_distance, max_weight) {}
  float truncation_distance;
  float max_weight;
  TSDValueConverter tsdf_values;
};

TEST_F(TSDValueConverterTest, DefaultValues) {
  EXPECT_EQ(tsdf_values.getUnknownWeightValue(), 0);
  EXPECT_EQ(tsdf_values.getUnknownTSDFValue(), 0);
  EXPECT_EQ(tsdf_values.getMinTSDF(), -truncation_distance);
  EXPECT_EQ(tsdf_values.getMaxTSDF(), truncation_distance);
  EXPECT_EQ(tsdf_values.getMinWeight(), 0.f);
  EXPECT_EQ(tsdf_values.getMaxWeight(), max_weight);
}

TEST_F(TSDValueConverterTest, ValueToTSDFConversions) {
  for (uint16 i = 1; i < 32768; ++i) {
    EXPECT_EQ(tsdf_values.TSDFToValue(tsdf_values.ValueToTSDF(i)), i);
  }
}

TEST_F(TSDValueConverterTest, ValueToTSDFConversionsWithUpdateMarker) {
  for (uint16 i = 1; i < 32768; ++i) {
    EXPECT_EQ(tsdf_values.TSDFToValue(
                  tsdf_values.ValueToTSDF(i + tsdf_values.getUpdateMarker())),
              i);
  }
}

TEST_F(TSDValueConverterTest, ValueToWeightConversions) {
  for (uint16 i = 1; i < 32768; ++i) {
    EXPECT_EQ(tsdf_values.WeightToValue(tsdf_values.ValueToWeight(i)), i);
  }
}

TEST_F(TSDValueConverterTest, ValueToWeightConversionsWithUpdateMarker) {
  for (uint16 i = 1; i < 32768; ++i) {
    EXPECT_EQ(tsdf_values.WeightToValue(
                  tsdf_values.ValueToWeight(i + tsdf_values.getUpdateMarker())),
              i);
  }
}

TEST_F(TSDValueConverterTest, TSDFToValueConversions) {
  uint16 num_samples = 1000;
  float tolerance = truncation_distance * 2.f / 32767.f;
  for (uint16 i = 0; i < num_samples; ++i) {
    float sdf_sample =
        -truncation_distance + i * 2.f * truncation_distance / num_samples;
    EXPECT_NEAR(tsdf_values.ValueToTSDF(tsdf_values.TSDFToValue(sdf_sample)),
                sdf_sample, tolerance);
  }
}

TEST_F(TSDValueConverterTest, WeightToValueConversions) {
  uint16 num_samples = 1000;
  float tolerance = max_weight / 32767.f;
  for (uint16 i = 0; i < num_samples; ++i) {
    float weight_sample = i * max_weight / num_samples;
    EXPECT_NEAR(
        tsdf_values.ValueToWeight(tsdf_values.WeightToValue(weight_sample)),
        weight_sample, tolerance);
  }
}

TEST_F(TSDValueConverterTest, WeightToValueOutOfRangeConversions) {
  float tolerance = max_weight / 32767.f;
  EXPECT_NEAR(
      tsdf_values.ValueToWeight(tsdf_values.WeightToValue(2.f * max_weight)),
      max_weight, tolerance);
  EXPECT_NEAR(tsdf_values.ValueToWeight(tsdf_values.WeightToValue(-max_weight)),
              0.f, tolerance);
}

TEST_F(TSDValueConverterTest, TSDFToValueOutOfRangeConversions) {
  float tolerance = truncation_distance * 2.f / 32767.f;
  EXPECT_NEAR(tsdf_values.ValueToTSDF(
                  tsdf_values.TSDFToValue(2.f * truncation_distance)),
              truncation_distance, tolerance);
  EXPECT_NEAR(tsdf_values.ValueToTSDF(
                  tsdf_values.TSDFToValue(-2.f * truncation_distance)),
              -truncation_distance, tolerance);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer