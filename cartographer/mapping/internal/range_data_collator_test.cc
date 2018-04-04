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

#include "cartographer/mapping/internal/range_data_collator.h"

#include "cartographer/common/time.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

const int kNumSamples = 10;

sensor::TimedPointCloudData CreateFakeRangeData(int from, int to) {
  double duration = common::ToSeconds(common::FromUniversal(to) -
                                      common::FromUniversal(from));
  sensor::TimedPointCloudData result{common::FromUniversal(to),
                                     Eigen::Vector3f(0., 1., 2.),
                                     sensor::TimedPointCloud(kNumSamples)};
  for (int i = 0; i < kNumSamples; ++i) {
    double fraction = static_cast<double>(i) / (kNumSamples - 1);
    double relative_time = (1.f - fraction) * -duration;
    result.ranges[i] = Eigen::Vector4f(1., 2., 3., relative_time);
  }
  return result;
}

bool ArePointTimestampsSorted(const sensor::TimedPointCloudOriginData& data) {
  std::vector<float> timestamps;
  timestamps.reserve(data.ranges.size());
  for (const auto& range : data.ranges) {
    timestamps.push_back(range.point_time[3]);
  }
  return std::is_sorted(timestamps.begin(), timestamps.end());
}

TEST(RangeDataCollatorTest, SingleSensor) {
  const std::string sensor_id = "single_sensor";
  RangeDataCollator collator({sensor_id});
  auto output_0 =
      collator.AddRangeData(sensor_id, CreateFakeRangeData(200, 300));
  EXPECT_EQ(common::ToUniversal(output_0.time), 300);
  EXPECT_EQ(output_0.origins.size(), 1);
  EXPECT_EQ(output_0.ranges.size(), kNumSamples);
  EXPECT_TRUE(ArePointTimestampsSorted(output_0));
  auto output_1 =
      collator.AddRangeData(sensor_id, CreateFakeRangeData(300, 500));
  EXPECT_EQ(common::ToUniversal(output_1.time), 500);
  EXPECT_EQ(output_1.origins.size(), 1);
  ASSERT_EQ(output_1.ranges.size(), kNumSamples);
  EXPECT_TRUE(ArePointTimestampsSorted(output_1));
  EXPECT_NEAR(common::ToUniversal(
                  output_1.time +
                  common::FromSeconds(output_1.ranges[0].point_time[3])),
              300, 2);
  auto output_2 =
      collator.AddRangeData(sensor_id, CreateFakeRangeData(-1000, 510));
  EXPECT_EQ(common::ToUniversal(output_2.time), 510);
  EXPECT_EQ(output_2.origins.size(), 1);
  EXPECT_EQ(output_2.ranges.size(), 1);
  EXPECT_EQ(output_2.ranges[0].point_time[3], 0.f);
  EXPECT_TRUE(ArePointTimestampsSorted(output_2));
}

TEST(RangeDataCollatorTest, SingleSensorEmptyData) {
  const std::string sensor_id = "single_sensor";
  RangeDataCollator collator({sensor_id});
  sensor::TimedPointCloudData empty_data{common::FromUniversal(300)};
  auto output_0 = collator.AddRangeData(sensor_id, empty_data);
  EXPECT_EQ(output_0.time, empty_data.time);
  EXPECT_EQ(output_0.ranges.size(), empty_data.ranges.size());
  EXPECT_TRUE(ArePointTimestampsSorted(output_0));
  auto output_1 =
      collator.AddRangeData(sensor_id, CreateFakeRangeData(300, 500));
  EXPECT_EQ(common::ToUniversal(output_1.time), 500);
  EXPECT_EQ(output_1.origins.size(), 1);
  ASSERT_EQ(output_1.ranges.size(), kNumSamples);
  EXPECT_TRUE(ArePointTimestampsSorted(output_1));
  EXPECT_NEAR(common::ToUniversal(
                  output_1.time +
                  common::FromSeconds(output_1.ranges[0].point_time[3])),
              300, 2);
  auto output_2 =
      collator.AddRangeData(sensor_id, CreateFakeRangeData(-1000, 510));
  EXPECT_EQ(common::ToUniversal(output_2.time), 510);
  EXPECT_EQ(output_2.origins.size(), 1);
  EXPECT_EQ(output_2.ranges.size(), 1);
  EXPECT_EQ(output_2.ranges[0].point_time[3], 0.f);
  EXPECT_TRUE(ArePointTimestampsSorted(output_2));
}

TEST(RangeDataCollatorTest, TwoSensors) {
  const std::string sensor_0 = "sensor_0";
  const std::string sensor_1 = "sensor_1";
  RangeDataCollator collator({sensor_0, sensor_1});
  auto output_0 =
      collator.AddRangeData(sensor_0, CreateFakeRangeData(200, 300));
  EXPECT_EQ(output_0.ranges.size(), 0);
  auto output_1 =
      collator.AddRangeData(sensor_1, CreateFakeRangeData(-1000, 310));
  EXPECT_EQ(output_1.origins.size(), 2);
  EXPECT_EQ(common::ToUniversal(output_1.time), 300);
  ASSERT_EQ(output_1.ranges.size(), 2 * kNumSamples - 1);
  EXPECT_NEAR(common::ToUniversal(
                  output_1.time +
                  common::FromSeconds(output_1.ranges[0].point_time[3])),
              -1000, 2);
  EXPECT_EQ(output_1.ranges.back().point_time[3], 0.f);
  EXPECT_TRUE(ArePointTimestampsSorted(output_1));
  auto output_2 =
      collator.AddRangeData(sensor_0, CreateFakeRangeData(300, 500));
  EXPECT_EQ(output_2.origins.size(), 2);
  EXPECT_EQ(common::ToUniversal(output_2.time), 310);
  ASSERT_EQ(output_2.ranges.size(), 2);
  EXPECT_NEAR(common::ToUniversal(
                  output_2.time +
                  common::FromSeconds(output_2.ranges[0].point_time[3])),
              300, 2);
  EXPECT_EQ(output_2.ranges.back().point_time[3], 0.f);
  EXPECT_TRUE(ArePointTimestampsSorted(output_2));
  // Sending the same sensor will flush everything before.
  auto output_3 =
      collator.AddRangeData(sensor_0, CreateFakeRangeData(600, 700));
  EXPECT_EQ(common::ToUniversal(output_3.time), 500);
  EXPECT_EQ(
      output_1.ranges.size() + output_2.ranges.size() + output_3.ranges.size(),
      3 * kNumSamples);
  EXPECT_EQ(output_3.ranges.back().point_time[3], 0.f);
  EXPECT_TRUE(ArePointTimestampsSorted(output_3));
}

TEST(RangeDataCollatorTest, ThreeSensors) {
  const std::string sensor_0 = "sensor_0";
  const std::string sensor_1 = "sensor_1";
  const std::string sensor_2 = "sensor_2";
  RangeDataCollator collator({sensor_0, sensor_1, sensor_2});
  auto output_0 =
      collator.AddRangeData(sensor_0, CreateFakeRangeData(100, 200));
  EXPECT_EQ(output_0.ranges.size(), 0);
  auto output_1 =
      collator.AddRangeData(sensor_1, CreateFakeRangeData(199, 250));
  EXPECT_EQ(output_1.ranges.size(), 0);
  auto output_2 =
      collator.AddRangeData(sensor_2, CreateFakeRangeData(210, 300));
  EXPECT_EQ(output_2.ranges.size(), kNumSamples + 1);
  EXPECT_TRUE(ArePointTimestampsSorted(output_2));
  auto output_3 =
      collator.AddRangeData(sensor_2, CreateFakeRangeData(400, 500));
  EXPECT_EQ(output_2.ranges.size() + output_3.ranges.size(), 3 * kNumSamples);
  EXPECT_TRUE(ArePointTimestampsSorted(output_3));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
