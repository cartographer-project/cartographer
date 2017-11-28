/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/sensor/map_by_time.h"

#include <deque>

#include "cartographer/common/time.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {

common::Time CreateTime(const int milliseconds) {
  return common::Time(common::FromMilliseconds(milliseconds));
}

struct Data {
  common::Time time;
};

struct NodeData {
  common::Time time;
};

TEST(MapByTimeTest, AppendAndViewTrajectory) {
  MapByTime<Data> map_by_time;
  map_by_time.Append(0, Data{CreateTime(10)});
  map_by_time.Append(42, Data{CreateTime(42)});
  map_by_time.Append(42, Data{CreateTime(43)});
  std::deque<Data> expected_data = {Data{CreateTime(42)}, Data{CreateTime(43)}};
  for (const Data& data : map_by_time.trajectory(42)) {
    ASSERT_FALSE(expected_data.empty());
    EXPECT_EQ(expected_data.front().time, data.time);
    expected_data.pop_front();
  }
  EXPECT_TRUE(expected_data.empty());
}

TEST(MapByTimeTest, Trimming) {
  MapByTime<Data> map_by_time;
  EXPECT_FALSE(map_by_time.HasTrajectory(42));
  map_by_time.Append(42, Data{CreateTime(1)});
  map_by_time.Append(42, Data{CreateTime(41)});
  map_by_time.Append(42, Data{CreateTime(42)});
  map_by_time.Append(42, Data{CreateTime(43)});
  map_by_time.Append(42, Data{CreateTime(47)});
  map_by_time.Append(42, Data{CreateTime(48)});
  map_by_time.Append(42, Data{CreateTime(49)});
  map_by_time.Append(42, Data{CreateTime(5000)});
  EXPECT_TRUE(map_by_time.HasTrajectory(42));
  // Trim one node.
  mapping::MapById<mapping::NodeId, NodeData> map_by_id;
  map_by_id.Append(42, NodeData{CreateTime(42)});
  map_by_id.Append(42, NodeData{CreateTime(46)});
  map_by_id.Append(42, NodeData{CreateTime(48)});
  map_by_time.Trim(map_by_id, mapping::NodeId{42, 1});
  map_by_id.Trim(mapping::NodeId{42, 1});
  ASSERT_TRUE(map_by_time.HasTrajectory(42));
  std::deque<Data> expected_data = {
      Data{CreateTime(1)},  Data{CreateTime(41)}, Data{CreateTime(42)},
      Data{CreateTime(48)}, Data{CreateTime(49)}, Data{CreateTime(5000)}};
  for (const Data& data : map_by_time.trajectory(42)) {
    ASSERT_FALSE(expected_data.empty());
    EXPECT_EQ(expected_data.front().time, data.time);
    expected_data.pop_front();
  }
  EXPECT_TRUE(expected_data.empty());
  // Trim everything.
  map_by_time.Trim(map_by_id, mapping::NodeId{42, 2});
  map_by_id.Trim(mapping::NodeId{42, 2});
  map_by_time.Trim(map_by_id, mapping::NodeId{42, 0});
  map_by_id.Trim(mapping::NodeId{42, 0});
  EXPECT_FALSE(map_by_time.HasTrajectory(42));
}

TEST(MapByTimeTest, TrimmingDoesNotCreateTrajectory) {
  MapByTime<Data> map_by_time;
  EXPECT_FALSE(map_by_time.HasTrajectory(42));
  mapping::MapById<mapping::NodeId, NodeData> map_by_id;
  map_by_id.Append(42, NodeData{CreateTime(42)});
  map_by_time.Trim(map_by_id, mapping::NodeId{42, 0});
  EXPECT_FALSE(map_by_time.HasTrajectory(42));
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
