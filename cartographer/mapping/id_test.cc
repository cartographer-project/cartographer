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

#include "cartographer/mapping/id.h"

#include <deque>
#include <iterator>
#include <utility>

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

static MapById<NodeId, int> CreateTestMapById() {
  MapById<NodeId, int> map_by_id;
  map_by_id.Append(7, 2);
  map_by_id.Append(42, 3);
  map_by_id.Append(0, 0);
  map_by_id.Append(0, 1);
  return map_by_id;
}

TEST(IdTest, EmptyMapById) {
  MapById<NodeId, int> map_by_id;
  EXPECT_TRUE(map_by_id.empty());
  const NodeId id = map_by_id.Append(42, 42);
  EXPECT_FALSE(map_by_id.empty());
  map_by_id.Trim(id);
  EXPECT_TRUE(map_by_id.empty());
}

TEST(IdTest, MapByIdIterator) {
  MapById<NodeId, int> map_by_id = CreateTestMapById();
  EXPECT_EQ(2, map_by_id.BeginOfTrajectory(7)->data);
  EXPECT_TRUE(std::next(map_by_id.BeginOfTrajectory(7)) ==
              map_by_id.EndOfTrajectory(7));
  std::deque<std::pair<NodeId, int>> expected_id_data = {
      {NodeId{0, 0}, 0},
      {NodeId{0, 1}, 1},
      {NodeId{7, 0}, 2},
      {NodeId{42, 0}, 3},
  };
  for (const auto& id_data : map_by_id) {
    EXPECT_EQ(expected_id_data.front().first, id_data.id);
    EXPECT_EQ(expected_id_data.front().second, id_data.data);
    ASSERT_FALSE(expected_id_data.empty());
    expected_id_data.pop_front();
  }
  EXPECT_TRUE(expected_id_data.empty());
}

TEST(IdTest, MapByIdTrajectoryRange) {
  MapById<NodeId, int> map_by_id = CreateTestMapById();
  std::deque<std::pair<NodeId, int>> expected_data = {
      {NodeId{0, 0}, 0},
      {NodeId{0, 1}, 1},
  };
  for (const auto& entry : map_by_id.trajectory(0)) {
    EXPECT_EQ(expected_data.front().first, entry.id);
    EXPECT_EQ(expected_data.front().second, entry.data);
    ASSERT_FALSE(expected_data.empty());
    expected_data.pop_front();
  }
  EXPECT_TRUE(expected_data.empty());
}

TEST(IdTest, MapByIdTrajectoryIdRange) {
  MapById<NodeId, int> map_by_id = CreateTestMapById();
  std::deque<int> expected_data = {0, 7, 42};
  for (const int trajectory_id : map_by_id.trajectory_ids()) {
    EXPECT_EQ(expected_data.front(), trajectory_id);
    ASSERT_FALSE(expected_data.empty());
    expected_data.pop_front();
  }
  EXPECT_TRUE(expected_data.empty());
}

TEST(IdTest, MapByIdIterateByTrajectories) {
  MapById<NodeId, int> map_by_id = CreateTestMapById();
  std::deque<std::pair<NodeId, int>> expected_id_data = {
      {NodeId{0, 0}, 0},
      {NodeId{0, 1}, 1},
      {NodeId{7, 0}, 2},
      {NodeId{42, 0}, 3},
  };
  for (int trajectory_id : map_by_id.trajectory_ids()) {
    for (const auto& entry : map_by_id.trajectory(trajectory_id)) {
      EXPECT_EQ(expected_id_data.front().first, entry.id);
      EXPECT_EQ(expected_id_data.front().second, entry.data);
      ASSERT_FALSE(expected_id_data.empty());
      expected_id_data.pop_front();
    }
  }
  EXPECT_TRUE(expected_id_data.empty());
}

TEST(IdTest, MapByIdPrevIterator) {
  MapById<NodeId, int> map_by_id;
  map_by_id.Append(42, 42);
  auto it = map_by_id.end();
  ASSERT_TRUE(it != map_by_id.begin());
  std::advance(it, -1);
  EXPECT_TRUE(it == map_by_id.begin());
}

TEST(IdTest, InsertIntoMapById) {
  MapById<NodeId, int> map_by_id;
  EXPECT_EQ(0, map_by_id.SizeOfTrajectoryOrZero(42));
  map_by_id.Append(42, 42);
  map_by_id.Insert(NodeId{42, 5}, 42);
  EXPECT_EQ(2, map_by_id.SizeOfTrajectoryOrZero(42));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
