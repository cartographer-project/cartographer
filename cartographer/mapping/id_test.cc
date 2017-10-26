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

#include <algorithm>
#include <deque>
#include <iterator>
#include <random>
#include <utility>

#include "cartographer/common/time.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

common::Time CreateTime(int secs) {
  return common::Time(std::chrono::seconds(secs));
}

struct Data {
  Data(int secs) : time_(CreateTime(secs)) {}

  const common::Time& time() const {
    return time_;
  }

  const common::Time time_;
};

template <typename IdType>
static MapById<IdType, int> CreateTestMapById() {
  MapById<IdType, int> map_by_id;
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
  MapById<NodeId, int> map_by_id = CreateTestMapById<NodeId>();
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
  MapById<NodeId, int> map_by_id = CreateTestMapById<NodeId>();
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
  MapById<NodeId, int> map_by_id = CreateTestMapById<NodeId>();
  std::deque<int> expected_data = {0, 7, 42};
  for (const int trajectory_id : map_by_id.trajectory_ids()) {
    EXPECT_EQ(expected_data.front(), trajectory_id);
    ASSERT_FALSE(expected_data.empty());
    expected_data.pop_front();
  }
  EXPECT_TRUE(expected_data.empty());
}

TEST(IdTest, MapByIdIterateByTrajectories) {
  MapById<NodeId, int> map_by_id = CreateTestMapById<NodeId>();
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

TEST(IdTest, FindNodeId) {
  MapById<NodeId, int> map_by_id;
  map_by_id.Append(42, 42);
  map_by_id.Append(42, 43);
  map_by_id.Append(42, 44);
  CHECK_EQ(map_by_id.find(NodeId{42, 1})->data, 43);
  EXPECT_TRUE(map_by_id.find(NodeId{42, 3}) == map_by_id.end());
}

TEST(IdTest, FindSubmapId) {
  MapById<SubmapId, int> map_by_id;
  map_by_id.Append(42, 42);
  map_by_id.Append(42, 43);
  map_by_id.Append(42, 44);
  CHECK_EQ(map_by_id.find(SubmapId{42, 1})->data, 43);
  EXPECT_TRUE(map_by_id.find(SubmapId{42, 3}) == map_by_id.end());
}

TEST(IdTest, LowerBoundEdgeCases) {
  MapById<SubmapId, Data> map_by_id;
  map_by_id.Append(0, Data(1));
  map_by_id.Append(2, Data(2));
  CHECK(map_by_id.lower_bound(1, CreateTime(10)) ==
        map_by_id.EndOfTrajectory(1));
  CHECK(map_by_id.lower_bound(2, CreateTime(3)) ==
        map_by_id.EndOfTrajectory(2));
  CHECK(map_by_id.lower_bound(2, CreateTime(1)) ==
        map_by_id.BeginOfTrajectory(2));
}

TEST(IdTest, LowerBound) {
  MapById<SubmapId, Data> map_by_id;
  map_by_id.Append(0, Data(1));
  map_by_id.Append(0, Data(2));
  map_by_id.Append(0, Data(4));
  map_by_id.Append(0, Data(5));
  CHECK(map_by_id.lower_bound(0, CreateTime(3)) ==
        (MapById<SubmapId, Data>::ConstIterator(map_by_id, SubmapId{0, 2})));
  CHECK(map_by_id.lower_bound(0, CreateTime(2)) ==
        (MapById<SubmapId, Data>::ConstIterator(map_by_id, SubmapId{0, 1})));
  CHECK(map_by_id.lower_bound(0, CreateTime(4)) ==
        (MapById<SubmapId, Data>::ConstIterator(map_by_id, SubmapId{0, 2})));
}

TEST(IdTest, LowerBoundFuzz) {
  constexpr int MAX_T_INCREMENT = 20;
  constexpr int MAX_N_NODES = 20;
  constexpr int N_TESTS = 100;

  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_int_distribution<int> dt_dist(1, MAX_T_INCREMENT);
  std::uniform_int_distribution<int> N_dist(1, MAX_N_NODES);

  for (int i = 0; i < N_TESTS; ++i) {
    const int N = N_dist(rng);
    int t = 0;
    MapById<SubmapId, Data> map_by_id;
    for (int j = 0; j < N; ++j) {
      t = t + dt_dist(rng);
      map_by_id.Append(0, Data(t));
    }
    std::uniform_int_distribution<int> t0_dist(1, N * MAX_T_INCREMENT + 1);
    int t0 = t0_dist(rng);
    auto it = map_by_id.lower_bound(0, CreateTime(t0));

    auto ground_truth = std::lower_bound(
        map_by_id.BeginOfTrajectory(0), map_by_id.EndOfTrajectory(0),
        CreateTime(t0),
        [](MapById<SubmapId, Data>::IdDataReference a, const common::Time& t) {
          return a.data.time() < t;
        });

    CHECK(ground_truth == it);
  }
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
