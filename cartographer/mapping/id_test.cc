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

common::Time CreateTime(const int milliseconds) {
  return common::Time(common::FromMilliseconds(milliseconds));
}

class Data {
 public:
  explicit Data(int milliseconds) : time_(CreateTime(milliseconds)) {}

  const common::Time& time() const { return time_; }

 private:
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
  EXPECT_EQ(map_by_id.trajectory_ids().begin(),
            map_by_id.trajectory_ids().end());
  int unknown_trajectory_id = 3;
  EXPECT_EQ(map_by_id.trajectory(unknown_trajectory_id).begin(),
            map_by_id.trajectory(unknown_trajectory_id).end());
  const NodeId id = map_by_id.Append(42, 42);
  EXPECT_FALSE(map_by_id.empty());
  map_by_id.Trim(id);
  EXPECT_TRUE(map_by_id.empty());
  EXPECT_EQ(0, map_by_id.size());
}

TEST(IdTest, DeleteTrajectory) {
  MapById<NodeId, int> map_by_id;
  int trajectory_id = 3;
  int other_trajectory_id = 5;
  map_by_id.Insert(NodeId{trajectory_id, 4}, 5);
  map_by_id.Insert(NodeId{trajectory_id, 5}, 7);
  map_by_id.Insert(NodeId{other_trajectory_id, 1}, 3);
  EXPECT_EQ(map_by_id.size(), 3);
  EXPECT_EQ(2, std::distance(map_by_id.trajectory_ids().begin(),
                             map_by_id.trajectory_ids().end()));
  map_by_id.Trim(NodeId{trajectory_id, 4});
  map_by_id.Trim(NodeId{trajectory_id, 5});
  EXPECT_EQ(0, std::distance(map_by_id.trajectory(trajectory_id).begin(),
                             map_by_id.trajectory(trajectory_id).end()));
  int invalid_trajectory_id = 2;
  EXPECT_EQ(map_by_id.trajectory(invalid_trajectory_id).begin(),
            map_by_id.trajectory(invalid_trajectory_id).end());
  EXPECT_EQ(map_by_id.size(), 1);
  EXPECT_EQ(1, std::distance(map_by_id.trajectory(other_trajectory_id).begin(),
                             map_by_id.trajectory(other_trajectory_id).end()));
  EXPECT_EQ(1, std::distance(map_by_id.trajectory_ids().begin(),
                             map_by_id.trajectory_ids().end()));
  EXPECT_FALSE(map_by_id.empty());
}

TEST(IdTest, MapByIdIterator) {
  MapById<NodeId, int> map_by_id = CreateTestMapById<NodeId>();
  EXPECT_EQ(4, map_by_id.size());
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
    ASSERT_FALSE(expected_id_data.empty());
    EXPECT_EQ(expected_id_data.front().first, id_data.id);
    EXPECT_EQ(expected_id_data.front().second, id_data.data);
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
    ASSERT_FALSE(expected_data.empty());
    EXPECT_EQ(expected_data.front().first, entry.id);
    EXPECT_EQ(expected_data.front().second, entry.data);
    expected_data.pop_front();
  }
  EXPECT_TRUE(expected_data.empty());
}

TEST(IdTest, MapByIdTrajectoryIdRange) {
  MapById<NodeId, int> map_by_id = CreateTestMapById<NodeId>();
  std::deque<int> expected_data = {0, 7, 42};
  for (const int trajectory_id : map_by_id.trajectory_ids()) {
    ASSERT_FALSE(expected_data.empty());
    EXPECT_EQ(expected_data.front(), trajectory_id);
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
      ASSERT_FALSE(expected_id_data.empty());
      EXPECT_EQ(expected_id_data.front().first, entry.id);
      EXPECT_EQ(expected_id_data.front().second, entry.data);
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
  EXPECT_TRUE(map_by_id.find(NodeId{41, 0}) == map_by_id.end());
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
  constexpr int kMaxTimeIncrement = 20;
  constexpr int kMaxNumNodes = 20;
  constexpr int kNumTests = 100;
  constexpr int kTrajectoryId = 1;

  std::mt19937 rng;
  std::uniform_int_distribution<int> dt_dist(1, kMaxTimeIncrement);
  std::uniform_int_distribution<int> N_dist(1, kMaxNumNodes);

  for (int i = 0; i < kNumTests; ++i) {
    const int N = N_dist(rng);
    int t = 0;
    MapById<SubmapId, Data> map_by_id;
    for (int j = 0; j < N; ++j) {
      t = t + dt_dist(rng);
      map_by_id.Append(kTrajectoryId, Data(t));
    }
    std::uniform_int_distribution<int> t0_dist(1, N * kMaxTimeIncrement + 1);
    int t0 = t0_dist(rng);
    auto it = map_by_id.lower_bound(kTrajectoryId, CreateTime(t0));

    auto ground_truth = std::lower_bound(
        map_by_id.BeginOfTrajectory(kTrajectoryId),
        map_by_id.EndOfTrajectory(kTrajectoryId), CreateTime(t0),
        [](MapById<SubmapId, Data>::IdDataReference a, const common::Time& t) {
          return a.data.time() < t;
        });

    CHECK(ground_truth == it);
  }
}

TEST(IdTest, LowerBoundTrimmedTrajectory) {
  constexpr int kTrajectoryId = 1;

  std::mt19937 rng;
  std::uniform_int_distribution<int> dt_dist(1, 20);

  const int N = 500;
  int t = 0;
  MapById<SubmapId, Data> map_by_id;
  for (int j = 0; j < N; ++j) {
    t = t + dt_dist(rng);
    map_by_id.Append(kTrajectoryId, Data(t));
  }

  // Choose random length of a trim segment.
  std::uniform_int_distribution<int> dt_trim_segment_length(
      1, static_cast<int>(N / 2));
  size_t trim_segment_length = dt_trim_segment_length(rng);
  // Choose random start for a trim_segment.
  std::uniform_int_distribution<int> dt_trim_segment_start(
      2, N - trim_segment_length - 1);
  size_t trim_segment_start_index = dt_trim_segment_start(rng);

  auto trim_segment_start = map_by_id.begin();
  std::advance(trim_segment_start, trim_segment_start_index);

  auto trim_segment_end = map_by_id.begin();
  std::advance(trim_segment_end,
               trim_segment_start_index + trim_segment_length);

  for (auto it = trim_segment_start; it != trim_segment_end;) {
    const auto this_it = it;
    ++it;
    map_by_id.Trim(this_it->id);
  }

  auto it = map_by_id.lower_bound(kTrajectoryId, CreateTime(0));

  auto ground_truth =
      std::lower_bound(map_by_id.BeginOfTrajectory(kTrajectoryId),
                       map_by_id.EndOfTrajectory(kTrajectoryId), CreateTime(0),
                       [](MapById<SubmapId, Data>::IdDataReference a,
                          const common::Time& t) { return a.data.time() < t; });

  EXPECT_EQ(ground_truth, it);
}

struct DataStruct {
  const common::Time time;
};

TEST(IdTest, LowerBoundFuzzWithStruct) {
  constexpr int kMaxTimeIncrement = 20;
  constexpr int kMaxNumNodes = 20;
  constexpr int kNumTests = 100;
  constexpr int kTrajectoryId = 1;

  std::mt19937 rng;
  std::uniform_int_distribution<int> dt_dist(1, kMaxTimeIncrement);
  std::uniform_int_distribution<int> N_dist(1, kMaxNumNodes);

  for (int i = 0; i < kNumTests; ++i) {
    const int N = N_dist(rng);
    int t = 0;
    MapById<SubmapId, DataStruct> map_by_id;
    for (int j = 0; j < N; ++j) {
      t = t + dt_dist(rng);
      map_by_id.Append(kTrajectoryId, DataStruct{CreateTime(t)});
    }
    std::uniform_int_distribution<int> t0_dist(1, N * kMaxTimeIncrement + 1);
    int t0 = t0_dist(rng);
    auto it = map_by_id.lower_bound(kTrajectoryId, CreateTime(t0));

    auto ground_truth = std::lower_bound(
        map_by_id.BeginOfTrajectory(kTrajectoryId),
        map_by_id.EndOfTrajectory(kTrajectoryId), CreateTime(t0),
        [](MapById<SubmapId, DataStruct>::IdDataReference a,
           const common::Time& t) { return a.data.time < t; });

    CHECK(ground_truth == it);
  }
}
}  // namespace
}  // namespace mapping
}  // namespace cartographer
