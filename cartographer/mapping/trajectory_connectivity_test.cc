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

#include "cartographer/mapping/trajectory_connectivity.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr int kNumTrajectories = 10;

TEST(TrajectoryConnectivityTest, TransitivelyConnected) {
  TrajectoryConnectivity trajectory_connectivity;

  // Make sure nothing's connected until we connect some things.
  for (int trajectory_a = 0; trajectory_a < kNumTrajectories; ++trajectory_a) {
    for (int trajectory_b = 0; trajectory_b < kNumTrajectories;
         ++trajectory_b) {
      EXPECT_EQ(trajectory_a == trajectory_b,
                trajectory_connectivity.TransitivelyConnected(trajectory_a,
                                                              trajectory_b));
    }
  }

  // Connect some stuff up.
  trajectory_connectivity.Connect(0, 1);
  EXPECT_TRUE(trajectory_connectivity.TransitivelyConnected(0, 1));
  trajectory_connectivity.Connect(8, 9);
  EXPECT_TRUE(trajectory_connectivity.TransitivelyConnected(8, 9));
  EXPECT_FALSE(trajectory_connectivity.TransitivelyConnected(0, 9));

  trajectory_connectivity.Connect(1, 8);
  for (int i : {0, 1}) {
    for (int j : {8, 9}) {
      EXPECT_TRUE(trajectory_connectivity.TransitivelyConnected(i, j));
    }
  }
}

TEST(TrajectoryConnectivityTest, EmptyConnectedComponents) {
  TrajectoryConnectivity trajectory_connectivity;
  auto connections = trajectory_connectivity.ConnectedComponents();
  EXPECT_EQ(0, connections.size());
}

TEST(TrajectoryConnectivityTest, ConnectedComponents) {
  TrajectoryConnectivity trajectory_connectivity;
  for (int i = 0; i <= 4; ++i) {
    trajectory_connectivity.Connect(0, i);
  }
  for (int i = 5; i <= 9; ++i) {
    trajectory_connectivity.Connect(5, i);
  }
  auto connections = trajectory_connectivity.ConnectedComponents();
  ASSERT_EQ(2, connections.size());
  // The clustering is arbitrary; we need to figure out which one is which.
  const std::vector<int>* zero_cluster = nullptr;
  const std::vector<int>* five_cluster = nullptr;
  if (std::find(connections[0].begin(), connections[0].end(), 0) !=
      connections[0].end()) {
    zero_cluster = &connections[0];
    five_cluster = &connections[1];
  } else {
    zero_cluster = &connections[1];
    five_cluster = &connections[0];
  }
  for (int i = 0; i <= 9; ++i) {
    EXPECT_EQ(i <= 4, std::find(zero_cluster->begin(), zero_cluster->end(),
                                i) != zero_cluster->end());
    EXPECT_EQ(i > 4, std::find(five_cluster->begin(), five_cluster->end(), i) !=
                         five_cluster->end());
  }
}

TEST(TrajectoryConnectivityTest, ConnectionCount) {
  TrajectoryConnectivity trajectory_connectivity;
  for (int i = 0; i < kNumTrajectories; ++i) {
    trajectory_connectivity.Connect(0, 1);
    // Permute the arguments to check invariance.
    EXPECT_EQ(i + 1, trajectory_connectivity.ConnectionCount(1, 0));
  }
  for (int i = 1; i < 9; ++i) {
    EXPECT_EQ(0, trajectory_connectivity.ConnectionCount(i, i + 1));
  }
}

TEST(TrajectoryConnectivityTest, ReflexiveConnectivity) {
  TrajectoryConnectivity trajectory_connectivity;
  EXPECT_TRUE(trajectory_connectivity.TransitivelyConnected(0, 0));
  EXPECT_EQ(0, trajectory_connectivity.ConnectionCount(0, 0));
  trajectory_connectivity.Add(0);
  EXPECT_TRUE(trajectory_connectivity.TransitivelyConnected(0, 0));
  EXPECT_EQ(0, trajectory_connectivity.ConnectionCount(0, 0));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
