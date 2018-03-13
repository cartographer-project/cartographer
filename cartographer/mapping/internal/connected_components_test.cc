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

#include "cartographer/mapping/internal/connected_components.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr int kNumTrajectories = 10;

TEST(ConnectedComponentsTest, TransitivelyConnected) {
  ConnectedComponents connected_components;

  // Make sure nothing's connected until we connect some things.
  for (int trajectory_a = 0; trajectory_a < kNumTrajectories; ++trajectory_a) {
    for (int trajectory_b = 0; trajectory_b < kNumTrajectories;
         ++trajectory_b) {
      EXPECT_EQ(trajectory_a == trajectory_b,
                connected_components.TransitivelyConnected(trajectory_a,
                                                           trajectory_b));
    }
  }

  // Connect some stuff up.
  connected_components.Connect(0, 1);
  EXPECT_TRUE(connected_components.TransitivelyConnected(0, 1));
  connected_components.Connect(8, 9);
  EXPECT_TRUE(connected_components.TransitivelyConnected(8, 9));
  EXPECT_FALSE(connected_components.TransitivelyConnected(0, 9));

  connected_components.Connect(1, 8);
  for (int i : {0, 1}) {
    for (int j : {8, 9}) {
      EXPECT_TRUE(connected_components.TransitivelyConnected(i, j));
    }
  }
}

TEST(ConnectedComponentsTest, EmptyConnectedComponents) {
  ConnectedComponents connected_components;
  auto connections = connected_components.Components();
  EXPECT_EQ(0, connections.size());
}

TEST(ConnectedComponentsTest, ConnectedComponents) {
  ConnectedComponents connected_components;
  for (int i = 0; i <= 4; ++i) {
    connected_components.Connect(0, i);
  }
  for (int i = 5; i <= 9; ++i) {
    connected_components.Connect(5, i);
  }
  auto connections = connected_components.Components();
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

TEST(ConnectedComponentsTest, ConnectionCount) {
  ConnectedComponents connected_components;
  for (int i = 0; i < kNumTrajectories; ++i) {
    connected_components.Connect(0, 1);
    // Permute the arguments to check invariance.
    EXPECT_EQ(i + 1, connected_components.ConnectionCount(1, 0));
  }
  for (int i = 1; i < 9; ++i) {
    EXPECT_EQ(0, connected_components.ConnectionCount(i, i + 1));
  }
}

TEST(ConnectedComponentsTest, ReflexiveConnectivity) {
  ConnectedComponents connected_components;
  EXPECT_TRUE(connected_components.TransitivelyConnected(0, 0));
  EXPECT_EQ(0, connected_components.ConnectionCount(0, 0));
  connected_components.Add(0);
  EXPECT_TRUE(connected_components.TransitivelyConnected(0, 0));
  EXPECT_EQ(0, connected_components.ConnectionCount(0, 0));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
