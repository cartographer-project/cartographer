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

#include "cartographer/mapping/trajectory_connectivity_state.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {

TEST(TrajectoryConnectivityStateTest, UnknownTrajectory) {
  TrajectoryConnectivityState state;
  state.Add(0);

  // Nothing is transitively connected to an unknown trajectory.
  EXPECT_FALSE(state.TransitivelyConnected(0, 1));
  EXPECT_EQ(state.LastConnectionTime(0, 1), common::Time());
}

TEST(TrajectoryConnectivityStateTest, NotConnected) {
  TrajectoryConnectivityState state;
  state.Add(0);
  state.Add(1);
  EXPECT_FALSE(state.TransitivelyConnected(0, 1));
  EXPECT_EQ(state.LastConnectionTime(0, 1), common::Time());
}

TEST(TrajectoryConnectivityStateTest, Connected) {
  TrajectoryConnectivityState state;
  state.Add(0);
  state.Add(1);
  state.Connect(0, 1, common::FromUniversal(123456));
  EXPECT_TRUE(state.TransitivelyConnected(0, 1));
  EXPECT_EQ(state.LastConnectionTime(0, 1), common::FromUniversal(123456));
}

TEST(TrajectoryConnectivityStateTest, UpdateConnectionTime) {
  TrajectoryConnectivityState state;
  state.Add(0);
  state.Add(1);
  state.Connect(0, 1, common::FromUniversal(123456));
  state.Connect(0, 1, common::FromUniversal(234567));
  EXPECT_TRUE(state.TransitivelyConnected(0, 1));
  EXPECT_EQ(state.LastConnectionTime(0, 1), common::FromUniversal(234567));

  // Connections with an earlier connection time do not update the last
  // connection time.
  state.Connect(0, 1, common::FromUniversal(123456));
  EXPECT_EQ(state.LastConnectionTime(0, 1), common::FromUniversal(234567));
}

TEST(TrajectoryConnectivityStateTest, JoinTwoComponents) {
  TrajectoryConnectivityState state;
  state.Add(0);
  state.Add(1);
  state.Add(2);
  state.Add(3);
  state.Connect(0, 1, common::FromUniversal(123456));
  state.Connect(2, 3, common::FromUniversal(123456));

  // Connect the two disjoint connected components.
  state.Connect(0, 2, common::FromUniversal(234567));
  EXPECT_TRUE(state.TransitivelyConnected(0, 2));
  EXPECT_TRUE(state.TransitivelyConnected(1, 3));

  // All bipartite trajectory pairs between the two connected components should
  // have the updated connection time.
  EXPECT_EQ(state.LastConnectionTime(0, 2), common::FromUniversal(234567));
  EXPECT_EQ(state.LastConnectionTime(0, 3), common::FromUniversal(234567));
  EXPECT_EQ(state.LastConnectionTime(1, 3), common::FromUniversal(234567));

  // A pair of trajectory IDs belonging to the same connected component should
  // be unaffected.
  EXPECT_EQ(state.LastConnectionTime(0, 1), common::FromUniversal(123456));
}

}  // namespace mapping
}  // namespace cartographer
