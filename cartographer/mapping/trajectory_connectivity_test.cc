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

#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/mapping_2d/submaps.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

class TrajectoryConnectivityTest : public ::testing::Test {
 protected:
  TrajectoryConnectivityTest() {
    for (int i = 0; i < 10; ++i) {
      auto parameter_dictionary = common::MakeDictionary(R"text(
          return {
            resolution = 0.05,
            half_length = 10.,
            num_laser_fans = 10,
            output_debug_images = false,
            laser_fan_inserter = {
              insert_free_space = true,
              hit_probability = 0.53,
              miss_probability = 0.495,
            },
          })text");
      trajectories_.emplace_back(new mapping_2d::Submaps(
          mapping_2d::CreateSubmapsOptions(parameter_dictionary.get())));
    }
  }

  // Helper function to avoid .get() noise.
  const Submaps* trajectory(int index) { return trajectories_.at(index).get(); }

  TrajectoryConnectivity trajectory_connectivity_;
  std::vector<std::unique_ptr<const Submaps>> trajectories_;
};

TEST_F(TrajectoryConnectivityTest, TransitivelyConnected) {
  // Make sure nothing's connected until we connect some things.
  for (auto& trajectory_a : trajectories_) {
    for (auto& trajectory_b : trajectories_) {
      EXPECT_FALSE(trajectory_connectivity_.TransitivelyConnected(
          trajectory_a.get(), trajectory_b.get()));
    }
  }

  // Connect some stuff up.
  trajectory_connectivity_.Connect(trajectory(0), trajectory(1));
  EXPECT_TRUE(trajectory_connectivity_.TransitivelyConnected(trajectory(0),
                                                             trajectory(1)));
  trajectory_connectivity_.Connect(trajectory(8), trajectory(9));
  EXPECT_TRUE(trajectory_connectivity_.TransitivelyConnected(trajectory(8),
                                                             trajectory(9)));
  EXPECT_FALSE(trajectory_connectivity_.TransitivelyConnected(trajectory(0),
                                                              trajectory(9)));

  trajectory_connectivity_.Connect(trajectory(1), trajectory(8));
  for (int i : {0, 1}) {
    for (int j : {8, 9}) {
      EXPECT_TRUE(trajectory_connectivity_.TransitivelyConnected(
          trajectory(i), trajectory(j)));
    }
  }
}

TEST_F(TrajectoryConnectivityTest, EmptyConnectedComponents) {
  auto connections = trajectory_connectivity_.ConnectedComponents();
  EXPECT_EQ(0, connections.size());
}

TEST_F(TrajectoryConnectivityTest, ConnectedComponents) {
  for (int i = 0; i <= 4; ++i) {
    trajectory_connectivity_.Connect(trajectory(0), trajectory(i));
  }
  for (int i = 5; i <= 9; ++i) {
    trajectory_connectivity_.Connect(trajectory(5), trajectory(i));
  }
  auto connections = trajectory_connectivity_.ConnectedComponents();
  ASSERT_EQ(2, connections.size());
  // The clustering is arbitrary; we need to figure out which one is which.
  const std::vector<const Submaps*>* zero_cluster = nullptr;
  const std::vector<const Submaps*>* five_cluster = nullptr;
  if (std::find(connections[0].begin(), connections[0].end(), trajectory(0)) !=
      connections[0].end()) {
    zero_cluster = &connections[0];
    five_cluster = &connections[1];
  } else {
    zero_cluster = &connections[1];
    five_cluster = &connections[0];
  }
  for (int i = 0; i <= 9; ++i) {
    EXPECT_EQ(i <= 4, std::find(zero_cluster->begin(), zero_cluster->end(),
                                trajectory(i)) != zero_cluster->end());
    EXPECT_EQ(i > 4, std::find(five_cluster->begin(), five_cluster->end(),
                               trajectory(i)) != five_cluster->end());
  }
}

TEST_F(TrajectoryConnectivityTest, ConnectionCount) {
  for (int i = 0; i < 10; ++i) {
    trajectory_connectivity_.Connect(trajectory(0), trajectory(1));
    // Permute the arguments to check invariance.
    EXPECT_EQ(i + 1, trajectory_connectivity_.ConnectionCount(trajectory(1),
                                                              trajectory(0)));
  }
  for (int i = 1; i < 9; ++i) {
    EXPECT_EQ(0, trajectory_connectivity_.ConnectionCount(trajectory(i),
                                                          trajectory(i + 1)));
  }
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
