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

#include "cartographer/mapping/sparse_pose_graph.h"

#include <algorithm>
#include <deque>
#include <vector>

#include "glog/logging.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace {

class FakeSubmaps : public Submaps {
 public:
  ~FakeSubmaps() override {}
  const Submap* Get(int) const override { LOG(FATAL) << "Not implemented."; }

  int size() const override { LOG(FATAL) << "Not implemented."; }

  void SubmapToProto(int, const transform::Rigid3d&,
                     proto::SubmapQuery::Response*) const override {
    LOG(FATAL) << "Not implemented.";
  }
};

TEST(SparsePoseGraphTest, TrajectoryFunctions) {
  std::vector<TrajectoryNode> trajectory_nodes;
  const std::vector<FakeSubmaps> submaps(5);
  std::deque<TrajectoryNode::ConstantData> constant_data;
  constexpr int kNumTrajectoryNodes = 10;
  for (int j = 0; j < kNumTrajectoryNodes; ++j) {
    for (size_t i = 0; i < submaps.size(); ++i) {
      constant_data.push_back({});
      constant_data.back().trajectory = &submaps[i];
      TrajectoryNode node;
      node.constant_data = &constant_data.back();
      trajectory_nodes.push_back(node);
    }
  }

  const std::unordered_map<const Submaps*, int> index_map = {{&submaps[0], 0},
                                                             {&submaps[1], 1},
                                                             {&submaps[2], 2},
                                                             {&submaps[3], 3},
                                                             {&submaps[4], 4}};
  std::vector<std::vector<TrajectoryNode>> grouped_nodes;
  std::vector<std::pair<int, int>> new_indices;
  GroupTrajectoryNodes(trajectory_nodes, index_map, &grouped_nodes,
                       &new_indices);

  ASSERT_EQ(grouped_nodes.size(), submaps.size());
  for (size_t i = 0; i < submaps.size(); ++i) {
    EXPECT_EQ(grouped_nodes[i].size(), kNumTrajectoryNodes);
    for (const auto& node : grouped_nodes[i]) {
      EXPECT_EQ(node.constant_data->trajectory, &submaps[i]);
    }
  }

  ASSERT_EQ(trajectory_nodes.size(), new_indices.size());
  for (size_t i = 0; i < new_indices.size(); ++i) {
    const auto index_pair = new_indices[i];
    EXPECT_EQ(trajectory_nodes[i].constant_data->trajectory,
              grouped_nodes[index_pair.first][index_pair.second]
                  .constant_data->trajectory);
  }
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
