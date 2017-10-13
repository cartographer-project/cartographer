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

#include "cartographer/mapping/pose_graph_trimmer.h"

#include <vector>

#include "cartographer/mapping/id.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

class FakePoseGraph : public Trimmable {
 public:
  ~FakePoseGraph() override {}

  int num_submaps(int trajectory_id) const override {
    return 17 - trimmed_submaps_.size();
  }

  void MarkSubmapAsTrimmed(const SubmapId& submap_id) override {
    trimmed_submaps_.push_back(submap_id);
  }

  std::vector<SubmapId> trimmed_submaps() { return trimmed_submaps_; }

 private:
  std::vector<SubmapId> trimmed_submaps_;
};

TEST(PureLocalizationTrimmerTest, MarksSubmapsAsExpected) {
  const int kTrajectoryId = 42;
  PureLocalizationTrimmer trimmer(kTrajectoryId, 15);
  FakePoseGraph fake_pose_graph;
  trimmer.Trim(&fake_pose_graph);

  const auto trimmed_submaps = fake_pose_graph.trimmed_submaps();
  ASSERT_EQ(2, trimmed_submaps.size());
  EXPECT_EQ((SubmapId{kTrajectoryId, 0}), trimmed_submaps[0]);
  EXPECT_EQ((SubmapId{kTrajectoryId, 1}), trimmed_submaps[1]);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
