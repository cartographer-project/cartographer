/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_TESTING_FAKE_TRIMMABLE_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_TESTING_FAKE_TRIMMABLE_H_

#include <vector>

#include "cartographer/mapping/pose_graph_trimmer.h"

namespace cartographer {
namespace mapping {
namespace testing {

class FakeTrimmable : public Trimmable {
 public:
  FakeTrimmable(int trajectory_id, int num_submaps) {
    for (int index = 0; index < num_submaps; ++index) {
      submaps_.push_back(SubmapId{trajectory_id, index});
    }
  }
  ~FakeTrimmable() override {}

  int num_submaps(const int trajectory_id) const override {
    return submaps_.size() - trimmed_submaps_.size();
  }

  std::vector<SubmapId> GetSubmapIds(int trajectory_id) const override {
    return submaps_;
  }

  MapById<SubmapId, PoseGraphInterface::SubmapData> GetAllSubmapData()
      const override {
    return {};
  }

  MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const override {
    return {};
  }

  std::vector<PoseGraphInterface::Constraint> GetConstraints() const override {
    return {};
  }

  void MarkSubmapAsTrimmed(const SubmapId& submap_id) override {
    trimmed_submaps_.push_back(submap_id);
  }

  bool IsFinished(const int trajectory_id) const override { return false; }

  std::vector<SubmapId> trimmed_submaps() { return trimmed_submaps_; }

 private:
  std::vector<SubmapId> submaps_;
  std::vector<SubmapId> trimmed_submaps_;
};

}  // namespace testing
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_TESTING_FAKE_TRIMMABLE_H_
