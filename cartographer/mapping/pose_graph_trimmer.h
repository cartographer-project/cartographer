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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_TRIMMER_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_TRIMMER_H_

#include "cartographer/mapping/id.h"

namespace cartographer {
namespace mapping {

// Implemented by the pose graph to provide thread-safe access to functions for
// trimming the graph.
class Trimmable {
 public:
  virtual ~Trimmable() {}

  // TODO(whess): This is all the functionality necessary for pure localization.
  // To be expanded as needed for lifelong mapping.
  virtual int num_submaps(int trajectory_id) const = 0;

  // Marks 'submap_id' and corresponding intra-submap nodes as trimmed. They
  // will no longer take part in scan matching, loop closure, visualization.
  // Submaps and nodes are only marked, the numbering remains unchanged.
  virtual void MarkSubmapAsTrimmed(const SubmapId& submap_id) = 0;
};

// An interface to implement algorithms that choose how to trim the pose graph.
class PoseGraphTrimmer {
 public:
  virtual ~PoseGraphTrimmer() {}

  // Called once after each pose graph optimization.
  virtual void Trim(Trimmable* pose_graph) = 0;
};

// Keeps the last 'num_submaps_to_keep' of the trajectory with 'trajectory_id'
// to implement localization without mapping.
class PureLocalizationTrimmer : public PoseGraphTrimmer {
 public:
  PureLocalizationTrimmer(int trajectory_id, int num_submaps_to_keep);
  ~PureLocalizationTrimmer() override {}

  void Trim(Trimmable* pose_graph) override;

 private:
  const int trajectory_id_;
  const int num_submaps_to_keep_;
  int num_submaps_trimmed_ = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_TRIMMER_H_
