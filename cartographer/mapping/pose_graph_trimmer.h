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
class TrimmingInterface {
 public:
  virtual ~TrimmingInterface() {}

  // TODO(whess): This is all the data necessary for pure localization. To be
  // expanded as needed for lifelong mapping.
  virtual int num_submaps(int trajectory_id) = 0;

  virtual void MarkSubmapAsTrimmed(const SubmapId& submap_id) = 0;
};

// An interface to implement algorithms that choose how to trim the pose graph.
// Useful for implementing lifelong mapping or pure localization.
class PoseGraphTrimmer {
 public:
  virtual ~PoseGraphTrimmer() {}

  // Called once after each pose graph optimization.
  virtual void Trim(TrimmingInterface* trimming) = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_TRIMMER_H_
