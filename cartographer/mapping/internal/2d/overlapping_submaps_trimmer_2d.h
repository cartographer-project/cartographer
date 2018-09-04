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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_OVERLAPPING_SUBMAPS_TRIMMER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_OVERLAPPING_SUBMAPS_TRIMMER_H_

#include "cartographer/common/port.h"
#include "cartographer/mapping/pose_graph_trimmer.h"

namespace cartographer {
namespace mapping {

// Trims submaps that have less than 'min_covered_cells_count' cells not
// overlapped by at least 'fresh_submaps_count` submaps.
class OverlappingSubmapsTrimmer2D : public PoseGraphTrimmer {
 public:
  OverlappingSubmapsTrimmer2D(uint16 fresh_submaps_count,
                              double min_covered_area,
                              uint16 min_added_submaps_count)
      : fresh_submaps_count_(fresh_submaps_count),
        min_covered_area_(min_covered_area),
        min_added_submaps_count_(min_added_submaps_count) {}
  ~OverlappingSubmapsTrimmer2D() override = default;

  void Trim(Trimmable* pose_graph) override;
  bool IsFinished() override { return finished_; }

 private:
  // Number of the most recent submaps to keep.
  const uint16 fresh_submaps_count_;
  // Minimum area of covered space to keep submap from trimming measured in m^2.
  const double min_covered_area_;
  // Number of added submaps before the trimmer is invoked.
  const uint16 min_added_submaps_count_;
  // Current finished submap count.
  uint16 current_submap_count_ = 0;

  bool finished_ = false;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_OVERLAPPING_SUBMAPS_TRIMMER_H_
