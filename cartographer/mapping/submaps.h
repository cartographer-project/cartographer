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

// Submaps is a sequence of maps to which scans are matched and into which scans
// are inserted.
//
// Except during initialization when only a single submap exists, there are
// always two submaps into which scans are inserted: an old submap that is used
// for matching, and a new one, which will be used for matching next, that is
// being initialized.
//
// Once a certain number of scans have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover,
// a "new" submap gets inserted.
#ifndef CARTOGRAPHER_MAPPING_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_SUBMAPS_H_

#include <memory>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Converts the given probability to log odds.
inline float Logit(float probability) {
  return std::log(probability / (1.f - probability));
}

const float kMaxLogOdds = Logit(kMaxProbability);
const float kMinLogOdds = Logit(kMinProbability);

// Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
// kMaxLogOdds] is mapped to [1, 255].
inline uint8 ProbabilityToLogOddsInteger(const float probability) {
  const int value = common::RoundToInt((Logit(probability) - kMinLogOdds) *
                                       254.f / (kMaxLogOdds - kMinLogOdds)) +
                    1;
  CHECK_LE(1, value);
  CHECK_GE(255, value);
  return value;
}

// An individual submap, which has an initial position 'origin', keeps track of
// which range data were inserted into it, and sets the
// 'finished_probability_grid' to be used for loop closing once the map no
// longer changes.
struct Submap {
  Submap(const Eigen::Vector3f& origin, int begin_range_data_index)
      : origin(origin),
        begin_range_data_index(begin_range_data_index),
        end_range_data_index(begin_range_data_index) {}

  transform::Rigid3d local_pose() const {
    return transform::Rigid3d::Translation(origin.cast<double>());
  }

  // Origin of this submap.
  Eigen::Vector3f origin;

  // This Submap contains RangeData with indices in the range
  // ['begin_range_data_index', 'end_range_data_index').
  int begin_range_data_index;
  int end_range_data_index;

  // The 'finished_probability_grid' when this submap is finished and will not
  // change anymore. Otherwise, this is nullptr and the next call to
  // InsertRangeData() will change the submap.
  const mapping_2d::ProbabilityGrid* finished_probability_grid = nullptr;
};

// A container of Submaps.
class Submaps {
 public:
  static constexpr uint8 kUnknownLogOdds = 0;

  Submaps();
  virtual ~Submaps();

  Submaps(const Submaps&) = delete;
  Submaps& operator=(const Submaps&) = delete;

  // Returns the index of the newest initialized Submap which can be
  // used for scan-to-map matching.
  int matching_index() const;

  // Returns the indices of the Submap into which point clouds will
  // be inserted.
  std::vector<int> insertion_indices() const;

  // Returns the Submap with the given 'index'. The same 'index' will always
  // return the same pointer, so that Submaps can be identified by it.
  virtual const Submap* Get(int index) const = 0;

  // Returns the number of Submaps.
  virtual int size() const = 0;

  // Fills data about the Submap with 'index' into the 'response'.
  virtual void SubmapToProto(
      int index, const std::vector<mapping::TrajectoryNode>& trajectory_nodes,
      const transform::Rigid3d& global_submap_pose,
      proto::SubmapQuery::Response* response) const = 0;

 protected:
  static void AddProbabilityGridToResponse(
      const transform::Rigid3d& local_submap_pose,
      const mapping_2d::ProbabilityGrid& probability_grid,
      proto::SubmapQuery::Response* response);
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SUBMAPS_H_
