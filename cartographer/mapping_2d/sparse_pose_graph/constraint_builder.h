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

#ifndef CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_
#define CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_

#include <array>
#include <deque>
#include <functional>
#include <limits>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/math.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/sparse_pose_graph/proto/constraint_builder_options.pb.h"
#include "cartographer/mapping/trajectory_connectivity.h"
#include "cartographer/mapping_2d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_2d/scan_matching/fast_correlative_scan_matcher.h"
#include "cartographer/mapping_2d/sparse_pose_graph/optimization_problem.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_3d/scan_matching/fast_correlative_scan_matcher.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/voxel_filter.h"

namespace cartographer {
namespace mapping_2d {
namespace sparse_pose_graph {

// Returns (map <- submap) where 'submap' is a coordinate system at the origin
// of the Submap.
transform::Rigid2d ComputeSubmapPose(const mapping::Submap& submap);

// Asynchronously computes constraints.
//
// Intermingle an arbitrary number of calls to MaybeAddConstraint() or
// MaybeAddGlobalConstraint, then call WhenDone(). After all computations are
// done the 'callback' will be called with the result and another
// MaybeAdd(Global)Constraint()/WhenDone() cycle can follow.
//
// This class is thread-safe.
class ConstraintBuilder {
 public:
  using Constraint = mapping::SparsePoseGraph::Constraint2D;
  using Result = std::vector<Constraint>;

  ConstraintBuilder(
      const mapping::sparse_pose_graph::proto::ConstraintBuilderOptions&
          options,
      common::ThreadPool* thread_pool);
  ~ConstraintBuilder();

  ConstraintBuilder(const ConstraintBuilder&) = delete;
  ConstraintBuilder& operator=(const ConstraintBuilder&) = delete;

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_index', and the 'point_cloud' for 'scan_index'.
  // The 'initial_pose' is relative to the 'submap'.
  //
  // The pointees of 'submap' and 'point_cloud' must stay valid until all
  // computations are finished.
  void MaybeAddConstraint(int submap_index, const mapping::Submap* submap,
                          int scan_index,
                          const sensor::PointCloud2D* point_cloud,
                          const transform::Rigid2d& initial_relative_pose);

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_index' and the 'point_cloud' for 'scan_index'. This performs
  // full-submap matching.
  //
  // The scan at 'scan_index' should be from trajectory 'scan_trajectory', and
  // the 'submap' should be from 'submap_trajectory'. The
  // 'trajectory_connectivity' is updated if the full-submap match succeeds.
  //
  // The pointees of 'submap' and 'point_cloud' must stay valid until all
  // computations are finished.
  void MaybeAddGlobalConstraint(
      int submap_index, const mapping::Submap* submap, int scan_index,
      const mapping::Submaps* scan_trajectory,
      const mapping::Submaps* submap_trajectory,
      mapping::TrajectoryConnectivity* trajectory_connectivity,
      const sensor::PointCloud2D* point_cloud);

  // Must be called after all computations related to 'scan_index' are added.
  void NotifyEndOfScan(const int scan_index);

  // Registers the 'callback' to be called with the results, after all
  // computations triggered by MaybeAddConstraint() have finished.
  void WhenDone(std::function<void(const Result&)> callback);

  // Returns the number of consecutive finished scans.
  int GetNumFinishedScans();

 private:
  struct SubmapScanMatcher {
    const ProbabilityGrid* probability_grid;
    std::unique_ptr<scan_matching::FastCorrelativeScanMatcher>
        fast_correlative_scan_matcher;
  };

  // Either schedules the 'work_item', or if needed, schedules the scan matcher
  // construction and queues the 'work_item'.
  void ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
      int submap_index, const ProbabilityGrid* submap,
      std::function<void()> work_item) REQUIRES(mutex_);

  // Constructs the scan matcher for a 'submap', then schedules its work items.
  void ConstructSubmapScanMatcher(int submap_index,
                                  const ProbabilityGrid* submap)
      EXCLUDES(mutex_);

  // Returns the scan matcher for a submap, which has to exist.
  const SubmapScanMatcher* GetSubmapScanMatcher(int submap_index)
      EXCLUDES(mutex_);

  // Runs in a background thread and does computations for an additional
  // constraint, assuming 'submap' and 'point_cloud' do not change anymore.
  // If 'match_full_submap' is true, and global localization succeeds, will
  // connect 'scan_trajectory' and 'submap_trajectory' in
  // 'trajectory_connectivity'.
  // As output, it may create a new Constraint in 'constraint'.
  void ComputeConstraint(
      int submap_index, const mapping::Submap* submap, int scan_index,
      const mapping::Submaps* scan_trajectory,
      const mapping::Submaps* submap_trajectory, bool match_full_submap,
      mapping::TrajectoryConnectivity* trajectory_connectivity,
      const sensor::PointCloud2D* point_cloud,
      const transform::Rigid2d& initial_relative_pose,
      std::unique_ptr<Constraint>* constraint) EXCLUDES(mutex_);

  // Decrements the 'pending_computations_' count. If all computations are done,
  // runs the 'when_done_' callback and resets the state.
  void FinishComputation(int computation_index) EXCLUDES(mutex_);

  const mapping::sparse_pose_graph::proto::ConstraintBuilderOptions options_;
  common::ThreadPool* thread_pool_;
  common::Mutex mutex_;

  // 'callback' set by WhenDone().
  std::unique_ptr<std::function<void(const Result&)>> when_done_
      GUARDED_BY(mutex_);

  // Index of the scan in reaction to which computations are currently
  // added. This is always the highest scan index seen so far, even when older
  // scans are matched against a new submap.
  int current_computation_ GUARDED_BY(mutex_) = 0;

  // For each added scan, maps to the number of pending computations that were
  // added for it.
  std::map<int, int> pending_computations_ GUARDED_BY(mutex_);

  // Constraints currently being computed in the background. A deque is used to
  // keep pointers valid when adding more entries.
  std::deque<std::unique_ptr<Constraint>> constraints_ GUARDED_BY(mutex_);

  // Map of already constructed scan matchers by 'submap_index'.
  std::map<int, SubmapScanMatcher> submap_scan_matchers_ GUARDED_BY(mutex_);

  // Map by 'submap_index' of scan matchers under construction, and the work
  // to do once construction is done.
  std::map<int, std::vector<std::function<void()>>> submap_queued_work_items_
      GUARDED_BY(mutex_);

  common::FixedRatioSampler sampler_;
  const sensor::AdaptiveVoxelFilter adaptive_voxel_filter_;
  scan_matching::CeresScanMatcher ceres_scan_matcher_;

  // Histogram of scan matcher scores.
  common::Histogram score_histogram_ GUARDED_BY(mutex_);
};

}  // namespace sparse_pose_graph
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_
