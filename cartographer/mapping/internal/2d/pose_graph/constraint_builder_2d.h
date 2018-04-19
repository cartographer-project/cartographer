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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_CONSTRAINT_BUILDER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_CONSTRAINT_BUILDER_2D_H_

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
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/pose_graph/proto/constraint_builder_options.pb.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping {
namespace pose_graph {

// Returns (map <- submap) where 'submap' is a coordinate system at the origin
// of the Submap.
transform::Rigid2d ComputeSubmapPose(const Submap2D& submap);

// Asynchronously computes constraints.
//
// Intermingle an arbitrary number of calls to MaybeAddConstraint() or
// MaybeAddGlobalConstraint, then call WhenDone(). After all computations are
// done the 'callback' will be called with the result and another
// MaybeAdd(Global)Constraint()/WhenDone() cycle can follow.
//
// This class is thread-safe.
class ConstraintBuilder2D {
 public:
  using Constraint = PoseGraphInterface::Constraint;
  using Result = std::vector<Constraint>;

  ConstraintBuilder2D(const proto::ConstraintBuilderOptions& options,
                      common::ThreadPool* thread_pool);
  ~ConstraintBuilder2D();

  ConstraintBuilder2D(const ConstraintBuilder2D&) = delete;
  ConstraintBuilder2D& operator=(const ConstraintBuilder2D&) = delete;

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id', and the 'compressed_point_cloud' for 'node_id'. The
  // 'initial_relative_pose' is relative to the 'submap'.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  void MaybeAddConstraint(const SubmapId& submap_id, const Submap2D* submap,
                          const NodeId& node_id,
                          const TrajectoryNode::Data* const constant_data,
                          const transform::Rigid2d& initial_relative_pose);

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id' and the 'compressed_point_cloud' for 'node_id'.
  // This performs full-submap matching.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  void MaybeAddGlobalConstraint(
      const SubmapId& submap_id, const Submap2D* submap, const NodeId& node_id,
      const TrajectoryNode::Data* const constant_data);

  // Must be called after all computations related to one node have been added.
  void NotifyEndOfNode();

  // Registers the 'callback' to be called with the results, after all
  // computations triggered by MaybeAddConstraint() have finished.
  void WhenDone(const std::function<void(const Result&)>& callback);

  // Returns the number of consecutive finished nodes.
  int GetNumFinishedNodes();

  // Delete data related to 'submap_id'.
  void DeleteScanMatcher(const SubmapId& submap_id);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  struct SubmapScanMatcher {
    const Grid2D* grid;
    std::unique_ptr<scan_matching::FastCorrelativeScanMatcher2D>
        fast_correlative_scan_matcher;
  };

  // Either schedules the 'work_item', or if needed, schedules the scan matcher
  // construction and queues the 'work_item'.
  void ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
      const SubmapId& submap_id, const Grid2D* grid,
      const std::function<void()>& work_item) REQUIRES(mutex_);

  // Constructs the scan matcher for a 'submap', then schedules its work items.
  void ConstructSubmapScanMatcher(const SubmapId& submap_id, const Grid2D* grid)
      EXCLUDES(mutex_);

  // Returns the scan matcher for a submap, which has to exist.
  const SubmapScanMatcher* GetSubmapScanMatcher(const SubmapId& submap_id)
      EXCLUDES(mutex_);

  // Runs in a background thread and does computations for an additional
  // constraint, assuming 'submap' and 'compressed_point_cloud' do not change
  // anymore. As output, it may create a new Constraint in 'constraint'.
  void ComputeConstraint(const SubmapId& submap_id, const Submap2D* submap,
                         const NodeId& node_id, bool match_full_submap,
                         const TrajectoryNode::Data* const constant_data,
                         const transform::Rigid2d& initial_relative_pose,
                         std::unique_ptr<Constraint>* constraint)
      EXCLUDES(mutex_);

  // Decrements the 'pending_computations_' count. If all computations are done,
  // runs the 'when_done_' callback and resets the state.
  void FinishComputation(int computation_index) EXCLUDES(mutex_);

  const pose_graph::proto::ConstraintBuilderOptions options_;
  common::ThreadPool* thread_pool_;
  common::Mutex mutex_;

  // 'callback' set by WhenDone().
  std::unique_ptr<std::function<void(const Result&)>> when_done_
      GUARDED_BY(mutex_);

  // Index of the node in reaction to which computations are currently
  // added. This is always the highest node index seen so far, even when older
  // nodes are matched against a new submap.
  int current_computation_ GUARDED_BY(mutex_) = 0;

  // For each added node, maps to the number of pending computations that were
  // added for it.
  std::map<int, int> pending_computations_ GUARDED_BY(mutex_);

  // Constraints currently being computed in the background. A deque is used to
  // keep pointers valid when adding more entries.
  std::deque<std::unique_ptr<Constraint>> constraints_ GUARDED_BY(mutex_);

  // Map of already constructed scan matchers by 'submap_id'.
  std::map<SubmapId, SubmapScanMatcher> submap_scan_matchers_
      GUARDED_BY(mutex_);

  // Map by 'submap_id' of scan matchers under construction, and the work
  // to do once construction is done.
  std::map<SubmapId, std::vector<std::function<void()>>>
      submap_queued_work_items_ GUARDED_BY(mutex_);

  common::FixedRatioSampler sampler_;
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;

  // Histogram of scan matcher scores.
  common::Histogram score_histogram_ GUARDED_BY(mutex_);
};

}  // namespace pose_graph
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_CONSTRAINT_BUILDER_2D_H_
