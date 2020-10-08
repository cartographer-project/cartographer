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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_3D_H_

#include <array>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/common/task.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/fast_correlative_scan_matcher_3d.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/pose_graph/constraint_builder_options.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping {
namespace constraints {

// Asynchronously computes constraints.
//
// Intermingle an arbitrary number of calls to 'MaybeAddConstraint',
// 'MaybeAddGlobalConstraint', and 'NotifyEndOfNode', then call 'WhenDone' once.
// After all computations are done the 'callback' will be called with the result
// and another MaybeAdd(Global)Constraint()/WhenDone() cycle can follow.
//
// This class is thread-safe.
class ConstraintBuilder3D {
 public:
  using Constraint = mapping::PoseGraphInterface::Constraint;
  using Result = std::vector<Constraint>;

  ConstraintBuilder3D(const proto::ConstraintBuilderOptions& options,
                      common::ThreadPoolInterface* thread_pool);
  ~ConstraintBuilder3D();

  ConstraintBuilder3D(const ConstraintBuilder3D&) = delete;
  ConstraintBuilder3D& operator=(const ConstraintBuilder3D&) = delete;

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id', and the 'compressed_point_cloud' for 'node_id'.
  //
  // 'global_node_pose' and 'global_submap_pose' are initial estimates of poses
  // in the global map frame, i.e. both are gravity aligned.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  void MaybeAddConstraint(const SubmapId& submap_id, const Submap3D* submap,
                          const NodeId& node_id,
                          const TrajectoryNode::Data* const constant_data,
                          const transform::Rigid3d& global_node_pose,
                          const transform::Rigid3d& global_submap_pose);

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id' and the 'compressed_point_cloud' for 'node_id'.
  // This performs full-submap matching.
  //
  // 'global_node_rotation' and 'global_submap_rotation' are initial estimates
  // of roll and pitch, i.e. their yaw is essentially ignored.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  void MaybeAddGlobalConstraint(
      const SubmapId& submap_id, const Submap3D* submap, const NodeId& node_id,
      const TrajectoryNode::Data* const constant_data,
      const Eigen::Quaterniond& global_node_rotation,
      const Eigen::Quaterniond& global_submap_rotation);

  // Must be called after all computations related to one node have been added.
  void NotifyEndOfNode();

  // Registers the 'callback' to be called with the results, after all
  // computations triggered by 'MaybeAdd*Constraint' have finished.
  // 'callback' is executed in the 'ThreadPool'.
  void WhenDone(const std::function<void(const Result&)>& callback);

  // Returns the number of consecutive finished nodes.
  int GetNumFinishedNodes();

  // Delete data related to 'submap_id'.
  void DeleteScanMatcher(const SubmapId& submap_id);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  struct SubmapScanMatcher {
    const HybridGrid* high_resolution_hybrid_grid = nullptr;
    const HybridGrid* low_resolution_hybrid_grid = nullptr;
    std::unique_ptr<scan_matching::FastCorrelativeScanMatcher3D>
        fast_correlative_scan_matcher;
    std::weak_ptr<common::Task> creation_task_handle;
  };

  // The returned 'grid' and 'fast_correlative_scan_matcher' must only be
  // accessed after 'creation_task_handle' has completed.
  const SubmapScanMatcher* DispatchScanMatcherConstruction(
      const SubmapId& submap_id, const Submap3D* submap)
      EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Runs in a background thread and does computations for an additional
  // constraint.
  // As output, it may create a new Constraint in 'constraint'.
  void ComputeConstraint(const SubmapId& submap_id, const NodeId& node_id,
                         bool match_full_submap,
                         const TrajectoryNode::Data* const constant_data,
                         const transform::Rigid3d& global_node_pose,
                         const transform::Rigid3d& global_submap_pose,
                         const SubmapScanMatcher& submap_scan_matcher,
                         std::unique_ptr<Constraint>* constraint)
      LOCKS_EXCLUDED(mutex_);

  void RunWhenDoneCallback() LOCKS_EXCLUDED(mutex_);

  const proto::ConstraintBuilderOptions options_;
  common::ThreadPoolInterface* thread_pool_;
  absl::Mutex mutex_;

  // 'callback' set by WhenDone().
  std::unique_ptr<std::function<void(const Result&)>> when_done_
      GUARDED_BY(mutex_);

  // TODO(gaschler): Use atomics instead of mutex to access these counters.
  // Number of the node in reaction to which computations are currently
  // added. This is always the number of nodes seen so far, even when older
  // nodes are matched against a new submap.
  int num_started_nodes_ GUARDED_BY(mutex_) = 0;

  int num_finished_nodes_ GUARDED_BY(mutex_) = 0;

  std::unique_ptr<common::Task> finish_node_task_ GUARDED_BY(mutex_);

  std::unique_ptr<common::Task> when_done_task_ GUARDED_BY(mutex_);

  // Constraints currently being computed in the background. A deque is used to
  // keep pointers valid when adding more entries. Constraint search results
  // with below-threshold scores are also 'nullptr'.
  std::deque<std::unique_ptr<Constraint>> constraints_ GUARDED_BY(mutex_);

  // Map of dispatched or constructed scan matchers by 'submap_id'.
  std::map<SubmapId, SubmapScanMatcher> submap_scan_matchers_
      GUARDED_BY(mutex_);
  std::map<SubmapId, common::FixedRatioSampler> per_submap_sampler_;

  scan_matching::CeresScanMatcher3D ceres_scan_matcher_;

  // Histograms of scan matcher scores.
  common::Histogram score_histogram_ GUARDED_BY(mutex_);
  common::Histogram rotational_score_histogram_ GUARDED_BY(mutex_);
  common::Histogram low_resolution_score_histogram_ GUARDED_BY(mutex_);
};

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_3D_H_
