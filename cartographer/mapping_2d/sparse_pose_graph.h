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

// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
//
// It is extended for submapping:
// Each scan has been matched against one or more submaps (adding a constraint
// for each match), both poses of scans and of submaps are to be optimized.
// All constraints are between a submap i and a scan j.

#ifndef CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/mapping/proto/scan_matching_progress.pb.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/trajectory_connectivity.h"
#include "cartographer/mapping_2d/sparse_pose_graph/constraint_builder.h"
#include "cartographer/mapping_2d/sparse_pose_graph/optimization_problem.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping_2d {

// Implements the SPA loop closure method.
class SparsePoseGraph : public mapping::SparsePoseGraph {
 public:
  SparsePoseGraph(
      const mapping::proto::SparsePoseGraphOptions& options,
      common::ThreadPool* thread_pool,
      std::deque<mapping::TrajectoryNode::ConstantData>* constant_node_data);
  ~SparsePoseGraph() override;

  SparsePoseGraph(const SparsePoseGraph&) = delete;
  SparsePoseGraph& operator=(const SparsePoseGraph&) = delete;

  // Adds a new 'laser_fan_in_pose' observation at 'time', and a 'pose'
  // that will later be optimized. The 'tracking_to_pose' is remembered so
  // that the optimized pose can be embedded into 3D. The 'pose' was determined
  // by scan matching against the 'matching_submap' and the scan was inserted
  // into the 'insertion_submaps'.
  void AddScan(common::Time time, const transform::Rigid3d& tracking_to_pose,
               const sensor::LaserFan& laser_fan_in_pose,
               const transform::Rigid2d& pose,
               const kalman_filter::Pose2DCovariance& pose_covariance,
               const mapping::Submaps* submaps,
               const mapping::Submap* matching_submap,
               const std::vector<const mapping::Submap*>& insertion_submaps)
      EXCLUDES(mutex_);

  void RunFinalOptimization() override;
  bool HasNewOptimizedPoses() override;
  mapping::proto::ScanMatchingProgress GetScanMatchingProgress() override;

  std::vector<std::vector<const mapping::Submaps*>> GetConnectedTrajectories()
      override;
  std::vector<transform::Rigid3d> GetSubmapTransforms(
      const mapping::Submaps& submaps) EXCLUDES(mutex_) override;
  transform::Rigid3d GetLocalToGlobalTransform(const mapping::Submaps& submaps)
      EXCLUDES(mutex_) override;
  std::vector<mapping::TrajectoryNode> GetTrajectoryNodes() override
      EXCLUDES(mutex_);
  std::vector<Constraint2D> constraints_2d() override;
  std::vector<Constraint3D> constraints_3d() override;

 private:
  struct SubmapState {
    const mapping::Submap* submap = nullptr;

    // Indices of the scans that were inserted into this map together with
    // constraints for them. They are not to be matched again when this submap
    // becomes 'finished'.
    std::set<int> scan_indices;

    // Whether in the current state of the background thread this submap is
    // finished. When this transitions to true, all scans are tried to match
    // against this submap. Likewise, all new scans are matched against submaps
    // which are finished.
    bool finished = false;

    // The trajectory to which this SubmapState belongs.
    const mapping::Submaps* trajectory = nullptr;
  };

  int GetSubmapIndex(const mapping::Submap* submap) const REQUIRES(mutex_) {
    const auto iterator = submap_indices_.find(submap);
    CHECK(iterator != submap_indices_.end());
    return iterator->second;
  }

  // Grows 'submap_transforms_' to have an entry for every element of 'submaps'.
  void GrowSubmapTransformsAsNeeded(
      const std::vector<const mapping::Submap*>& submaps) REQUIRES(mutex_);

  // Adds constraints for a scan, and starts scan matching in the background.
  void ComputeConstraintsForScan(
      int scan_index, const mapping::Submaps* scan_trajectory,
      const mapping::Submap* matching_submap,
      std::vector<const mapping::Submap*> insertion_submaps,
      const mapping::Submap* finished_submap, const transform::Rigid2d& pose,
      const kalman_filter::Pose2DCovariance& covariance) REQUIRES(mutex_);

  // Adds constraints for older scans whenever a new submap is finished.
  void ComputeConstraintsForOldScans(const mapping::Submap* submap)
      REQUIRES(mutex_);

  // Registers the callback to run the optimization once all constraints have
  // been computed, that will also do all work that queue up in 'scan_queue_'.
  void HandleScanQueue() REQUIRES(mutex_);

  // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
  // all computations have finished.
  void WaitForAllComputations() EXCLUDES(mutex_);

  // Runs the optimization. Callers have to make sure, that there is only one
  // optimization being run at a time.
  void RunOptimization() EXCLUDES(mutex_);

  // Handles a new 'work_item'.
  void AddWorkItem(std::function<void()> work_item) REQUIRES(mutex_);

  // Adds extrapolated transforms, so that there are transforms for all submaps.
  std::vector<transform::Rigid3d> ExtrapolateSubmapTransforms(
      const std::vector<transform::Rigid2d>& submap_transforms,
      const mapping::Submaps& submaps) const REQUIRES(mutex_);

  const mapping::proto::SparsePoseGraphOptions options_;
  common::Mutex mutex_;

  // If it exists, further scans must be added to this queue, and will be
  // considered later.
  std::unique_ptr<std::deque<std::function<void()>>> scan_queue_
      GUARDED_BY(mutex_);

  // How our various trajectories are related.
  mapping::TrajectoryConnectivity trajectory_connectivity_ GUARDED_BY(mutex_);

  // We globally localize a fraction of the scans from each trajectory.
  std::unordered_map<const mapping::Submaps*,
                     std::unique_ptr<common::FixedRatioSampler>>
      global_localization_samplers_ GUARDED_BY(mutex_);

  // Number of scans added since last loop closure.
  int num_scans_since_last_loop_closure_ GUARDED_BY(mutex_) = 0;

  // Whether the optimization has to be run before more data is added.
  bool run_loop_closure_ GUARDED_BY(mutex_) = false;

  // Current optimization problem.
  sparse_pose_graph::OptimizationProblem optimization_problem_;
  sparse_pose_graph::ConstraintBuilder constraint_builder_ GUARDED_BY(mutex_);
  std::vector<Constraint2D> constraints_;
  std::vector<transform::Rigid2d> initial_point_cloud_poses_;
  std::vector<transform::Rigid2d> point_cloud_poses_;  // (map <- point cloud)
  std::vector<transform::Rigid2d> submap_transforms_;  // (map <- submap)

  // Submaps get assigned an index and state as soon as they are seen, even
  // before they take part in the background computations.
  std::map<const mapping::Submap*, int> submap_indices_ GUARDED_BY(mutex_);
  std::vector<SubmapState> submap_states_ GUARDED_BY(mutex_);

  // Whether to return true on the next call to HasNewOptimizedPoses().
  bool has_new_optimized_poses_ GUARDED_BY(mutex_) = false;

  // Connectivity structure of our trajectories.
  std::vector<std::vector<const mapping::Submaps*>> connected_components_;
  // Trajectory to connected component ID.
  std::map<const mapping::Submaps*, size_t> reverse_connected_components_;

  // Data that are currently being shown.
  //
  // Deque to keep references valid for the background computation when adding
  // new data.
  std::deque<mapping::TrajectoryNode::ConstantData>* constant_node_data_;
  std::vector<mapping::TrajectoryNode> trajectory_nodes_ GUARDED_BY(mutex_);

  // Current submap transforms used for displaying data.
  std::vector<transform::Rigid2d> optimized_submap_transforms_
      GUARDED_BY(mutex_);
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_H_
