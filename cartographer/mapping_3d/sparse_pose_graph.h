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

#ifndef CARTOGRAPHER_MAPPING_3D_SPARSE_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_3D_SPARSE_POSE_GRAPH_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/trajectory_connectivity.h"
#include "cartographer/mapping_3d/sparse_pose_graph/constraint_builder.h"
#include "cartographer/mapping_3d/sparse_pose_graph/optimization_problem.h"
#include "cartographer/mapping_3d/submaps.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping_3d {

// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
//
// It is extended for submapping in 3D:
// Each scan has been matched against one or more submaps (adding a constraint
// for each match), both poses of scans and of submaps are to be optimized.
// All constraints are between a submap i and a scan j.
class SparsePoseGraph : public mapping::SparsePoseGraph {
 public:
  SparsePoseGraph(const mapping::proto::SparsePoseGraphOptions& options,
                  common::ThreadPool* thread_pool);
  ~SparsePoseGraph() override;

  SparsePoseGraph(const SparsePoseGraph&) = delete;
  SparsePoseGraph& operator=(const SparsePoseGraph&) = delete;

  // Adds a new 'range_data_in_tracking' observation at 'time', and a 'pose'
  // that will later be optimized. The 'pose' was determined by scan matching
  // against 'insertion_submaps.front()' and the scan was inserted into the
  // 'insertion_submaps'. If 'insertion_submaps.front().finished()' is 'true',
  // this submap was inserted into for the last time.
  void AddScan(
      common::Time time, const sensor::RangeData& range_data_in_tracking,
      const transform::Rigid3d& pose, int trajectory_id,
      const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
      EXCLUDES(mutex_);

  // Adds new IMU data to be used in the optimization.
  void AddImuData(int trajectory_id, const sensor::ImuData& imu_data);

  void FreezeTrajectory(int trajectory_id) override;
  void AddSubmapFromProto(int trajectory_id,
                          const transform::Rigid3d& initial_pose,
                          const mapping::proto::Submap& submap) override;
  void AddTrimmer(std::unique_ptr<mapping::PoseGraphTrimmer> trimmer) override;
  void RunFinalOptimization() override;
  std::vector<std::vector<int>> GetConnectedTrajectories() override;
  int num_submaps(int trajectory_id) EXCLUDES(mutex_) override;
  mapping::SparsePoseGraph::SubmapData GetSubmapData(
      const mapping::SubmapId& submap_id) EXCLUDES(mutex_) override;
  std::vector<std::vector<mapping::SparsePoseGraph::SubmapData>>
  GetAllSubmapData() EXCLUDES(mutex_) override;
  transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id)
      EXCLUDES(mutex_) override;
  std::vector<std::vector<mapping::TrajectoryNode>> GetTrajectoryNodes()
      override EXCLUDES(mutex_);
  std::vector<Constraint> constraints() override EXCLUDES(mutex_);

 private:
  // The current state of the submap in the background threads. When this
  // transitions to kFinished, all scans are tried to match against this submap.
  // Likewise, all new scans are matched against submaps which are finished.
  enum class SubmapState { kActive, kFinished, kTrimmed };
  struct SubmapData {
    std::shared_ptr<const Submap> submap;

    // IDs of the scans that were inserted into this map together with
    // constraints for them. They are not to be matched again when this submap
    // becomes 'finished'.
    std::set<mapping::NodeId> node_ids;

    SubmapState state = SubmapState::kActive;
  };

  // Handles a new work item.
  void AddWorkItem(const std::function<void()>& work_item) REQUIRES(mutex_);

  // Grows the optimization problem to have an entry for every element of
  // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
  std::vector<mapping::SubmapId> GrowSubmapTransformsAsNeeded(
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
      REQUIRES(mutex_);

  // Adds constraints for a scan, and starts scan matching in the background.
  void ComputeConstraintsForScan(
      int trajectory_id,
      std::vector<std::shared_ptr<const Submap>> insertion_submaps,
      bool newly_finished_submap, const transform::Rigid3d& pose)
      REQUIRES(mutex_);

  // Computes constraints for a scan and submap pair.
  void ComputeConstraint(const mapping::NodeId& node_id,
                         const mapping::SubmapId& submap_id) REQUIRES(mutex_);

  // Adds constraints for older scans whenever a new submap is finished.
  void ComputeConstraintsForOldScans(const mapping::SubmapId& submap_id)
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

  // Computes the local to global frame transform based on the given optimized
  // 'submap_transforms'.
  transform::Rigid3d ComputeLocalToGlobalTransform(
      const std::vector<std::vector<sparse_pose_graph::SubmapData>>&
          submap_transforms,
      int trajectory_id) const REQUIRES(mutex_);

  mapping::SparsePoseGraph::SubmapData GetSubmapDataUnderLock(
      const mapping::SubmapId& submap_id) REQUIRES(mutex_);

  const mapping::proto::SparsePoseGraphOptions options_;
  common::Mutex mutex_;

  // If it exists, further scans must be added to this queue, and will be
  // considered later.
  std::unique_ptr<std::deque<std::function<void()>>> scan_queue_
      GUARDED_BY(mutex_);

  // How our various trajectories are related.
  mapping::TrajectoryConnectivity trajectory_connectivity_ GUARDED_BY(mutex_);

  // We globally localize a fraction of the scans from each trajectory.
  std::unordered_map<int, std::unique_ptr<common::FixedRatioSampler>>
      global_localization_samplers_ GUARDED_BY(mutex_);

  // Number of scans added since last loop closure.
  int num_scans_since_last_loop_closure_ GUARDED_BY(mutex_) = 0;

  // Whether the optimization has to be run before more data is added.
  bool run_loop_closure_ GUARDED_BY(mutex_) = false;

  // Current optimization problem.
  sparse_pose_graph::OptimizationProblem optimization_problem_;
  sparse_pose_graph::ConstraintBuilder constraint_builder_ GUARDED_BY(mutex_);
  std::vector<Constraint> constraints_ GUARDED_BY(mutex_);

  // Submaps get assigned an ID and state as soon as they are seen, even
  // before they take part in the background computations.
  mapping::NestedVectorsById<SubmapData, mapping::SubmapId> submap_data_
      GUARDED_BY(mutex_);

  // Connectivity structure of our trajectories by IDs.
  std::vector<std::vector<int>> connected_components_;
  // Trajectory ID to connected component ID.
  std::map<int, size_t> reverse_connected_components_;

  // Data that are currently being shown.
  mapping::NestedVectorsById<mapping::TrajectoryNode, mapping::NodeId>
      trajectory_nodes_ GUARDED_BY(mutex_);
  int num_trajectory_nodes_ GUARDED_BY(mutex_) = 0;

  // Current submap transforms used for displaying data.
  std::vector<std::vector<sparse_pose_graph::SubmapData>>
      optimized_submap_transforms_ GUARDED_BY(mutex_);

  // List of all trimmers to consult when optimizations finish.
  std::vector<std::unique_ptr<mapping::PoseGraphTrimmer>> trimmers_
      GUARDED_BY(mutex_);

  // Set of all frozen trajectories not being optimized.
  std::set<int> frozen_trajectories_ GUARDED_BY(mutex_);

  // Allows querying and manipulating the pose graph by the 'trimmers_'. The
  // 'mutex_' of the pose graph is held while this class is used.
  class TrimmingHandle : public mapping::Trimmable {
   public:
    TrimmingHandle(SparsePoseGraph* parent);
    ~TrimmingHandle() override {}

    int num_submaps(int trajectory_id) const override;
    void MarkSubmapAsTrimmed(const mapping::SubmapId& submap_id) override;

   private:
    SparsePoseGraph* const parent_;
  };
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SPARSE_POSE_GRAPH_H_
