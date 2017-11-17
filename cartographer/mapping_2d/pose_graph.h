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

#ifndef CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_H_

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
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/mapping/trajectory_connectivity_state.h"
#include "cartographer/mapping_2d/pose_graph/constraint_builder.h"
#include "cartographer/mapping_2d/pose_graph/optimization_problem.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping_2d {

// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
//
// It is extended for submapping:
// Each node has been matched against one or more submaps (adding a constraint
// for each match), both poses of nodes and of submaps are to be optimized.
// All constraints are between a submap i and a node j.
class PoseGraph : public mapping::PoseGraph {
 public:
  PoseGraph(const mapping::proto::PoseGraphOptions& options,
            common::ThreadPool* thread_pool);
  ~PoseGraph() override;

  PoseGraph(const PoseGraph&) = delete;
  PoseGraph& operator=(const PoseGraph&) = delete;

  // Adds a new node with 'constant_data'. Its 'constant_data->local_pose' was
  // determined by scan matching against 'insertion_submaps.front()' and the
  // node data was inserted into the 'insertion_submaps'. If
  // 'insertion_submaps.front().finished()' is 'true', data was inserted into
  // this submap for the last time.
  mapping::NodeId AddNode(
      std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data,
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
      EXCLUDES(mutex_);

  void AddImuData(int trajectory_id, const sensor::ImuData& imu_data) override
      EXCLUDES(mutex_);
  void AddOdometryData(int trajectory_id,
                       const sensor::OdometryData& odometry_data) override
      EXCLUDES(mutex_);
  void AddFixedFramePoseData(
      int trajectory_id,
      const sensor::FixedFramePoseData& fixed_frame_pose_data);

  void FinishTrajectory(int trajectory_id) override;
  void FreezeTrajectory(int trajectory_id) override;
  void AddSubmapFromProto(const transform::Rigid3d& global_submap_pose,
                          const mapping::proto::Submap& submap) override;
  void AddNodeFromProto(const transform::Rigid3d& global_pose,
                        const mapping::proto::Node& node) override;
  void AddNodeToSubmap(const mapping::NodeId& node_id,
                       const mapping::SubmapId& submap_id) override;
  void AddSerializedConstraints(
      const std::vector<Constraint>& constraints) override;
  void AddTrimmer(std::unique_ptr<mapping::PoseGraphTrimmer> trimmer) override;
  void RunFinalOptimization() override;
  std::vector<std::vector<int>> GetConnectedTrajectories() override;
  mapping::PoseGraph::SubmapData GetSubmapData(
      const mapping::SubmapId& submap_id) EXCLUDES(mutex_) override;
  mapping::MapById<mapping::SubmapId, mapping::PoseGraph::SubmapData>
  GetAllSubmapData() EXCLUDES(mutex_) override;
  transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id)
      EXCLUDES(mutex_) override;
  mapping::MapById<mapping::NodeId, mapping::TrajectoryNode>
  GetTrajectoryNodes() override EXCLUDES(mutex_);
  sensor::MapByTime<sensor::ImuData> GetImuData() override EXCLUDES(mutex_);
  sensor::MapByTime<sensor::OdometryData> GetOdometryData() override
      EXCLUDES(mutex_);
  std::vector<Constraint> constraints() override EXCLUDES(mutex_);
  void SetInitialTrajectoryPose(int from_trajectory_id, int to_trajectory_id,
                                const transform::Rigid3d& pose,
                                const common::Time time) override
      EXCLUDES(mutex_);
  transform::Rigid3d GetInterpolatedGlobalTrajectoryPose(
      int trajectory_id, const common::Time time) const REQUIRES(mutex_);

 private:
  // The current state of the submap in the background threads. When this
  // transitions to kFinished, all nodes are tried to match against this submap.
  // Likewise, all new nodes are matched against submaps which are finished.
  enum class SubmapState { kActive, kFinished };
  struct SubmapData {
    std::shared_ptr<const Submap> submap;

    // IDs of the nodes that were inserted into this map together with
    // constraints for them. They are not to be matched again when this submap
    // becomes 'finished'.
    std::set<mapping::NodeId> node_ids;

    SubmapState state = SubmapState::kActive;
  };

  // Handles a new work item.
  void AddWorkItem(const std::function<void()>& work_item) REQUIRES(mutex_);

  // Adds connectivity and sampler for a trajectory if it does not exist.
  void AddTrajectoryIfNeeded(int trajectory_id) REQUIRES(mutex_);

  // Grows the optimization problem to have an entry for every element of
  // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
  std::vector<mapping::SubmapId> InitializeGlobalSubmapPoses(
      int trajectory_id, const common::Time time,
      const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
      REQUIRES(mutex_);

  // Adds constraints for a node, and starts scan matching in the background.
  void ComputeConstraintsForNode(
      const mapping::NodeId& node_id,
      std::vector<std::shared_ptr<const Submap>> insertion_submaps,
      bool newly_finished_submap) REQUIRES(mutex_);

  // Computes constraints for a node and submap pair.
  void ComputeConstraint(const mapping::NodeId& node_id,
                         const mapping::SubmapId& submap_id) REQUIRES(mutex_);

  // Adds constraints for older nodes whenever a new submap is finished.
  void ComputeConstraintsForOldNodes(const mapping::SubmapId& submap_id)
      REQUIRES(mutex_);

  // Registers the callback to run the optimization once all constraints have
  // been computed, that will also do all work that queue up in 'work_queue_'.
  void HandleWorkQueue() REQUIRES(mutex_);

  // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
  // all computations have finished.
  void WaitForAllComputations() EXCLUDES(mutex_);

  // Runs the optimization. Callers have to make sure, that there is only one
  // optimization being run at a time.
  void RunOptimization() EXCLUDES(mutex_);

  // Computes the local to global map frame transform based on the given
  // 'global_submap_poses'.
  transform::Rigid3d ComputeLocalToGlobalTransform(
      const mapping::MapById<mapping::SubmapId, pose_graph::SubmapData>&
          global_submap_poses,
      int trajectory_id) const REQUIRES(mutex_);

  mapping::PoseGraph::SubmapData GetSubmapDataUnderLock(
      const mapping::SubmapId& submap_id) REQUIRES(mutex_);

  common::Time GetLatestNodeTime(const mapping::NodeId& node_id,
                                 const mapping::SubmapId& submap_id) const
      REQUIRES(mutex_);

  // Updates the trajectory connectivity structure with a new constraint.
  void UpdateTrajectoryConnectivity(const Constraint& constraint)
      REQUIRES(mutex_);

  const mapping::proto::PoseGraphOptions options_;
  common::Mutex mutex_;

  // If it exists, further work items must be added to this queue, and will be
  // considered later.
  std::unique_ptr<std::deque<std::function<void()>>> work_queue_
      GUARDED_BY(mutex_);

  // How our various trajectories are related.
  mapping::TrajectoryConnectivityState trajectory_connectivity_state_;

  // We globally localize a fraction of the nodes from each trajectory.
  std::unordered_map<int, std::unique_ptr<common::FixedRatioSampler>>
      global_localization_samplers_ GUARDED_BY(mutex_);

  // Number of nodes added since last loop closure.
  int num_nodes_since_last_loop_closure_ GUARDED_BY(mutex_) = 0;

  // Whether the optimization has to be run before more data is added.
  bool run_loop_closure_ GUARDED_BY(mutex_) = false;

  // Current optimization problem.
  pose_graph::OptimizationProblem optimization_problem_;
  pose_graph::ConstraintBuilder constraint_builder_ GUARDED_BY(mutex_);
  std::vector<Constraint> constraints_ GUARDED_BY(mutex_);

  // Submaps get assigned an ID and state as soon as they are seen, even
  // before they take part in the background computations.
  mapping::MapById<mapping::SubmapId, SubmapData> submap_data_
      GUARDED_BY(mutex_);

  // Data that are currently being shown.
  mapping::MapById<mapping::NodeId, mapping::TrajectoryNode> trajectory_nodes_
      GUARDED_BY(mutex_);
  int num_trajectory_nodes_ GUARDED_BY(mutex_) = 0;

  // Global submap poses currently used for displaying data.
  mapping::MapById<mapping::SubmapId, pose_graph::SubmapData>
      global_submap_poses_ GUARDED_BY(mutex_);

  // List of all trimmers to consult when optimizations finish.
  std::vector<std::unique_ptr<mapping::PoseGraphTrimmer>> trimmers_
      GUARDED_BY(mutex_);

  // Set of all frozen trajectories not being optimized.
  std::set<int> frozen_trajectories_ GUARDED_BY(mutex_);

  // Set of all initial trajectory poses.
  std::map<int, InitialTrajectoryPose> initial_trajectory_poses_
      GUARDED_BY(mutex_);

  // Allows querying and manipulating the pose graph by the 'trimmers_'. The
  // 'mutex_' of the pose graph is held while this class is used.
  class TrimmingHandle : public mapping::Trimmable {
   public:
    TrimmingHandle(PoseGraph* parent);
    ~TrimmingHandle() override {}

    int num_submaps(int trajectory_id) const override;
    void MarkSubmapAsTrimmed(const mapping::SubmapId& submap_id)
        REQUIRES(parent_->mutex_) override;

   private:
    PoseGraph* const parent_;
  };
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_H_
