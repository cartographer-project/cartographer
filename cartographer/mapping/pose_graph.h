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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_H_

#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/pose_graph_options.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/map_by_time.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

proto::PoseGraphOptions CreatePoseGraphOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class PoseGraph {
 public:
  // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
  // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
  // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
  struct Constraint {
    struct Pose {
      transform::Rigid3d zbar_ij;
      double translation_weight;
      double rotation_weight;
    };

    SubmapId submap_id;  // 'i' in the paper.
    NodeId node_id;      // 'j' in the paper.

    // Pose of the node 'j' relative to submap 'i'.
    Pose pose;

    // Differentiates between intra-submap (where node 'j' was inserted into
    // submap 'i') and inter-submap constraints (where node 'j' was not inserted
    // into submap 'i').
    enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
  };

  struct SubmapData {
    std::shared_ptr<const Submap> submap;
    transform::Rigid3d pose;
  };

  struct InitialTrajectoryPose {
    int to_trajectory_id;
    transform::Rigid3d relative_pose;
    common::Time time;
  };

  PoseGraph() {}
  virtual ~PoseGraph() {}

  PoseGraph(const PoseGraph&) = delete;
  PoseGraph& operator=(const PoseGraph&) = delete;

  // Inserts an IMU measurement.
  virtual void AddImuData(int trajectory_id,
                          const sensor::ImuData& imu_data) = 0;

  // Inserts an odometry measurement.
  virtual void AddOdometryData(int trajectory_id,
                               const sensor::OdometryData& odometry_data) = 0;

  // Finishes the given trajectory.
  virtual void FinishTrajectory(int trajectory_id) = 0;

  // Freezes a trajectory. Poses in this trajectory will not be optimized.
  virtual void FreezeTrajectory(int trajectory_id) = 0;

  // Adds a 'submap' from a proto with the given 'global_pose' to the
  // appropriate frozen trajectory.
  virtual void AddSubmapFromProto(const transform::Rigid3d& global_pose,
                                  const proto::Submap& submap) = 0;

  // Adds a 'node' from a proto with the given 'global_pose' to the
  // appropriate frozen trajectory.
  virtual void AddNodeFromProto(const transform::Rigid3d& global_pose,
                                const proto::Node& node) = 0;

  // Adds information that 'node_id' was inserted into 'submap_id'. The submap
  // has to be deserialized first.
  virtual void AddNodeToSubmap(const NodeId& node_id,
                               const SubmapId& submap_id) = 0;

  // Adds serialized constraints. The corresponding trajectory nodes and submaps
  // have to be deserialized before calling this function.
  virtual void AddSerializedConstraints(
      const std::vector<Constraint>& constraints) = 0;

  // Adds a 'trimmer'. It will be used after all data added before it has been
  // included in the pose graph.
  virtual void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) = 0;

  // Computes optimized poses.
  virtual void RunFinalOptimization() = 0;

  // Gets the current trajectory clusters.
  virtual std::vector<std::vector<int>> GetConnectedTrajectories() = 0;

  // Returns the current optimized transform and submap itself for the given
  // 'submap_id'. Returns 'nullptr' for the 'submap' member if the submap does
  // not exist (anymore).
  virtual SubmapData GetSubmapData(const SubmapId& submap_id) = 0;

  // Returns data for all submaps.
  virtual MapById<SubmapId, SubmapData> GetAllSubmapData() = 0;

  // Returns the transform converting data in the local map frame (i.e. the
  // continuous, non-loop-closed frame) into the global map frame (i.e. the
  // discontinuous, loop-closed frame).
  virtual transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) = 0;

  // Returns the current optimized trajectories.
  virtual MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() = 0;

  // Serializes the constraints and trajectories.
  proto::PoseGraph ToProto();

  // Returns the IMU data.
  virtual sensor::MapByTime<sensor::ImuData> GetImuData() = 0;

  // Returns the odometry data.
  virtual sensor::MapByTime<sensor::OdometryData> GetOdometryData() = 0;

  // Returns the collection of constraints.
  virtual std::vector<Constraint> constraints() = 0;

  // Sets a relative initial pose 'relative_pose' for 'from_trajectory_id' with
  // respect to 'to_trajectory_id' at time 'time'.
  virtual void SetInitialTrajectoryPose(int from_trajectory_id,
                                        int to_trajectory_id,
                                        const transform::Rigid3d& pose,
                                        const common::Time time) = 0;
};

std::vector<PoseGraph::Constraint> FromProto(
    const ::google::protobuf::RepeatedPtrField<
        ::cartographer::mapping::proto::PoseGraph::Constraint>&
        constraint_protos);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_H_
