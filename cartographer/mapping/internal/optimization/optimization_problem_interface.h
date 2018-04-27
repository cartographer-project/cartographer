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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_INTERFACE_H_

#include <map>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/time.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/map_by_time.h"
#include "cartographer/sensor/odometry_data.h"

namespace cartographer {
namespace mapping {
namespace optimization {

// Implements the SPA loop closure method.
template <typename NodeDataType, typename SubmapDataType,
          typename RigidTransformType>
class OptimizationProblemInterface {
 public:
  using Constraint = PoseGraphInterface::Constraint;
  using LandmarkNode = PoseGraphInterface::LandmarkNode;

  OptimizationProblemInterface() {}
  virtual ~OptimizationProblemInterface() {}

  OptimizationProblemInterface(const OptimizationProblemInterface&) = delete;
  OptimizationProblemInterface& operator=(const OptimizationProblemInterface&) =
      delete;

  virtual void AddImuData(int trajectory_id,
                          const sensor::ImuData& imu_data) = 0;
  virtual void AddOdometryData(int trajectory_id,
                               const sensor::OdometryData& odometry_data) = 0;
  virtual void AddTrajectoryNode(int trajectory_id,
                                 const NodeDataType& node_data) = 0;
  virtual void InsertTrajectoryNode(const NodeId& node_id,
                                    const NodeDataType& node_data) = 0;
  virtual void TrimTrajectoryNode(const NodeId& node_id) = 0;
  virtual void AddSubmap(int trajectory_id,
                         const RigidTransformType& global_submap_pose) = 0;
  virtual void InsertSubmap(const SubmapId& submap_id,
                            const RigidTransformType& global_submap_pose) = 0;
  virtual void TrimSubmap(const SubmapId& submap_id) = 0;
  virtual void SetMaxNumIterations(int32 max_num_iterations) = 0;

  // Optimizes the global poses.
  virtual void Solve(
      const std::vector<Constraint>& constraints,
      const std::set<int>& frozen_trajectories,
      const std::map<std::string, LandmarkNode>& landmark_nodes) = 0;

  virtual const MapById<NodeId, NodeDataType>& node_data() const = 0;
  virtual const MapById<SubmapId, SubmapDataType>& submap_data() const = 0;
  virtual const std::map<std::string, transform::Rigid3d>& landmark_data()
      const = 0;
  virtual const sensor::MapByTime<sensor::ImuData>& imu_data() const = 0;
  virtual const sensor::MapByTime<sensor::OdometryData>& odometry_data()
      const = 0;
};

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_INTERFACE_H_
