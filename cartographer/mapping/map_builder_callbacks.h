#pragma once

#include <optional>

#include "cartographer/mapping/internal/constraints/constraint_builder_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/fast_correlative_scan_matcher_3d.h"

namespace cartographer::mapping {

struct MapBuilderCallbacks {
  // Gets called on every loop closure search attempt
  std::function<void(
    scan_matching::FastCorrelativeScanMatcher3D::Result,  // Coarse search
    std::optional<constraints::ConstraintBuilder3D::Constraint> 
  )> loop_closure_cb{nullptr};

  // Called with the trajectory node and the intra-submap constraint in global slam
  std::function<void(const TrajectoryNode&, const constraints::ConstraintBuilder3D::Constraint&)> node_insertion_cb{nullptr}; 
    
  // Called when a trajectory node is initially created in local slam
  std::function<void(const TrajectoryNode&)> local_slam_node_cb{nullptr};

  // Optimization 
  std::function<void(const ceres::Solver::Summary&)> optimization_cb{nullptr};

  // Remaining work items in the queue
  std::function<void(int)> work_items_queue_cb{nullptr};

};

}  // namespace cartographer::mapping
