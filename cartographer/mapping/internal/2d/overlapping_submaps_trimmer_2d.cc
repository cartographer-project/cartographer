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

#include "cartographer/mapping/internal/2d/overlapping_submaps_trimmer_2d.h"

#include <algorithm>

#include "cartographer/mapping/2d/submap_2d.h"

namespace cartographer {
namespace mapping {
namespace {

class SubmapCoverageGrid2D {
 public:
  // Aliases for documentation only (no type-safety).
  using CellId = std::pair<int64 /* x cells */, int64 /* y cells */>;
  using StoredType = std::vector<std::pair<SubmapId, common::Time>>;

  explicit SubmapCoverageGrid2D(const Eigen::Vector2d& offset)
      : offset_(offset) {}

  void AddPoint(const Eigen::Vector2d& point, const SubmapId& submap_id,
                const common::Time& time) {
    CellId cell_id{common::RoundToInt64(offset_(0) - point(0)),
                   common::RoundToInt64(offset_(1) - point(1))};
    cells_[cell_id].emplace_back(submap_id, time);
  }

  const std::map<CellId, StoredType>& cells() const { return cells_; }

 private:
  const Eigen::Vector2d offset_;
  std::map<CellId, StoredType> cells_;
};

Eigen::Vector2d GetCornerOfFirstSubmap(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data) {
  auto submap_2d = std::static_pointer_cast<const Submap2D>(
      submap_data.begin()->data.submap);
  return submap_2d->probability_grid().limits().max();
}

// Iterates over every cell in a submap, transforms the center of the cell to
// the global frame and then adds the submap id and the timestamp of the most
// recent range data insertion into the global grid.
std::set<SubmapId> AddSubmapsToSubmapCoverageGrid2D(
    const std::map<SubmapId, common::Time>& submap_freshness,
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data,
    SubmapCoverageGrid2D* coverage_grid) {
  std::set<SubmapId> all_submap_ids;

  for (const auto& submap : submap_data) {
    auto freshness = submap_freshness.find(submap.id);
    if (freshness == submap_freshness.end()) continue;
    if (!submap.data.submap->finished()) continue;
    all_submap_ids.insert(submap.id);
    const ProbabilityGrid& probability_grid =
        std::static_pointer_cast<const Submap2D>(submap.data.submap)
            ->probability_grid();
    // Iterate over every cell in a submap.
    Eigen::Array2i offset;
    CellLimits cell_limits;
    probability_grid.ComputeCroppedLimits(&offset, &cell_limits);
    if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
      LOG(WARNING) << "Empty grid found in submap ID = " << submap.id;
      continue;
    }
    const transform::Rigid2d projected_submap_pose =
        transform::Project2D(submap.data.pose);
    for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
      const Eigen::Array2i index = xy_index + offset;
      if (!probability_grid.IsKnown(index)) continue;
      const transform::Rigid2d center_of_cell_in_global_frame =
          projected_submap_pose *
          transform::Rigid2d::Translation({index.x() + 0.5, index.y() + 0.5});
      coverage_grid->AddPoint(center_of_cell_in_global_frame.translation(),
                              submap.id, freshness->second);
    }
  }
  return all_submap_ids;
}

// Uses intra-submap constraints and trajectory node timestamps to identify time
// of the last range data insertion to the submap.
std::map<SubmapId, common::Time> ComputeSubmapFreshness(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data,
    const MapById<NodeId, TrajectoryNode>& trajectory_nodes,
    const std::vector<PoseGraphInterface::Constraint>& constraints) {
  std::map<SubmapId, common::Time> submap_freshness;

  // Find the node with the largest NodeId per SubmapId.
  std::map<SubmapId, NodeId> submap_to_latest_node;
  for (const PoseGraphInterface::Constraint& constraint : constraints) {
    if (constraint.tag != PoseGraphInterface::Constraint::INTRA_SUBMAP) {
      continue;
    }
    auto submap_to_node = submap_to_latest_node.find(constraint.submap_id);
    if (submap_to_node == submap_to_latest_node.end()) {
      submap_to_latest_node.insert(
          std::make_pair(constraint.submap_id, constraint.node_id));
      continue;
    }
    submap_to_node->second =
        std::max(submap_to_node->second, constraint.node_id);
  }

  // Find timestamp of every latest node.
  for (const auto& submap_id_to_node_id : submap_to_latest_node) {
    auto submap_data_item = submap_data.find(submap_id_to_node_id.first);
    if (submap_data_item == submap_data.end()) {
      LOG(WARNING) << "Intra-submap constraint between SubmapID = "
                   << submap_id_to_node_id.first << " and NodeID "
                   << submap_id_to_node_id.second << " is missing submap data";
      continue;
    }
    auto latest_node_id = trajectory_nodes.find(submap_id_to_node_id.second);
    if (latest_node_id == trajectory_nodes.end()) continue;
    submap_freshness[submap_data_item->id] = latest_node_id->data.time();
  }
  return submap_freshness;
}

// Returns IDs of submaps that have less than 'min_covered_cells_count' cells
// not overlapped by at least 'fresh_submaps_count' submaps.
std::vector<SubmapId> FindSubmapIdsToTrim(
    const SubmapCoverageGrid2D& coverage_grid,
    const std::set<SubmapId>& all_submap_ids, uint16 fresh_submaps_count,
    uint16 min_covered_cells_count) {
  std::map<SubmapId, uint16> submap_to_covered_cells_count;
  for (const auto& cell : coverage_grid.cells()) {
    std::vector<std::pair<SubmapId, common::Time>> submaps_per_cell(
        cell.second);

    // In case there are several submaps covering the cell, only the freshest
    // submaps are kept.
    if (submaps_per_cell.size() > fresh_submaps_count) {
      // Sort by time in descending order.
      std::sort(submaps_per_cell.begin(), submaps_per_cell.end(),
                [](const std::pair<SubmapId, common::Time>& left,
                   const std::pair<SubmapId, common::Time>& right) {
                  return left.second > right.second;
                });
      submaps_per_cell.erase(submaps_per_cell.begin() + fresh_submaps_count,
                             submaps_per_cell.end());
    }
    for (const std::pair<SubmapId, common::Time>& submap : submaps_per_cell) {
      ++submap_to_covered_cells_count[submap.first];
    }
  }
  std::vector<SubmapId> submap_ids_to_keep;
  for (const auto& id_to_cells_count : submap_to_covered_cells_count) {
    if (id_to_cells_count.second < min_covered_cells_count) continue;
    submap_ids_to_keep.push_back(id_to_cells_count.first);
  }

  DCHECK(std::is_sorted(submap_ids_to_keep.begin(), submap_ids_to_keep.end()));
  std::vector<SubmapId> result;
  std::set_difference(all_submap_ids.begin(), all_submap_ids.end(),
                      submap_ids_to_keep.begin(), submap_ids_to_keep.end(),
                      std::back_inserter(result));
  return result;
}

}  // namespace

void OverlappingSubmapsTrimmer2D::Trim(Trimmable* pose_graph) {
  const auto submap_data = pose_graph->GetAllSubmapData();
  if (submap_data.size() == 0) return;

  const auto constraints = pose_graph->GetConstraints();
  const auto trajectory_nodes = pose_graph->GetTrajectoryNodes();

  SubmapCoverageGrid2D coverage_grid(GetCornerOfFirstSubmap(submap_data));

  const std::set<SubmapId> all_submap_ids = AddSubmapsToSubmapCoverageGrid2D(
      ComputeSubmapFreshness(submap_data, trajectory_nodes, constraints),
      submap_data, &coverage_grid);
  const std::vector<SubmapId> submap_ids_to_remove =
      FindSubmapIdsToTrim(coverage_grid, all_submap_ids, fresh_submaps_count_,
                          min_covered_cells_count_);
  for (const SubmapId& id : submap_ids_to_remove) {
    pose_graph->MarkSubmapAsTrimmed(id);
  }
}

}  // namespace mapping
}  // namespace cartographer
