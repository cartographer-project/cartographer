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

// This is an implementation of a 3D branch-and-bound algorithm similar to
// FastCorrelativeScanMatcher2D.

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_3D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/port.h"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/3d/scan_matching/precomputation_grid_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"
#include "cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_3d.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::FastCorrelativeScanMatcherOptions3D
CreateFastCorrelativeScanMatcherOptions3D(
    common::LuaParameterDictionary* parameter_dictionary);

class PrecomputationGridStack3D {
 public:
  PrecomputationGridStack3D(
      const HybridGrid& hybrid_grid,
      const proto::FastCorrelativeScanMatcherOptions3D& options);

  const PrecomputationGrid3D& Get(int depth) const {
    return precomputation_grids_.at(depth);
  }

  int max_depth() const { return precomputation_grids_.size() - 1; }

 private:
  std::vector<PrecomputationGrid3D> precomputation_grids_;
};

struct DiscreteScan3D;
struct Candidate3D;

// Used to compute scores between 0 and 1 how well the given pose matches.
using MatchingFunction = std::function<float(const transform::Rigid3f&)>;

class FastCorrelativeScanMatcher3D {
 public:
  struct Result {
    float score;
    transform::Rigid3d pose_estimate;
    float rotational_score;
    float low_resolution_score;
  };

  FastCorrelativeScanMatcher3D(
      const HybridGrid& hybrid_grid,
      const HybridGrid* low_resolution_hybrid_grid,
      const std::vector<TrajectoryNode>& nodes,
      const proto::FastCorrelativeScanMatcherOptions3D& options);
  ~FastCorrelativeScanMatcher3D();

  FastCorrelativeScanMatcher3D(const FastCorrelativeScanMatcher3D&) = delete;
  FastCorrelativeScanMatcher3D& operator=(const FastCorrelativeScanMatcher3D&) =
      delete;

  // Aligns the node with the given 'constant_data' within the 'hybrid_grid'
  // given 'global_node_pose' and 'global_submap_pose'. 'Result' is only
  // returned if a score above 'min_score' (excluding equality) is possible.
  std::unique_ptr<Result> Match(const transform::Rigid3d& global_node_pose,
                                const transform::Rigid3d& global_submap_pose,
                                const TrajectoryNode::Data& constant_data,
                                float min_score) const;

  // Aligns the node with the given 'constant_data' within the 'hybrid_grid'
  // given rotations which are expected to be approximately gravity aligned.
  // 'Result' is only returned if a score above 'min_score' (excluding equality)
  // is possible.
  std::unique_ptr<Result> MatchFullSubmap(
      const Eigen::Quaterniond& global_node_rotation,
      const Eigen::Quaterniond& global_submap_rotation,
      const TrajectoryNode::Data& constant_data, float min_score) const;

 private:
  struct SearchParameters {
    const int linear_xy_window_size;     // voxels
    const int linear_z_window_size;      // voxels
    const double angular_search_window;  // radians
    const MatchingFunction* const low_resolution_matcher;
  };

  std::unique_ptr<Result> MatchWithSearchParameters(
      const SearchParameters& search_parameters,
      const transform::Rigid3f& global_node_pose,
      const transform::Rigid3f& global_submap_pose,
      const sensor::PointCloud& point_cloud,
      const Eigen::VectorXf& rotational_scan_matcher_histogram,
      const Eigen::Quaterniond& gravity_alignment, float min_score) const;
  DiscreteScan3D DiscretizeScan(const SearchParameters& search_parameters,
                                const sensor::PointCloud& point_cloud,
                                const transform::Rigid3f& pose,
                                float rotational_score) const;
  std::vector<DiscreteScan3D> GenerateDiscreteScans(
      const SearchParameters& search_parameters,
      const sensor::PointCloud& point_cloud,
      const Eigen::VectorXf& rotational_scan_matcher_histogram,
      const Eigen::Quaterniond& gravity_alignment,
      const transform::Rigid3f& global_node_pose,
      const transform::Rigid3f& global_submap_pose) const;
  std::vector<Candidate3D> GenerateLowestResolutionCandidates(
      const SearchParameters& search_parameters, int num_discrete_scans) const;
  void ScoreCandidates(int depth,
                       const std::vector<DiscreteScan3D>& discrete_scans,
                       std::vector<Candidate3D>* const candidates) const;
  std::vector<Candidate3D> ComputeLowestResolutionCandidates(
      const SearchParameters& search_parameters,
      const std::vector<DiscreteScan3D>& discrete_scans) const;
  Candidate3D BranchAndBound(const SearchParameters& search_parameters,
                             const std::vector<DiscreteScan3D>& discrete_scans,
                             const std::vector<Candidate3D>& candidates,
                             int candidate_depth, float min_score) const;
  transform::Rigid3f GetPoseFromCandidate(
      const std::vector<DiscreteScan3D>& discrete_scans,
      const Candidate3D& candidate) const;

  const proto::FastCorrelativeScanMatcherOptions3D options_;
  const float resolution_;
  const int width_in_voxels_;
  std::unique_ptr<PrecomputationGridStack3D> precomputation_grid_stack_;
  const HybridGrid* const low_resolution_hybrid_grid_;
  RotationalScanMatcher rotational_scan_matcher_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_3D_H_
