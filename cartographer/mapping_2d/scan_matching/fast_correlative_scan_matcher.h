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

// This is an implementation of the algorithm described in "Real-Time
// Correlative Scan Matching" by Olson.
//
// It is similar to the RealTimeCorrelativeScanMatcher but has a different
// trade-off: Scan matching is faster because more effort is put into the
// precomputation done for a given map. However, this map is immutable after
// construction.

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/port.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/scan_matching/correlative_scan_matcher.h"
#include "cartographer/mapping_2d/scan_matching/proto/fast_correlative_scan_matcher_options.pb.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

proto::FastCorrelativeScanMatcherOptions
CreateFastCorrelativeScanMatcherOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// A precomputed grid that contains in each cell (x0, y0) the maximum
// probability in the width x width area defined by x0 <= x < x0 + width and
// y0 <= y < y0.
class PrecomputationGrid {
 public:
  PrecomputationGrid(const ProbabilityGrid& probability_grid,
                     const CellLimits& limits, int width,
                     std::vector<float>* reusable_intermediate_grid);

  // Returns a value between 0 and 255 to represent probabilities between
  // kMinProbability and kMaxProbability.
  int GetValue(const Eigen::Array2i& xy_index) const {
    const Eigen::Array2i local_xy_index = xy_index - offset_;
    // The static_cast<unsigned> is for performance to check with 2 comparisons
    // xy_index.x() < offset_.x() || xy_index.y() < offset_.y() ||
    // local_xy_index.x() >= wide_limits_.num_x_cells ||
    // local_xy_index.y() >= wide_limits_.num_y_cells
    // instead of using 4 comparisons.
    if (static_cast<unsigned>(local_xy_index.x()) >=
            static_cast<unsigned>(wide_limits_.num_x_cells) ||
        static_cast<unsigned>(local_xy_index.y()) >=
            static_cast<unsigned>(wide_limits_.num_y_cells)) {
      return 0;
    }
    const int stride = wide_limits_.num_x_cells;
    return cells_[local_xy_index.x() + local_xy_index.y() * stride];
  }

  // Maps values from [0, 255] to [kMinProbability, kMaxProbability].
  static float ToProbability(float value) {
    return mapping::kMinProbability +
           value *
               ((mapping::kMaxProbability - mapping::kMinProbability) / 255.f);
  }

 private:
  uint8 ComputeCellValue(float probability) const;

  // Offset of the precomputation grid in relation to the 'probability_grid'
  // including the additional 'width' - 1 cells.
  const Eigen::Array2i offset_;

  // Size of the precomputation grid.
  const CellLimits wide_limits_;

  // Probabilites mapped to 0 to 255.
  std::vector<uint8> cells_;
};

class PrecomputationGridStack;

// An implementation of "Real-Time Correlative Scan Matching" by Olson.
class FastCorrelativeScanMatcher {
 public:
  FastCorrelativeScanMatcher(
      const ProbabilityGrid& probability_grid,
      const proto::FastCorrelativeScanMatcherOptions& options);
  ~FastCorrelativeScanMatcher();

  FastCorrelativeScanMatcher(const FastCorrelativeScanMatcher&) = delete;
  FastCorrelativeScanMatcher& operator=(const FastCorrelativeScanMatcher&) =
      delete;

  // Aligns 'point_cloud' within the 'probability_grid' given an
  // 'initial_pose_estimate'. If a score above 'min_score' (excluding equality)
  // is possible, true is returned, and 'score' and 'pose_estimate' are updated
  // with the result.
  bool Match(const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud, float min_score,
             float* score, transform::Rigid2d* pose_estimate) const;

  // Aligns 'point_cloud' within the full 'probability_grid', i.e., not
  // restricted to the configured search window. If a score above 'min_score'
  // (excluding equality) is possible, true is returned, and 'score' and
  // 'pose_estimate' are updated with the result.
  bool MatchFullSubmap(const sensor::PointCloud& point_cloud, float min_score,
                       float* score, transform::Rigid2d* pose_estimate) const;

 private:
  // The actual implementation of the scan matcher, called by Match() and
  // MatchFullSubmap() with appropriate 'initial_pose_estimate' and
  // 'search_parameters'.
  bool MatchWithSearchParameters(
      SearchParameters search_parameters,
      const transform::Rigid2d& initial_pose_estimate,
      const sensor::PointCloud& point_cloud, float min_score, float* score,
      transform::Rigid2d* pose_estimate) const;
  std::vector<Candidate> ComputeLowestResolutionCandidates(
      const std::vector<DiscreteScan>& discrete_scans,
      const SearchParameters& search_parameters) const;
  std::vector<Candidate> GenerateLowestResolutionCandidates(
      const SearchParameters& search_parameters) const;
  void ScoreCandidates(const PrecomputationGrid& precomputation_grid,
                       const std::vector<DiscreteScan>& discrete_scans,
                       const SearchParameters& search_parameters,
                       std::vector<Candidate>* const candidates) const;
  Candidate BranchAndBound(const std::vector<DiscreteScan>& discrete_scans,
                           const SearchParameters& search_parameters,
                           const std::vector<Candidate>& candidates,
                           int candidate_depth, float min_score) const;

  const proto::FastCorrelativeScanMatcherOptions options_;
  MapLimits limits_;
  std::unique_ptr<PrecomputationGridStack> precomputation_grid_stack_;
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_
