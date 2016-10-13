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
// The correlative scan matching algorithm is exhaustively evaluating the scan
// matching search space. As described by the paper, the basic steps are:
//
// 1) Evaluate the probability p(z|xi, m) over the entire 3D search window using
// the low-resolution table.
// 2) Find the best voxel in the low-resolution 3D space that has not already
// been considered. Denote this value as Li. If Li < Hbest, terminate: Hbest is
// the best scan matching alignment.
// 3) Evaluate the search volume inside voxel i using the high resolution table.
// Suppose the log-likelihood of this voxel is Hi. Note that Hi <= Li since the
// low-resolution map overestimates the log likelihoods. If Hi > Hbest, set
// Hbest = Hi.
//
// This can be made even faster by transforming the scan exactly once over some
// discretized range.

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H_

#include <iostream>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/scan_matching/correlative_scan_matcher.h"
#include "cartographer/mapping_2d/scan_matching/proto/real_time_correlative_scan_matcher_options.pb.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

proto::RealTimeCorrelativeScanMatcherOptions
CreateRealTimeCorrelativeScanMatcherOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

// An implementation of "Real-Time Correlative Scan Matching" by Olson.
class RealTimeCorrelativeScanMatcher {
 public:
  explicit RealTimeCorrelativeScanMatcher(
      const proto::RealTimeCorrelativeScanMatcherOptions& options);

  RealTimeCorrelativeScanMatcher(const RealTimeCorrelativeScanMatcher&) =
      delete;
  RealTimeCorrelativeScanMatcher& operator=(
      const RealTimeCorrelativeScanMatcher&) = delete;

  // Aligns 'point_cloud' within the 'probability_grid' given an
  // 'initial_pose_estimate' then updates 'pose_estimate' with the result and
  // returns the score.
  double Match(const transform::Rigid2d& initial_pose_estimate,
               const sensor::PointCloud& point_cloud,
               const ProbabilityGrid& probability_grid,
               transform::Rigid2d* pose_estimate) const;

  // Computes the score for each Candidate in a collection. The cost is computed
  // as the sum of probabilities, different from the Ceres CostFunctions:
  // http://ceres-solver.org/modeling.html
  //
  // Visible for testing.
  void ScoreCandidates(const ProbabilityGrid& probability_grid,
                       const std::vector<DiscreteScan>& discrete_scans,
                       const SearchParameters& search_parameters,
                       std::vector<Candidate>* candidates) const;

 private:
  std::vector<Candidate> GenerateExhaustiveSearchCandidates(
      const SearchParameters& search_parameters) const;

  const proto::RealTimeCorrelativeScanMatcherOptions options_;
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H_
