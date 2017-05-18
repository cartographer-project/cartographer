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

#include "cartographer/mapping_3d/sparse_pose_graph/constraint_builder.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/mapping_3d/scan_matching/proto/ceres_scan_matcher_options.pb.h"
#include "cartographer/mapping_3d/scan_matching/proto/fast_correlative_scan_matcher_options.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {
namespace sparse_pose_graph {

ConstraintBuilder::ConstraintBuilder(
    const mapping::sparse_pose_graph::proto::ConstraintBuilderOptions& options,
    common::ThreadPool* const thread_pool)
    : options_(options),
      thread_pool_(thread_pool),
      sampler_(options.sampling_ratio()),
      adaptive_voxel_filter_(options.adaptive_voxel_filter_options()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options_3d()) {}

ConstraintBuilder::~ConstraintBuilder() {
  common::MutexLocker locker(&mutex_);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(pending_computations_.size(), 0);
  CHECK_EQ(submap_queued_work_items_.size(), 0);
  CHECK(when_done_ == nullptr);
}

void ConstraintBuilder::MaybeAddConstraint(
    const mapping::SubmapId& submap_id, const Submap* const submap,
    const mapping::NodeId& node_id,
    const sensor::CompressedPointCloud* const compressed_point_cloud,
    const std::vector<mapping::TrajectoryNode>& submap_nodes,
    const transform::Rigid3d& initial_pose) {
  if (initial_pose.translation().norm() > options_.max_constraint_distance()) {
    return;
  }
  if (sampler_.Pulse()) {
    common::MutexLocker locker(&mutex_);
    constraints_.emplace_back();
    auto* const constraint = &constraints_.back();
    ++pending_computations_[current_computation_];
    const int current_computation = current_computation_;
    ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
        submap_id, submap_nodes, &submap->high_resolution_hybrid_grid,
        [=]() EXCLUDES(mutex_) {
          ComputeConstraint(submap_id, submap, node_id,
                            false,   /* match_full_submap */
                            nullptr, /* trajectory_connectivity */
                            compressed_point_cloud, initial_pose, constraint);
          FinishComputation(current_computation);
        });
  }
}

void ConstraintBuilder::MaybeAddGlobalConstraint(
    const mapping::SubmapId& submap_id, const Submap* const submap,
    const mapping::NodeId& node_id,
    const sensor::CompressedPointCloud* const compressed_point_cloud,
    const std::vector<mapping::TrajectoryNode>& submap_nodes,
    const Eigen::Quaterniond& gravity_alignment,
    mapping::TrajectoryConnectivity* const trajectory_connectivity) {
  common::MutexLocker locker(&mutex_);
  constraints_.emplace_back();
  auto* const constraint = &constraints_.back();
  ++pending_computations_[current_computation_];
  const int current_computation = current_computation_;
  ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
      submap_id, submap_nodes, &submap->high_resolution_hybrid_grid,
      [=]() EXCLUDES(mutex_) {
        ComputeConstraint(
            submap_id, submap, node_id, true, /* match_full_submap */
            trajectory_connectivity, compressed_point_cloud,
            transform::Rigid3d::Rotation(gravity_alignment), constraint);
        FinishComputation(current_computation);
      });
}

void ConstraintBuilder::NotifyEndOfScan() {
  common::MutexLocker locker(&mutex_);
  ++current_computation_;
}

void ConstraintBuilder::WhenDone(
    const std::function<void(const ConstraintBuilder::Result&)> callback) {
  common::MutexLocker locker(&mutex_);
  CHECK(when_done_ == nullptr);
  when_done_ =
      common::make_unique<std::function<void(const Result&)>>(callback);
  ++pending_computations_[current_computation_];
  const int current_computation = current_computation_;
  thread_pool_->Schedule(
      [this, current_computation] { FinishComputation(current_computation); });
}

void ConstraintBuilder::ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
    const mapping::SubmapId& submap_id,
    const std::vector<mapping::TrajectoryNode>& submap_nodes,
    const HybridGrid* const submap, const std::function<void()> work_item) {
  if (submap_scan_matchers_[submap_id].fast_correlative_scan_matcher !=
      nullptr) {
    thread_pool_->Schedule(work_item);
  } else {
    submap_queued_work_items_[submap_id].push_back(work_item);
    if (submap_queued_work_items_[submap_id].size() == 1) {
      thread_pool_->Schedule([=]() {
        ConstructSubmapScanMatcher(submap_id, submap_nodes, submap);
      });
    }
  }
}

void ConstraintBuilder::ConstructSubmapScanMatcher(
    const mapping::SubmapId& submap_id,
    const std::vector<mapping::TrajectoryNode>& submap_nodes,
    const HybridGrid* const submap) {
  auto submap_scan_matcher =
      common::make_unique<scan_matching::FastCorrelativeScanMatcher>(
          *submap, submap_nodes,
          options_.fast_correlative_scan_matcher_options_3d());
  common::MutexLocker locker(&mutex_);
  submap_scan_matchers_[submap_id] = {submap, std::move(submap_scan_matcher)};
  for (const std::function<void()>& work_item :
       submap_queued_work_items_[submap_id]) {
    thread_pool_->Schedule(work_item);
  }
  submap_queued_work_items_.erase(submap_id);
}

const ConstraintBuilder::SubmapScanMatcher*
ConstraintBuilder::GetSubmapScanMatcher(const mapping::SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  const SubmapScanMatcher* submap_scan_matcher =
      &submap_scan_matchers_[submap_id];
  CHECK(submap_scan_matcher->fast_correlative_scan_matcher != nullptr);
  return submap_scan_matcher;
}

void ConstraintBuilder::ComputeConstraint(
    const mapping::SubmapId& submap_id, const Submap* const submap,
    const mapping::NodeId& node_id, bool match_full_submap,
    mapping::TrajectoryConnectivity* trajectory_connectivity,
    const sensor::CompressedPointCloud* const compressed_point_cloud,
    const transform::Rigid3d& initial_pose,
    std::unique_ptr<OptimizationProblem::Constraint>* constraint) {
  const SubmapScanMatcher* const submap_scan_matcher =
      GetSubmapScanMatcher(submap_id);
  const sensor::PointCloud point_cloud = compressed_point_cloud->Decompress();
  const sensor::PointCloud filtered_point_cloud =
      adaptive_voxel_filter_.Filter(point_cloud);

  // The 'constraint_transform' (submap 'i' <- scan 'j') is computed from the
  // initial guess 'initial_pose' for (submap 'i' <- scan 'j') and a
  // 'filtered_point_cloud' in 'j'.
  float score = 0.;
  transform::Rigid3d pose_estimate;

  if (match_full_submap) {
    if (submap_scan_matcher->fast_correlative_scan_matcher->MatchFullSubmap(
            initial_pose.rotation(), filtered_point_cloud, point_cloud,
            options_.global_localization_min_score(), &score, &pose_estimate)) {
      CHECK_GT(score, options_.global_localization_min_score());
      CHECK_GE(node_id.trajectory_id, 0);
      CHECK_GE(submap_id.trajectory_id, 0);
      trajectory_connectivity->Connect(node_id.trajectory_id,
                                       submap_id.trajectory_id);
    } else {
      return;
    }
  } else {
    if (submap_scan_matcher->fast_correlative_scan_matcher->Match(
            initial_pose, filtered_point_cloud, point_cloud,
            options_.min_score(), &score, &pose_estimate)) {
      // We've reported a successful local match.
      CHECK_GT(score, options_.min_score());
    } else {
      return;
    }
  }
  {
    common::MutexLocker locker(&mutex_);
    score_histogram_.Add(score);
  }

  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.
  ceres::Solver::Summary unused_summary;
  transform::Rigid3d constraint_transform;
  ceres_scan_matcher_.Match(
      pose_estimate, pose_estimate,
      {{&filtered_point_cloud, submap_scan_matcher->hybrid_grid}},
      &constraint_transform, &unused_summary);

  constraint->reset(new OptimizationProblem::Constraint{
      submap_id,
      node_id,
      {constraint_transform,
       1. / std::sqrt(options_.lower_covariance_eigenvalue_bound()) *
           kalman_filter::PoseCovariance::Identity()},
      OptimizationProblem::Constraint::INTER_SUBMAP});

  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with " << filtered_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      const transform::Rigid3d difference =
          initial_pose.inverse() * constraint_transform;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << transform::GetAngle(difference);
    }
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}

void ConstraintBuilder::FinishComputation(const int computation_index) {
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    common::MutexLocker locker(&mutex_);
    if (--pending_computations_[computation_index] == 0) {
      pending_computations_.erase(computation_index);
    }
    if (pending_computations_.empty()) {
      CHECK_EQ(submap_queued_work_items_.size(), 0);
      if (when_done_ != nullptr) {
        for (const std::unique_ptr<OptimizationProblem::Constraint>&
                 constraint : constraints_) {
          if (constraint != nullptr) {
            result.push_back(*constraint);
          }
        }
        if (options_.log_matches()) {
          LOG(INFO) << constraints_.size() << " computations resulted in "
                    << result.size() << " additional constraints.";
          LOG(INFO) << "Score histogram:\n" << score_histogram_.ToString(10);
        }
        constraints_.clear();
        callback = std::move(when_done_);
        when_done_.reset();
      }
    }
  }
  if (callback != nullptr) {
    (*callback)(result);
  }
}

int ConstraintBuilder::GetNumFinishedScans() {
  common::MutexLocker locker(&mutex_);
  if (pending_computations_.empty()) {
    return current_computation_;
  }
  return pending_computations_.begin()->first;
}

}  // namespace sparse_pose_graph
}  // namespace mapping_3d
}  // namespace cartographer
