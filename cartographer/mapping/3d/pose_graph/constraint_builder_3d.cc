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

#include "cartographer/mapping/3d/pose_graph/constraint_builder_3d.h"

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
#include "cartographer/mapping/3d/scan_matching/proto/ceres_scan_matcher_options_3d.pb.h"
#include "cartographer/mapping/3d/scan_matching/proto/fast_correlative_scan_matcher_options_3d.pb.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/gauge.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace pose_graph {

static auto* kConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kConstraintsFoundMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsFoundMetric = metrics::Counter::Null();
static auto* kQueueLengthMetric = metrics::Gauge::Null();
static auto* kConstraintScoresMetric = metrics::Histogram::Null();
static auto* kConstraintRotationalScoresMetric = metrics::Histogram::Null();
static auto* kConstraintLowResolutionScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintRotationalScoresMetric =
    metrics::Histogram::Null();
static auto* kGlobalConstraintLowResolutionScoresMetric =
    metrics::Histogram::Null();

ConstraintBuilder3D::ConstraintBuilder3D(
    const proto::ConstraintBuilderOptions& options,
    common::ThreadPool* const thread_pool)
    : options_(options),
      thread_pool_(thread_pool),
      sampler_(options.sampling_ratio()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options_3d()) {}

ConstraintBuilder3D::~ConstraintBuilder3D() {
  common::MutexLocker locker(&mutex_);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(pending_computations_.size(), 0);
  CHECK_EQ(submap_queued_work_items_.size(), 0);
  CHECK(when_done_ == nullptr);
}

void ConstraintBuilder3D::MaybeAddConstraint(
    const SubmapId& submap_id, const Submap3D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const std::vector<TrajectoryNode>& submap_nodes,
    const transform::Rigid3d& global_node_pose,
    const transform::Rigid3d& global_submap_pose) {
  if ((global_node_pose.translation() - global_submap_pose.translation())
          .norm() > options_.max_constraint_distance()) {
    return;
  }
  if (sampler_.Pulse()) {
    common::MutexLocker locker(&mutex_);
    constraints_.emplace_back();
    kQueueLengthMetric->Set(constraints_.size());
    auto* const constraint = &constraints_.back();
    ++pending_computations_[current_computation_];
    const int current_computation = current_computation_;
    ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
        submap_id, submap_nodes, submap, [=]() EXCLUDES(mutex_) {
          ComputeConstraint(submap_id, node_id, false, /* match_full_submap */
                            constant_data, global_node_pose, global_submap_pose,
                            constraint);
          FinishComputation(current_computation);
        });
  }
}

void ConstraintBuilder3D::MaybeAddGlobalConstraint(
    const SubmapId& submap_id, const Submap3D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const std::vector<TrajectoryNode>& submap_nodes,
    const Eigen::Quaterniond& global_node_rotation,
    const Eigen::Quaterniond& global_submap_rotation) {
  common::MutexLocker locker(&mutex_);
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  ++pending_computations_[current_computation_];
  const int current_computation = current_computation_;
  ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
      submap_id, submap_nodes, submap, [=]() EXCLUDES(mutex_) {
        ComputeConstraint(
            submap_id, node_id, true, /* match_full_submap */
            constant_data, transform::Rigid3d::Rotation(global_node_rotation),
            transform::Rigid3d::Rotation(global_submap_rotation), constraint);
        FinishComputation(current_computation);
      });
}

void ConstraintBuilder3D::NotifyEndOfNode() {
  common::MutexLocker locker(&mutex_);
  ++current_computation_;
}

void ConstraintBuilder3D::WhenDone(
    const std::function<void(const ConstraintBuilder3D::Result&)>& callback) {
  common::MutexLocker locker(&mutex_);
  CHECK(when_done_ == nullptr);
  when_done_ =
      common::make_unique<std::function<void(const Result&)>>(callback);
  ++pending_computations_[current_computation_];
  const int current_computation = current_computation_;
  thread_pool_->Schedule(
      [this, current_computation] { FinishComputation(current_computation); });
}

void ConstraintBuilder3D::ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
    const SubmapId& submap_id, const std::vector<TrajectoryNode>& submap_nodes,
    const Submap3D* const submap, const std::function<void()>& work_item) {
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

void ConstraintBuilder3D::ConstructSubmapScanMatcher(
    const SubmapId& submap_id, const std::vector<TrajectoryNode>& submap_nodes,
    const Submap3D* const submap) {
  auto submap_scan_matcher =
      common::make_unique<scan_matching::FastCorrelativeScanMatcher3D>(
          submap->high_resolution_hybrid_grid(),
          &submap->low_resolution_hybrid_grid(), submap_nodes,
          options_.fast_correlative_scan_matcher_options_3d());
  common::MutexLocker locker(&mutex_);
  submap_scan_matchers_[submap_id] = {&submap->high_resolution_hybrid_grid(),
                                      &submap->low_resolution_hybrid_grid(),
                                      std::move(submap_scan_matcher)};
  for (const std::function<void()>& work_item :
       submap_queued_work_items_[submap_id]) {
    thread_pool_->Schedule(work_item);
  }
  submap_queued_work_items_.erase(submap_id);
}

const ConstraintBuilder3D::SubmapScanMatcher*
ConstraintBuilder3D::GetSubmapScanMatcher(const SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  const SubmapScanMatcher* submap_scan_matcher =
      &submap_scan_matchers_[submap_id];
  CHECK(submap_scan_matcher->fast_correlative_scan_matcher != nullptr);
  return submap_scan_matcher;
}

void ConstraintBuilder3D::ComputeConstraint(
    const SubmapId& submap_id, const NodeId& node_id, bool match_full_submap,
    const TrajectoryNode::Data* const constant_data,
    const transform::Rigid3d& global_node_pose,
    const transform::Rigid3d& global_submap_pose,
    std::unique_ptr<Constraint>* constraint) {
  const SubmapScanMatcher* const submap_scan_matcher =
      GetSubmapScanMatcher(submap_id);

  // The 'constraint_transform' (submap i <- node j) is computed from:
  // - a 'high_resolution_point_cloud' in node j and
  // - the initial guess 'initial_pose' (submap i <- node j).
  std::unique_ptr<scan_matching::FastCorrelativeScanMatcher3D::Result>
      match_result;

  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  if (match_full_submap) {
    kGlobalConstraintsSearchedMetric->Increment();
    match_result =
        submap_scan_matcher->fast_correlative_scan_matcher->MatchFullSubmap(
            global_node_pose.rotation(), global_submap_pose.rotation(),
            *constant_data, options_.global_localization_min_score());
    if (match_result != nullptr) {
      CHECK_GT(match_result->score, options_.global_localization_min_score());
      CHECK_GE(node_id.trajectory_id, 0);
      CHECK_GE(submap_id.trajectory_id, 0);
      kGlobalConstraintsFoundMetric->Increment();
      kGlobalConstraintScoresMetric->Observe(match_result->score);
      kGlobalConstraintRotationalScoresMetric->Observe(
          match_result->rotational_score);
      kGlobalConstraintLowResolutionScoresMetric->Observe(
          match_result->low_resolution_score);
    } else {
      return;
    }
  } else {
    kConstraintsSearchedMetric->Increment();
    match_result = submap_scan_matcher->fast_correlative_scan_matcher->Match(
        global_node_pose, global_submap_pose, *constant_data,
        options_.min_score());
    if (match_result != nullptr) {
      // We've reported a successful local match.
      CHECK_GT(match_result->score, options_.min_score());
      kConstraintsFoundMetric->Increment();
      kConstraintScoresMetric->Observe(match_result->score);
      kConstraintRotationalScoresMetric->Observe(
          match_result->rotational_score);
      kConstraintLowResolutionScoresMetric->Observe(
          match_result->low_resolution_score);
    } else {
      return;
    }
  }
  {
    common::MutexLocker locker(&mutex_);
    score_histogram_.Add(match_result->score);
    rotational_score_histogram_.Add(match_result->rotational_score);
    low_resolution_score_histogram_.Add(match_result->low_resolution_score);
  }

  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.
  ceres::Solver::Summary unused_summary;
  transform::Rigid3d constraint_transform;
  ceres_scan_matcher_.Match(match_result->pose_estimate.translation(),
                            match_result->pose_estimate,
                            {{&constant_data->high_resolution_point_cloud,
                              submap_scan_matcher->high_resolution_hybrid_grid},
                             {&constant_data->low_resolution_point_cloud,
                              submap_scan_matcher->low_resolution_hybrid_grid}},
                            &constraint_transform, &unused_summary);

  constraint->reset(new Constraint{
      submap_id,
      node_id,
      {constraint_transform, options_.loop_closure_translation_weight(),
       options_.loop_closure_rotation_weight()},
      Constraint::INTER_SUBMAP});

  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->high_resolution_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      // Compute the difference between (submap i <- node j) according to loop
      // closure ('constraint_transform') and according to global SLAM state.
      const transform::Rigid3d difference = global_node_pose.inverse() *
                                            global_submap_pose *
                                            constraint_transform;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << transform::GetAngle(difference);
    }
    info << " with score " << std::setprecision(1) << 100. * match_result->score
         << "%.";
    LOG(INFO) << info.str();
  }
}

void ConstraintBuilder3D::FinishComputation(const int computation_index) {
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
        for (const std::unique_ptr<Constraint>& constraint : constraints_) {
          if (constraint != nullptr) {
            result.push_back(*constraint);
          }
        }
        if (options_.log_matches()) {
          LOG(INFO) << constraints_.size() << " computations resulted in "
                    << result.size() << " additional constraints.";
          LOG(INFO) << "Score histogram:\n" << score_histogram_.ToString(10);
          LOG(INFO) << "Rotational score histogram:\n"
                    << rotational_score_histogram_.ToString(10);
          LOG(INFO) << "Low resolution score histogram:\n"
                    << low_resolution_score_histogram_.ToString(10);
        }
        constraints_.clear();
        callback = std::move(when_done_);
        when_done_.reset();
      }
    }
    kQueueLengthMetric->Set(constraints_.size());
  }
  if (callback != nullptr) {
    (*callback)(result);
  }
}

int ConstraintBuilder3D::GetNumFinishedNodes() {
  common::MutexLocker locker(&mutex_);
  if (pending_computations_.empty()) {
    return current_computation_;
  }
  return pending_computations_.begin()->first;
}

void ConstraintBuilder3D::DeleteScanMatcher(const SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  CHECK(pending_computations_.empty());
  submap_scan_matchers_.erase(submap_id);
}

void ConstraintBuilder3D::RegisterMetrics(metrics::FamilyFactory* factory) {
  auto* counts = factory->NewCounterFamily(
      "/mapping/3d/pose_graph/constraint_builder/constraints",
      "Constraints computed");
  kConstraintsSearchedMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "searched"}});
  kConstraintsFoundMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "found"}});
  kGlobalConstraintsSearchedMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "searched"}});
  kGlobalConstraintsFoundMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "found"}});
  auto* queue_length = factory->NewGaugeFamily(
      "/mapping/3d/pose_graph/constraint_builder/queue_length", "Queue length");
  kQueueLengthMetric = queue_length->Add({{}});
  auto boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = factory->NewHistogramFamily(
      "/mapping/3d/pose_graph/constraint_builder/scores",
      "Constraint scores built", boundaries);
  kConstraintScoresMetric =
      scores->Add({{"search_region", "local"}, {"kind", "score"}});
  kConstraintRotationalScoresMetric =
      scores->Add({{"search_region", "local"}, {"kind", "rotational_score"}});
  kConstraintLowResolutionScoresMetric = scores->Add(
      {{"search_region", "local"}, {"kind", "low_resolution_score"}});
  kGlobalConstraintScoresMetric =
      scores->Add({{"search_region", "global"}, {"kind", "score"}});
  kGlobalConstraintRotationalScoresMetric =
      scores->Add({{"search_region", "global"}, {"kind", "rotational_score"}});
  kGlobalConstraintLowResolutionScoresMetric = scores->Add(
      {{"search_region", "global"}, {"kind", "low_resolution_score"}});
}

}  // namespace pose_graph
}  // namespace mapping
}  // namespace cartographer
