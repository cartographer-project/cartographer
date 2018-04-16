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

#include "cartographer/mapping/internal/2d/pose_graph/constraint_builder_2d.h"

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
#include "cartographer/mapping/2d/scan_matching/proto/ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/mapping/2d/scan_matching/proto/fast_correlative_scan_matcher_options_2d.pb.h"
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
static auto* kGlobalConstraintScoresMetric = metrics::Histogram::Null();

transform::Rigid2d ComputeSubmapPose(const Submap2D& submap) {
  return transform::Project2D(submap.local_pose());
}

ConstraintBuilder2D::ConstraintBuilder2D(
    const pose_graph::proto::ConstraintBuilderOptions& options,
    common::ThreadPool* const thread_pool)
    : options_(options),
      thread_pool_(thread_pool),
      sampler_(options.sampling_ratio()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options()) {}

ConstraintBuilder2D::~ConstraintBuilder2D() {
  common::MutexLocker locker(&mutex_);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(node_index_to_constraint_search_tasks_.size(), 0);
  CHECK(when_done_ == nullptr);
}

void ConstraintBuilder2D::MaybeAddConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose) {
  if (initial_relative_pose.translation().norm() >
      options_.max_constraint_distance()) {
    return;
  }
  LOG(INFO) << "ConstraintBuilder2D::MaybeAddConstraint";
  if (sampler_.Pulse()) {
    common::MutexLocker locker(&mutex_);
    constraints_.emplace_back();
    kQueueLengthMetric->Set(constraints_.size());
    auto* const constraint = &constraints_.back();
    DispatchScanMatcherConstructionAndWorkItem(
        submap_id, &submap->probability_grid(), [=]() EXCLUDES(mutex_) {
          LOG(INFO) << "Execute ConstraintBuilder2D::MaybeAddConstraint";
          ComputeConstraint(submap_id, submap, node_id,
                            false, /* match_full_submap */
                            constant_data, initial_relative_pose, constraint);
        });
  }
}

void ConstraintBuilder2D::MaybeAddGlobalConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data) {
  common::MutexLocker locker(&mutex_);
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  DispatchScanMatcherConstructionAndWorkItem(
      submap_id, &submap->probability_grid(), [=]() EXCLUDES(mutex_) {
        LOG(INFO) << "Execute ConstraintBuilder2D::MaybeAddGlobalConstraint";
        ComputeConstraint(
            submap_id, submap, node_id, true, /* match_full_submap */
            constant_data, transform::Rigid2d::Identity(), constraint);
      });
}

void ConstraintBuilder2D::NotifyEndOfNode() {
  common::MutexLocker locker(&mutex_);
  if (!node_index_to_constraint_search_tasks_.count(current_node_index_)) {
    ++num_finished_nodes_;
  }
  // TODO: Create task that depends on all constraint search for node.
  // TODO(gaschler): Delete task when done.
  auto* task = new common::Task;
  int current_node_index = current_node_index_;
  task->SetWorkItem([this, current_node_index] {
    LOG(INFO) << "Execute ConstraintBuilder2D::NotifyEndOfNode";
    FinishComputation(current_node_index);
  });
  auto& list = node_index_to_constraint_search_tasks_[current_node_index_];
  for (auto it = list.begin(); it != list.end(); ++it) {
    task->AddDependency(&*it);
  }
  task->Dispatch(thread_pool_);
  ++current_node_index_;
}

void ConstraintBuilder2D::WhenDone(
    const std::function<void(const ConstraintBuilder2D::Result&)>& callback) {
  common::MutexLocker locker(&mutex_);
  CHECK(when_done_ == nullptr);
  when_done_ =
      common::make_unique<std::function<void(const Result&)>>(callback);
  // TODO(gaschler): Remove pending_computations_ and current_computation.

  auto* task = new common::Task;
  int current_node_index = current_node_index_;
  task->SetWorkItem([this, current_node_index] {
    LOG(INFO) << "Execute ConstraintBuilder2D::WhenDone";
    FinishComputation(current_node_index);
  });
  auto& list = node_index_to_constraint_search_tasks_[current_node_index_];
  for (auto it = list.begin(); it != list.end(); ++it) {
    task->AddDependency(&*it);
  }
  // TODO: check if this is sufficient.
#if 0
  for (common::Task& constraint_search_task :
      node_index_to_constraint_search_tasks_[current_node_index_]) {
    task->AddDependency(&constraint_search_task);
  }
#endif
  task->Dispatch(thread_pool_);
}

void ConstraintBuilder2D::DispatchScanMatcherConstructionAndWorkItem(
    const SubmapId& submap_id, const ProbabilityGrid* submap,
    const std::function<void()>& work_item) {
  auto it = submap_scan_matchers_.find(submap_id);
  if (it == submap_scan_matchers_.end()) {
    // TODO(gaschler): Use emplace.
    // auto insert_result =
    // submap_scan_matchers_.emplace(std::make_pair(submap_id,
    // SubmapScanMatcher())); it = insert_result.first;
    submap_scan_matchers_[submap_id];
    it = submap_scan_matchers_.find(submap_id);
    auto& submap_scan_matcher = it->second;
    submap_scan_matcher.probability_grid = submap;
    // Create scan matcher factory.
    // TODO(gaschler): Make const copy of options in constructor.
    auto& scan_matcher_options =
        options_.fast_correlative_scan_matcher_options();
    submap_scan_matcher.scan_matcher_factory.SetWorkItem(
        [&submap_scan_matcher, &scan_matcher_options]() {
          // TODO(gaschler): This doesn't need mutex_?
          LOG(INFO) << "Execute scan_matcher_factory";
          submap_scan_matcher.fast_correlative_scan_matcher =
              common::make_unique<scan_matching::FastCorrelativeScanMatcher2D>(
                  *submap_scan_matcher.probability_grid, scan_matcher_options);
        });
    LOG(INFO) << "Dispatch scan_matcher_factory";
    submap_scan_matcher.scan_matcher_factory.Dispatch(thread_pool_);
  }

  // TODO(gaschler): Delete task when done.
  auto& list = node_index_to_constraint_search_tasks_[current_node_index_];
  list.emplace_front();
  common::Task& task = list.front();
  task.SetWorkItem(work_item);
  task.AddDependency(&it->second.scan_matcher_factory);
  task.Dispatch(thread_pool_);
  LOG(INFO) << "Dispatch ComputeConstraint";
}

const ConstraintBuilder2D::SubmapScanMatcher*
ConstraintBuilder2D::GetSubmapScanMatcher(const SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  const SubmapScanMatcher* submap_scan_matcher =
      &submap_scan_matchers_[submap_id];
  CHECK(submap_scan_matcher->fast_correlative_scan_matcher != nullptr);
  return submap_scan_matcher;
}

void ConstraintBuilder2D::ComputeConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, bool match_full_submap,
    const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    std::unique_ptr<ConstraintBuilder2D::Constraint>* constraint) {
  const transform::Rigid2d initial_pose =
      ComputeSubmapPose(*submap) * initial_relative_pose;
  const SubmapScanMatcher* const submap_scan_matcher =
      GetSubmapScanMatcher(submap_id);

  // The 'constraint_transform' (submap i <- node j) is computed from:
  // - a 'filtered_gravity_aligned_point_cloud' in node j,
  // - the initial guess 'initial_pose' for (map <- node j),
  // - the result 'pose_estimate' of Match() (map <- node j).
  // - the ComputeSubmapPose() (map <- submap i)
  float score = 0.;
  transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();

  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  if (match_full_submap) {
    kGlobalConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher->fast_correlative_scan_matcher->MatchFullSubmap(
            constant_data->filtered_gravity_aligned_point_cloud,
            options_.global_localization_min_score(), &score, &pose_estimate)) {
      CHECK_GT(score, options_.global_localization_min_score());
      CHECK_GE(node_id.trajectory_id, 0);
      CHECK_GE(submap_id.trajectory_id, 0);
      kGlobalConstraintsFoundMetric->Increment();
      kGlobalConstraintScoresMetric->Observe(score);
    } else {
      return;
    }
  } else {
    kConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher->fast_correlative_scan_matcher->Match(
            initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
            options_.min_score(), &score, &pose_estimate)) {
      // We've reported a successful local match.
      CHECK_GT(score, options_.min_score());
      kConstraintsFoundMetric->Increment();
      kConstraintScoresMetric->Observe(score);
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
  ceres_scan_matcher_.Match(pose_estimate.translation(), pose_estimate,
                            constant_data->filtered_gravity_aligned_point_cloud,
                            *submap_scan_matcher->probability_grid,
                            &pose_estimate, &unused_summary);

  const transform::Rigid2d constraint_transform =
      ComputeSubmapPose(*submap).inverse() * pose_estimate;
  constraint->reset(new Constraint{submap_id,
                                   node_id,
                                   {transform::Embed3D(constraint_transform),
                                    options_.loop_closure_translation_weight(),
                                    options_.loop_closure_rotation_weight()},
                                   Constraint::INTER_SUBMAP});

  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->filtered_gravity_aligned_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      const transform::Rigid2d difference =
          initial_pose.inverse() * pose_estimate;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << std::abs(difference.normalized_angle());
    }
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}

void ConstraintBuilder2D::FinishComputation(const int node_index) {
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    common::MutexLocker locker(&mutex_);
    ++num_finished_nodes_;
    for (common::Task& constraint_search_task :
         node_index_to_constraint_search_tasks_[node_index]) {
      CHECK_EQ(constraint_search_task.GetState(), common::Task::COMPLETED);
    }
    node_index_to_constraint_search_tasks_.erase(node_index);
    LOG(WARNING) << "FinishComputation node_index=" << node_index;
#if 0
    if (--pending_computations_[computation_index] == 0) {
      num_finished_nodes_ = pending_computations_.begin()->first;
      pending_computations_.erase(computation_index);
      LOG(INFO) << "Finished computation_index=" << computation_index;
    }
#endif
    if (node_index_to_constraint_search_tasks_.empty()) {
      // CHECK_EQ(submap_queued_work_items_.size(), 0);
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
        }
        constraints_.clear();
        callback = std::move(when_done_);
        when_done_.reset();
      }
    }
    kQueueLengthMetric->Set(constraints_.size());
  }
  if (callback != nullptr) {
    LOG(ERROR) << "Call back when_done_";
    (*callback)(result);
  } else {
    LOG(ERROR) << "when_done_ == nullptr is fishy";
  }
}

int ConstraintBuilder2D::GetNumFinishedNodes() {
  common::MutexLocker locker(&mutex_);
  LOG(INFO) << "GetNumFinishedNodes ";
  LOG(INFO) << "current_node_index_ " << current_node_index_;
  LOG(INFO) << "num_finished_nodes_ " << num_finished_nodes_;
  LOG(INFO) << "pending_computations_.begin()->first "
            << node_index_to_constraint_search_tasks_.begin()->first;
  if (node_index_to_constraint_search_tasks_.empty()) {
    // CHECK_EQ(num_finished_nodes_, current_node_index_);
    return current_node_index_;
  }
  // CHECK_EQ(num_finished_nodes_, pending_computations_.begin()->first);
  return node_index_to_constraint_search_tasks_.begin()->first;
}

void ConstraintBuilder2D::DeleteScanMatcher(const SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  CHECK(node_index_to_constraint_search_tasks_.empty());
  submap_scan_matchers_.erase(submap_id);
}

void ConstraintBuilder2D::RegisterMetrics(metrics::FamilyFactory* factory) {
  auto* counts = factory->NewCounterFamily(
      "/mapping/2d/pose_graph/constraint_builder/constraints",
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
      "/mapping/2d/pose_graph/constraint_builder/queue_length", "Queue length");
  kQueueLengthMetric = queue_length->Add({});
  auto boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = factory->NewHistogramFamily(
      "/mapping/2d/pose_graph/constraint_builder/scores",
      "Constraint scores built", boundaries);
  kConstraintScoresMetric = scores->Add({{"search_region", "local"}});
  kGlobalConstraintScoresMetric = scores->Add({{"search_region", "global"}});
}

}  // namespace pose_graph
}  // namespace mapping
}  // namespace cartographer
