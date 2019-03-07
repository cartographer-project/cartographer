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

#include "cartographer/cloud/metrics/prometheus/family_factory.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/metrics/register.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "prometheus/exposer.h"
#include "prometheus/metric_family.h"

namespace cartographer {
namespace cloud {
namespace metrics {
namespace prometheus {
namespace {

using Label = ::prometheus::ClientMetric::Label;

static auto* kCounter = ::cartographer::metrics::Counter::Null();
static auto* kGauge = ::cartographer::metrics::Gauge::Null();
static auto* kScoresMetric = ::cartographer::metrics::Histogram::Null();

const char kLabelKey[] = "kind";
const char kLabelValue[] = "score";
const std::array<double, 5> kObserveScores = {{-1, 0.11, 0.2, 0.5, 2}};

class Algorithm {
 public:
  static void RegisterMetrics(::cartographer::metrics::FamilyFactory* factory) {
    auto boundaries = ::cartographer::metrics::Histogram::FixedWidth(0.05, 20);
    auto* scores_family = factory->NewHistogramFamily(
        "/algorithm/scores", "Scores achieved", boundaries);
    kScoresMetric = scores_family->Add({{kLabelKey, kLabelValue}});
  }
  void Run() {
    for (double score : kObserveScores) {
      kScoresMetric->Observe(score);
    }
  }
};

TEST(MetricsTest, CollectCounter) {
  FamilyFactory factory;
  auto* counter_family = factory.NewCounterFamily("/test/hits", "Hits");
  kCounter = counter_family->Add({{kLabelKey, kLabelValue}});
  kCounter->Increment();
  kCounter->Increment(5);
  double expected_value = 1 + 5;
  std::vector<::prometheus::MetricFamily> collected;
  {
    std::shared_ptr<::prometheus::Collectable> collectable;
    CHECK(collectable = factory.GetCollectable().lock());
    collected = collectable->Collect();
  }
  ASSERT_EQ(collected.size(), 1);
  ASSERT_EQ(collected[0].metric.size(), 1);
  EXPECT_THAT(
      collected[0].metric.at(0).label,
      testing::AllOf(
          testing::ElementsAre(testing::Field(&Label::name, kLabelKey)),
          testing::ElementsAre(testing::Field(&Label::value, kLabelValue))));
  EXPECT_THAT(collected[0].metric.at(0).counter.value,
              testing::DoubleEq(expected_value));
}

TEST(MetricsTest, CollectGauge) {
  FamilyFactory factory;
  auto* gauge_family =
      factory.NewGaugeFamily("/test/queue/length", "Length of some queue");
  kGauge = gauge_family->Add({{kLabelKey, kLabelValue}});
  kGauge->Increment();
  kGauge->Increment(5);
  kGauge->Decrement();
  kGauge->Decrement(2);
  double expected_value = 1 + 5 - 1 - 2;
  std::vector<::prometheus::MetricFamily> collected;
  {
    std::shared_ptr<::prometheus::Collectable> collectable;
    CHECK(collectable = factory.GetCollectable().lock());
    collected = collectable->Collect();
  }
  ASSERT_EQ(collected.size(), 1);
  ASSERT_EQ(collected[0].metric.size(), 1);
  EXPECT_THAT(
      collected[0].metric.at(0).label,
      testing::AllOf(
          testing::ElementsAre(testing::Field(&Label::name, kLabelKey)),
          testing::ElementsAre(testing::Field(&Label::value, kLabelValue))));
  EXPECT_THAT(collected[0].metric.at(0).gauge.value,
              testing::DoubleEq(expected_value));
}

TEST(MetricsTest, CollectHistogram) {
  FamilyFactory registry;
  Algorithm::RegisterMetrics(&registry);

  Algorithm algorithm;
  algorithm.Run();
  std::vector<::prometheus::MetricFamily> collected;
  {
    std::shared_ptr<::prometheus::Collectable> collectable;
    CHECK(collectable = registry.GetCollectable().lock());
    collected = collectable->Collect();
  }
  ASSERT_EQ(collected.size(), 1);
  ASSERT_EQ(collected[0].metric.size(), 1);
  EXPECT_THAT(
      collected[0].metric.at(0).label,
      testing::AllOf(
          testing::ElementsAre(testing::Field(&Label::name, kLabelKey)),
          testing::ElementsAre(testing::Field(&Label::value, kLabelValue))));
  EXPECT_THAT(collected[0].metric.at(0).histogram.sample_count,
              testing::Eq(kObserveScores.size()));
  EXPECT_EQ(collected[0].metric.at(0).histogram.bucket.at(0).cumulative_count,
            1);
}

TEST(MetricsTest, RunExposerServer) {
  FamilyFactory registry;
  Algorithm::RegisterMetrics(&registry);
  ::cartographer::metrics::RegisterAllMetrics(&registry);
  ::prometheus::Exposer exposer("0.0.0.0:9100");
  exposer.RegisterCollectable(registry.GetCollectable());

  Algorithm algorithm;
  algorithm.Run();
}

}  // namespace
}  // namespace prometheus
}  // namespace metrics
}  // namespace cloud
}  // namespace cartographer
