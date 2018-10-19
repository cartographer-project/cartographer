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

#include "absl/container/flat_hash_map.h"
#include "absl/memory/memory.h"
#include "prometheus/counter.h"
#include "prometheus/family.h"
#include "prometheus/gauge.h"
#include "prometheus/histogram.h"

namespace cartographer {
namespace cloud {
namespace metrics {
namespace prometheus {
namespace {

using BucketBoundaries = ::cartographer::metrics::Histogram::BucketBoundaries;

// Creates or looks up already existing objects from a wrapper map.
template <typename WrapperMap,
          typename ObjectPtr = typename WrapperMap::key_type,
          typename Wrapper = typename WrapperMap::mapped_type::element_type>
Wrapper* GetOrCreateWrapper(ObjectPtr object_ptr, WrapperMap* wrapper_map,
                            std::mutex* wrapper_mutex) {
  std::lock_guard<std::mutex> lock(*wrapper_mutex);
  auto wrappers_itr = wrapper_map->find(object_ptr);
  if (wrappers_itr == wrapper_map->end()) {
    auto wrapper = absl::make_unique<Wrapper>(object_ptr);
    auto* ptr = wrapper.get();
    (*wrapper_map)[object_ptr] = std::unique_ptr<Wrapper>(std::move(wrapper));
    return ptr;
  }
  return wrappers_itr->second.get();
}

class Counter : public ::cartographer::metrics::Counter {
 public:
  explicit Counter(::prometheus::Counter* prometheus)
      : prometheus_(prometheus) {}

  void Increment() override { prometheus_->Increment(); }
  void Increment(double by_value) override { prometheus_->Increment(by_value); }

 private:
  ::prometheus::Counter* prometheus_;
};

class CounterFamily
    : public ::cartographer::metrics::Family<::cartographer::metrics::Counter> {
 public:
  explicit CounterFamily(
      ::prometheus::Family<::prometheus::Counter>* prometheus)
      : prometheus_(prometheus) {}

  Counter* Add(const std::map<std::string, std::string>& labels) override {
    ::prometheus::Counter* counter = &prometheus_->Add(labels);
    return GetOrCreateWrapper<>(counter, &wrappers_, &wrappers_mutex_);
  }

 private:
  ::prometheus::Family<::prometheus::Counter>* prometheus_;
  std::mutex wrappers_mutex_;
  absl::flat_hash_map<::prometheus::Counter*, std::unique_ptr<Counter>>
      wrappers_;
};

class Gauge : public ::cartographer::metrics::Gauge {
 public:
  explicit Gauge(::prometheus::Gauge* prometheus) : prometheus_(prometheus) {}

  void Decrement() override { prometheus_->Decrement(); }
  void Decrement(double by_value) override { prometheus_->Decrement(by_value); }
  void Increment() override { prometheus_->Increment(); }
  void Increment(double by_value) override { prometheus_->Increment(by_value); }
  void Set(double value) override { prometheus_->Set(value); }

 private:
  ::prometheus::Gauge* prometheus_;
};

class GaugeFamily
    : public ::cartographer::metrics::Family<::cartographer::metrics::Gauge> {
 public:
  explicit GaugeFamily(::prometheus::Family<::prometheus::Gauge>* prometheus)
      : prometheus_(prometheus) {}

  Gauge* Add(const std::map<std::string, std::string>& labels) override {
    ::prometheus::Gauge* gauge = &prometheus_->Add(labels);
    return GetOrCreateWrapper<>(gauge, &wrappers_, &wrappers_mutex_);
  }

 private:
  ::prometheus::Family<::prometheus::Gauge>* prometheus_;
  std::mutex wrappers_mutex_;
  absl::flat_hash_map<::prometheus::Gauge*, std::unique_ptr<Gauge>> wrappers_;
};

class Histogram : public ::cartographer::metrics::Histogram {
 public:
  explicit Histogram(::prometheus::Histogram* prometheus)
      : prometheus_(prometheus) {}

  void Observe(double value) override { prometheus_->Observe(value); }

 private:
  ::prometheus::Histogram* prometheus_;
};

class HistogramFamily : public ::cartographer::metrics::Family<
                            ::cartographer::metrics::Histogram> {
 public:
  HistogramFamily(::prometheus::Family<::prometheus::Histogram>* prometheus,
                  const BucketBoundaries& boundaries)
      : prometheus_(prometheus), boundaries_(boundaries) {}

  Histogram* Add(const std::map<std::string, std::string>& labels) override {
    ::prometheus::Histogram* histogram = &prometheus_->Add(labels, boundaries_);
    return GetOrCreateWrapper<>(histogram, &wrappers_, &wrappers_mutex_);
  }

 private:
  ::prometheus::Family<::prometheus::Histogram>* prometheus_;
  std::mutex wrappers_mutex_;
  absl::flat_hash_map<::prometheus::Histogram*, std::unique_ptr<Histogram>>
      wrappers_;
  const BucketBoundaries boundaries_;
};

}  // namespace

FamilyFactory::FamilyFactory()
    : registry_(std::make_shared<::prometheus::Registry>()) {}

::cartographer::metrics::Family<::cartographer::metrics::Counter>*
FamilyFactory::NewCounterFamily(const std::string& name,
                                const std::string& description) {
  auto& family = ::prometheus::BuildCounter()
                     .Name(name)
                     .Help(description)
                     .Register(*registry_);
  auto wrapper = absl::make_unique<CounterFamily>(&family);
  auto* ptr = wrapper.get();
  counters_.emplace_back(std::move(wrapper));
  return ptr;
}

::cartographer::metrics::Family<::cartographer::metrics::Gauge>*
FamilyFactory::NewGaugeFamily(const std::string& name,
                              const std::string& description) {
  auto& family = ::prometheus::BuildGauge()
                     .Name(name)
                     .Help(description)
                     .Register(*registry_);
  auto wrapper = absl::make_unique<GaugeFamily>(&family);
  auto* ptr = wrapper.get();
  gauges_.emplace_back(std::move(wrapper));
  return ptr;
}

::cartographer::metrics::Family<::cartographer::metrics::Histogram>*
FamilyFactory::NewHistogramFamily(const std::string& name,
                                  const std::string& description,
                                  const BucketBoundaries& boundaries) {
  auto& family = ::prometheus::BuildHistogram()
                     .Name(name)
                     .Help(description)
                     .Register(*registry_);
  auto wrapper = absl::make_unique<HistogramFamily>(&family, boundaries);
  auto* ptr = wrapper.get();
  histograms_.emplace_back(std::move(wrapper));
  return ptr;
}

std::weak_ptr<::prometheus::Collectable> FamilyFactory::GetCollectable() const {
  return registry_;
}

}  // namespace prometheus
}  // namespace metrics
}  // namespace cloud
}  // namespace cartographer
