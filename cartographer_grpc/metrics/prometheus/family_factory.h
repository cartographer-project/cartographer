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

#ifndef CARTOGRAPHER_GRPC_METRICS_PROMETHEUS_FAMILY_FACTORY_H_
#define CARTOGRAPHER_GRPC_METRICS_PROMETHEUS_FAMILY_FACTORY_H_

#include <memory>
#include <string>

#include "prometheus/registry.h"
#include "cartographer/metrics/family_factory.h"

namespace cartographer_grpc {
namespace metrics {
namespace prometheus {

class FamilyFactory : public cartographer::metrics::FamilyFactory {
 public:
  FamilyFactory();

  cartographer::metrics::HistogramFamily* NewHistogramFamily(
      const std::string& name, const std::string& description) override;

  std::weak_ptr<::prometheus::Collectable> GetCollectable() const;

 private:
  std::vector<std::unique_ptr<cartographer::metrics::HistogramFamily>>
      histograms_;
  std::shared_ptr<::prometheus::Registry> registry_;
};

}  // namespace prometheus
}  // namespace metrics
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_METRICS_PROMETHEUS_FAMILY_FACTORY_H_
