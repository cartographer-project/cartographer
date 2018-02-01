#include "cartographer_grpc/metrics/prometheus/family_factory.h"

#include "cartographer/common/make_unique.h"
#include "prometheus/family.h"
#include "prometheus/histogram.h"

namespace cartographer_grpc {
namespace metrics {
namespace prometheus {

namespace {

class Histogram : public cartographer::metrics::Histogram {
 public:
  Histogram(::prometheus::Histogram* prometheus) : prometheus_(prometheus) {}

  void Observe(double value) override { prometheus_->Observe(value); }

 private:
  ::prometheus::Histogram* prometheus_;
};

class HistogramFamily : public cartographer::metrics::HistogramFamily {
 public:
  HistogramFamily(::prometheus::Family<::prometheus::Histogram>* prometheus)
      : prometheus_(prometheus) {}

  Histogram* Add(const std::map<std::string, std::string>& labels,
                 const Histogram::BucketBoundaries& boundaries) override {
    ::prometheus::Histogram* histogram = &prometheus_->Add(labels, boundaries);
    auto wrapper = new Histogram(histogram);
    wrappers_.emplace_back(wrapper);
    return wrapper;
  }

 private:
  ::prometheus::Family<::prometheus::Histogram>* prometheus_;
  std::vector<std::unique_ptr<Histogram>> wrappers_;
};

}  // namespace

FamilyFactory::FamilyFactory()
    : registry_(std::make_shared<::prometheus::Registry>()) {}

cartographer::metrics::HistogramFamily* FamilyFactory::NewHistogramFamily(
    const std::string& name, const std::string& description) {
  auto& family = ::prometheus::BuildHistogram()
                     .Name(name)
                     .Help(description)
                     .Register(*registry_);
  auto wrapper = new HistogramFamily(&family);
  histograms_.emplace_back(wrapper);
  return wrapper;
}

std::weak_ptr<::prometheus::Collectable> FamilyFactory::GetCollectable() const {
  return registry_;
}

}  // namespace prometheus
}  // namespace metrics
}  // namespace cartographer_grpc
