#include "cartographer/metrics/register.h"

#include "cartographer/mapping_2d/pose_graph/constraint_builder.h"
#include "cartographer/mapping_3d/pose_graph/constraint_builder.h"

namespace cartographer {
namespace metrics {

void RegisterAllMetrics(FamilyFactory *registry) {
  mapping_2d::pose_graph::ConstraintBuilder::RegisterMetrics(registry);
  mapping_3d::pose_graph::ConstraintBuilder::RegisterMetrics(registry);
}

}  // namespace metrics
}  // namespace cartographer
