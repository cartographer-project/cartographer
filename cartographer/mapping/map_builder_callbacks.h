#pragma once

#include <optional>

#include "cartographer/mapping/internal/constraints/constraint_builder_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/fast_correlative_scan_matcher_3d.h"

namespace cartographer::mapping {

struct MapBuilderCallbacks {
  std::function<void(
    scan_matching::FastCorrelativeScanMatcher3D::Result,  // Course search
    std::optional<constraints::ConstraintBuilder3D::Constraint> 
  )> loop_closure_cb{nullptr};
};

}  // namespace cartographer::mapping
