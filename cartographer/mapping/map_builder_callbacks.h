#pragma once

namespace cartographer::mapping {

struct MapBuilderCallbacks {
  std::function<void()> loop_closure_cb{nullptr};
};

}  // namespace cartographer::mapping
