#include "cartographer/cloud/map_builder_server_interface.h"

#include "cartographer/cloud/internal/map_builder_server.h"
#include "cartographer/common/make_unique.h"

namespace cartographer {
namespace cloud {

void RegisterMapBuilderServerMetrics(metrics::FamilyFactory* factory) {
  MapBuilderServer::RegisterMetrics(factory);
}

}  // namespace cloud
}  // namespace cartographer
