#include "cartographer/cloud/map_builder_server_interface.h"

#include "cartographer/cloud/internal/map_builder_server.h"
#include "cartographer/common/make_unique.h"

namespace cartographer {
namespace cloud {

void RegisterMapBuilderServerMetrics(metrics::FamilyFactory* factory) {
  MapBuilderServer::RegisterMetrics(factory);
}

std::unique_ptr<MapBuilderServerInterface> CreateMapBuilderServer(
    const proto::MapBuilderServerOptions& map_builder_server_options,
    std::unique_ptr<mapping::MapBuilderInterface> map_builder) {
  return common::make_unique<MapBuilderServer>(map_builder_server_options,
                                               std::move(map_builder));
}

}  // namespace cloud
}  // namespace cartographer
