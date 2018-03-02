#include "cartographer_grpc/map_builder_server_interface.h"

#include "cartographer/common/make_unique.h"
#include "cartographer_grpc/internal/map_builder_server.h"

namespace cartographer {
namespace cloud {

std::unique_ptr<MapBuilderServerInterface> CreateMapBuilderServer(
    const proto::MapBuilderServerOptions& map_builder_server_options,
    std::unique_ptr<mapping::MapBuilderInterface> map_builder) {
  return common::make_unique<MapBuilderServer>(map_builder_server_options,
                                               std::move(map_builder));
}

}  // namespace cloud
}  // namespace cartographer
