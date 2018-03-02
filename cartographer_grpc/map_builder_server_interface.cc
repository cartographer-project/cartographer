#include "cartographer_grpc/map_builder_server_interface.h"

#include "cartographer/common/make_unique.h"
#include "cartographer_grpc/internal/map_builder_server.h"

namespace cartographer_grpc {

std::unique_ptr<MapBuilderServerInterface> CreateMapBuilderServer(
    const proto::MapBuilderServerOptions& map_builder_server_options,
    std::unique_ptr<::cartographer::mapping::MapBuilderInterface> map_builder) {
  return ::cartographer::common::make_unique<MapBuilderServer>(
      map_builder_server_options, std::move(map_builder));
}

}  // namespace cartographer_grpc
