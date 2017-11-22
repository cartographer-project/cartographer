#include "cartographer_grpc/framework/server.h"

namespace cartographer_grpc {
namespace framework {

Service::Service(const std::map<std::string, RpcHandlerInfo>& rpc_handlers)
    : rpc_handlers_(rpc_handlers) {}

}  // namespace framework
}  // namespace cartographer_grpc
