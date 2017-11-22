#include "cartographer_grpc/framework/rpc.h"

#include "glog/logging.h"
#include "grpc++/impl/codegen/service_type.h"

namespace cartographer_grpc {
namespace framework {

Rpc::Rpc(const RpcHandlerInfo& rpc_handler_info)
    : rpc_handler_info_(rpc_handler_info) {
}

::grpc::internal::RpcMethod::RpcType Rpc::rpc_type() const {
  return rpc_handler_info_.rpc_type;
}

::grpc::internal::ServerAsyncStreamingInterface* Rpc::responder() {
  LOG(FATAL) << "Not yet implemented";
  return nullptr;
}

Rpc::RpcState* Rpc::GetState(State state) {
  switch (state) {
    case State::NEW_CONNECTION:
      return &new_connection_state_;
    case State::READ:
      return &read_state_;
    case State::WRITE:
      return &write_state_;
    case State::DONE:
      return &done_state_;
  }
  LOG(FATAL) << "Never reached.";
}

ActiveRpcs::~ActiveRpcs() {
  std::lock_guard<std::mutex> lock(mu_);
  if (!rpcs_.empty()) {
    LOG(FATAL) << "RPCs still in flight!";
  }
}

Rpc* ActiveRpcs::Add(std::unique_ptr<Rpc> rpc) {
  std::lock_guard<std::mutex> lock(mu_);
  const auto result = rpcs_.emplace(rpc.release());
  CHECK(result.second) << "RPC already active.";
  return *result.first;
}

bool ActiveRpcs::Remove(Rpc* rpc) {
  std::lock_guard<std::mutex> lock(mu_);
  auto it = rpcs_.find(rpc);
  if (it != rpcs_.end()) {
    delete rpc;
    rpcs_.erase(it);
    return true;
  } else {
    return false;
  }
}

}  // namespace framework
}  // namespace cartographer_grpc
