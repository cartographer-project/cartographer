#include "work_queue.h"

namespace cartographer::mapping {

WorkQueueCharacterization characterize(const std::unique_ptr<WorkQueue>& queue) {
  WorkQueueCharacterization characterization;
  if (!queue) {
    std::cerr << "Work queue == nullptr" << std::endl;
    return characterization;
  }
  characterization.front_of_queue_time = queue->front().time;
  for (auto& work_item : *queue) {
    if (characterization.queue_distribution.count(work_item.work_item_type)) {
      characterization.queue_distribution[work_item.work_item_type] += 1;
    }
    characterization.queue_distribution[work_item.work_item_type] = 1;
  }
  return characterization;
}

std::string to_string(WorkItemType t) {
  switch(t) {
    case CHANGE_TRAJECTORY_STATE: 
      return "change_trajectory_state";
    case OPTIMIZATION_ADD_IMU_DATA:
      return "optimization_add_imu_data";
    case OPTIMIZATION_ADD_ODOM_DATA: 
      return "optimization_add_odom_data";
    case OPTIMIZATION_ADD_LANDMARK_DATA:
      return "optimization_add_landmark_data";
    case OPTIMIZATION_ADD_FIXED_FRAME_DATA:
      return "optimization_add_fixed_frame_data";
    case OPTIMIZATION_RUN_FINAL: 
      return "optimization_solve";
    case OPTIMIZATION_INSERT_SUBMAP: 
      return "optimization_insert_submap";
    case COMPUTE_CONSTRAINTS:
      return "compute_constraints";
    case NODE_TRAJECTORY_INSERTION: 
      return "node_trajectory_insertion";
    case NODE_SUBMAP_INSERTION:
      return "node_submap_insertion";
    case UNLABELED_ITEM:
      return "unlabeled";
    default:
      return "unlabeled";
  }
  return "unlabeled";
}


}  // namespace cartographer::mapping
