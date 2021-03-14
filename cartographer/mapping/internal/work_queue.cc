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
    if (characterization.queue_distrobution.count(work_item.work_item_type)) {
      characterization.queue_distrobution[work_item.work_item_type] += 1;
    }
    characterization.queue_distrobution[work_item.work_item_type] = 1;
  }
  return characterization;
}

}  // namespace cartographer::mapping
