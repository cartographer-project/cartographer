/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_COMMON_LOCKLESS_QUEUE_H_
#define CARTOGRAPHER_COMMON_LOCKLESS_QUEUE_H_

#include <atomic>
#include <list>
#include <memory>

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

// Lock-less queue which is thread safe for concurrent data producers and a
// single data consumer.
//
// This lockless queue implementation is adapted from
// https://github.com/resonance-audio/resonance-audio/blob/master/resonance_audio/utils/lockless_task_queue.h
template <typename T>
class LocklessQueue {
 public:
  LocklessQueue() {
    free_list_head_ = nullptr;
    incoming_data_list_head_ = nullptr;
  }

  ~LocklessQueue() {
    FreeNodes(&free_list_head_);
    FreeNodes(&incoming_data_list_head_);
  }

  // Pushes the data item into the queue.
  void Push(std::unique_ptr<T> t) {
    Node* const free_node = PopNodeFromFreeList();
    CHECK(free_node);
    free_node->data = std::move(t);
    PushNodeToList(&incoming_data_list_head_, free_node);
  }

  // Pops the oldest data item from the queue. If the queue is empty returns a
  // nullptr.
  std::unique_ptr<T> Pop() {
    PopAllDataNodes();
    if (outgoing_data_items_.size() > 0u) {
      std::unique_ptr<T> data = std::move(outgoing_data_items_.front());
      outgoing_data_items_.pop_front();
      return std::move(data);
    }
    return nullptr;
  }

 private:
  // Node to model a single-linked list.
  struct Node {
    Node() = default;

    // Dummy copy constructor to enable vector::resize allocation.
    Node(const Node& node) : next() {}

    // User data.
    std::unique_ptr<T> data;

    // Pointer to next node.
    std::atomic<Node*> next;
  };

  // Deallocates all nodes of the list starting with 'head'.
  void FreeNodes(std::atomic<Node*>* head) {
    Node* node_itr = head->exchange(nullptr);
    while (node_itr != nullptr) {
      Node* next_node_ptr = node_itr->next;
      delete node_itr;
      node_itr = next_node_ptr;
    }
  }

  // Pushes a node to the front of a list.
  void PushNodeToList(std::atomic<Node*>* list_head, Node* node) {
    DCHECK(list_head);
    DCHECK(node);
    Node* list_head_ptr;
    do {
      list_head_ptr = list_head->load();
      node->next = list_head_ptr;
    } while (!std::atomic_compare_exchange_strong_explicit(
        list_head, &list_head_ptr, node, std::memory_order_release,
        std::memory_order_relaxed));
  }

  // Pops a node from the front of the free node list. If the list is empty
  // constructs a new node instance.
  Node* PopNodeFromFreeList() {
    Node* list_head_ptr;
    Node* list_head_next_ptr;
    do {
      list_head_ptr = free_list_head_.load();
      if (list_head_ptr == nullptr) {
        return new Node;
      }
      list_head_next_ptr = list_head_ptr->next.load();
    } while (!std::atomic_compare_exchange_strong_explicit(
        &free_list_head_, &list_head_ptr, list_head_next_ptr,
        std::memory_order_relaxed, std::memory_order_relaxed));
    return list_head_ptr;
  }

  // Pops all data nodes from the list thread-safe
  void PopAllDataNodes() {
    Node* node_itr = incoming_data_list_head_.exchange(nullptr);
    std::list<std::unique_ptr<T>> temp_data_list;
    while (node_itr != nullptr) {
      Node* next_node_ptr = node_itr->next;
      temp_data_list.push_front(std::move(node_itr->data));
      node_itr->data = nullptr;
      PushNodeToList(&free_list_head_, node_itr);
      node_itr = next_node_ptr;
    }
    outgoing_data_items_.splice(outgoing_data_items_.end(), temp_data_list);
  }

  // Pointer to head node of free list.
  std::atomic<Node*> free_list_head_;

  // Pointer to head node of data list.
  std::atomic<Node*> incoming_data_list_head_;

  // List of data items in FIFO order.
  std::list<std::unique_ptr<T>> outgoing_data_items_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_LOCKLESS_QUEUE_H_