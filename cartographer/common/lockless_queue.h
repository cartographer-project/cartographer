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
#include <memory>

#include "absl/memory/memory.h"
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
    data_list_head_ = nullptr;
    data_list_tail_ = nullptr;
  }

  ~LocklessQueue() {
    FreeNodes(free_list_head_.exchange(nullptr));
    FreeNodes(incoming_data_list_head_.exchange(nullptr));
    FreeNodes(data_list_head_);
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
    SwapLists();
    if (data_list_head_ != nullptr) {
      Node* node = data_list_head_;
      data_list_head_ = data_list_head_->next;
      std::unique_ptr<T> data = std::move(node->data);
      PushNodeToList(&free_list_head_, node);
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
  void FreeNodes(Node* node) {
    while (node != nullptr) {
      Node* next_node_ptr = node->next;
      delete node;
      node = next_node_ptr;
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

  // Swaps the incoming data list for an empty list and appends all items
  // to 'data_list_tail_'.
  void SwapLists() {
    Node* node_itr = incoming_data_list_head_.exchange(nullptr);
    if (node_itr == nullptr) {
      // There is no data on the incoming list.
      return;
    }
    // The first node of the incoming data list will become the tail of the
    // data list.
    Node* const data_list_tail = node_itr;

    // Reverses the list order. After this operation 'prev_node_itr' points to
    // head of the new data list items.
    Node* prev_node_itr = nullptr;
    while (node_itr != nullptr) {
      Node* const next_node_ptr = node_itr->next;
      node_itr->next = prev_node_itr;
      prev_node_itr = node_itr;
      node_itr = next_node_ptr;
    }

    // If the previous data list was empty, replace head rather than appending
    // to the list.
    if (data_list_tail_ == nullptr) {
      data_list_head_ = prev_node_itr;
    } else {
      data_list_tail_->next = prev_node_itr;
    }
    data_list_tail_ = data_list_tail;
  }

  // Pointer to head node of free list.
  std::atomic<Node*> free_list_head_;

  // Pointer to head node of incoming data list, which is in FILO order.
  std::atomic<Node*> incoming_data_list_head_;

  // Pointer to head node of data list.
  Node* data_list_head_;

  // Pointer to tail node of data list.
  Node* data_list_tail_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_LOCKLESS_QUEUE_H_
