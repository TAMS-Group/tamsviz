// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "document.h"
#include "snapshot.h"

#include <deque>

template <class T> struct History {
  typedef T Type;
  size_t limit = 100;
  struct Item {
    std::string label;
    std::shared_ptr<const Snapshot<T>> snapshot;
  };
  std::deque<std::shared_ptr<Item>> undo_stack, redo_stack;
  std::shared_ptr<Item> current;
  // int64_t save_counter = 0;
  bool locked = false;
  void clear() {
    undo_stack.clear();
    redo_stack.clear();
    current = nullptr;
    // save_counter = 0;
  }
  bool canUndo() const { return !undo_stack.empty(); }
  void undo(T &v) {
    if (canUndo() && !locked) {
      redo_stack.push_front(current);
      current = undo_stack.back();
      undo_stack.pop_back();
      // save_counter--;
      if (undo_stack.size() + redo_stack.size() > limit) {
        redo_stack.pop_back();
      }
      current->snapshot->apply(v);
    }
  }
  bool canRedo() const { return !redo_stack.empty(); }
  void redo(T &v) {
    if (canRedo() && !locked) {
      undo_stack.push_back(current);
      // save_counter++;
      current = redo_stack.front();
      redo_stack.pop_front();
      if (undo_stack.size() + redo_stack.size() > limit) {
        undo_stack.pop_front();
      }
      current->snapshot->apply(v);
    }
  }
};
