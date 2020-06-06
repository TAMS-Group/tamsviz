// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include <functional>

class Destructor {
  std::function<void()> fn;

public:
  Destructor() {}
  Destructor(const Destructor &) = delete;
  Destructor &operator=(const Destructor &) = delete;
  Destructor(const std::function<void()> &fn) : fn(fn) {}
  Destructor &operator=(const std::function<void()> &f) { fn = f; }
  ~Destructor() {
    if (fn) {
      fn();
    }
  }
};
