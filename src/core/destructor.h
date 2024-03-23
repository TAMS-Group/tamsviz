// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include <functional>

class Destructor {
  std::function<void()> fn;

public:
  Destructor() {}
  Destructor(const Destructor &) = delete;
  Destructor &operator=(const Destructor &) = delete;
  Destructor(const std::function<void()> &fn) : fn(fn) {}
  Destructor &operator=(const std::function<void()> &f) {
    fn = f;
    return *this;
  }
  ~Destructor() {
    if (fn) {
      fn();
    }
  }
};
