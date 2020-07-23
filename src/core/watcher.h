// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "variant.h"

class Watcher {
  Variant _snapshot;

public:
  template <class... Args> bool changed(const Args &... args) {
    if (std::type_index(typeid(std::make_tuple(args...))) == _snapshot.type()) {
      auto tuple = std::make_tuple(args...);
      if (tuple != _snapshot.value<decltype(tuple)>()) {
        _snapshot = Variant(tuple);
        return true;
      }
    } else {
      _snapshot = Variant(std::make_tuple(args...));
      return true;
    }
    return false;
  }
};
