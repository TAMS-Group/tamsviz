// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <memory>
#include <typeindex>

class Variant {
  std::shared_ptr<void> _ptr = nullptr;
  std::type_index _type = typeid(void);

public:
  std::type_index type() const { return _type; }
  template <class T> const T &value() const {
    return *static_cast<const T *>(_ptr.get());
  }
  template <class T> void assign(const T &v) {
    _type = typeid(T);
    _ptr = std::make_shared<T>(v);
  }
  Variant() {}
  template <class T> explicit Variant(const T &v) { assign(v); }
  void clear() {
    _ptr = nullptr;
    _type = typeid(void);
  }
};
