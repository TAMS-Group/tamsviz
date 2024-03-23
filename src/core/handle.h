// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include <cstdint>
#include <memory>
#include <string>

class Object;

uint64_t handleObjectId(const Object *object);

template <class T> class Handle {
  uint64_t _id = 0;

public:
  Handle(const std::shared_ptr<T> &p = nullptr) { reset(p); }
  void reset(uint64_t id) { _id = id; }
  void reset(const std::shared_ptr<T> &p = nullptr) {
    _id = handleObjectId(p ? (Object *)p.get() : nullptr);
  }
  void operator=(const std::shared_ptr<T> &p) { reset(p); }
  template <class R>
  std::shared_ptr<T> resolve(const std::shared_ptr<R> &root) {
    if (_id == 0) {
      return nullptr;
    }
    std::shared_ptr<T> r;
    root->recurseObjects([&r, this](const std::shared_ptr<Object> &o) {
      if (handleObjectId(o.get()) == _id) {
        if (auto x = std::dynamic_pointer_cast<T>(o)) {
          r = x;
        }
      }
    });
    return r;
  }
  const uint64_t id() const { return _id; }
};

template <class T> uint64_t getHandleId(const std::shared_ptr<T> &p) {
  return p ? p->id() : uint64_t(0);
}

template <class T> uint64_t getHandleId(const Handle<T> &h) { return h.id(); }

#define HANDLE_COMPARE_OP(op)                                                  \
                                                                               \
  template <class L, class R>                                                  \
  bool operator op(const Handle<L> &l, const std::shared_ptr<R> &r) {          \
    return getHandleId(l) op getHandleId(r);                                   \
  }                                                                            \
                                                                               \
  template <class L, class R>                                                  \
  bool operator op(const std::shared_ptr<L> &l, const Handle<R> &r) {          \
    return getHandleId(l) op getHandleId(r);                                   \
  }                                                                            \
                                                                               \
  template <class L, class R>                                                  \
  bool operator op(const Handle<L> &l, const Handle<R> &r) {                   \
    return getHandleId(l) op getHandleId(r);                                   \
  }

HANDLE_COMPARE_OP(==)
HANDLE_COMPARE_OP(!=)
HANDLE_COMPARE_OP(<)
HANDLE_COMPARE_OP(>)
HANDLE_COMPARE_OP(<=)
HANDLE_COMPARE_OP(>=)
