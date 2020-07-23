// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "property.h"

#include <map>
#include <memory>
#include <mutex>

#include <Eigen/Dense>

class Message;
template <class M> class Subscriber;

template <class T> class Optional {
  T _value;
  bool _valid = false;
  inline void check() const {
    if (!_valid) {
      throw std::runtime_error("invalid optional");
    }
  }

public:
  inline Optional() : _valid(false) {}
  inline Optional(const T &value) : _value(value), _valid(true) {}
  inline operator bool() const { return _valid; }
  inline const T &value() const {
    check();
    return _value;
  }
  inline const T &operator*() const {
    check();
    return _value;
  }
  inline const T *operator->() const {
    check();
    return &_value;
  }
};

class Frame;

class Transformer {
  struct Data;
  std::shared_ptr<Data> _data;
  std::string _root_name;

public:
  Transformer(bool subscribe = true);
  Transformer(const Transformer &) = delete;
  Transformer &operator=(const Transformer &) = delete;
  void update(const std::string &root);
  void clear();
  void push(const std::shared_ptr<const Message> &message);
  const std::string &root() const { return _root_name; }
  std::vector<std::string> list();
  friend class Frame;
};

class Frame {
  bool _active = false;
  std::string _name;

public:
  Frame();
  explicit Frame(const std::string &name);
  Frame(const Frame &other);
  Frame &operator=(const Frame &other);
  ~Frame();
  Optional<Eigen::Isometry3d>
  pose(const std::shared_ptr<Transformer> &transformer);
  const std::string &name() const { return _name; }
  void name(const std::string &name);
  bool empty() const { return _name.empty(); }
};

inline void toString(const Frame &v, std::string &s) { s = v.name(); }
inline void fromString(Frame &v, const std::string &s) { v.name(s); }
inline bool operator==(const Frame &a, const Frame &b) {
  return a.name() == b.name();
}
inline bool operator!=(const Frame &a, const Frame &b) {
  return a.name() != b.name();
}
template <> struct DefaultPropertyAttributes<Frame> {
  static void initialize(PropertyAttributes *attributes);
};
