// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

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

class Transformer {
  struct Data;
  std::shared_ptr<Data> _data;

public:
  Transformer(bool subscribe = true);
  Transformer(const Transformer &) = delete;
  Transformer &operator=(const Transformer &) = delete;
  Optional<Eigen::Isometry3d> lookup(const std::string &frame);
  void update(const std::string &root);
  void clear();
  void push(const std::shared_ptr<Message> &message);
  std::vector<std::string> list();
};
