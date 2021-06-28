// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <tuple>

#include "event.h"

class LoaderThread {
  std::thread _thread;
  std::mutex _mutex;
  std::condition_variable _condition;
  std::deque<std::pair<std::shared_ptr<void>, std::function<void()>>> _queue;
  bool _stop = false;
  std::atomic<size_t> _count;

public:
  void start(const std::shared_ptr<void> &owner,
             const std::function<void()> &fnc);
  void cancel(const std::shared_ptr<void> &owner);
  size_t count() const { return _count; }
  LoaderThread();
  ~LoaderThread();
  Event<void()> started, finished;
  static const std::shared_ptr<LoaderThread> &instance();
};

template <class T> struct Loader {
  struct Data {
    std::shared_ptr<T> data;
    bool ready = false;
  };
  std::shared_ptr<Data> _data;
  std::function<std::shared_ptr<T>()> _constructor;
  void _load() {
    if (!_data) {
      _data = std::make_shared<Data>();
      auto d = _data;
      auto ctor = _constructor;
      LoaderThread::instance()->start(_data, [d, ctor]() {
        d->data = ctor();
        d->ready = true;
      });
    }
  }

public:
  template <class... Args> Loader(const Args &... args) {
    _constructor = [args...]() { return std::make_shared<T>(args...); };
  }
  Loader() {}
  Loader(const Loader &) = delete;
  Loader &operator=(const Loader &) = delete;
  ~Loader() { clear(); }
  std::shared_ptr<T> load() {
    _load();
    if (_data->ready) {
      return _data->data;
    } else {
      return nullptr;
    }
  }
  void clear() {
    if (_data) {
      LoaderThread::instance()->cancel(_data);
      _data = nullptr;
    }
  }
};
