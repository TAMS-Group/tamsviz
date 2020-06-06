// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include <atomic>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <vector>

class QObject;
template <class T> class QPointer;

class EventBase {
  std::string name;

public:
  EventBase() {}
  EventBase(const std::string &name) : name(name) {}
  static bool
  listenerCheckQPointer(const std::shared_ptr<QPointer<const QObject>> &q);
  static std::shared_ptr<QPointer<const QObject>>
  listenerMakeQPointer(const QObject *o);
  void begin();
  void end();
};

template <class FNC> class Event : public EventBase {

  struct Listener {
    std::function<FNC> callback;
    virtual bool check() const = 0;
    template <class... ARGS> void call(const ARGS &... args) const {
      callback(args...);
    }
    virtual std::shared_ptr<void> pin() { return nullptr; }
    Listener(const std::function<FNC> &callback) : callback(callback) {}
    virtual ~Listener() {}
  };
  std::list<std::shared_ptr<Listener>> listeners;

  struct Listener0 : Listener {
    bool check() const override { return true; }
    Listener0(const std::function<FNC> &callback) : Listener(callback) {}
  };

  template <class T> struct ListenerP : Listener {
    std::weak_ptr<T> instance;
    ListenerP(const std::shared_ptr<T> &instance,
              const std::function<FNC> &callback)
        : Listener(callback), instance(instance) {}
    bool check() const override {
      std::shared_ptr<T> ptr = instance.lock();
      return ptr != nullptr;
    }
    virtual std::shared_ptr<void> pin() { return instance.lock(); }
  };

  struct ListenerQ : Listener {
    std::shared_ptr<QPointer<const QObject>> instance;
    ListenerQ(const std::shared_ptr<QPointer<const QObject>> &instance,
              const std::function<FNC> &callback)
        : Listener(callback), instance(instance) {}
    bool check() const override {
      return EventBase::listenerCheckQPointer(instance);
    }
  };

  std::recursive_mutex mutex;

public:
  Event() {}
  Event(const std::string &name) : EventBase(name) {}

  void clear() {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    listeners.clear();
  }

  void connect(const std::function<FNC> &callback) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    listeners.push_back(std::make_shared<Listener0>(callback));
  }

  template <class T>
  void connect(const std::shared_ptr<T> &instance,
               const std::function<FNC> &callback) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    listeners.push_back(std::make_shared<ListenerP<T>>(instance, callback));
  }

  void connect(const QObject *obj, const std::function<FNC> &callback) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    listeners.push_back(
        std::make_shared<ListenerQ>(listenerMakeQPointer(obj), callback));
  }

  template <class... ARGS> void operator()(const ARGS &... args) {
    begin();
    std::vector<std::shared_ptr<Listener>> ll;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex);
      ll.reserve(listeners.size());
      for (auto it = listeners.begin(); it != listeners.end();) {
        if ((*it)->check()) {
          ll.push_back(*it);
          it++;
        } else {
          it = listeners.erase(it);
        }
      }
    }
    for (auto &l : ll) {
      auto ptr = l->pin();
      if (l->check()) {
        l->call(args...);
      }
    }
    end();
  }
};

class EventFlag {
  struct Data {
    std::atomic<int> value;
    Data() { value = 0; }
  };
  std::shared_ptr<Data> _data = std::make_shared<Data>();

public:
  EventFlag(Event<void()> &event) {
    Data *ptr = _data.get();
    event.connect(_data, [ptr]() { ptr->value = 1; });
  }
  bool poll() { return _data->value.exchange(0); }
};
