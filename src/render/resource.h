// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "../core/event.h"

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

void loadResource(const std::string &url, std::string &data);
void loadResource(const std::string &url, std::vector<uint8_t> &data);

struct ResourceEvents {
  static ResourceEvents &instance();
  Event<void()> reload{"reload"};
};

class ResourceBase : public std::enable_shared_from_this<ResourceBase> {
  bool _invalidate_connected = false;
  EventFlag _invalidated{ResourceEvents::instance().reload};

protected:
  void cleanup(const std::function<void()> &callback);
  bool invalidated() { _invalidated.poll(); }

public:
  static void setCleanupFunction(
      const std::function<void(const std::function<void()> &)> &callback);
  ResourceBase() {}
  ResourceBase(const ResourceBase &) = delete;
  ResourceBase &operator=(const ResourceBase &) = delete;
};

template <class T, class... ARGS> class ResourceManager {
  std::mutex _mutex;
  std::map<std::tuple<ARGS...>, std::weak_ptr<T>> _map;

public:
  std::shared_ptr<T> load(const ARGS &... args) {
    std::lock_guard<std::mutex> lock(_mutex);
    auto &item = _map[std::make_tuple(args...)];
    std::shared_ptr<T> ret = item.lock();
    if (!ret) {
      ret = std::make_shared<T>(args...);
    }
    item = ret;
    return ret;
  }
  std::shared_ptr<T> fetchExisting(const ARGS &... args) {
    std::lock_guard<std::mutex> lock(_mutex);
    auto &item = _map[std::make_tuple(args...)];
    return item.lock();
  }
  void clear() {
    std::lock_guard<std::mutex> lock(_mutex);
    _map.clear();
  }
};
