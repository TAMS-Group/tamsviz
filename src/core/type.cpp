// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "type.h"

#include "object.h"

#include <atomic>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

uint64_t handleObjectId(const Object *object) {
  return object ? object->id() : 0;
}

struct TypeRegistry {
  std::mutex mutex;
  std::unordered_map<std::string, std::shared_ptr<Type>> names;
  std::unordered_map<std::type_index, std::shared_ptr<Type>> ids;
  static const std::shared_ptr<TypeRegistry> &instance() {
    static auto ret = std::make_shared<TypeRegistry>();
    return ret;
  }
};

void Type::_registerType(const std::shared_ptr<Type> &t) {
  auto reg = TypeRegistry::instance();
  std::unique_lock<std::mutex> lock(reg->mutex);
  reg->names[t->name()] = t;
  reg->ids[t->typeId()] = t;
}

std::shared_ptr<Type> Type::tryFind(const std::string &name) {
  auto reg = TypeRegistry::instance();
  std::unique_lock<std::mutex> lock(reg->mutex);
  auto iter = reg->names.find(name);
  if (iter != reg->names.end()) {
    return iter->second;
  } else {
    return nullptr;
  }
}

std::shared_ptr<Type> Type::find(const std::string &name) {
  if (auto ret = tryFind(name)) {
    return ret;
  } else {
    throw std::runtime_error("type not found: " + name);
  }
}

std::shared_ptr<Type> Type::tryFind(const std::type_index &id) {
  auto reg = TypeRegistry::instance();
  std::unique_lock<std::mutex> lock(reg->mutex);
  auto iter = reg->ids.find(id);
  if (iter != reg->ids.end()) {
    return iter->second;
  } else {
    return nullptr;
  }
}

std::shared_ptr<Type> Type::find(const std::type_index &id) {
  if (auto ret = tryFind(id)) {
    return ret;
  } else {
    throw std::runtime_error(std::string() + "type not found: " + id.name());
  }
}

std::vector<std::shared_ptr<Type>> Type::list() const {
  auto reg = TypeRegistry::instance();
  std::unique_lock<std::mutex> lock(reg->mutex);
  std::vector<std::shared_ptr<Type>> ret;
  for (auto &p : reg->ids) {
    auto type = p.second;
    for (auto base = type; base; base = base->base()) {
      if (base.get() == this) {
        ret.push_back(type);
        break;
      }
    }
  }
  return ret;
}
