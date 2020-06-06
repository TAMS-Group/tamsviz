// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "type.h"

#include <atomic>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

struct TypeRegistry {
  std::mutex mutex;
  std::unordered_map<std::string, std::shared_ptr<Type>> names;
  std::unordered_map<std::type_index, std::shared_ptr<Type>> ids;
  static const std::shared_ptr<TypeRegistry> &instance() {
    static auto ret = std::make_shared<TypeRegistry>();
    return ret;
  }
};

void getObjectProperties(const std::shared_ptr<Object> &v,
                         std::vector<Property> &props) {
  props.clear();
  if (v) {
    for (auto &prop : v->properties()) {
      props.push_back(prop);
    }
  }
}

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

thread_local int g_properties_unlocked_counter = 0;
void Property::checkUnlocked() {
  if (g_properties_unlocked_counter <= 0) {
    throw std::runtime_error("Use LockScope to safely access properties");
  }
}
void Property::unlockScope(int i) { g_properties_unlocked_counter += i; }

void forEachObject(void *context,
                   void (*f)(void *, const Object *, const Object *),
                   const Object *parent, const Object *child) {
  if (!child) {
    return;
  }
  f(context, parent, child);
  for (auto &property : child->objectProperties()) {
    property.forEachObject(context, f, child);
  }
}

Object::Object() {
  if (g_properties_unlocked_counter <= 0) {
    throw std::runtime_error("Use LockScope to safely construct Objects");
  }
  assignNewId();
  // registerObject(this);
  // static std::vector<std::shared_ptr<Object>> g_object_ptr_test;
  // g_object_ptr_test.push_back(shared_from_this());
}
void Object::assignNewId() {
  static std::atomic<uint64_t> counter;
  _id = (++counter);
  if (g_properties_unlocked_counter <= 0) {
    throw std::runtime_error("Use LockScope to safely call assignNewId");
  }
}
Object::~Object() {
  // deregisterObject(this);
}
void Object::setId(uint64_t id) {
  // deregisterObject(this);
  _id = id;
  // registerObject(this);
}

void removeObject(const std::shared_ptr<Object> &parent, void *object) {
  if (parent) {
    for (auto &property : parent->properties()) {
      property.remove(object);
    }
  }
}
