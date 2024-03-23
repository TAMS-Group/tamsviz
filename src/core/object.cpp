// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "object.h"

#include <mutex>
#include <vector>

void getObjectProperties(const std::shared_ptr<Object> &v,
                         std::vector<Property> &props) {
  props.clear();
  if (v) {
    for (auto &prop : v->properties()) {
      props.push_back(prop);
    }
  }
}

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

class ObjectIDFactory {
  std::mutex _mutex;
  uint64_t _counter = 1;

public:
  static ObjectIDFactory *instance() {
    static ObjectIDFactory instance;
    return &instance;
  }
  uint64_t generate() {
    std::lock_guard<std::mutex> lock(_mutex);
    return ++_counter;
  }
  void update(uint64_t min) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_counter < min) {
      _counter = min;
    }
  }
};

thread_local int g_properties_unlocked_counter = 0;
void Property::checkUnlocked() {
  if (g_properties_unlocked_counter <= 0) {
    throw std::runtime_error("Use LockScope to safely access properties");
  }
}
void Property::unlockScope(int i) { g_properties_unlocked_counter += i; }

Object::Object() {
  if (g_properties_unlocked_counter <= 0) {
    throw std::runtime_error("Use LockScope to safely construct Objects");
  }
  assignNewId();
}

void Object::assignNewId() {
  _id = ObjectIDFactory::instance()->generate();
  if (g_properties_unlocked_counter <= 0) {
    throw std::runtime_error("Use LockScope to safely call assignNewId");
  }
}

Object::~Object() {}

void Object::setId(uint64_t id) {
  _id = id;
  ObjectIDFactory::instance()->update(id);
}

void removeObject(const std::shared_ptr<Object> &parent, void *object) {
  if (parent) {
    for (auto &property : parent->properties()) {
      property.remove(object);
    }
  }
}

std::shared_ptr<const Type> Object::type() const {
  return Type::find(typeid(*this));
}

PropertyList<const Property> Object::properties() const {
  return PropertyList<const Property>(_properties.data(),
                                      _properties.data() + _properties.size());
}

PropertyList<const Property> Object::objectProperties() const {
  return PropertyList<const Property>(_object_properties.data(),
                                      _object_properties.data() +
                                          _object_properties.size());
}

PropertyList<Property> Object::properties() {
  return PropertyList<Property>(_properties.data(),
                                _properties.data() + _properties.size());
}

PropertyList<Property> Object::objectProperties() {
  return PropertyList<Property>(_object_properties.data(),
                                _object_properties.data() +
                                    _object_properties.size());
}

void Object::addProperty(Property property) {
  _properties.push_back(property);
  if (property.canContainObjects()) {
    _object_properties.push_back(property);
  }
}
