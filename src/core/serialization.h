// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "object.h"
#include "variant.h"

#include <map>

class SerializationTypeError : public std::runtime_error {
protected:
  std::string _type_name;
  SerializationTypeError(const std::string &type_name);

public:
  const std::string &typeName() const { return _type_name; }
  virtual void createReplacement() const = 0;
};

template <class T> struct SerializationTypeReplacement : T {};

template <class T>
class SerializationTypeErrorImpl : public SerializationTypeError {

public:
  SerializationTypeErrorImpl(const std::string &type_name)
      : SerializationTypeError(type_name) {}
  virtual void createReplacement() const {
    Type::global<SerializationTypeReplacement<T>>(typeName(), Type::find<T>());
  }
};

template <class T>
auto serialize(const T &v) ->
    typename std::enable_if<toStringSupported((T *)nullptr), Variant>::type {
  std::string ret;
  toString(v, ret);
  return Variant(ret);
}
template <class T>
auto deserialize(T &v, const Variant &r) ->
    typename std::enable_if<fromStringSupported((T *)nullptr), void>::type {
  fromString(v, r.value<std::string>());
}

template <class T>
auto serialize(const T &v) ->
    typename std::enable_if<propertiesSupported((T *)nullptr), Variant>::type {
  std::map<std::string, Variant> r;
  std::vector<Property> pp;
  properties(*(T *)&v, pp);
  for (auto &property : pp) {
    r[property.displayName()] = property.serialize();
  }
  return Variant(r);
}
template <class T>
auto deserialize(T &value, const Variant &variant) ->
    typename std::enable_if<propertiesSupported((T *)nullptr), void>::type {
  auto map = variant.value<std::map<std::string, Variant>>();
  std::vector<Property> pp;
  properties(*(T *)&value, pp);
  for (auto &property : pp) {
    if (map.find(property.name()) != map.end()) {
      property.deserialize(map[property.name()]);
    } else if (map.find(property.displayName()) != map.end()) {
      property.deserialize(map[property.displayName()]);
    }
  }
}

template <class T> Variant serialize(const std::vector<T> &v) {
  std::vector<Variant> r;
  for (auto &x : v) {
    r.push_back(serialize(x));
  }
  return Variant(r);
}
template <class T> void deserialize(std::vector<T> &v, const Variant &x) {
  auto &r = x.value<std::vector<Variant>>();
  v.resize(r.size());
  for (size_t i = 0; i < v.size(); i++) {
    deserialize(v[i], r[i]);
  }
}

template <class T> Variant serialize(const Handle<T> &v) {
  return Variant(std::to_string(v.id()));
}
template <class T> void deserialize(Handle<T> &handle, const Variant &variant) {
  handle.reset(std::stoul(variant.value<std::string>()));
}

template <class T>
static std::shared_ptr<const Type>
findSerializationType(const std::shared_ptr<T> &p) {
  if (p == nullptr || typeid(*p) == typeid(T)) {
    return nullptr;
  }
  return Type::find(typeid(*p));
}
template <class T> Variant serialize(const std::shared_ptr<T> &v) {
  if (v) {
    std::map<std::string, Variant> r;
    for (auto &property : v->properties()) {
      r[property.name()] = property.serialize();
    }
    if (auto type = findSerializationType(v)) {
      r["type"] = Variant(std::string(type->name()));
    }
    r["id"] = Variant(std::to_string(v->id()));
    return Variant(r);
  } else {
    return Variant(std::string("null"));
  }
}
template <class T>
void deserialize(std::shared_ptr<T> &object, const Variant &variant) {
  if (variant.type() == typeid(std::string)) {
    auto s = variant.value<std::string>();
    if (s == "null" || s == "") {
      object = nullptr;
    } else {
      throw std::runtime_error("invalid value");
    }
  } else {
    auto map = variant.value<std::map<std::string, Variant>>();
    if (map.find("type") != map.end()) {
      std::string type_name = map["type"].value<std::string>();
      auto type = Type::tryFind(type_name);
      if (!type) {
        throw SerializationTypeErrorImpl<T>(type_name);
      }
      if (object == nullptr ||
          std::type_index(typeid(*object)) != type->typeId()) {
        object = type->instantiate<T>();
      }
    } else {
      std::shared_ptr<Type> type = Type::find<T>();
      if (object == nullptr ||
          std::type_index(typeid(*object)) != type->typeId()) {
        object = type->instantiate<T>();
      }
    }
    if (map.find("id") != map.end()) {
      object->setId(std::stoull(map["id"].value<std::string>()));
    }
    for (auto &property : object->properties()) {
      if (map.find(property.name()) != map.end()) {
        property.deserialize(map[property.name()]);
      }
    }
  }
}

template <class T>
Variant PropertyInfoImpl<T>::serialize(const void *value) const {
  return ::serialize(*(T *)value);
}
template <class T>
void PropertyInfoImpl<T>::deserialize(void *value, const Variant &v) const {
  ::deserialize(*(T *)value, v);
}

void toYAML(const Variant &v, std::ostream &stream);
std::string toYAML(const Variant &v);

Variant parseYAML(const std::string &str);
