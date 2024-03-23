// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include <functional>
#include <memory>
#include <stdexcept>
#include <typeindex>
#include <vector>

#include "handle.h"

inline void toString(const std::string &v, std::string &s) { s = v; }
inline void toString(const bool &v, std::string &s) {
  if (v) {
    s = "true";
  } else {
    s = "false";
  }
}
inline void toString(float v, std::string &s) { s = std::to_string(v); }
inline void toString(double v, std::string &s) { s = std::to_string(v); }
inline void toString(int v, std::string &s) { s = std::to_string(v); }
inline void toString(long v, std::string &s) { s = std::to_string(v); }
inline void toString(long long v, std::string &s) { s = std::to_string(v); }
inline void toString(unsigned long v, std::string &s) { s = std::to_string(v); }
inline void toString(unsigned long long v, std::string &s) {
  s = std::to_string(v);
}

inline void fromString(std::string &v, const std::string &s) { v = s; }
inline void fromString(float &v, const std::string &s) { v = std::stof(s); }
inline void fromString(double &v, const std::string &s) { v = std::stod(s); }
inline void fromString(int &v, const std::string &s) { v = std::stoi(s); }
inline void fromString(long &v, const std::string &s) { v = std::stol(s); }
inline void fromString(long long &v, const std::string &s) {
  v = std::stoll(s);
}
inline void fromString(unsigned long &v, const std::string &s) {
  v = std::stoul(s);
}
inline void fromString(unsigned long long &v, const std::string &s) {
  v = std::stoull(s);
}
inline void fromString(bool &v, const std::string &s) {
  if (s == "true" || s == "1" || s == "True") {
    v = true;
    return;
  }
  if (s == "false" || s == "0" || s == "False") {
    v = false;
    return;
  }
  throw std::invalid_argument("s");
}

class SnapshotBase;
class Object;
class Property;

static inline bool tryToString(...) { return false; }
template <class T>
static inline auto tryToString(const T *v, std::string &s)
    -> decltype(toString(*v, s), bool()) {
  toString(*v, s);
  return true;
}

static inline void fromStringOrThrow(...) {
  throw std::invalid_argument("type");
}
template <class T>
static inline auto fromStringOrThrow(T *v, const std::string &s)
    -> decltype(fromString(*v, s)) {
  fromString(*v, s);
}

static constexpr bool toStringSupported(...) { return false; }
template <class T>
static constexpr auto toStringSupported(T *v)
    -> decltype(toString(*v, *(std::string *)nullptr), true) {
  return true;
}

static constexpr bool fromStringSupported(...) { return false; }
template <class T>
static constexpr auto fromStringSupported(T *v)
    -> decltype(fromString(*v, *(std::string *)nullptr), true) {
  return true;
}

static constexpr bool propertiesSupported(...) { return false; }
template <class T>
static constexpr auto propertiesSupported(T *v)
    -> decltype(properties(*v, *(std::vector<Property> *)nullptr), true) {
  return true;
}

inline void removeObject(...) {}
template <class T>
void removeObject(std::vector<std::shared_ptr<T>> &parent, void *object) {
  for (auto it = parent.begin(); it != parent.end();) {
    if (it->get() == object) {
      it = parent.erase(it);
    } else {
      removeObject(*it, object);
      it++;
    }
  }
}
void removeObject(const std::shared_ptr<Object> &parent, void *object);

struct StopObjectRecursionTag {};

typedef std::function<bool(const std::shared_ptr<Object> &o)> SnapshotFilter;

template <class T>
auto forEachObject(void *context,
                   void (*f)(void *, const Object *, const Object *),
                   const Object *parent, const T &child) ->
    typename std::enable_if<propertiesSupported((T *)nullptr) ||
                                fromStringSupported((T *)nullptr),
                            StopObjectRecursionTag>::type {
  return StopObjectRecursionTag();
}

void forEachObject(void *context,
                   void (*f)(void *, const Object *, const Object *),
                   const Object *parent, const Object *child);

template <class T>
void forEachObject(void *context,
                   void (*f)(void *, const Object *, const Object *),
                   const Object *parent, const std::shared_ptr<T> &v) {
  forEachObject(context, f, parent, v.get());
}

template <class T>
void forEachObject(void *context,
                   void (*f)(void *, const Object *, const Object *),
                   const Object *parent, const std::vector<T> &v) {
  for (auto &e : v) {
    forEachObject(context, f, parent, e);
  }
}

template <class T>
StopObjectRecursionTag
forEachObject(void *context, void (*f)(void *, const Object *, const Object *),
              const Object *parent, const Handle<T> &v) {
  return StopObjectRecursionTag();
}

static void tryGetProperties(...) {}
template <class X>
static auto tryGetProperties(X *v, std::vector<Property> &props)
    -> decltype(properties(*v, props)) {
  properties(*v, props);
}
void getObjectProperties(const std::shared_ptr<Object> &v,
                         std::vector<Property> &props);
template <class O>
static void tryGetProperties(std::shared_ptr<O> *v,
                             std::vector<Property> &props) {
  getObjectProperties(*v, props);
}

struct Type {
protected:
  std::string _name;
  std::type_index _type_id = typeid(void);
  std::shared_ptr<Type> _base;
  bool _constructable = false;
  std::function<std::shared_ptr<void>()> _create;
  std::string _category;

  static void _registerType(const std::shared_ptr<Type> &t);

  template <class T> void _initConstructor(...) {
    _create = []() -> std::shared_ptr<void> {
      throw std::runtime_error(
          "can't instantiate class without default constructor");
    };
  }
  template <class T> void _initConstructor(decltype(new T()) i) {
    _create = []() -> std::shared_ptr<void> {
      return std::static_pointer_cast<void>(std::make_shared<T>());
    };
    _constructable = true;
  }

  Type() {}
  Type(const Type &) = delete;
  Type &operator=(const Type &) = delete;
  virtual ~Type() {}

public:
  const std::type_index &typeId() const { return _type_id; }
  const std::string &name() const { return _name; }
  const std::shared_ptr<Type> &base() const { return _base; }
  bool constructable() const { return _constructable; }
  const std::string &category() const { return _category; }

  template <class T> std::shared_ptr<T> instantiate() const {
    for (const Type *t = this; t; t = t->_base.get()) {
      if (t->typeId() == typeid(T)) {
        return std::static_pointer_cast<T>(_create());
      }
    }
    return nullptr;
  }

  static std::shared_ptr<Type> tryFind(const std::string &name);
  static std::shared_ptr<Type> tryFind(const std::type_index &id);
  static std::shared_ptr<Type> find(const std::string &name);
  static std::shared_ptr<Type> find(const std::type_index &id);
  template <class T> static std::shared_ptr<Type> find() {
    return find(typeid(T));
  }
  template <class T> static std::shared_ptr<Type> tryFind() {
    return tryFind(typeid(T));
  }

  std::vector<std::shared_ptr<Type>> list() const;

  template <class T>
  static std::shared_ptr<Type>
  global(const std::string &name = "",
         const std::shared_ptr<Type> &base = nullptr,
         const std::string &category = "") {
    if (auto ret = tryFind<T>()) {
      return ret;
    }
    auto ret = create<T>(name, base, category);
    _registerType(ret);
    return ret;
  }

  template <class T>
  static std::shared_ptr<Type>
  create(const std::string &name = "",
         const std::shared_ptr<Type> &base = nullptr,
         const std::string &category = "");

  virtual bool tryToString(const void *p, std::string &s) const = 0;
  virtual bool fromStringSupported() const = 0;
  virtual void fromStringOrThrow(void *p, const std::string &s) const = 0;
  virtual void properties(const void *ptr,
                          std::vector<Property> &properties) const = 0;

protected:
  static bool _tryToPointerList(...) { return false; }
  template <class T>
  static bool _tryToPointerList(const std::vector<std::shared_ptr<T>> *list,
                                std::vector<std::shared_ptr<void>> &out) {
    out.assign(list->begin(), list->end());
    return true;
  }

  static bool _tryFromPointerList(...) { return false; }
  template <class T>
  static bool
  _tryFromPointerList(std::vector<std::shared_ptr<T>> *list,
                      const std::vector<std::shared_ptr<void>> &in) {
    list->clear();
    list->reserve(in.size());
    for (auto &v : in) {
      list->push_back(std::static_pointer_cast<T>(v));
    }
    return true;
  }

public:
  virtual bool
  tryToPointerList(const void *object,
                   std::vector<std::shared_ptr<void>> &out) const = 0;
  virtual bool
  tryFromPointerList(void *object,
                     const std::vector<std::shared_ptr<void>> &in) const = 0;
};

template <class T> class TypeImpl : public Type {

public:
  TypeImpl(const std::string &name, const std::shared_ptr<Type> &base,
           const std::string &category) {
    _name = name;
    _base = base;
    _category = category;
    _type_id = typeid(T);
    _initConstructor<T>(0);
  }
  virtual bool tryToString(const void *p, std::string &s) const override {
    return ::tryToString((const T *)p, s);
  }
  virtual bool fromStringSupported() const {
    return ::fromStringSupported((T *)nullptr);
  }
  virtual void fromStringOrThrow(void *value,
                                 const std::string &s) const override {
    ::fromStringOrThrow((T *)value, s);
  }
  virtual void properties(const void *ptr,
                          std::vector<Property> &props) const override {
    tryGetProperties((T *)ptr, props);
  }
  virtual bool
  tryToPointerList(const void *object,
                   std::vector<std::shared_ptr<void>> &out) const override {
    return _tryToPointerList(static_cast<const T *>(object), out);
  }
  virtual bool tryFromPointerList(
      void *object,
      const std::vector<std::shared_ptr<void>> &in) const override {
    return _tryFromPointerList(static_cast<T *>(object), in);
  }
};

template <class T>
std::shared_ptr<Type> Type::create(const std::string &name,
                                   const std::shared_ptr<Type> &base,
                                   const std::string &category) {
  return std::shared_ptr<Type>(new TypeImpl<T>(name, base, category));
}

#define DECLARE_TYPE_STRINGIFY_2(x) #x
#define DECLARE_TYPE_STRINGIFY(x) DECLARE_TYPE_STRINGIFY_2(x)

#define DECLARE_TYPE_IMPL(Derived, Base, ...)                                  \
  static_assert(std::is_base_of<Base, Derived>::value,                         \
                "incorrect base class");                                       \
  static std::shared_ptr<Type> _##Base##_##Derived = Type::global<Derived>(    \
      #Derived,                                                                \
      (typeid(Base) != typeid(Derived)) ? Type::find(typeid(Base)) : nullptr);
#define DECLARE_TYPE(Type, ...) DECLARE_TYPE_IMPL(Type, ##__VA_ARGS__, Type)

#define DECLARE_TYPE_C(Derived, Base, Category)                                \
  static_assert(std::is_base_of<Base, Derived>::value,                         \
                "incorrect base class");                                       \
  static std::shared_ptr<Type> _##Base##_##Derived = Type::global<Derived>(    \
      #Derived, Type::find(typeid(Base)), DECLARE_TYPE_STRINGIFY_2(Category));
