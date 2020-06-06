// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <typeindex>
#include <vector>

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

template <class T> class Handle {
  uint64_t _id = 0;

public:
  Handle(const std::shared_ptr<T> &p = nullptr) { reset(p); }
  void reset(uint64_t id) { _id = id; }
  void reset(const std::shared_ptr<T> &p = nullptr) { _id = (p ? p->id() : 0); }
  void operator=(const std::shared_ptr<T> &p) { reset(p); }
  template <class R>
  std::shared_ptr<T> resolve(const std::shared_ptr<R> &root) {
    if (_id == 0) {
      return nullptr;
    }
    std::shared_ptr<T> r;
    root->recurse([&r, this](const std::shared_ptr<Object> &o) {
      if (auto x = std::dynamic_pointer_cast<T>(o)) {
        if (x->id() == _id) {
          r = x;
        }
      }
    });
    return r;
  }
  const uint64_t id() const { return _id; }
};
template <class T, class S>
bool operator==(const Handle<T> &h, const std::shared_ptr<S> &p) {
  if (p == nullptr) {
    return h.id() == 0;
  } else {
    return h.id() == p->id();
  }
}
template <class T, class S>
bool operator==(const std::shared_ptr<S> &p, const Handle<T> &h) {
  if (p == nullptr) {
    return h.id() == 0;
  } else {
    return h.id() == p->id();
  }
}
template <class T, class S>
bool operator!=(const Handle<T> &h, const std::shared_ptr<S> &p) {
  return !(h == p);
}
template <class T, class S>
bool operator!=(const std::shared_ptr<S> &p, const Handle<T> &h) {
  return !(h == p);
}

class Variant {
  std::shared_ptr<void> _ptr = nullptr;
  std::type_index _type = typeid(void);

public:
  std::type_index type() const { return _type; }
  template <class T> const T &value() const {
    return *static_cast<const T *>(_ptr.get());
  }
  template <class T> void assign(const T &v) {
    _type = typeid(T);
    _ptr = std::make_shared<T>(v);
  }
  Variant() {}
  template <class T> explicit Variant(const T &v) { assign(v); }
  void clear() {
    _ptr = nullptr;
    _type = typeid(void);
  }
};

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
         const std::shared_ptr<Type> &base = nullptr) {
    if (auto ret = tryFind<T>()) {
      return ret;
    }
    auto ret = create<T>(name, base);
    _registerType(ret);
    return ret;
  }

  template <class T>
  static std::shared_ptr<Type>
  create(const std::string &name = "",
         const std::shared_ptr<Type> &base = nullptr);

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
  TypeImpl(const std::string &name, const std::shared_ptr<Type> &base) {
    _name = name;
    _base = base;
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
                                   const std::shared_ptr<Type> &base) {
  return std::shared_ptr<Type>(new TypeImpl<T>(name, base));
}

#define DECLARE_TYPE_IMPL(Derived, Base, ...)                                  \
  static_assert(std::is_base_of<Base, Derived>::value,                         \
                "incorrect base class");                                       \
  static std::shared_ptr<Type> _##Base##_##Derived = Type::global<Derived>(    \
      #Derived,                                                                \
      (typeid(Base) != typeid(Derived)) ? Type::find(typeid(Base)) : nullptr);
#define DECLARE_TYPE(Type, ...) DECLARE_TYPE_IMPL(Type, ##__VA_ARGS__, Type)

struct PropertyAttributes {
  double min = std::numeric_limits<double>::quiet_NaN();
  double max = std::numeric_limits<double>::quiet_NaN();
  double step_scale = std::numeric_limits<double>::quiet_NaN();
  double wrap = false;
  std::function<std::vector<std::string>(const Property &)> list;
};

class PropertyInfo {
private:
  std::string _name;
  std::shared_ptr<const Type> _type;
  std::shared_ptr<const PropertyAttributes> _attributes;

protected:
  template <class T>
  void init(const std::string &name,
            const std::shared_ptr<PropertyAttributes> &attributes) {
    static std::shared_ptr<Type> t = Type::create<T>();
    _type = t;
    _name = name;
    if (attributes) {
      _attributes = attributes;
    } else {
      static auto default_attributes = std::make_shared<PropertyAttributes>();
      _attributes = default_attributes;
    }
  }
  PropertyInfo() {}

public:
  const std::shared_ptr<const PropertyAttributes> &attributes() const {
    return _attributes;
  }
  const std::string &name() const { return _name; }
  const std::shared_ptr<const Type> &type() const { return _type; }
  virtual void forEachObject(void *context,
                             void (*f)(void *, const Object *, const Object *),
                             const Object *parent, void *value) const = 0;
  virtual void remove(void *parent, void *object) const = 0;
  virtual bool canContainObjects() const = 0;
  virtual Variant serialize(const void *ptr) const = 0;
  virtual void deserialize(void *ptr, const Variant &v) const = 0;
  virtual void assign(void *ptr, const Variant &v) const = 0;
  virtual Variant toVariant(const void *ptr) const = 0;
  virtual std::shared_ptr<const SnapshotBase>
  save(const void *value, const std::shared_ptr<const SnapshotBase> &x,
       const SnapshotFilter &filter) const = 0;
  virtual void
  applySnapshot(void *value,
                const std::shared_ptr<const SnapshotBase> &x) const = 0;
};

template <class T> class PropertyInfoImpl : public PropertyInfo {

public:
  PropertyInfoImpl(const std::string &name,
                   const std::shared_ptr<PropertyAttributes> &attributes) {
    init<T>(name, attributes);
  }
  virtual void forEachObject(void *context,
                             void (*f)(void *, const Object *, const Object *),
                             const Object *parent, void *value) const override {
    ::forEachObject(context, f, parent, *(T *)value);
  }
  virtual void remove(void *parent, void *object) const override {
    removeObject(*(T *)parent, object);
  }
  virtual bool canContainObjects() const override {
    return !std::is_same<decltype(::forEachObject(nullptr, nullptr, nullptr,
                                                  *(T *)nullptr)),
                         StopObjectRecursionTag>::value;
  }
  virtual Variant serialize(const void *value) const override;
  virtual void deserialize(void *value, const Variant &v) const override;
  virtual void assign(void *ptr, const Variant &v) const {
    if (v.type() != typeid(T)) {
      throw std::runtime_error("type mismatch");
    }
    *(T *)ptr = v.value<T>();
  }
  virtual Variant toVariant(const void *ptr) const {
    return Variant(*(T *)ptr);
  }
  virtual std::shared_ptr<const SnapshotBase>
  save(const void *value, const std::shared_ptr<const SnapshotBase> &x,
       const SnapshotFilter &filter) const override;
  virtual void
  applySnapshot(void *value,
                const std::shared_ptr<const SnapshotBase> &x) const override;
};

class Property {
  void *_value = nullptr;
  std::shared_ptr<const PropertyInfo> _info;

public:
  Property() {}
  template <class T>
  inline Property(const std::shared_ptr<PropertyInfo> &info, T *value)
      : _info(info), _value(value) {}
  inline const char *name() const { return _info->name().c_str(); }
  inline const std::shared_ptr<const PropertyInfo> &info() const {
    return _info;
  }
  inline const std::shared_ptr<const PropertyAttributes> &attributes() const {
    return _info->attributes();
  }
  inline std::shared_ptr<const SnapshotBase>
  save(const std::shared_ptr<const SnapshotBase> &x,
       const SnapshotFilter &filter) const {
    return _info->save(_value, x, filter);
  }
  inline void applySnapshot(const std::shared_ptr<const SnapshotBase> &x) {
    _info->applySnapshot(_value, x);
  }
  inline const std::type_index &typeId() const {
    return _info->type()->typeId();
  }
  inline bool tryToString(std::string &s) const {
    return _info->type()->tryToString(_value, s);
  }
  inline bool fromStringSupported() const {
    return _info->type()->fromStringSupported();
  }
  inline void fromStringOrThrow(const std::string &s) {
    _info->type()->fromStringOrThrow(_value, s);
  }
  inline void expand(std::vector<Property> &properties) const {
    _info->type()->properties(_value, properties);
  }
  inline void forEachObject(void *context,
                            void (*f)(void *, const Object *, const Object *),
                            const Object *parent) const {
    _info->forEachObject(context, f, parent, _value);
  }
  inline void remove(void *object) { _info->remove(_value, object); }
  inline bool canContainObjects() const { return _info->canContainObjects(); }
  inline Variant serialize() const { return _info->serialize(_value); }
  inline void deserialize(const Variant &v) { _info->deserialize(_value, v); }
  std::string displayName() const {
    std::string n = name();
    n.at(0) = std::toupper(n[0]);
    return n;
  }
  inline void assign(const Variant &variant) { _info->assign(_value, variant); }
  inline Variant read() const { return _info->toVariant(_value); }
  template <class T> inline void set(const T &v) {
    checkUnlocked();
    if (typeId() != typeid(T)) {
      throw std::runtime_error("property type mismatch");
    }
    return *static_cast<T *>(_value);
  }
  template <class T> inline const T &get() const {
    checkUnlocked();
    if (typeId() != typeid(T)) {
      throw std::runtime_error("property type mismatch");
    }
    return *static_cast<const T *>(_value);
  }
  inline void *valuePointer() {
    checkUnlocked();
    return _value;
  }
  inline const void *valuePointer() const {
    checkUnlocked();
    return _value;
  }
  static void checkUnlocked();
  static void unlockScope(int i);
};

template <class T> class PropertyList {
  T *_begin, *_end;

public:
  PropertyList(T *begin, T *end) : _begin(begin), _end(end) {}
  T *begin() const { return _begin; }
  T *end() const { return _end; }
  T &operator[](size_t i) const { return _begin[i]; }
  size_t size() const { return _end - _begin; }
};

template <class T> struct DefaultPropertyAttributes {
  static inline void initialize(PropertyAttributes *attributes) {}
};

#define GENERATE_PROPERTY_ATTRIBUTES(type, ...)                                \
  []() {                                                                       \
    struct Attr : PropertyAttributes {                                         \
      Attr() {                                                                 \
        DefaultPropertyAttributes<type>::initialize(                           \
            (PropertyAttributes *)this);                                       \
        __VA_ARGS__;                                                           \
      }                                                                        \
    };                                                                         \
    static auto attr = std::make_shared<Attr>();                               \
    return attr;                                                               \
  }()

#define PROPERTY_FIRST_ARGUMENT_2(a, ...) a
#define PROPERTY_FIRST_ARGUMENT(...) PROPERTY_FIRST_ARGUMENT_2(__VA_ARGS__, )

#define PROPERTY_SKIP_FIRST_2(a, ...) __VA_ARGS__
#define PROPERTY_SKIP_FIRST(...) PROPERTY_SKIP_FIRST_2(__VA_ARGS__, 0)

#define PROPERTY(type, name, ...)                                              \
private:                                                                       \
  type _property_##name = type(PROPERTY_FIRST_ARGUMENT(__VA_ARGS__));          \
  struct Property##_##name {                                                   \
    template <class O, class T>                                                \
    inline Property##_##name(O *object, T *value) {                            \
      static std::string xname = []() {                                        \
        std::string s = #name;                                                 \
        s[0] = std::toupper(s[0]);                                             \
        return s;                                                              \
      }();                                                                     \
      static auto info =                                                       \
          std::shared_ptr<PropertyInfo>(new PropertyInfoImpl<T>(               \
              xname, GENERATE_PROPERTY_ATTRIBUTES(                             \
                         type, PROPERTY_SKIP_FIRST(__VA_ARGS__))));            \
      object->addProperty(Property(info, value));                              \
    }                                                                          \
  };                                                                           \
  Property##_##name _##name##_property =                                       \
      Property##_##name(this, &_property_##name);                              \
                                                                               \
public:                                                                        \
  inline const type &name() const {                                            \
    Property::checkUnlocked();                                                 \
    return _property_##name;                                                   \
  }                                                                            \
  inline type &name() {                                                        \
    Property::checkUnlocked();                                                 \
    return _property_##name;                                                   \
  }                                                                            \
  inline void name(const type &value) {                                        \
    Property::checkUnlocked();                                                 \
    _property_##name = value;                                                  \
  }

class Object : public std::enable_shared_from_this<Object> {
  std::vector<Property> _properties, _object_properties;
  uint64_t _id = 0;

public:
  Object();
  virtual ~Object();
  Object(const Object &) = delete;
  Object &operator=(const Object &) = delete;
  std::shared_ptr<Type> type() const {
    // TODO: cache
    return Type::find(typeid(*this));
  }
  inline PropertyList<const Property> properties() const {
    return PropertyList<const Property>(
        _properties.data(), _properties.data() + _properties.size());
  }
  inline PropertyList<const Property> objectProperties() const {
    return PropertyList<const Property>(_object_properties.data(),
                                        _object_properties.data() +
                                            _object_properties.size());
  }
  inline PropertyList<Property> properties() {
    return PropertyList<Property>(_properties.data(),
                                  _properties.data() + _properties.size());
  }
  inline PropertyList<Property> objectProperties() {
    return PropertyList<Property>(_object_properties.data(),
                                  _object_properties.data() +
                                      _object_properties.size());
  }
  void addProperty(Property property) {
    _properties.push_back(property);
    if (property.canContainObjects()) {
      _object_properties.push_back(property);
    }
  }
  uint64_t id() const { return _id; }
  void assignNewId();
  void setId(uint64_t id);
  template <class F>
  auto recurse(const F &f)
      -> decltype(f(std::shared_ptr<Object>(), std::shared_ptr<Object>())) {
    forEachObject((void *)&f,
                  [](void *context, const Object *parent, const Object *child) {
                    auto *f = (F *)context;
                    auto p = parent ? ((Object *)parent)->shared_from_this()
                                    : nullptr;
                    if (auto c = (child ? ((Object *)child)->shared_from_this()
                                        : nullptr)) {
                      (*f)(p, c);
                    }
                  },
                  nullptr, this);
  }
  template <class F>
  auto recurse(const F &f) -> decltype(f(std::shared_ptr<Object>())) {
    forEachObject((void *)&f,
                  [](void *context, const Object *parent, const Object *child) {
                    auto *f = (F *)context;
                    if (auto c = (child ? ((Object *)child)->shared_from_this()
                                        : nullptr)) {
                      (*f)(c);
                    }
                  },
                  nullptr, this);
  }
};
DECLARE_TYPE(Object);

class Watcher {
  Variant _snapshot;

public:
  template <class... Args> bool changed(const Args &... args) {
    if (std::type_index(typeid(std::make_tuple(args...))) == _snapshot.type()) {
      auto tuple = std::make_tuple(args...);
      if (tuple != _snapshot.value<decltype(tuple)>()) {
        _snapshot = Variant(tuple);
        return true;
      }
    } else {
      _snapshot = Variant(std::make_tuple(args...));
      return true;
    }
    return false;
  }
};

#define STRUCT_BEGIN(typename)                                                 \
  static void properties(typename &v, std::vector<Property> &properties) {

#define STRUCT_FIELD(fieldname, ...)                                           \
  properties.emplace_back(                                                     \
      []() {                                                                   \
        static std::shared_ptr<PropertyInfo> info =                            \
            std::make_shared<PropertyInfoImpl<                                 \
                typename std::remove_reference<decltype(v.fieldname)>::type>>( \
                #fieldname, GENERATE_PROPERTY_ATTRIBUTES(                      \
                                typename std::remove_reference<decltype(       \
                                    v.fieldname)>::type,                       \
                                __VA_ARGS__));                                 \
        return info;                                                           \
      }(),                                                                     \
      &v.fieldname);

#define STRUCT_PROPERTY(propname, ...)                                         \
  properties.emplace_back(                                                     \
      []() {                                                                   \
        static std::shared_ptr<PropertyInfo> info = std::make_shared<          \
            PropertyInfoImpl<typename std::remove_reference<decltype(          \
                v.propname())>::type>>(                                        \
            #propname,                                                         \
            GENERATE_PROPERTY_ATTRIBUTES(                                      \
                typename std::remove_reference<decltype(v.propname())>::type,  \
                __VA_ARGS__));                                                 \
        return info;                                                           \
      }(),                                                                     \
      &v.propname());

#define STRUCT_END() }

#define DECLARE_STRUCT_PROPERTY(type, name, def)                               \
protected:                                                                     \
  type _##name = def;                                                          \
                                                                               \
public:                                                                        \
  inline const type &name() const { return _##name; }                          \
  inline type &name() { return _##name; }
