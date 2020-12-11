// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "type.h"
#include "variant.h"

#include <limits>

struct AutoCompletion {
  std::vector<std::string> items;
  bool completed = false;
};

struct PropertyAttributes {
  double min = std::numeric_limits<double>::quiet_NaN();
  double max = std::numeric_limits<double>::quiet_NaN();
  double step_scale = std::numeric_limits<double>::quiet_NaN();
  double wrap = false;
  bool hidden = false;
  std::function<std::vector<std::string>(const Property &)> list;
  std::function<void(const Property &, const std::string &, AutoCompletion &)>
      complete;
};

template <class T> bool expandList(std::vector<Property> &list, T &v);
template <class T>
bool expandList(std::vector<Property> &list, std::vector<T> &v);

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
  virtual bool expandList(std::vector<Property> &list,
                          const void *ptr) const = 0;
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
  virtual bool expandList(std::vector<Property> &list,
                          const void *ptr) const override {
    return ::expandList(list, *(T *)ptr);
  }
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
  inline bool expandList(std::vector<Property> &list) {
    return _info->expandList(list, _value);
  }
  static void checkUnlocked();
  static void unlockScope(int i);
};

template <class T> bool expandList(std::vector<Property> &list, T &v) {
  return false;
}
template <class T>
bool expandList(std::vector<Property> &list, std::vector<T> &v) {
  static auto attr = std::make_shared<PropertyAttributes>();
  for (size_t i = 0; i < v.size(); i++) {
    list.push_back(Property(
        std::make_shared<PropertyInfoImpl<T>>(std::to_string(i), attr), &v[i]));
  }
  return true;
}

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
