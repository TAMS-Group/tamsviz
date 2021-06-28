// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "property.h"

template <class T>
auto properties(T &v, std::vector<Property> &properties) -> decltype(
    (*(T *)nullptr).properties(*((std::vector<Property> *)nullptr))) {
  v.properties(properties);
}

#define AUTO_STRUCT_BEGIN(name)                                                \
  struct name {                                                                \
    name() {}                                                                  \
    bool operator==(const name &other) const {                                 \
      return Properties::equals(*this, other);                                 \
    }                                                                          \
    bool operator!=(const name &other) const {                                 \
      return !Properties::equals(*this, other);                                \
    }                                                                          \
    void properties(std::vector<Property> &p) {                                \
      Properties::properties(*this, p);                                        \
    }                                                                          \
    typedef struct {                                                           \
      template <class T> static bool equals(const T &a, const T &b) {          \
        return true;                                                           \
      }                                                                        \
      template <class T>                                                       \
      static void properties(T &v, std::vector<Property> &p) {}

#define AUTO_STRUCT_FIELD(type, name, ...)                                     \
  }                                                                            \
  Base_##name;                                                                 \
  type name = type(PROPERTY_FIRST_ARGUMENT(__VA_ARGS__));                      \
  typedef struct : Base_##name {                                               \
    template <class T> static bool equals(const T &a, const T &b) {            \
      return Base_##name::equals(a, b) && (a.name == b.name);                  \
    }                                                                          \
    template <class T>                                                         \
    static void properties(T &v, std::vector<Property> &p) {                   \
      Base_##name::properties(v, p);                                           \
      p.emplace_back(                                                          \
          []() {                                                               \
            static std::shared_ptr<PropertyInfo> info =                        \
                std::make_shared<PropertyInfoImpl<type>>(                      \
                    #name, GENERATE_PROPERTY_ATTRIBUTES(type, __VA_ARGS__));   \
            return info;                                                       \
          }(),                                                                 \
          &v.name);                                                            \
    }

#define AUTO_STRUCT_IMPL_CONCAT_2(a, b) a##b
#define AUTO_STRUCT_IMPL_CONCAT(a, b) AUTO_STRUCT_IMPL_CONCAT_2(a, b)
#define AUTO_STRUCT_EXTRA(...)                                                 \
  }                                                                            \
  AUTO_STRUCT_IMPL_CONCAT(Base_, __LINE__);                                    \
  __VA_ARGS__;                                                                 \
  typedef struct : AUTO_STRUCT_IMPL_CONCAT(Base_, __LINE__) {

#define AUTO_STRUCT_END()                                                      \
  }                                                                            \
  Properties;                                                                  \
  }                                                                            \
  ;

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
