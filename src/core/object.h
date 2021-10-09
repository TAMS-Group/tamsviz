// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "property.h"

class Object : public std::enable_shared_from_this<Object> {
  std::vector<Property> _properties, _object_properties;
  uint64_t _id = 0;
  static void _dummyVoid() {}

public:
  Object();
  virtual ~Object();
  Object(const Object &) = delete;
  Object &operator=(const Object &) = delete;
  std::shared_ptr<const Type> type() const;
  PropertyList<const Property> properties() const;
  PropertyList<const Property> objectProperties() const;
  PropertyList<Property> properties();
  PropertyList<Property> objectProperties();
  void addProperty(Property property);
  uint64_t id() const { return _id; }
  void assignNewId();
  void setId(uint64_t id);
  template <class F>
  auto recurseObjects(const F &f)
      -> decltype(f(std::shared_ptr<Object>(), std::shared_ptr<Object>()),
                  _dummyVoid()) {
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
  auto recurseObjects(const F &f)
      -> decltype(f(std::shared_ptr<Object>()), _dummyVoid()) {
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
