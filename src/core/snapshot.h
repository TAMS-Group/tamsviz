// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "object.h"

#include <unordered_map>

struct SnapshotBase {
  SnapshotBase();
  SnapshotBase(const SnapshotBase &other);
  virtual ~SnapshotBase();
  static size_t instanceCount();
  mutable uint64_t counter = 0;
};

template <class T>
struct Snapshot : std::enable_if<fromStringSupported((T *)nullptr) ||
                                     propertiesSupported((T *)nullptr),
                                 SnapshotBase>::type {
  T value;
  static std::shared_ptr<const Snapshot<T>>
  save(const T &v, const std::shared_ptr<const Snapshot<T>> &s,
       const SnapshotFilter &filter) {
    if (s && s->value == v) {
      return s;
    } else {
      auto ret = std::make_shared<Snapshot>();
      ret->value = v;
      return ret;
    }
  }
  void apply(T &v) const { v = value; }
};

template <class T> struct Snapshot<std::shared_ptr<T>> : SnapshotBase {
  uint64_t id = 0;
  std::shared_ptr<const Type> type;
  bool nonzero = false;
  std::vector<std::shared_ptr<const SnapshotBase>> property_snapshots;
  static std::shared_ptr<const Type> findType(const std::shared_ptr<T> &p) {
    if (p == nullptr) {
      return nullptr;
    }
    return Type::find(typeid(*p));
  }
  static std::shared_ptr<const Snapshot<std::shared_ptr<T>>>
  save(const std::shared_ptr<T> &v,
       const std::shared_ptr<const Snapshot<std::shared_ptr<T>>> &s0,
       const SnapshotFilter &filter) {
    std::shared_ptr<Snapshot<std::shared_ptr<T>>> ret;
    if (s0 == nullptr || s0->nonzero != (bool)v || (v && (s0->id != v->id())) ||
        (v && s0->type->typeId() != std::type_index(typeid(*v)))) {
      ret = std::make_shared<Snapshot<std::shared_ptr<T>>>();
      if (v != nullptr) {
        ret->property_snapshots.resize(v->properties().size());
      }
      ret->id = (v ? v->id() : 0);
      ret->type = findType(v);
      ret->nonzero = (bool)v;
    }
    if (v != nullptr) {
      auto properties = v->properties();
      for (size_t property_index = 0; property_index < properties.size();
           property_index++) {
        auto &property = properties[property_index];
        auto previous_snapshot =
            ((s0 != nullptr && property_index < s0->property_snapshots.size() &&
              s0->nonzero && s0->type->typeId() == std::type_index(typeid(*v)))
                 ? s0->property_snapshots.at(property_index)
                 : nullptr);
        auto property_snapshot =
            ((property.canContainObjects() || filter == nullptr ||
              previous_snapshot == nullptr || filter(v))
                 ? property.save(previous_snapshot, filter)
                 : previous_snapshot);
        // auto property_snapshot =
        //    property.save(previous_snapshot, (filter != v ? filter :
        //    nullptr));
        if (property_snapshot == nullptr) {
          throw std::runtime_error("failed to save property");
        }
        if (property_snapshot != previous_snapshot && ret == nullptr) {
          ret = std::make_shared<Snapshot<std::shared_ptr<T>>>(*s0);
          ret->property_snapshots.resize(v->properties().size());
        }
        if (ret) {
          ret->property_snapshots.at(property_index) = property_snapshot;
        }
      }
    } else {
      if (ret) {
        ret->property_snapshots.clear();
      }
    }
    return ret ? ret : s0;
  }
  void apply(std::shared_ptr<T> &v) const {
    if (!nonzero) {
      v = nullptr;
      return;
    } else {
      if (v == nullptr || type != Type::find(typeid(*v))) {
        v = type->instantiate<T>();
      }
      v->setId(id);
      {
        auto properties = v->properties();
        for (size_t property_index = 0; property_index < properties.size();
             property_index++) {
          auto &property = properties[property_index];
          property.applySnapshot(property_snapshots.at(property_index));
        }
      }
    }
  }
};

template <class T> struct Snapshot<std::vector<T>> : SnapshotBase {
  std::vector<std::shared_ptr<const Snapshot<T>>> snapshots;
  static std::shared_ptr<const Snapshot>
  save(const std::vector<T> &v, const std::shared_ptr<const Snapshot> &s0,
       const SnapshotFilter &filter) {
    std::shared_ptr<Snapshot<std::vector<T>>> ret;
    if (s0 == nullptr || (s0->snapshots.size() != v.size())) {
      ret = std::make_shared<Snapshot<std::vector<T>>>();
      ret->snapshots.resize(v.size());
    }
    for (size_t i = 0; i < v.size(); i++) {
      auto shot = Snapshot<T>::save(v[i],
                                    (s0 != nullptr && i < s0->snapshots.size())
                                        ? s0->snapshots[i]
                                        : nullptr,
                                    filter);
      if (ret != nullptr || i >= s0->snapshots.size() ||
          shot != s0->snapshots[i]) {
        if (ret == nullptr) {
          if (s0) {
            ret = std::make_shared<Snapshot<std::vector<T>>>(*s0);
          } else {
            ret = std::make_shared<Snapshot<std::vector<T>>>();
          }
          ret->snapshots.resize(v.size());
        }
        ret->snapshots[i] = shot;
      }
    }
    return ret ? ret : s0;
  }
  inline void shuffle(...) const {}
  template <class X> void shuffle(std::vector<std::shared_ptr<X>> &v) const {
    // if (v.size() != snapshots.size()) {
    std::unordered_map<uint64_t, std::shared_ptr<X>> id_map;
    id_map.reserve(v.size());
    for (auto &x : v) {
      if (x) {
        id_map[x->id()] = x;
      }
    }
    v.resize(snapshots.size());
    for (size_t i = 0; i < snapshots.size(); i++) {
      auto iter = id_map.find(snapshots[i]->id);
      if (iter != id_map.end()) {
        v[i] = iter->second;
      } else {
        v[i] = nullptr;
      }
    }
    //}
  }
  void apply(std::vector<T> &v) const {
    shuffle(v);
    v.resize(snapshots.size());
    for (size_t i = 0; i < snapshots.size(); i++) {
      snapshots[i]->apply(v[i]);
    }
  }
};

template <class T> struct Snapshot<Handle<T>> : SnapshotBase {
  uint64_t id = 0;
  static std::shared_ptr<const Snapshot>
  save(const Handle<T> &v, const std::shared_ptr<const Snapshot> &s0,
       const SnapshotFilter &filter) {
    if (s0 && s0->id == v.id()) {
      return s0;
    } else {
      auto ret = std::make_shared<Snapshot>();
      ret->id = v.id();
      return ret;
    }
  }
  void apply(Handle<T> &v) const { v.reset(id); }
};

template <class T>
std::shared_ptr<const SnapshotBase>
PropertyInfoImpl<T>::save(const void *value,
                          const std::shared_ptr<const SnapshotBase> &x,
                          const SnapshotFilter &filter) const {
  auto snapshot = std::dynamic_pointer_cast<const Snapshot<T>>(x);
  if (x && !snapshot) {
    throw std::runtime_error("snapshot type mismatch");
  }
  return Snapshot<T>::save(*(const T *)value, snapshot, filter);
}

template <class T>
void PropertyInfoImpl<T>::applySnapshot(
    void *value, const std::shared_ptr<const SnapshotBase> &x) const {
  std::dynamic_pointer_cast<const Snapshot<T>>(x)->apply(*(T *)value);
}
