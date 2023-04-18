// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "document.h"
#include "history.h"
#include "snapshot.h"

#include <mutex>
#include <unordered_set>

class BagPlayer;
class AnnotationTrack;

struct Selection {
  std::vector<uint64_t> _objects;
  void operator=(const std::shared_ptr<Object> &o);
  bool contains(const std::shared_ptr<Object> &o) const;
  std::vector<std::shared_ptr<Object>>
  resolve(const std::shared_ptr<Object> &root) const;
  void clear();
  void add(const std::shared_ptr<Object> &object);
  void toggle(const std::shared_ptr<Object> &object);
  void erase(const std::shared_ptr<Object> &object);
  bool empty() const;
  size_t size() const;
};
bool operator==(const Selection &a, const Selection &b);
bool operator!=(const Selection &a, const Selection &b);
STRUCT_BEGIN(Selection);
STRUCT_FIELD(_objects);
STRUCT_END();

struct Workspace : Object {
private:
  std::unordered_set<std::shared_ptr<Object>> object_ptr_test;

public:
  std::shared_ptr<BagPlayer> player;
  std::shared_ptr<History<std::shared_ptr<Workspace>>> history;
  Event<void()> modified{"modified"};
  PROPERTY(std::shared_ptr<Document>, document, std::make_shared<Document>());
  std::shared_ptr<const Snapshot<std::shared_ptr<Document>>> saved_document;
  PROPERTY(Selection, selection);
  PROPERTY(Handle<AnnotationTrack>, currentAnnotationTrack);
  Workspace();
  ~Workspace();
  std::vector<std::string> listTopics(const std::string &type_name);
  std::vector<std::string>
  listTopics(const std::initializer_list<std::string> &type_names);
};
DECLARE_TYPE(Workspace, Object);

struct GlobalEvents {
  static std::shared_ptr<GlobalEvents> instance();
  Event<void()> redraw{"redraw"};
};

class ObjectScope {
public:
  ObjectScope();
  ~ObjectScope();
  ObjectScope(const ObjectScope &) = delete;
  ObjectScope &operator=(const ObjectScope &) = delete;
};

class LockScope : public ObjectScope {
  static std::shared_ptr<Workspace> &workspace_instance();
  static std::recursive_mutex &mutex_instance();
  std::lock_guard<std::recursive_mutex> lock{mutex_instance()};

public:
  LockScope();
  ~LockScope();
  LockScope(const LockScope &) = delete;
  LockScope &operator=(const LockScope &) = delete;
  inline std::shared_ptr<Workspace> &ws() { return workspace_instance(); }
  inline const std::shared_ptr<Workspace> &ws() const {
    return workspace_instance();
  }
  inline std::shared_ptr<Workspace> &operator()() {
    return workspace_instance();
  }
  inline const std::shared_ptr<Workspace> &operator()() const {
    return workspace_instance();
  }
  inline Workspace &operator*() { return *ws(); }
  inline const Workspace &operator*() const { return *ws(); }
  inline Workspace *operator->() { return ws().get(); }
  inline const Workspace *operator->() const { return ws().get(); }
};

class ActionScope : public LockScope {
  std::string label;
  std::shared_ptr<const Object> object;
  bool aggregate = false;

public:
  ActionScope(const std::string &label,
              const std::shared_ptr<const Object> &object = nullptr,
              bool aggregate = false);
  ~ActionScope();
};
