// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "workspace.h"

#include "bagplayer.h"
#include "history.h"
#include "log.h"
#include "topic.h"
#include "transformer.h"

#include <QTimer>

std::shared_ptr<GlobalEvents> GlobalEvents::instance() {
  static auto instance = std::make_shared<GlobalEvents>();
  return instance;
}

void Selection::operator=(const std::shared_ptr<Object> &o) {
  _objects.clear();
  if (o) {
    _objects = {o->id()};
  }
}
bool Selection::contains(const std::shared_ptr<Object> &o) const {
  if (o) {
    for (auto id : _objects) {
      if (id == o->id()) {
        return true;
      }
    }
  }
  return false;
}
std::vector<std::shared_ptr<Object>>
Selection::resolve(const std::shared_ptr<Object> &root) const {
  std::vector<std::shared_ptr<Object>> ret;
  root->recurseObjects([&](const std::shared_ptr<Object> &o) {
    for (auto id : _objects) {
      if (o->id() == id) {
        ret.push_back(o);
      }
    }
  });
  return ret;
}
void Selection::clear() { _objects.clear(); }
void Selection::add(const std::shared_ptr<Object> &object) {
  if (object) {
    for (auto &id : _objects) {
      if (id == object->id()) {
        return;
      }
    }
    _objects.push_back(object->id());
  }
}
bool Selection::empty() const { return _objects.empty(); }
size_t Selection::size() const { return _objects.size(); }
void Selection::toggle(const std::shared_ptr<Object> &object) {
  if (object) {
    if (contains(object)) {
      erase(object);
    } else {
      add(object);
    }
  }
}
void Selection::erase(const std::shared_ptr<Object> &object) {
  if (object) {
    for (auto it = _objects.begin(); it != _objects.end();) {
      if (*it == object->id()) {
        it = _objects.erase(it);
      } else {
        it++;
      }
    }
  }
}
bool operator==(const Selection &a, const Selection &b) {
  if (a._objects.size() != b._objects.size()) {
    return false;
  }
  for (size_t i = 0; i < a._objects.size(); i++) {
    if (a._objects.at(i) != b._objects.at(i)) {
      return false;
    }
  }
  return true;
}
bool operator!=(const Selection &a, const Selection &b) { return !(a == b); }

Workspace::Workspace() {

  history = std::make_shared<History<std::shared_ptr<Workspace>>>();

  if (0) {
    modified.connect([this]() {
      recurseObjects([this](const std::shared_ptr<Object> &o) {
        if (o && o.get() != this) {
          object_ptr_test.insert(o);
        }
      });
    });
  }

  modified.connect([this]() {
    document()->display()->refreshRecursive();
    if (document()->window()) {
      document()->window()->refreshRecursive();
    }
  });
}
Workspace::~Workspace() { object_ptr_test.clear(); }
std::vector<std::string> Workspace::listTopics(const std::string &type_name) {
  if (player) {
    return player->listTopics(type_name);
  } else {
    return TopicManager::instance()->listTopics(type_name);
  }
}
std::vector<std::string>
Workspace::listTopics(const std::initializer_list<std::string> &type_names) {
  std::set<std::string> ret;
  for (auto &type_name : type_names) {
    for (auto &v : listTopics(type_name)) {
      ret.insert(v);
    }
  }
  return std::vector<std::string>(ret.begin(), ret.end());
}

ObjectScope::ObjectScope() { Property::unlockScope(+1); }
ObjectScope::~ObjectScope() { Property::unlockScope(-1); }

LockScope::LockScope() {
  auto &w = ws();
  if (w && w->history && !w->history->current) {
    auto item = std::make_shared<History<std::shared_ptr<Workspace>>::Item>();
    item->snapshot =
        Snapshot<std::shared_ptr<Workspace>>::save(ws(), nullptr, nullptr);
    ws()->history->current = item;
  }
}
LockScope::~LockScope() {}

std::shared_ptr<Workspace> &LockScope::workspace_instance() {
  static std::shared_ptr<Workspace> instance = std::make_shared<Workspace>();
  return instance;
}
static std::recursive_mutex g_mutex_instance;
std::recursive_mutex &LockScope::mutex_instance() { return g_mutex_instance; }

static std::vector<ActionScope *> g_current_action_scopes;
ActionScope::ActionScope(const std::string &label,
                         const std::shared_ptr<const Object> &object,
                         bool aggregate)
    : label(label), object(object), aggregate(aggregate) {
  LOG_INFO("begin " << label);
  ws()->history->locked = true;
  g_current_action_scopes.push_back(this);
  if (!ws()->history->current) {
    auto item = std::make_shared<History<std::shared_ptr<Workspace>>::Item>();
    item->snapshot =
        Snapshot<std::shared_ptr<Workspace>>::save(ws(), nullptr, nullptr);
    ws()->history->current = item;
  }
}
ActionScope::~ActionScope() {
  LOG_INFO("done " << label);
  if (g_current_action_scopes.back() != this) {
    throw std::runtime_error("overlapping actions");
  }
  g_current_action_scopes.pop_back();
  {
    std::shared_ptr<const Snapshot<std::shared_ptr<Workspace>>> new_snapshot =
        Snapshot<std::shared_ptr<Workspace>>::save(
            ws(), ws()->history->current->snapshot,
            [&](const std::shared_ptr<Object> &o) {
              return object == nullptr || o == ws() || o == object;
            });
    if (new_snapshot != ws()->history->current->snapshot) {
      auto item = std::make_shared<History<std::shared_ptr<Workspace>>::Item>();
      item->label = label;
      item->snapshot = new_snapshot;
      if (aggregate && ws()->history->current->label == label &&
          ws()->history->canUndo()) {
        ws()->history->current = item;
      } else {
        ws()->history->undo_stack.push_back(ws()->history->current);
        if (ws()->history->undo_stack.size() > ws()->history->limit) {
          ws()->history->undo_stack.pop_front();
        }
        ws()->history->redo_stack.clear();
        ws()->history->current = item;
      }
    }
  }
  ws()->history->locked = false;
  QTimer::singleShot(0, []() {
    LockScope ws;
    ws->modified();
  });
}
