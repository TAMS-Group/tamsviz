// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "event.h"

#include "log.h"

#include <QObject>
#include <QPointer>

bool EventBase::listenerCheckQPointer(
    const std::shared_ptr<QPointer<const QObject>> &p) {
  QPointer<const QObject> q = *p;
  return (bool)q;
}

std::shared_ptr<QPointer<const QObject>>
EventBase::listenerMakeQPointer(const QObject *o) {
  return std::make_shared<QPointer<const QObject>>(o);
}

void EventBase::begin() {
  if (!name.empty()) {
    // LOG_DEBUG("event " << name << " begin");
  }
}

void EventBase::end() {
  if (!name.empty()) {
    // LOG_DEBUG("event " << name << " end");
  }
}
