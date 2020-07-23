// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "guicommon.h"

#include "../core/type.h"

#include <map>
#include <unordered_map>

class ActionScope;
class Property;
class View;

class PropertyGridWidget : public QDockWidget {
  std::shared_ptr<Property> drag_property;
  QTreeWidgetItem *drag_item = nullptr;
  TreeWidget *view = nullptr;
  uint64_t object_id = 0;
  std::weak_ptr<Object> object_ptr;
  void sync(QTreeWidgetItem *parent_item, std::vector<Property> properties);
  void sync();
  bool isPropertyDraggable(QTreeWidgetItem *item);

public:
  PropertyGridWidget();
};
