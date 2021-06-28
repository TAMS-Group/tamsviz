// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "guicommon.h"

class Object;

class DisplayTreeWidget : public QDockWidget {
  TreeWidget *view = nullptr;
  std::vector<
      std::pair<std::weak_ptr<Object>, std::shared_ptr<QTreeWidgetItem>>>
      items;
  void sync();
  bool updating = false;

public:
  DisplayTreeWidget();
  ~DisplayTreeWidget();
};
