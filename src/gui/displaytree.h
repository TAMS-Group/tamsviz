// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "guicommon.h"

class Object;

class DisplayTreeWidget : public QDockWidget {
  TreeWidget *view = nullptr;
  // QAbstractButton *delete_button = nullptr;
  std::vector<
      std::pair<std::weak_ptr<Object>, std::shared_ptr<QTreeWidgetItem>>>
      items;
  void sync();
  bool updating = false;

public:
  DisplayTreeWidget();
  ~DisplayTreeWidget();
};
