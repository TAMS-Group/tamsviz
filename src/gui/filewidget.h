// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "guicommon.h"

class FileWidget : public QDockWidget {
 public:
  FileWidget();
  ~FileWidget();
  virtual void showEvent(QShowEvent *event) override;
  virtual void hideEvent(QHideEvent *event) override;
};
