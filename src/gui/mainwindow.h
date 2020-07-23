// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "guicommon.h"

class MainWindow : public QMainWindow {

  QMenu *open_recent_menu = nullptr;

protected:
  virtual bool event(QEvent *event) override;

public:
  bool closeDocument();
  bool closeBag();
  bool saveDocument();
  bool saveDocument(const QString &path);
  bool saveDocumentAs();
  void addRecentFile(const QString &path);
  void openBag(const QString &path);
  void openBrowse();
  void openDocument(const QString &path);
  void openAny(const QString &path);
  void updateRecentMenu();

  MainWindow();
  ~MainWindow();
  virtual void closeEvent(QCloseEvent *event) override;
  static MainWindow *instance();
};
