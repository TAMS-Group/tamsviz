// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <QtWidgets>

#include <memory>
#include <vector>

#include "../core/log.h"

class TreeWidget : public QTreeWidget {
public:
  QModelIndex indexFromItem(QTreeWidgetItem *item, int column = 0) const {
    return QTreeWidget::indexFromItem(item, column);
  }
  QTreeWidgetItem *itemFromIndex(const QModelIndex &index) const {
    return QTreeWidget::itemFromIndex(index);
  }
};

void startOnMainThreadAsync(const std::function<void()> &fun);

void startOnObjectThreadAsync(QObject *object,
                              const std::function<void()> &fun);

QIcon _createTextIcon(const QString &str);
#define TEXT_ICON(str)                                                         \
  ([]() {                                                                      \
    static QIcon icon = _createTextIcon(str);                                  \
    return icon;                                                               \
  }())

QIcon _createMaterialIcon(const QString &str, double padding = 0.1);
#define MATERIAL_ICON(...)                                                     \
  ([]() {                                                                      \
    static QIcon icon = _createMaterialIcon(__VA_ARGS__);                      \
    return icon;                                                               \
  }())

QIcon _create_FA_R_Icon(const QString &str, double padding = 0.1);
#define FA_R_ICON(...)                                                         \
  ([]() {                                                                      \
    static QIcon icon = _create_FA_R_Icon(__VA_ARGS__);                        \
    return icon;                                                               \
  }())

QIcon _create_FA_S_Icon(const QString &str, double padding = 0.1);
#define FA_S_ICON(...)                                                         \
  ([]() {                                                                      \
    static QIcon icon = _create_FA_S_Icon(__VA_ARGS__);                        \
    return icon;                                                               \
  }())

class FlatButton : public QPushButton {
  void init();

public:
  FlatButton() { init(); }
  FlatButton(const QIcon &icon) {
    setIcon(icon);
    init();
  }
  FlatButton(const QIcon &icon, const QString &text) {
    setIcon(icon);
    setText(text);
    init();
  }
  FlatButton(const QString &text) {
    setText(text);
    init();
  }
  virtual QSize minimumSizeHint() const override;
  virtual QSize sizeHint() const override;
  virtual void paintEvent(QPaintEvent *event) override;
  ~FlatButton();
};
