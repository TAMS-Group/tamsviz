// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "guicommon.h"

#include "../core/document.h"
#include "../core/tracks.h"

#include <atomic>
#include <typeindex>

class WindowBase : public Window, public QFrame {

protected:
  WindowBase();
  ~WindowBase();

protected:
  void changeEvent(QEvent *event);
};
DECLARE_TYPE(WindowBase, Window);

class ContentWindowBase : public WindowBase {
  QHBoxLayout *bar = nullptr;
  QVBoxLayout *layout = nullptr;
  QWidget *spacer = nullptr;
  QWidget *content_window = nullptr;
  // QStaticText _annotation_hud_text;
  std::string _annotation_hud_string;

protected:
  ContentWindowBase();
  void setContentWidget(QWidget *widget);
  void addToolWidget(QWidget *widget);
  void addToolWidgetRight(QWidget *widget);
  void replace(const std::shared_ptr<Window> &new_window);
  QWidget *contentWidget() { return content_window; }
  void paintAnnotationHUD(QPainter *painter,
                          const std::shared_ptr<const Type> &type);
};
DECLARE_TYPE(ContentWindowBase, WindowBase);

class EmptyWindow : public ContentWindowBase {
public:
  EmptyWindow();
};
DECLARE_TYPE(EmptyWindow, ContentWindowBase);

class SplitWindowBase : public WindowBase {
  Qt::Orientation orientation;
  QWidget *splitter = nullptr;

protected:
  SplitWindowBase(Qt::Orientation orientation);
  ~SplitWindowBase();
  void sync();

public:
  PROPERTY(double, position, 0.5);
  PROPERTY(std::shared_ptr<Window>, a, nullptr);
  PROPERTY(std::shared_ptr<Window>, b, nullptr);
};
DECLARE_TYPE(SplitWindowBase, WindowBase);

class SplitWindowHorizontal : public SplitWindowBase {
public:
  SplitWindowHorizontal() : SplitWindowBase(Qt::Horizontal) {}
};
DECLARE_TYPE(SplitWindowHorizontal, SplitWindowBase);

class SplitWindowVertical : public SplitWindowBase {
public:
  SplitWindowVertical() : SplitWindowBase(Qt::Vertical) {}
};
DECLARE_TYPE(SplitWindowVertical, SplitWindowBase);
