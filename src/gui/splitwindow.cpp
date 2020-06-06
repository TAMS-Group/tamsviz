// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "splitwindow.h"

#include "mainwindow.h"

#include "../core/log.h"
#include "../core/workspace.h"

WindowBase::WindowBase() {
  LOG_DEBUG("new split window created");
  setParent(MainWindow::instance());
}
WindowBase::~WindowBase() { LOG_DEBUG("closing split window"); }
void WindowBase::changeEvent(QEvent *event) {
  if (event->type() == QEvent::ParentChange) {
    LOG_DEBUG("parent of " << typeid(*this).name() << " changed to "
                           << (parent() != nullptr ? typeid(*parent()).name()
                                                   : "nullptr"));
  }
}

static void replaceWindow(const std::shared_ptr<Object> &old_window,
                          const std::shared_ptr<Window> &new_window) {
  LockScope ws;
  std::shared_ptr<Object> parent;
  ws->recurse(
      [&](const std::shared_ptr<Object> &p, const std::shared_ptr<Object> &c) {
        if (c == old_window) {
          parent = p;
        }
      });
  if (parent) {
    for (auto &property : parent->properties()) {
      if (property.typeId() == typeid(std::shared_ptr<Window>)) {
        if (property.read().value<std::shared_ptr<Window>>() == old_window) {
          property.assign(Variant(new_window));
        }
      }
    }
  }
}

SplitWindowBase::SplitWindowBase(Qt::Orientation orientation)
    : orientation(orientation) {
  class Splitter : public QWidget {
    SplitWindowBase *parent = nullptr;
    bool aggregate = false;

  public:
    Splitter(SplitWindowBase *parent) : QWidget(parent), parent(parent) {}
    virtual void mousePressEvent(QMouseEvent *event) override {
      event->accept();
      aggregate = false;
    }
    virtual void mouseMoveEvent(QMouseEvent *event) override {
      event->accept();
      if (event->buttons() == Qt::LeftButton) {
        ActionScope ws("Drag Splitter", nullptr, aggregate);
        aggregate = true;
        QPoint pos = parent->mapFromGlobal(event->globalPos());
        double p = 0.5;
        if (parent->orientation == Qt::Vertical) {
          p = (pos.x()) * 1.0 / parent->width();
        } else {
          p = (pos.y()) * 1.0 / parent->height();
        }
        parent->position() = std::max(0.05, std::min(0.95, p));
        LOG_DEBUG("splitter position " << parent->position());
        // parent->sync();
        ws->modified();
      }
    }
    virtual void mouseReleaseEvent(QMouseEvent *event) override {
      event->accept();
    }
  };
  QBoxLayout *layout = nullptr;
  Splitter *splitter = new Splitter(this);
  if (orientation == Qt::Horizontal) {
    layout = new QVBoxLayout(this);
    splitter->setCursor(Qt::SplitVCursor);
  } else {
    layout = new QHBoxLayout(this);
    splitter->setCursor(Qt::SplitHCursor);
  }
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  LockScope ws;
  a() = std::make_shared<EmptyWindow>();
  b() = std::make_shared<EmptyWindow>();
  splitter->setMinimumSize(8, 8);
  this->splitter = splitter;
  sync();
  ws->modified.connect(this, [this, layout, splitter]() { sync(); });
  layout->addWidget(splitter);
  setLayout(layout);
}
SplitWindowBase::~SplitWindowBase() {
  LOG_DEBUG("~SplitWindowBase");
  LOG_DEBUG("mainwindow: " << MainWindow::instance());
  LockScope ws;
  std::dynamic_pointer_cast<WindowBase>(a())->setParent(MainWindow::instance());
  std::dynamic_pointer_cast<WindowBase>(b())->setParent(MainWindow::instance());
}
void SplitWindowBase::sync() {
  LockScope ws;
  auto *layout = (QBoxLayout *)this->layout();
  if (!layout->itemAt(0) ||
      (dynamic_cast<WindowBase *>(a().get()) != layout->itemAt(0)->widget())) {
    if (auto *item = layout->takeAt(0)) {
      item->widget()->setParent(MainWindow::instance());
    }
    layout->insertWidget(0, (WindowBase *)a().get());
  }
  if (layout->count() != 3) {
    layout->takeAt(1);
    layout->insertWidget(1, splitter);
  }
  if (!layout->itemAt(2) ||
      (dynamic_cast<WindowBase *>(b().get()) != layout->itemAt(2)->widget())) {
    if (auto *item = layout->takeAt(2)) {
      item->widget()->setParent(MainWindow::instance());
    }
    layout->insertWidget(2, (WindowBase *)b().get());
  }
  ((WindowBase *)a().get())
      ->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  splitter->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  ((WindowBase *)b().get())
      ->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  int range = (orientation == Qt::Vertical) ? width() : height();
  std::array<int, 3> s;
  s[0] = (int)std::round(position() * range);
  s[1] = 0;
  s[2] = range - s[0];
  for (size_t i = 0; i < s.size(); i++) {
    if (layout->stretch(i) != s[i]) {
      layout->setStretch(i, s[i]);
    }
  }
}

void ContentWindowBase::replace(const std::shared_ptr<Window> &new_window) {
  LOG_DEBUG("replacing window with " << typeid(*new_window).name());
  LockScope ws;
  auto me = std::dynamic_pointer_cast<Window>(this->shared_from_this());
  auto snapshot = Snapshot<std::shared_ptr<Window>>::save(me, nullptr, nullptr);
  QTimer::singleShot(0, this, [snapshot, new_window, this]() {
    ActionScope ws("Change Window Type");
    auto me = std::dynamic_pointer_cast<Window>(this->shared_from_this());
    replaceWindow(me, new_window);
    if (auto split_window =
            std::dynamic_pointer_cast<SplitWindowBase>(new_window)) {
      // TODO: handle recursive ids, maybe just use serializer
      snapshot->apply(split_window->a());
      snapshot->apply(split_window->b());
      split_window->b()->assignNewId();
    }
    ws->modified();
  });
}
ContentWindowBase::ContentWindowBase() {
  // setFrameStyle(QFrame::Sunken || QFrame::Box);
  layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  bar = new QHBoxLayout(this);
  layout->addLayout(bar);
  bar->setSpacing(0);
  layout->setSpacing(0);
  {
    auto *button = new FlatButton();
    button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    QTimer::singleShot(0, this, [button, this]() {
      QMenu *menu = new QMenu(this);
      auto types = Type::find<Window>()->list();
      for (auto &type : types) {
        if (type->typeId() == typeid(SplitWindowHorizontal) ||
            type->typeId() == typeid(SplitWindowVertical)) {
          continue;
        }
        if (!type->constructable()) {
          continue;
        }
        QString label = type->name().c_str();
        label = label.replace("Window", "");
        // label += " ";
        // label += type->typeId().name();
        if (type->typeId() == typeid(*this)) {
          button->setText(label);
        }
        connect(menu->addAction(label), &QAction::triggered, this,
                [type, this](bool checked) {
                  LockScope ws;
                  replace(type->instantiate<Window>());
                });
      }
      button->setMenu(menu);
    });
    bar->addWidget(button);
  }
  {
    class Spacer : public QWidget {
      ContentWindowBase *_parent = nullptr;

    public:
      Spacer(ContentWindowBase *parent) : QWidget(parent), _parent(parent) {}

    protected:
      virtual void mousePressEvent(QMouseEvent *event) override {
        QWidget::mousePressEvent(event);
        LockScope ws;
        ws->selection() = _parent->shared_from_this();
        ws->modified();
      }
    };
    spacer = new Spacer(this);
    spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    bar->addWidget(spacer);
  }

  {
    auto *button = new FlatButton();
    // button->setIcon(createIcon(QChar((int)0x1F031)));
    // button->setIcon(createIcon(QChar(0x2015)));
    // button->setIcon(MATERIAL_ICON("vertical_split"));
    button->setIcon(MATERIAL_ICON("border_vertical"));
    button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    connect(button, &QPushButton::clicked, this, [this]() {
      LockScope ws;
      replace(std::make_shared<SplitWindowVertical>());
    });
    bar->addWidget(button);
  }
  {
    auto *button = new FlatButton();
    // button->setIcon(createIcon(QChar(0x007c)));
    // button->setIcon(MATERIAL_ICON("horizontal_split"));
    button->setIcon(MATERIAL_ICON("border_horizontal"));
    button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    connect(button, &QPushButton::clicked, this, [this]() {
      LockScope ws;
      replace(std::make_shared<SplitWindowHorizontal>());
    });
    bar->addWidget(button);
  }
  {
    // auto *button = new QPushButton(QString(QChar(0x2715)), this);
    auto *button = new FlatButton();
    // button->setIcon(createIcon(QChar(0x2715)));
    button->setIcon(style()->standardIcon(QStyle::SP_DockWidgetCloseButton));
    button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    bar->addWidget(button);
    connect(button, &QAbstractButton::clicked, this, [this]() {
      QTimer::singleShot(0, this, [this]() {
        // setUpdatesEnabled(false);
        LockScope ws;
        auto me = std::dynamic_pointer_cast<Window>(this->shared_from_this());
        std::shared_ptr<Object> window_old;
        std::shared_ptr<Window> window_new;
        ws->recurse([&](const std::shared_ptr<Object> &window) {
          if (auto split = std::dynamic_pointer_cast<SplitWindowBase>(window)) {
            if (split->a() == me) {
              window_old = split;
              window_new = split->b();
              split->b() = std::make_shared<EmptyWindow>();
            }
            if (split->b() == me) {
              window_old = split;
              window_new = split->a();
              split->a() = std::make_shared<EmptyWindow>();
            }
          }
          if (auto parent = std::dynamic_pointer_cast<Document>(window)) {
            if (typeid(*me) != typeid(EmptyWindow)) {
              window_old = me;
              window_new = std::make_shared<EmptyWindow>();
            }
          }
        });
        if (window_old) {
          ActionScope action("Close Split Window");
          replaceWindow(window_old, window_new);
          ws->modified();
        }
        // setUpdatesEnabled(true);
      });
    });
  }
  setContentWidget(new QWidget(this));
  setLayout(layout);
}
void ContentWindowBase::addToolWidget(QWidget *widget) {
  widget->setParent(this);
  bar->insertWidget(bar->indexOf(spacer), widget);
}
void ContentWindowBase::addToolWidgetRight(QWidget *widget) {
  widget->setParent(this);
  bar->insertWidget(bar->indexOf(spacer) + 1, widget);
}

void ContentWindowBase::setContentWidget(QWidget *widget) {
  if (content_window) {
    layout->removeWidget(content_window);
    delete content_window;
  }
  content_window = widget;
  content_window->setParent(this);
  content_window->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  layout->addWidget(content_window);
  layout->setStretch(0, 0);
  layout->setStretch(1, 100);
}

EmptyWindow::EmptyWindow() {
  /*auto *label = new QLabel("Empty");
  label->setStyleSheet("background: #888;");
  label->setAlignment(Qt::AlignCenter);
  setContentWidget(label);
  */
  auto *view = new QWidget(this);
  view->setStyleSheet("background: #000;");
  setContentWidget(view);
}
