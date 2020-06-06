// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "displaytree.h"

#include "../core/log.h"
#include "../core/workspace.h"

#include <unordered_map>

DisplayTreeWidget::DisplayTreeWidget() : QDockWidget("Display Tree") {
  LockScope ws;
  class TreeWidget2 : public TreeWidget {
    DisplayTreeWidget *parent = nullptr;

  public:
    TreeWidget2(DisplayTreeWidget *parent) : parent(parent) {
      setParent(parent);
    }
    virtual void dropEvent(QDropEvent *event) override {
      LOG_DEBUG("drop");
      LockScope ws;
      LOG_DEBUG(currentItem()->text(0).toStdString());
      QTreeWidgetItem *dragged_item = currentItem();
      QTreeWidget::dropEvent(event);
      QTreeWidgetItem *new_parent_item = dragged_item->parent();
      std::shared_ptr<Object> dragged_object;
      std::shared_ptr<DisplayGroupBase> new_parent_display;
      for (auto &item : parent->items) {
        if (item.second.get() == dragged_item) {
          dragged_object = item.first.lock();
        }
        if (item.second.get() == new_parent_item) {
          new_parent_display =
              std::dynamic_pointer_cast<DisplayGroupBase>(item.first.lock());
        }
      }
      if (!dragged_object) {
        LOG_ERROR("dragged object lost");
        ws->modified();
        return;
      }
      if (!new_parent_display) {
        LOG_WARN("no drop target");
        ws->modified();
        return;
      }
      std::shared_ptr<DisplayGroupBase> old_parent_display;
      ws()->document()->display()->recurse(
          [&](const std::shared_ptr<Display> &parent,
              const std::shared_ptr<Display> &child) {
            if (child == dragged_object) {
              old_parent_display =
                  std::dynamic_pointer_cast<DisplayGroupBase>(parent);
            }
          });
      if (!old_parent_display) {
        LOG_ERROR("parent display lost");
        ws->modified();
        return;
      }
      if (old_parent_display == new_parent_display) {
        LOG_DEBUG("same parent");
        ws->modified();
        return;
      }
      if (auto dragged_display =
              std::dynamic_pointer_cast<Display>(dragged_object)) {
        ActionScope action("Drag&Drop");
        bool erased = false;
        for (size_t i = 0; i < old_parent_display->displays().size(); i++) {
          if (old_parent_display->displays()[i] == dragged_object) {
            old_parent_display->displays().erase(
                old_parent_display->displays().begin() + i);
            erased = true;
            break;
          }
        }
        if (erased) {
          new_parent_display->displays().push_back(dragged_display);
        }
      }
    }
  };
  view = new TreeWidget2(this);
  view->setSelectionMode(QAbstractItemView::ExtendedSelection);
  view->setEditTriggers(QAbstractItemView::DoubleClicked |
                        QAbstractItemView::EditKeyPressed |
                        QAbstractItemView::AnyKeyPressed);
  view->setDragEnabled(true);
  view->viewport()->setAcceptDrops(true);
  view->setDropIndicatorShown(true);
  view->setDragDropMode(QAbstractItemView::InternalMove);
  QWidget *content = new QWidget();
  auto *layout = new QVBoxLayout(content);
  layout->setContentsMargins(0, 0, 0, 0);
  auto *bar = new QHBoxLayout(content);
  bar->setSpacing(0);
  layout->setSpacing(0);
  {
    auto *button = new FlatButton("Create");
    button->setIcon(TEXT_ICON("+"));
    bar->addWidget(button);
    QMenu *menu = new QMenu(button);
    std::map<QString, std::function<void()>> actions;
    for (auto &type : Type::find<Display>()->list()) {
      if (type->typeId() == typeid(WorldDisplay)) {
        continue;
      }
      if (!type->constructable()) {
        continue;
      }
      actions[QString(type->name().c_str()).replace("Display", "")] = [type]() {
        ActionScope ws(std::string("Create ") + type->name());
        auto new_display = type->instantiate<Display>();
        static size_t counter = 1;
        new_display->name(std::string() + type->name() + "" +
                          std::to_string(counter++));
        ws->document()->display()->displays().push_back(new_display);
        ws->selection() = new_display;
        ws->modified();
      };
    }
    for (auto p : actions) {
      connect(menu->addAction(p.first), &QAction::triggered, this,
              [p](bool checked) { p.second(); });
    }
    button->setMenu(menu);
  }
  layout->addLayout(bar);
  layout->addWidget(view);
  setWidget(content);
  view->setHeaderHidden(true);
  connect(view, &QTreeWidget::itemChanged, this,
          [this](QTreeWidgetItem *item, int column) {
            LockScope ws;
            if (column == 0) {
              for (auto &p : items) {
                if (p.second.get() == item) {
                  if (auto display =
                          std::dynamic_pointer_cast<Display>(p.first.lock())) {
                    ActionScope ws("Name", display);
                    display->name(item->text(0).toStdString());
                    ws->modified();
                  }
                }
              }
            }
          });
  connect(view, &QTreeWidget::itemSelectionChanged, this, [this]() {
    LOG_DEBUG("selection changed");
    LockScope ws;
    if (updating)
      return;
    ws->selection().clear();
    for (auto *item : view->selectedItems()) {
      for (auto &p : items) {
        if (p.second.get() == item) {
          if (auto object = p.first.lock()) {
            ws->selection().add(object);
          }
        }
      }
    };
    ws->modified();
  });
  sync();
  ws->modified.connect(this, [this]() { sync(); });
  QTimer::singleShot(0, this, [this] { view->setFocus(Qt::OtherFocusReason); });
}

void DisplayTreeWidget::sync() {
  LockScope ws;
  updating = true;
  std::unordered_set<std::shared_ptr<Object>> all_objects;
  ws->document()->display()->recurse(
      [&](const std::shared_ptr<Object> &o) { all_objects.insert(o); });
  std::unordered_map<std::shared_ptr<Object>, std::shared_ptr<QTreeWidgetItem>>
      object_to_item;
  {
    for (auto &p : items) {
      std::shared_ptr<Object> object = p.first.lock();
      if (object && all_objects.find(object) != all_objects.end()) {
        object_to_item[object] = p.second;
      } else {
        p.second->takeChildren();
      }
    }
    items.clear();
  }
  ws->document()->display()->recurse(
      [&object_to_item](const std::shared_ptr<Display> &display) {
        if (object_to_item.find(display) == object_to_item.end()) {
          auto item = std::make_shared<QTreeWidgetItem>();
          item->setFlags(item->flags() | Qt::ItemIsEditable);
          object_to_item[display] = item;
        }
      });
  ws->document()->display()->recurse(
      [&object_to_item, this](const std::shared_ptr<Display> &parent,
                              const std::shared_ptr<Display> &display) {
        LockScope ws;
        std::shared_ptr<QTreeWidgetItem> parent_item;
        if (parent) {
          parent_item = object_to_item[parent];
        }
        std::shared_ptr<QTreeWidgetItem> item = object_to_item[display];
        if (display->name().empty()) {
          item->setText(0, display->type()->name().c_str());
        } else {
          item->setText(0, display->name().c_str());
        }
        item->setSelected(ws->selection().contains(display));
        if (item->parent() != parent_item.get()) {
          if (item->parent()) {
            item->parent()->removeChild(item.get());
          } else {
            this->view->takeTopLevelItem(
                this->view->indexOfTopLevelItem((item.get())));
          }
          if (parent_item) {
            parent_item->addChild(item.get());
          } else {
            this->view->addTopLevelItem(item.get());
          }
        }
        if (item->treeWidget() == nullptr) {
          this->view->addTopLevelItem(item.get());
        }
      });
  for (auto &p : object_to_item) {
    items.emplace_back(p.first, p.second);
  }
  updating = false;
}

DisplayTreeWidget::~DisplayTreeWidget() {
  LockScope ws;
  LOG_DEBUG("~DisplayTreeWidget begin" << parent());
  view->setCurrentItem(nullptr);
  for (auto &item : items) {
    item.second->takeChildren();
  }
  LOG_DEBUG("~DisplayTreeWidget clear" << parent());
  items.clear();
  LOG_DEBUG("~DisplayTreeWidget end" << parent());
}
