// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "propertygrid.h"

#include "../core/history.h"
#include "../core/log.h"
#include "../core/serialization.h"
#include "../core/workspace.h"

Q_DECLARE_METATYPE(Property);
static constexpr int PropertyRole = Qt::UserRole + 1;
static Property itemProperty(const QModelIndex &index) {
  return index.data(PropertyRole).value<Property>();
}
static Property itemProperty(const QTreeWidgetItem *item) {
  return item->data(1, PropertyRole).value<Property>();
}

PropertyGridWidget::PropertyGridWidget() : QDockWidget("Properties") {
  LockScope ws;
  class PropertyTreeWidget : public TreeWidget {
    PropertyGridWidget *property_grid = nullptr;
    QPoint drag_start_pos;
    double drag_start_value = 0.0;
    bool aggregate = false;

  public:
    PropertyTreeWidget(PropertyGridWidget *property_grid)
        : property_grid(property_grid) {
      setParent(property_grid);
    }
    /*virtual void paintEvent(QPaintEvent *event) override {
      TreeWidget::paintEvent(event);
  }*/
    virtual void mousePressEvent(QMouseEvent *event) override {
      QTreeWidget::mousePressEvent(event);
      if (property_grid->drag_property) {
        LockScope ws;
        drag_start_pos = event->pos();
        auto object = property_grid->object_ptr.lock();
        if (!object) {
          LOG_DEBUG("object is null");
          return;
        }
        auto &t = property_grid->drag_property->typeId();
        std::string value_string;
        property_grid->drag_property->tryToString(value_string);
        drag_start_value = std::stod(value_string);
      }
      aggregate = false;
    }
    virtual void mouseMoveEvent(QMouseEvent *event) override {
      if (property_grid->drag_property) {
        LockScope ws;
        auto object = property_grid->object_ptr.lock();
        if (!object) {
          LOG_DEBUG("object is null");
          return;
        }
        auto &t = property_grid->drag_property->typeId();
        double value = 0.0;
        if (std::isfinite(property_grid->drag_property->attributes()->min) &&
            std::isfinite(property_grid->drag_property->attributes()->max)) {
          value = drag_start_value + (event->x() - event->y() -
                                      drag_start_pos.x() + drag_start_pos.y()) *
                                         1.0 / columnWidth(1);
        } else {
          int step_div = 2;
          double step_size = 0.005;
          if (t != typeid(float) && t != typeid(double)) {
            step_div = 10;
            step_size = 1;
          }
          if (std::isfinite(
                  property_grid->drag_property->attributes()->step_scale)) {
            step_size *= property_grid->drag_property->attributes()->step_scale;
          }
          int delta = (event->x() - event->y() - drag_start_pos.x() +
                       drag_start_pos.y()) /
                      step_div;
          value = drag_start_value + delta * step_size;
        }
        if (property_grid->drag_property->attributes()->wrap) {
          value -= property_grid->drag_property->attributes()->min;
          value /= property_grid->drag_property->attributes()->max -
                   property_grid->drag_property->attributes()->min;
          value -= std::floor(value);
          value *= property_grid->drag_property->attributes()->max -
                   property_grid->drag_property->attributes()->min;
          value += property_grid->drag_property->attributes()->min;
        } else {
          if (value < property_grid->drag_property->attributes()->min) {
            value = property_grid->drag_property->attributes()->min;
          }
          if (value > property_grid->drag_property->attributes()->max) {
            value = property_grid->drag_property->attributes()->max;
          }
        }
        LOG_INFO("drag " << value << " "
                         << property_grid->drag_property->attributes()->min
                         << " "
                         << property_grid->drag_property->attributes()->max);
        ActionScope action(std::string("Drag Property ") +
                               property_grid->drag_property->displayName(),
                           /*object*/ nullptr, aggregate);
        aggregate = true;
        property_grid->drag_property->fromStringOrThrow(std::to_string(value));
        ws->modified();
      } else {
        QTreeWidget::mouseMoveEvent(event);
      }
    }
    virtual void mouseReleaseEvent(QMouseEvent *event) override {
      unsetCursor();

      aggregate = false;
      property_grid->drag_property = nullptr;
      property_grid->drag_item = nullptr;
      QTreeWidget::mouseReleaseEvent(event);
    }
  };
  view = new PropertyTreeWidget(this);
  setWidget(view);
  ws->modified.connect(this, [this]() { sync(); });
  view->setColumnCount(2);
  view->setEditTriggers(QAbstractItemView::DoubleClicked |
                        QAbstractItemView::EditKeyPressed |
                        QAbstractItemView::AnyKeyPressed);
  view->setSelectionMode(QAbstractItemView::SingleSelection);
  view->setHeaderLabels({"Name", "Value"});

  view->header()->setSectionResizeMode(1, QHeaderView::Stretch);
  view->setIndentation(10);

  class ItemDelegateBase : public QStyledItemDelegate {
    PropertyGridWidget *_parent = nullptr;

  public:
    ItemDelegateBase(PropertyGridWidget *parent)
        : QStyledItemDelegate(parent), _parent(parent) {}
    void drawLines(QPainter *painter, const QStyleOptionViewItem &option,
                   const QModelIndex &index) const {
      painter->setPen(QPen(QApplication::palette().brush(QPalette::Window), 0));
      painter->setBrush(QBrush(Qt::transparent));

      auto rect = option.rect;
      rect.setX(_parent->view->columnViewportPosition(index.column()));
      rect.setWidth(_parent->view->columnWidth(index.column()));
      if (index.column() == 0) {
        rect.setX(rect.x() - 1);
        rect.setWidth(rect.width() + 1);
      }
      painter->drawRect(rect);
    }
    void paint(QPainter *painter, const QStyleOptionViewItem &option,
               const QModelIndex &index) const {
      QStyledItemDelegate::paint(painter, option, index);
      drawLines(painter, option, index);
    }
  };
  {
    class EditDelegate : public ItemDelegateBase {
      PropertyGridWidget *property_grid = nullptr;

    public:
      EditDelegate(PropertyGridWidget *property_grid)
          : ItemDelegateBase(property_grid), property_grid(property_grid) {}
      virtual QWidget *createEditor(QWidget *parent,
                                    const QStyleOptionViewItem &option,
                                    const QModelIndex &index) const override {

        return nullptr;
      }
    };
    view->setItemDelegateForColumn(0, new EditDelegate(this));
  }

  connect(view, &QTreeWidget::currentItemChanged, view, [this]() {
    if (view->currentColumn() != 1) {
      view->setCurrentItem(view->currentItem(), 1);
    }
  });
  {
    class EditDelegate : public ItemDelegateBase {
      PropertyGridWidget *property_grid = nullptr;

    public:
      EditDelegate(PropertyGridWidget *property_grid)
          : ItemDelegateBase(property_grid), property_grid(property_grid) {}
      virtual void paint(QPainter *painter, const QStyleOptionViewItem &option,
                         const QModelIndex &index) const override {

        drawLines(painter, option, index);

        auto rect = option.rect.marginsRemoved(QMargins(1, 1, 0, 0));

        {
          auto color = ((option.state & QStyle::State_Active) &&
                        (option.state & QStyle::State_Selected))
                           ? option.palette.color(QPalette::Highlight)
                           : option.palette.color(QPalette::Base);
          painter->fillRect(rect, QBrush(color));
        }

        if (auto object = property_grid->object_ptr.lock()) {
          Property property = itemProperty(index);
          if (std::isfinite(property.attributes()->min) &&
              std::isfinite(property.attributes()->max)) {
            double v = index.data(Qt::DisplayRole).toDouble();
            rect.setWidth(std::round(
                rect.width() *
                ((v - property.attributes()->min) /
                 (property.attributes()->max - property.attributes()->min))));
            {
              auto color = option.palette.color(QPalette::Text);
              color.setAlphaF(0.1);
              if ((option.state & QStyle::State_Active) &&
                  (option.state & QStyle::State_Selected)) {
                color.setAlphaF(1.0 / 3.0);
              }
              painter->fillRect(rect, QBrush(color));
            }
          }
        }

        QStyleOptionViewItem opt = option;
        initStyleOption(&opt, index);
        opt.backgroundBrush = QBrush(Qt::transparent);
        opt.state &= ~QStyle::State_Selected;
        if ((option.state & QStyle::State_Active) &&
            (option.state & QStyle::State_Selected)) {
          opt.palette.setBrush(QPalette::Text, opt.palette.highlightedText());
        }
        QStyle *style =
            (option.widget ? option.widget->style() : QApplication::style());
        style->drawControl(QStyle::CE_ItemViewItem, &opt, painter,
                           option.widget);
      }
      virtual bool editorEvent(QEvent *event, QAbstractItemModel *model,
                               const QStyleOptionViewItem &option,
                               const QModelIndex &index) override {
        LockScope ws;
        bool ret =
            QStyledItemDelegate::editorEvent(event, model, option, index);
        if (auto *item = property_grid->view->itemFromIndex(index)) {
          if (event->type() == QEvent::MouseButtonPress) {
            if (auto *mouse_event = dynamic_cast<QMouseEvent *>(event)) {
              if (mouse_event->button() == Qt::LeftButton) {
                auto current_property = itemProperty(index);
                if (property_grid->isPropertyDraggable(item)) {
                  property_grid->drag_property =
                      std::make_shared<Property>(current_property);
                  property_grid->drag_item = item;
                  property_grid->view->setCursor(Qt::ClosedHandCursor);
                }
                if (current_property.typeId() == typeid(bool)) {
                  ActionScope ws("Toggle");
                  bool v = current_property.read().value<bool>();
                  v = !v;
                  current_property.assign(Variant(v));
                  ws->modified();
                }
              }
            }
          }
        }
        return ret;
      }
      virtual QWidget *createEditor(QWidget *parent,
                                    const QStyleOptionViewItem &option,
                                    const QModelIndex &index) const override {
        LockScope ws;
        auto *item = property_grid->view->itemFromIndex(index);
        if (!item) {
          LOG_ERROR("tree item not found");
          return nullptr;
        }
        auto current_property = itemProperty(index);
        if (current_property.typeId() == typeid(bool)) {
          LOG_DEBUG("bool");
          ActionScope ws("Toggle");
          bool v = current_property.read().value<bool>();
          v = !v;
          current_property.assign(Variant(v));
          ws->modified();
          return nullptr;
        }
        if (current_property.fromStringSupported()) {
          LOG_DEBUG("editable");
          if (current_property.attributes()->list) {
            LOG_DEBUG("combo box");
            auto *editor = new QComboBox(parent);
            editor->setEditable(true);
            editor->setFrame(false);
            editor->setStyleSheet("border: none;");
            for (auto &v :
                 current_property.attributes()->list(current_property)) {
              editor->addItem(v.c_str());
            }
            connect(
                editor,
                static_cast<void (QComboBox::*)(int)>(&QComboBox::activated),
                [editor, index, this](int i) {
                  setModelData(editor, property_grid->view->model(), index);
                });
            return editor;
          } else {
            LOG_DEBUG("line edit");

            auto editor = new QLineEdit(parent);
            editor->setFrame(false);
            if (auto complete = current_property.attributes()->complete) {
              auto *completion_model = new QStringListModel(editor);
              auto *completer = new QCompleter(completion_model, editor);
              completer->setCaseSensitivity(Qt::CaseInsensitive);
              completer->setCompletionMode(
                  QCompleter::UnfilteredPopupCompletion);
              completer->setMaxVisibleItems(20);
              completer->setWidget(editor);
              auto update = [current_property, completion_model, complete,
                             editor, completer](bool show) {
                QString text = editor->text();
                QStringList l;
                AutoCompletion completion;
                {
                  LockScope ws;
                  complete(current_property, text.toStdString(), completion);
                  for (auto &s : completion.items) {
                    QString qs = QString(s.c_str());
                    l.push_back(qs);
                  }
                }
                l.sort();
                {
                  QStringList a, b;
                  for (auto &s : l) {
                    if (s.startsWith(text) && s.size() > text.size()) {
                      a.push_back(s);
                    } else {
                      b.push_back(s);
                    }
                  }
                  l = a + b;
                }
                if (!l.isEmpty()) {
                  completion_model->setStringList(l);
                  completer->complete();
                  if (!completion.completed || show) {
                    LOG_DEBUG("show auto completion list");
                    completer->popup()->show();
                  } else {
                    LOG_DEBUG("hide auto completion list");
                    completer->popup()->hide();
                  }
                } else {
                  LOG_DEBUG("not auto completions, hide list");
                  completer->popup()->hide();
                }
              };
              connect(editor, &QLineEdit::textEdited, editor,
                      [update, completer](const QString &) { update(false); });
              connect(completer,
                      static_cast<void (QCompleter::*)(const QString &text)>(
                          &QCompleter::activated),
                      [editor, update, completer](const QString &text) {
                        if (text != editor->text()) {
                          editor->setText(text);
                          update(false);
                        }
                      });
              QTimer::singleShot(0, completer,
                                 [completer, update]() { update(true); });
            }

            return editor;
          }
        } else {
          LOG_DEBUG("not editable");
          return nullptr;
        }
      }
      virtual void setModelData(QWidget *editor, QAbstractItemModel *model,
                                const QModelIndex &index) const override {
        LockScope ws;
        if (editor) {
          auto *item = property_grid->view->itemFromIndex(index);
          if (!item) {
            LOG_ERROR("tree item not found");
            return;
          }
          std::shared_ptr<Object> object = property_grid->object_ptr.lock();
          auto property = itemProperty(index);
          std::string value =
              editor->property(editor->metaObject()->userProperty().name())
                  .toString()
                  .toStdString();
          {
            std::string v;
            property.tryToString(v);
            if (value == v) {
              LOG_DEBUG("property unchanged");
              return;
            }
          }
          {
            ActionScope action(
                std::string("Change ") +
                property.displayName() /*,
object*/);
            try {
              property.fromStringOrThrow(value);
              LOG_DEBUG("property changed");
            } catch (std::exception &ex) {
              LOG_ERROR("failed to parse value string");
            }
          }
          {
            std::string v;
            property.tryToString(v);
            model->setData(index, v.c_str(), 0);
          }
        }
      }
    };
    view->setItemDelegateForColumn(1, new EditDelegate(this));
  }
}

bool PropertyGridWidget::isPropertyDraggable(QTreeWidgetItem *item) {
  LockScope ws;
  if (item) {
    if (auto object = object_ptr.lock()) {
      if (!item->data(1, PropertyRole).isNull()) {
        auto current_property = itemProperty(item);
        std::type_index t = current_property.typeId();
        return t == typeid(double) || t == typeid(float) ||
               t == typeid(short) || t == typeid(int) || t == typeid(long) ||
               t == typeid(long long) || t == typeid(unsigned short) ||
               t == typeid(unsigned int) || t == typeid(unsigned long) ||
               t == typeid(unsigned long long);
      }
    }
  }
  return false;
}

void PropertyGridWidget::sync(QTreeWidgetItem *parent_item,
                              std::vector<Property> properties) {
  LockScope ws;
  size_t rows = 0;
  for (auto &property : properties) {
    if (property.attributes()->hidden) {
      continue;
    }
    std::string s;
    bool ok = property.tryToString(s);
    std::vector<Property> children;
    if (!ok) {
      property.expand(children);
      if (!children.empty()) {
        ok = true;
      }
    }
    if (!ok) {
      ok = property.expandList(children);
      LOG_DEBUG("list elements " << children.size());
    }
    if (!ok) {
      LOG_DEBUG("failed to analyze property " << property.name());
      continue;
    }
    rows++;
    QTreeWidgetItem *item = nullptr;
    bool is_new_item = false;
    if (parent_item) {
      item = parent_item->child(rows - 1);
      if (!item) {
        item = new QTreeWidgetItem();
        item->setFlags(item->flags() | Qt::ItemIsEditable);
        parent_item->addChild(item);
        is_new_item = true;
      }
    } else {
      item = view->topLevelItem(rows - 1);
      if (!item) {
        item = new QTreeWidgetItem();
        view->addTopLevelItem(item);
        is_new_item = true;
      }
    }
    item->setData(1, PropertyRole, QVariant::fromValue(property));
    if (is_new_item) {
      item->setText(0, property.displayName().c_str());
    }
    item->setText(1, QString::fromStdString(s));
    item->setFlags(item->flags() | Qt::ItemIsEditable);
    if (isPropertyDraggable(item)) {
      item->setData(1, Qt::DecorationRole, MATERIAL_ICON("swap_vert"));
    }
    if (property.typeId() == typeid(bool)) {
      if (property.get<bool>()) {
        item->setData(1, Qt::DecorationRole, MATERIAL_ICON("check_box"));
      } else {
        item->setData(1, Qt::DecorationRole,
                      MATERIAL_ICON("check_box_outline_blank"));
      }
    }
    if (children.size()) {
      sync(item, children);
      if (is_new_item) {
        item->setExpanded(true);
      }
    }
  }
  if (parent_item) {
    while (parent_item->childCount() > rows) {
      delete parent_item->takeChild(rows);
    }
  } else {
    while (view->topLevelItemCount() > rows) {
      delete view->topLevelItem(rows);
    }
  }
}

void PropertyGridWidget::sync() {
  LOG_DEBUG("PropertyGridWidget::sync");
  LockScope ws;
  auto selected_objects = ws->selection().resolve(ws());
  auto object =
      (selected_objects.size() == 1 ? selected_objects.front() : nullptr);
  if (object == nullptr || object_id != object->id()) {
    while (view->topLevelItemCount()) {
      delete view->topLevelItem(0);
    }
  }

  object_id = (object ? object->id() : 0);
  object_ptr = object;
  if (object != nullptr) {
    auto props = object->properties();
    sync(nullptr, std::vector<Property>(props.begin(), props.end()));
  }
}
