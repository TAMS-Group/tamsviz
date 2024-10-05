// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "mainwindow.h"

#include "../core/bagplayer.h"
#include "../core/history.h"
#include "../core/log.h"
#include "../core/profiler.h"
#include "../core/serialization.h"
#include "../core/snapshot.h"
#include "../core/workspace.h"
#include "displaytree.h"
#include "imagewindow.h"
#include "propertygrid.h"
#include "renderwindow.h"
#include "scenewindow.h"
#include "searchwidget.h"
#include "splitwindow.h"
#include "timeline.h"
#include "filewidget.h"

#include <ros/package.h>
#include <ros/ros.h>

#include <signal.h>

struct ClipboardItem : Object {
  PROPERTY(std::shared_ptr<Object>, object);
  PROPERTY(Handle<Object>, parent);
};
DECLARE_TYPE(ClipboardItem, Object)

struct ClipboardData : Object {
  PROPERTY(std::vector<std::shared_ptr<ClipboardItem>>, clipboard);
};
DECLARE_TYPE(ClipboardData, Object)

bool MainWindow::event(QEvent *event) { return QMainWindow::event(event); }

void MainWindow::addRecentFile(const QString &path) {
  {
    QSettings settings;
    QStringList files = settings.value("recent").toStringList();
    files.removeAll(path);
    files.prepend(path);
    while (files.size() > 10) {
      files.removeLast();
    }
    settings.setValue("recent", files);
  }
  updateRecentMenu();
}

void MainWindow::updateRecentMenu() {
  if (open_recent_menu) {
    QSettings settings;
    QStringList files = settings.value("recent").toStringList();
    open_recent_menu->clear();
    open_recent_menu->setEnabled(!files.isEmpty());
    for (const QString &file : files) {
      QObject::connect(open_recent_menu->addAction(QFileInfo(file).fileName()),
                       &QAction::triggered, this,
                       [this, file](bool checked) { openAny(file); });
    }
    open_recent_menu->addSeparator();
    QObject::connect(
        open_recent_menu->addAction(tr("Clear List")), &QAction::triggered,
        this, [this](bool checked) {
          if (QMessageBox::question(
                  this, tr("Clear recent file list?"),
                  tr("Do you want to clear the list of recent files?")) ==
              QMessageBox::Yes) {
            QSettings settings;
            settings.setValue("recent", QStringList());
            updateRecentMenu();
          }
        });
  }
}

void MainWindow::openDocument(const QString &path) {
  if (!closeDocument()) {
    return;
  }
  LockScope ws;
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    QMessageBox::critical(this, "Error", "Failed to open file.");
    return;
  }
  auto contents = file.readAll().toStdString();
  Variant v;
  try {
    v = parseYAML(contents);
  } catch (const std::exception &ex) {
    QMessageBox::critical(
        this, "Error",
        tr("Failed to load file. Parsing error.\n%1").arg(ex.what()));
    return;
  }
  std::shared_ptr<Document> displays;
  while (true) {
    try {
      deserialize(displays, v);
    } catch (const SerializationTypeError &ex) {
      auto button = QMessageBox::critical(
          this, "Error",
          tr("Type not found: %1\nIncompatible version or missing plug-ins?")
              .arg(ex.typeName().c_str()),
          QMessageBox::Ignore | QMessageBox::Cancel);
      if (button == QMessageBox::Ignore) {
        ex.createReplacement();
        continue;
      } else {
        return;
      }
    } catch (const std::exception &ex) {
      QMessageBox::critical(
          this, "Error",
          tr("Failed to load file. Deserialization error.\n%1").arg(ex.what()));
      return;
    }
    break;
  }
  displays->path = path.toStdString();
  ws->document() = displays;
  ws->saved_document = Snapshot<std::shared_ptr<Document>>::save(
      ws->document(), ws->saved_document, nullptr);
  ws->history->clear();
  ws->modified();
  addRecentFile(path);
  LOG_SUCCESS("opened " << displays->path);
}

void MainWindow::findAndOpenBag(const std::string &name) {
  LockScope ws;
  if (ws->player && ws->player->fileName() == name) {
    return;
  }
  {
    QFileInfo f(QFileInfo(QString::fromStdString(ws->document()->path)).dir(),
                QString::fromStdString(name));
    if (f.exists()) {
      openBag(f.absoluteFilePath());
      return;
    }
  }
  if (auto player = ws->player) {
    QFileInfo f(QFileInfo(QString::fromStdString(player->path())).dir(),
                QString::fromStdString(name));
    if (f.exists()) {
      openBag(f.absoluteFilePath());
      return;
    }
  }
  {
    QString path = QFileDialog::getOpenFileName(
        this, tr("Locate Bag File"), QString::fromStdString(name),
        tr("Bags (*.bag)"), nullptr, QFileDialog::DontUseNativeDialog);
    if (path.isNull()) {
      return;
    }
    openBag(path);
    return;
  }
}

void MainWindow::openBrowse() {
  LockScope ws;
  QString path = QFileDialog::getOpenFileName(
      this, tr("Open File"), QString(),
      tr("Supported file types (*.tmv *.bag);;Documents (*.tmv);;Bags "
         "(*.bag)"),
      nullptr, QFileDialog::DontUseNativeDialog);
  if (path.isNull()) {
    return;
  }
  openAny(path);
}

void MainWindow::openAny(const QString &path) {
  if (path.toLower().endsWith(".bag")) {
    openBag(path);
    return;
  }
  if (path.toLower().endsWith(".tmv")) {
    if (!closeDocument()) {
      return;
    }
    openDocument(path);
    return;
  }
  LOG_ERROR("unknown file type");
  QMessageBox::critical(this, "Error", tr("Unknown file type"));
}

bool MainWindow::closeDocument() {
  LockScope ws;
  if ((ws->saved_document !=
       Snapshot<std::shared_ptr<Document>>::save(
           ws->document(), ws->saved_document, nullptr)) &&
      (ws->history->canUndo() || ws->history->canRedo()) && !embedded) {
    // if ((ws->history->canUndo() || ws->history->canRedo()) && !embedded) {
    auto rs = QMessageBox::question(
        this, tr("Save changes?"),
        tr("The current document has been modified. Do "
           "you want to save your changes?"),
        QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel,
        QMessageBox::Cancel);
    if (rs == QMessageBox::Cancel) {
      return false;
    }
    if (rs == QMessageBox::No) {
      // continue
    }
    if (rs == QMessageBox::Yes) {
      if (!saveDocument()) {
        return false;
      }
    }
  }
  ws->document() = std::make_shared<Document>();
  ws->document()->window() = std::make_shared<SceneWindow>();
  ws->saved_document = Snapshot<std::shared_ptr<Document>>::save(
      ws->document(), ws->saved_document, nullptr);
  ws->history->clear();
  ws->modified();
  LOG_DEBUG("closed");
  return true;
}

void MainWindow::openBag(const QString &path) {
  QProgressDialog progress(tr("Loading bag..."), QString(), 0, 0);
  progress.setModal(true);
  progress.setWindowFlags(Qt::Window | Qt::WindowTitleHint |
                          Qt::CustomizeWindowHint);
  progress.show();
  {
    LockScope ws;
    ws->player.reset();
    ws->modified();
  }
  std::string error;
  volatile bool finished = false;
  std::shared_ptr<BagPlayer> player;
  std::thread thread([&]() {
    try {
      player = std::make_shared<BagPlayer>(path.toStdString());
    } catch (const std::exception &ex) {
      LOG_ERROR("failed to load bag " << ex.what());
      error = ex.what();
    }
    if (player) {
      LOG_DEBUG("bag loaded");
      player->changed.connect([]() { GlobalEvents::instance()->redraw(); });
    }
    LOG_DEBUG("bag loader thread finished");
    finished = true;
  });
  while (!finished) {
    QThread::msleep(10);
    QApplication::processEvents();
  }
  thread.join();
  progress.close();
  if (player) {
    {
      LockScope ws;
      ws->player = player;
      ws->modified();
    }
    addRecentFile(path);
    player->rewind();
  } else {
    progress.close();
    LOG_ERROR("failed to load bag file " << error);
    QMessageBox::critical(this, "Error",
                          tr("Failed to load file. Deserialization error.\n%1")
                              .arg(error.c_str()));
  }
}

bool MainWindow::closeBag() {
  LockScope ws;
  if (!ws->player) {
    return true;
  }
  ws->player = nullptr;
  ws->modified();
  return true;
}

bool MainWindow::saveDocument(const QString &path) {
  LockScope ws;
  QSaveFile file(path);
  file.open(QIODevice::WriteOnly);
  file.write(toYAML(serialize(ws->document())).c_str());
  if (file.commit()) {
    ws->document()->path = path.toStdString();
    ws->saved_document = Snapshot<std::shared_ptr<Document>>::save(
        ws->document(), ws->saved_document, nullptr);
    LOG_SUCCESS("saved " << path.toStdString());
    addRecentFile(path);
    ws->modified();
    return true;
  } else {
    QMessageBox::critical(this, tr("Error"), tr("Failed to write file"));
    return false;
  }
}

bool MainWindow::saveDocument() {
  LockScope ws;
  if (ws->document()->path.size()) {
    return saveDocument(ws->document()->path.c_str());
  } else {
    return saveDocumentAs();
  }
}

bool MainWindow::saveDocumentAs() {
  LockScope ws;
  QFileDialog dialog(this, QString(), QString(), tr("Documents (*.tmv)"));
  dialog.selectFile(ws->document()->path.c_str());
  dialog.setAcceptMode(QFileDialog::AcceptSave);
  dialog.setDefaultSuffix(tr(".tmv"));
  if (dialog.exec() == QDialog::Accepted) {
    QString fname = dialog.selectedFiles().value(0);
    return saveDocument(fname);
  } else {
    return false;
  }
}

MainWindow *g_main_window_instance = nullptr;
MainWindow *MainWindow::instance() { return g_main_window_instance; }

MainWindow::MainWindow(bool embedded) {
  this->embedded = embedded;

  g_main_window_instance = this;

  LockScope ws;

  setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);
  setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
  // setTabPosition(Qt::BottomDockWidgetArea, QTabWidget::East);
  auto *display_tree = new DisplayTreeWidget();
  auto *property_grid = new PropertyGridWidget();
  auto *timeline_widget = new TimelineWidget();
  addDockWidget(Qt::LeftDockWidgetArea, display_tree);
  addDockWidget(Qt::LeftDockWidgetArea, property_grid);
  addDockWidget(Qt::BottomDockWidgetArea, timeline_widget);
  ws->modified.connect(this, [this]() {
    LockScope ws;
    /*
    if (ws->document()->path.empty()) {
      setWindowTitle(QString("%1").arg(qApp->applicationName()));
    } else {
      setWindowTitle(
          QString("%1 - %2")
              .arg(QFileInfo(ws->document()->path.c_str()).fileName())
              .arg(qApp->applicationName()));
    }
    */
    QString title;
    if (!ws->document()->path.empty()) {
      title =
          QFileInfo(QString::fromStdString(ws->document()->path)).fileName();
    }
    if (ws->player) {
      if (!title.isEmpty()) {
        title += " - ";
      }
      title += QString::fromStdString(ws->player->fileName());
    }
    if (!title.isEmpty()) {
      title += " - ";
    }
    title += qApp->applicationName();
    setWindowTitle(title);
  });

  auto *search_widget = new SearchWidget();
  addDockWidget(Qt::RightDockWidgetArea, search_widget);
  search_widget->hide();
  search_widget->toggleViewAction()->setIcon(QIcon::fromTheme("edit-find"));

  auto *file_widget = new FileWidget();
  addDockWidget(Qt::RightDockWidgetArea, file_widget);
  file_widget->hide();
  file_widget->toggleViewAction()->setIcon(QIcon::fromTheme("folder"));

  menuBar()->setNativeMenuBar(false);
  auto *toolbar = addToolBar(tr("Tools"));
  toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);

  if (!embedded) {
    auto setupAction = [&](QAction *item, const std::function<void()> &callback,
                           const std::function<bool()> &predicate) {
      if (callback) {
        QObject::connect(item, &QAction::triggered, this,
                         [item, callback](bool checked) { callback(); });
      }
      if (predicate) {
        ws->modified.connect(
            item, [item, predicate]() { item->setEnabled(predicate()); });
        item->setEnabled(predicate());
      }
    };
    auto createMenuItem = [&](QMenu *menu, const char *label,
                              const std::function<void()> &callback = nullptr,
                              const std::function<bool()> &predicate =
                                  nullptr) {
      auto *item = menu->addAction(tr(label));
      setupAction(item, callback, predicate);
      return item;
    };
    auto createToolbarItem =
        [&](const char *label, QIcon icon,
            const std::function<void()> &callback = nullptr,
            const std::function<bool()> &predicate = nullptr) {
          auto *item = toolbar->addAction(tr(label));
          item->setIcon(icon);
          setupAction(item, callback, predicate);
          return item;
        };
    auto createMenuAndToolbarItem =
        [&](QMenu *menu, const char *label, QIcon icon,
            const std::function<void()> &callback = nullptr,
            const std::function<bool()> &predicate = nullptr) {
          createToolbarItem(label, icon, callback, predicate);
          auto *menu_item = createMenuItem(menu, label, callback, predicate);
          // menu_item->setIcon(icon);
          return menu_item;
        };
    {
      auto *menu = menuBar()->addMenu(tr("&File"));
      createMenuItem(menu, "&New", [this]() {
        closeDocument();
      })->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_N));
      createMenuAndToolbarItem(menu, "&Open", QIcon::fromTheme("document-open"),
                               [this]() { openBrowse(); })
          ->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_O));
      open_recent_menu = menu->addMenu(tr("Open &Recent"));
      createMenuAndToolbarItem(menu, "&Save", QIcon::fromTheme("document-save"),
                               [this]() { saveDocument(); })
          ->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_S));
      createMenuItem(menu, "Save &As...", [this]() {
        saveDocumentAs();
      })->setShortcut(QKeySequence(Qt::CTRL + Qt::SHIFT + Qt::Key_S));
      createMenuItem(menu, "&Close Document", [this]() {
        closeDocument();
      })->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_W));
      createMenuItem(menu, "Close &Bag", [this]() {
        closeBag();
      })->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_B));
      createMenuItem(menu, "E&xit", [this]() {
        close();
      })->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q));
    }
    {
      auto *menu = menuBar()->addMenu(tr("&Edit"));
      createMenuAndToolbarItem(
          menu, "&Undo",  // FA_S_ICON("undo", 0.15),
          QIcon::fromTheme("edit-undo"),
          [this]() {
            LockScope ws;
            ws->history->undo(ws());
            ws->modified();
          },
          []() {
            LockScope ws;
            return ws->history->canUndo();
          })
          ->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_Z));
      createMenuAndToolbarItem(
          menu, "&Redo",  // FA_S_ICON("redo", 0.15),
          QIcon::fromTheme("edit-redo"),
          [this]() {
            LockScope ws;
            ws->history->redo(ws());
            ws->modified();
          },
          []() {
            LockScope ws;
            return ws->history->canRedo();
          })
          ->setShortcuts({QKeySequence(Qt::CTRL + Qt::Key_Y),
                          QKeySequence(Qt::CTRL + Qt::SHIFT + Qt::Key_Z)});
      createMenuAndToolbarItem(
          menu, "&Copy",  // FA_R_ICON("copy"),
          QIcon::fromTheme("edit-copy"),
          [this]() {
            LOG_INFO("copy to clipboard");
            LockScope ws;
            QGuiApplication::clipboard()->clear();
            std::unordered_set<std::shared_ptr<Object>> selection_set;
            for (auto &o : ws->selection().resolve(ws())) {
              selection_set.insert(o);
            }
            auto clipboard_data = std::make_shared<ClipboardData>();
            std::unordered_set<std::shared_ptr<Object>> clipboard_set;
            ws()->recurseObjects([&](const std::shared_ptr<Object> &parent,
                                     const std::shared_ptr<Object> &child) {
              if (clipboard_set.find(parent) != clipboard_set.end()) {
                clipboard_set.insert(child);
              } else {
                if (selection_set.find(child) != selection_set.end()) {
                  clipboard_set.insert(child);
                  auto clipboard_item = std::make_shared<ClipboardItem>();
                  clipboard_item->object() = child;
                  clipboard_item->parent() = parent;
                  clipboard_data->clipboard().push_back(clipboard_item);
                }
              }
            });
            std::string data =
                "#" ROS_PACKAGE_NAME "\n" + toYAML(serialize(clipboard_data));
            QGuiApplication::clipboard()->setText(data.c_str());
          },
          []() {
            LockScope ws;
            return !ws()->selection().empty();
          })
          ->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_C));
      {
        auto paste_callback = [this]() {
          if (!QGuiApplication::clipboard()->text().startsWith(
                  "#" ROS_PACKAGE_NAME)) {
            LOG_ERROR("invalid clipboard data");
            return;
          }
          ActionScope ws("Paste");
          Variant v;
          try {
            v = parseYAML(QGuiApplication::clipboard()->text().toStdString());
          } catch (const std::exception &ex) {
            QMessageBox::critical(
                this, "Error",
                tr("Clipboard parsing error.\n%1").arg(ex.what()));
            return;
          }
          std::shared_ptr<ClipboardData> clipboard_data;
          try {
            deserialize(clipboard_data, v);
          } catch (const std::exception &ex) {
            QMessageBox::critical(
                this, "Error",
                tr("Clipboard deserialization error.\n%1").arg(ex.what()));
            return;
          }
          if (!clipboard_data) {
            return;
          }
          ws->selection().clear();
          for (auto &item : clipboard_data->clipboard()) {
            if (!item || !item->object()) {
              continue;
            }
            auto parent = item->parent().resolve(ws());
            if (!parent) {
              continue;
            }
            for (auto &property : parent->properties()) {
              std::vector<std::shared_ptr<void>> list;
              if (property.info()->type()->tryToPointerList(
                      property.valuePointer(), list)) {
                item->object()->assignNewId();
                list.push_back(item->object());
                if (property.info()->type()->tryFromPointerList(
                        property.valuePointer(), list)) {
                  ws->selection().add(item->object());
                  break;
                }
              }
            }
          }
          ws->modified();
        };
        auto *paste_menu_item = createMenuItem(menu, "&Paste", paste_callback);
        auto *paste_toolbar_item =
            createToolbarItem("Paste",
                              // FA_R_ICON("clipboard"),
                              QIcon::fromTheme("edit-paste"), paste_callback);
        paste_toolbar_item->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_V));
        auto update_paste = [paste_menu_item, paste_toolbar_item]() {
          bool ok = QGuiApplication::clipboard()->text().startsWith(
              "#" ROS_PACKAGE_NAME);
          paste_menu_item->setEnabled(ok);
          paste_toolbar_item->setEnabled(ok);
        };
        connect(QGuiApplication::clipboard(), &QClipboard::dataChanged, this,
                update_paste);
        update_paste();
      }
      createMenuAndToolbarItem(
          menu, "&Delete",  // FA_R_ICON("trash-alt"),
          QIcon::fromTheme("edit-delete"),
          [this]() {
            LOG_INFO("delete");
            ActionScope ws("Delete");
            for (auto selected : ws->selection().resolve(ws())) {
              removeObject(ws(), (void *)selected.get());
              ws->modified();
            }
          },
          []() {
            LockScope ws;
            return !ws()->selection().empty();
          })
          ->setShortcut(QKeySequence(Qt::Key_Delete));
      createMenuItem(
          menu, "Deselect",
          [this]() {
            LockScope ws;
            ws->selection().clear();
            ws->modified();
            if (auto *w = qApp->focusWidget()) {
              w->clearFocus();
            }
          },
          []() {
            LockScope ws;
            return !ws()->selection().empty();
          })
          ->setShortcut(QKeySequence(Qt::Key_Escape));
      createMenuItem(menu, "Select next annotation", [this]() {
        ActionScope ws("Select next annotation track");
        LOG_DEBUG("selecting next annotation track");
        auto curr = ws->currentAnnotationTrack().resolve(ws());
        size_t j = 0;
        auto &tt = ws->document()->timeline()->tracks();
        for (size_t i = 0; i < tt.size(); i++) {
          if (tt[i] == curr) {
            j = (i + 1) % tt.size();
          }
        }
        LOG_DEBUG("selecting annotation track j");
        for (size_t i = 0; i < tt.size(); i++) {
          auto t = tt[j];
          if (std::dynamic_pointer_cast<AnnotationTrack>(t)) {
            break;
          }
          j++;
        }
        auto t = tt[j];
        auto &selection = ws->selection();
        selection.clear();
        selection.add(t);
        ws->currentAnnotationTrack() =
            std::dynamic_pointer_cast<AnnotationTrack>(t);
        ws->modified();
        //     selection.add(ws->document()->timeline()->tracks()[jtrack]);
        // ActionScope ws("Select annotation track");
        // LOG_DEBUG("select next annotation");
        // if (!ws->document()->timeline()->tracks().empty()) {
        //   // auto selection_list =
        //   //     ws->selection().resolve(ws->document()->timeline());
        //   // for (auto selected : selection_list) {
        //   //   while (true) {
        //   //     if (auto track =
        //   // std::dynamic_pointer_cast<AnnotationTrack>(selected)) {
        //   //     }
        //   //     if (auto track =
        //   // std::dynamic_pointer_cast<AnnotationTrack>(selected)) {
        //   //     }
        //   //     break;
        //   //   }
        //   // }
        //   auto &selection = ws->selection();
        //   size_t jtrack = 0;
        //   for (size_t itrack = 0;
        //        itrack < ws->document()->timeline()->tracks().size();
        //        itrack++) {
        //     auto &track = ws->document()->timeline()->tracks()[itrack];
        //     if (selection.contains(track)) {
        //       jtrack = itrack + 1;
        //     }
        //     if (auto annotation_track =
        //             std::dynamic_pointer_cast<AnnotationTrack>(track)) {
        //       for (auto &branch : annotation_track->branches()) {
        //         for (auto &span : branch->spans()) {
        //           if (selection.contains(span)) {
        //             jtrack = itrack + 1;
        //           }
        //           for (auto &annotation : span->annotations()) {
        //             if (selection.contains(annotation)) {
        //               jtrack = itrack + 1;
        //             }
        //           }
        //         }
        //       }
        //     }
        //   }
        //   jtrack = (jtrack % ws->document()->timeline()->tracks().size());
        //   {
        //     LOG_DEBUG("selecting annotation track " << jtrack);
        //     selection.clear();
        //     selection.add(ws->document()->timeline()->tracks()[jtrack]);
        //   }
        // } else {
        //   LOG_DEBUG("no annotation tracks");
        // }
        // ws->modified();
      })->setShortcut(QKeySequence(Qt::Key_Tab));
      /*createMenuItem(menu, "Reload",
                     [this]() { ResourceEvents::instance().reload(); })
          ->setShortcut(QKeySequence(Qt::Key_F5));*/
      createMenuAndToolbarItem(menu, "Reload", QIcon::fromTheme("view-refresh"),
                               [this]() {
                                 ResourceEvents::instance().reload();
                                 GlobalEvents::instance()->redraw();
                               })
          ->setShortcut(QKeySequence(Qt::Key_F5));
    }
    toolbar->addAction(search_widget->toggleViewAction());
    toolbar->addAction(file_widget->toggleViewAction());

    {
      auto *menu = menuBar()->addMenu(tr("&Windows"));
      for (auto *dock : findChildren<QDockWidget *>()) {
        menu->addAction(dock->toggleViewAction());
      }
      // menu->addAction("x")->setShortcut(QKeySequence(Qt::Key_Alt));
      // menu->addAction("x")->setShortcut(QKeySequence(Qt::Key_AltGr));
    }
  }

  {
    auto update = [this]() {
      LockScope ws;
      auto *new_central_widget =
          ws() && ws->document() && ws->document()->window()
              ? (ContentWindowBase *)ws->document()->window().get()
              : nullptr;
      if (centralWidget() != new_central_widget) {
        LOG_DEBUG(
            "changing central widget from "
            << (centralWidget() ? typeid(*centralWidget()).name() : "null")
            << " to "
            << (new_central_widget ? typeid(*new_central_widget).name()
                                   : "null"));
        if (centralWidget()) {
          QWidget *w = takeCentralWidget();
          w->setParent(this);
          w->hide();
        }
        setCentralWidget(new_central_widget);
        new_central_widget->show();
      }
    };
    ws->modified.connect(this, update);
    update();
  }
  updateRecentMenu();
  resize(std::min(QGuiApplication::primaryScreen()->geometry().width() * 3 / 4,
                  1280),
         std::min(QGuiApplication::primaryScreen()->geometry().height() * 3 / 4,
                  800));
  ws->saved_document = Snapshot<std::shared_ptr<Document>>::save(
      ws->document(), ws->saved_document, nullptr);
  closeDocument();
  if (!embedded) {
    move(pos() + (QGuiApplication::primaryScreen()->geometry().center() -
                  geometry().center()));
    show();
    qApp->processEvents();
    display_tree->widget()->setFixedSize(
        QSize(display_tree->width(), height() / 4));
    qApp->processEvents();
    display_tree->widget()->setFixedSize(
        QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
  }
}

MainWindow::~MainWindow() {
  /*if (centralWidget()) {
    auto *central = takeCentralWidget();
    central->setParent(nullptr);
  }*/
  g_main_window_instance = nullptr;
}

void MainWindow::closeEvent(QCloseEvent *event) {
  if (!closeBag()) {
    event->ignore();
    return;
  }
  if (!closeDocument()) {
    event->ignore();
    return;
  }
  LockScope ws;
  event->accept();
  /*if (ws->history->save_counter != 0) {
    event->ignore();
  } else {
    event->accept();
}*/
}
