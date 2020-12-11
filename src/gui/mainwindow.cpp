// TAMSVIZ
// (c) 2020 Philipp Ruppel

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
#include "splitwindow.h"
#include "timeline.h"

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

bool MainWindow::event(QEvent *event) { QMainWindow::event(event); }

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
      open_recent_menu->addAction(tr("Clear List")), &QAction::triggered, this,
      [this](bool checked) {
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
    QString path = QFileDialog::getOpenFileName(this, tr("Locate Bag File"),
                                                QString::fromStdString(name),
                                                tr("Bags (*.bag)"));
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
         "(*.bag)"));
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
  if (ws->saved_document != Snapshot<std::shared_ptr<Document>>::save(
                                ws->document(), ws->saved_document, nullptr)) {
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
  LockScope ws;
  QProgressDialog progress(tr("Loading bag..."), QString(), 0, 0);
  progress.setModal(true);
  progress.setWindowFlags(Qt::Window | Qt::WindowTitleHint |
                          Qt::CustomizeWindowHint);
  progress.show();
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
    ws->player = player;
    ws->modified();
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

MainWindow::MainWindow() {

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
    if (ws->document()->path.empty()) {
      setWindowTitle(QString("%1").arg(qApp->applicationName()));
    } else {
      setWindowTitle(
          QString("%1 - %2")
              .arg(QFileInfo(ws->document()->path.c_str()).fileName())
              .arg(qApp->applicationName()));
    }
  });

  menuBar()->setNativeMenuBar(false);
  auto *toolbar = addToolBar(tr("Tools"));
  toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
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
                            const std::function<bool()> &predicate = nullptr) {
    auto *item = menu->addAction(tr(label));
    setupAction(item, callback, predicate);
    return item;
  };
  auto createToolbarItem = [&](const char *label, QIcon icon,
                               const std::function<void()> &callback = nullptr,
                               const std::function<bool()> &predicate =
                                   nullptr) {
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
    createMenuItem(menu, "&New", [this]() { closeDocument(); })
        ->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_N));
    createMenuAndToolbarItem(menu, "&Open", QIcon::fromTheme("document-open"),
                             [this]() { openBrowse(); })
        ->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_O));
    open_recent_menu = menu->addMenu(tr("Open &Recent"));
    createMenuAndToolbarItem(menu, "&Save", QIcon::fromTheme("document-save"),
                             [this]() { saveDocument(); })
        ->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_S));
    createMenuItem(menu, "Save &As...", [this]() { saveDocumentAs(); })
        ->setShortcut(QKeySequence(Qt::CTRL + Qt::SHIFT + Qt::Key_S));
    createMenuItem(menu, "&Close Document", [this]() { closeDocument(); })
        ->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_W));
    createMenuItem(menu, "Close &Bag", [this]() { closeBag(); })
        ->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_B));
    createMenuItem(menu, "E&xit", [this]() { close(); })
        ->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q));
  }
  {
    auto *menu = menuBar()->addMenu(tr("&Edit"));
    createMenuAndToolbarItem(menu, "&Undo", // FA_S_ICON("undo", 0.15),
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
    createMenuAndToolbarItem(menu, "&Redo", // FA_S_ICON("redo", 0.15),
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
        menu, "&Copy", // FA_R_ICON("copy"),
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
          ws()->recurse([&](const std::shared_ptr<Object> &parent,
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
              this, "Error", tr("Clipboard parsing error.\n%1").arg(ex.what()));
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
    createMenuAndToolbarItem(menu, "&Delete", // FA_R_ICON("trash-alt"),
                             QIcon::fromTheme("edit-delete"),
                             [this]() {
                               LOG_INFO("delete");
                               ActionScope ws("Delete");
                               for (auto selected :
                                    ws->selection().resolve(ws())) {
                                 removeObject(ws(), (void *)selected.get());
                                 ws->modified();
                               }
                             },
                             []() {
                               LockScope ws;
                               return !ws()->selection().empty();
                             })
        ->setShortcut(QKeySequence(Qt::Key_Delete));
    createMenuItem(menu, "Deselect",
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

  {
    auto *menu = menuBar()->addMenu(tr("&Windows"));
    for (auto *dock : findChildren<QDockWidget *>()) {
      menu->addAction(dock->toggleViewAction());
    }
    // menu->addAction("x")->setShortcut(QKeySequence(Qt::Key_Alt));
    // menu->addAction("x")->setShortcut(QKeySequence(Qt::Key_AltGr));
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
  {
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

int main(int argc, char **argv) {

  {
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    {
      QSurfaceFormat format;
      format.setVersion(3, 2);
      format.setProfile(QSurfaceFormat::CompatibilityProfile);
      format.setSamples(16);
      QSurfaceFormat::setDefaultFormat(format);
    }

    QCoreApplication::setOrganizationName("TAMS");
    QCoreApplication::setApplicationName(QString(ROS_PACKAGE_NAME).toUpper());

    QApplication app(argc, argv);

    ros::init(argc, argv, ROS_PACKAGE_NAME, ros::init_options::NoSigintHandler);
    ros::NodeHandle node("~");
    signal(SIGINT, [](int sig) {
      LOG_DEBUG("shutting down");
      if (auto *app = qApp) {
        app->exit();
      }
      ros::shutdown();
    });

    console_bridge::noOutputHandler();

    LOG_DEBUG("package name " << ROS_PACKAGE_NAME);
    LOG_DEBUG("package path " << ros::package::getPath(ROS_PACKAGE_NAME));

    // ProfilerThread profiler_thread;

    {
      {
        RenderThread::instance();
        MainWindow main_window;
        qDebug() << "styles" << QStyleFactory::keys();
        {
          ros::AsyncSpinner spinner(0);
          spinner.start();
          app.exec();
        }
        RenderThread::instance()->stop();

        // wait for async cleanup from renderthread to be finished
        qApp->processEvents();

        {
          LockScope ws;

          // windows will be deleted by the document
          auto widgets = main_window.findChildren<QWidget *>();
          for (auto *w : widgets) {
            if (dynamic_cast<Window *>(w)) {
              w->setParent(nullptr);
            }
          }

          ws()->document() = std::make_shared<Document>();
        }
      }
      {
        LockScope ws;
        ws() = nullptr;
      }
    }
  }
  LOG_DEBUG("shut down");
}
