// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include <pybind11/pybind11.h>

#include "../core/workspace.h"
#include "../gui/mainwindow.h"
#include "../gui/renderthread.h"
#include "../gui/renderwindow.h"

#include <console_bridge/console.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <signal.h>

namespace py = pybind11;

struct PyTAMSVIZ2 {

  std::shared_ptr<QApplication> app;
  std::shared_ptr<ros::NodeHandle> node;
  std::shared_ptr<ros::AsyncSpinner> spinner;

  /*void render() {
    LockScope ws;
    ws->document()->display()->renderSyncRecursive();
  }*/

  // ~PyTAMSVIZ2() { shutdown(); }

  void shutdown() {

    LOG_INFO("shutdown started");

    spinner.reset();

    LOG_INFO("shutdown phase 2");

    RenderThread::instance()->stop();

    LOG_INFO("shutdown phase 3");

    // wait for async cleanup from renderthread to be finished
    qApp->processEvents();

    LOG_INFO("shutdown phase 4");

    {
      LockScope ws;
      ws() = nullptr;
    }

    LOG_INFO("shutdown phase 5");

    spinner.reset();
    node.reset();
    app.reset();

    LOG_INFO("shutdown finished");
  }

  void start() {

    LOG_DEBUG("starting pytamsviz");

    static int argc = 0;
    static char *argv[0];

    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    {
      QSurfaceFormat format;
      format.setVersion(3, 2);
      format.setProfile(QSurfaceFormat::CompatibilityProfile);
      // format.setSamples(16);
      format.setSamples(0);
      QSurfaceFormat::setDefaultFormat(format);
    }

    ros::init(argc, argv, ROS_PACKAGE_NAME,
              ros::init_options::AnonymousName |
                  ros::init_options::NoSigintHandler);

    app = std::make_shared<QApplication>(argc, argv);
    app->setOrganizationName("TAMS");
    app->setApplicationName(QString(ROS_PACKAGE_NAME).toUpper());

    node = std::make_shared<ros::NodeHandle>("~");
    signal(SIGINT, [](int sig) {
      LOG_DEBUG("shutting down");
      ros::shutdown();
    });

    spinner = std::make_shared<ros::AsyncSpinner>(0);
    spinner->start();

    RenderThread::instance();

    LOG_DEBUG("pytamsviz started");
  }

  void open(const std::string &path) {

    LockScope ws;

    QFile file(path.c_str());
    if (!file.open(QIODevice::ReadOnly)) {
      throw std::runtime_error("file not found");
      return;
    }
    auto contents = file.readAll().toStdString();

    Variant vdoc = parseYAML(contents);

    auto mdoc = vdoc.value<std::map<std::string, Variant>>();
    for (auto &x : mdoc) {
      LOG_INFO(x.first);
    }
    mdoc.erase("Window");
    vdoc = Variant(mdoc);

    std::shared_ptr<Document> document;
    deserialize(document, vdoc);
    document->path = path;
    ws->document() = document;
    ws->history->clear();
    ws->modified();
    LOG_SUCCESS("opened " << document->path);
  }

  void exec() {
    LOG_DEBUG("pytamsviz exec begin");
    ros::waitForShutdown();
    LOG_DEBUG("pytamsviz exec end");
  }
};

struct PyTAMSVIZ {

  std::shared_ptr<QApplication> app;
  std::shared_ptr<ros::NodeHandle> node;
  std::shared_ptr<MainWindow> main_window;
  std::shared_ptr<ros::AsyncSpinner> spinner;

  ~PyTAMSVIZ() { LOG_INFO("pytamsviz dtor"); }

  void start(bool embedded) {

    static int argc = 0;
    static char *argv[0];

    if (embedded) {
      g_show_split_window_bars = false;
    }

    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    {
      QSurfaceFormat format;
      format.setVersion(3, 2);
      format.setProfile(QSurfaceFormat::CompatibilityProfile);
      // format.setSamples(16);
      format.setSamples(0);
      QSurfaceFormat::setDefaultFormat(format);
    }

    ros::init(argc, argv, ROS_PACKAGE_NAME,
              ros::init_options::AnonymousName |
                  ros::init_options::NoSigintHandler);

    app = std::make_shared<QApplication>(argc, argv);
    app->setOrganizationName("TAMS");
    app->setApplicationName(QString(ROS_PACKAGE_NAME).toUpper());

    node = std::make_shared<ros::NodeHandle>("~");
    signal(SIGINT, [](int sig) {
      LOG_DEBUG("shutting down");
      if (auto *app = qApp) {
        app->exit();
      }
      ros::shutdown();
    });

    console_bridge::noOutputHandler();

    RenderThread::instance();

    main_window = std::make_shared<MainWindow>(embedded);

    spinner = std::make_shared<ros::AsyncSpinner>(0);
    spinner->start();

    if (embedded) {
      main_window->menuBar()->hide();
      for (auto *bar : main_window->findChildren<QToolBar *>()) {
        bar->hide();
      }
      for (auto *dock : main_window->findChildren<QDockWidget *>()) {
        dock->hide();
      }
    }
  }

  void exec() { app->exec(); }

  void open(const std::string &filename) {
    main_window->openAny(filename.c_str());
  }

  void attach(uint64_t ptr) {
    main_window->setWindowFlags(Qt::Widget);
    QWidget *parent = (QWidget *)ptr;
    main_window->setParent(parent);
    if (auto *layout = parent->layout()) {
      layout->addWidget(&*main_window);
    }
  }

  void show() { main_window->show(); }

  void shutdown() {

    LOG_INFO("shutdown started");

    spinner.reset();

    LOG_INFO("shutdown phase 2");

    RenderThread::instance()->stop();

    LOG_INFO("shutdown phase 3");

    // wait for async cleanup from renderthread to be finished
    qApp->processEvents();

    LOG_INFO("shutdown phase 4");

    {
      LockScope ws;

      // windows will be deleted by the document
      auto widgets = main_window->findChildren<QWidget *>();
      for (auto *w : widgets) {
        if (dynamic_cast<Window *>(w)) {
          w->setParent(nullptr);
        }
      }

      ws()->document() = std::make_shared<Document>();
    }

    LOG_INFO("shutdown phase 5");

    main_window.reset();

    LOG_INFO("shutdown phase 6");

    {
      LockScope ws;
      ws() = nullptr;
    }

    LOG_INFO("shutdown phase 7");

    spinner.reset();
    main_window.reset();
    node.reset();
    app.reset();

    LOG_INFO("shutdown finished");
  }
};

PYBIND11_MODULE(pytamsviz, m) {

  py::class_<PyTAMSVIZ>(m, "TAMSVIZ")
      .def(py::init<>())
      .def("start", [](PyTAMSVIZ *_this) { _this->start(false); })
      .def("start",
           [](PyTAMSVIZ *_this, bool embedded) { _this->start(embedded); })
      .def("exec", &PyTAMSVIZ::exec)
      .def("open", &PyTAMSVIZ::open)
      .def("attach", &PyTAMSVIZ::attach)
      .def("show", &PyTAMSVIZ::show)
      .def("shutdown", &PyTAMSVIZ::shutdown) //
      ;

  py::class_<PyTAMSVIZ2>(m, "TAMSVIZ2")
      .def(py::init<>())
      .def("start", &PyTAMSVIZ2::start)
      .def("exec", &PyTAMSVIZ2::exec)
      .def("open", &PyTAMSVIZ2::open)
      .def("shutdown", &PyTAMSVIZ2::shutdown)
      //
      ;
}
