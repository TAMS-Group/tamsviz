// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "../core/workspace.h"
#include "../displays/camera.h"
#include "../gui/mainwindow.h"
#include "../gui/renderthread.h"
#include "../gui/renderwindow.h"

#include <boost/thread/barrier.hpp>
#include <console_bridge/console.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <signal.h>

namespace py = pybind11;

PYBIND11_MODULE(pytamsviz, m) {

  static std::shared_ptr<QApplication> app;
  static std::shared_ptr<ros::NodeHandle> node;
  static std::shared_ptr<MainWindow> main_window;
  static std::shared_ptr<ros::AsyncSpinner> spinner;

  static int argc = 0;
  static char *argv[0];

  QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
  {
    QSurfaceFormat format;
    format.setVersion(3, 2);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    format.setSamples(0);
    QSurfaceFormat::setDefaultFormat(format);
  }

  ros::init(argc, argv, ROS_PACKAGE_NAME,
            ros::init_options::AnonymousName |
                ros::init_options::NoSigintHandler);

  app = std::make_shared<QApplication>(argc, argv);
  app->setOrganizationName("TAMS");
  app->setApplicationName(QString(ROS_PACKAGE_NAME).toUpper());

  static auto init = []() {
    if (!app) {
      throw std::runtime_error("tamsviz already shut down");
    }

    std::setlocale(LC_NUMERIC, "en_US.UTF-8");

    node = std::make_shared<ros::NodeHandle>("~");
    signal(SIGINT, [](int sig) {
      LOG_DEBUG("shutting down");
      if (app) {
        app->exit();
      }
      ros::shutdown();
    });

    console_bridge::noOutputHandler();

    {
      LockScope ws;
      ws.ws() = std::make_shared<Workspace>();
    }
  };

  // ---

  m.def("ok", []() { return ros::ok(); });

  m.def("message", [](const std::string &topic, const std::string &hash,
                      const std::string &name, const std::string &definition,
                      const std::string &data) {
    if (auto topic_instance = Topic::instance(topic, false)) {
      auto type = MessageType::instance(hash, name, definition);
      auto message = std::make_shared<Message>();
      message->type(type);
      ros::serialization::IStream stream((uint8_t *)data.data(), data.size());
      message->read(stream);
      message->time(ros::Time::now());
      topic_instance->publish(message);
    }
  });

  m.def("render", []() {
    boost::barrier b(2);
    RenderThread::instance()->invalidate([&b]() { b.wait(); });
    b.wait();
  });

  // m.def("thread",
  //       [](const std::function<void()> &f) { std::thread(f).detach(); });

  // m.def("interval", [](double interval, const std::function<void()> &f) {
  //   node->createTimer(ros::Duration(interval),
  //                     boost::function<void(const ros::TimerEvent &)>(
  //                         [f](const ros::TimerEvent &) { f(); }));
  // });

  m.def("start_offscreen", []() {
    init();
    RenderThread::start();
    spinner = std::make_shared<ros::AsyncSpinner>(0);
    spinner->start();
  });

  m.def("start_editor", []() {
    init();
    main_window = std::make_shared<MainWindow>(false);
    main_window->show();
    RenderThread::start();
    spinner = std::make_shared<ros::AsyncSpinner>(0);
    spinner->start();
  });

  m.def("image", [](const std::string &name) {
    std::shared_ptr<sensor_msgs::Image> image_message;
    {
      LockScope ws;
      ws->document()->display()->recurseDisplays(
          [&](const std::shared_ptr<Display> &display) {
            if (auto cam = std::dynamic_pointer_cast<CameraDisplay>(display)) {
              if (cam->prefix() == name) {
                image_message = cam->image();
              }
            }
          });
    }
    py::array_t<uint8_t> ret;
    if (image_message) {
      ret.resize(std::vector<size_t>(
          {image_message->height, image_message->width, 4}));
      {
        auto r = ret.mutable_data();
        for (size_t i = 0; i < image_message->data.size(); i++) {
          r[i] = image_message->data[i];
        }
      }
    }
    return ret;
  });

  m.def("start_embedded", [](uint64_t ptr) {
    init();
    g_show_split_window_bars = false;
    main_window = std::make_shared<MainWindow>(true);
    main_window->menuBar()->hide();
    for (auto *bar : main_window->findChildren<QToolBar *>()) {
      bar->hide();
    }
    for (auto *dock : main_window->findChildren<QDockWidget *>()) {
      dock->hide();
    }
    main_window->setWindowFlags(Qt::Widget);
    QWidget *parent = (QWidget *)ptr;
    main_window->setParent(parent);
    if (auto *layout = parent->layout()) {
      layout->addWidget(&*main_window);
    }
    main_window->show();
    RenderThread::start();
    spinner = std::make_shared<ros::AsyncSpinner>(0);
    spinner->start();
  });

  m.def("open", [](const std::string &path) {
    if (main_window) {
      main_window->openDocument(path.c_str());
    } else {
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
  });

  static std::function<void()> shutdown = []() {
    // LOG_INFO("shutdown started");
    spinner.reset();

    // LOG_INFO("shutdown phase 2");
    RenderThread::stop();

    // LOG_INFO("shutdown phase 3");
    // wait for async cleanup from renderthread to be finished
    app->processEvents();

    // LOG_INFO("shutdown phase 4");
    if (main_window) {
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

    // LOG_INFO("shutdown phase 5");
    if (main_window) {
      main_window.reset();
    }

    // LOG_INFO("shutdown phase 6");
    {
      LockScope ws;
      ws() = nullptr;
    }

    // LOG_INFO("shutdown phase 7");
    spinner.reset();
    main_window.reset();
    node.reset();
    app.reset();
    //  LOG_INFO("shutdown finished");
  };

  m.def("run", []() {
    if (main_window) {
      app->exec();
    } else {
      ros::waitForShutdown();
    }
    shutdown();
  });

  m.def("shutdown", []() { shutdown(); });
}
