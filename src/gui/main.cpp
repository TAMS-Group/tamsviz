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

#include <ros/package.h>
#include <ros/ros.h>

#include <signal.h>

int main(int argc, char **argv) {

  {

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

    QApplication app(argc, argv);
    app.setOrganizationName("TAMS");
    app.setApplicationName(QString(ROS_PACKAGE_NAME).toUpper());

    std::setlocale(LC_NUMERIC, "en_US.UTF-8");

    QCommandLineParser parser;
    parser.setApplicationDescription(
        "TAMSVIZ - Visualization and annotation tool for ROS");
    parser.addHelpOption();

    parser.addPositionalArgument("files", "Bag and visualization files to open",
                                 "[files...]");

    QCommandLineOption opt_profiler("profiler", "Run with profiler enabled");
    parser.addOption(opt_profiler);

    QCommandLineOption opt_maximize("maximize", "Maximize window");
    parser.addOption(opt_maximize);

    parser.process(app);

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

    std::unique_ptr<ProfilerThread> profiler_thread;
    if (parser.isSet(opt_profiler)) {
      profiler_thread.reset(new ProfilerThread());
    }

    {
      {
        RenderThread::instance();
        MainWindow main_window;
        // qDebug() << "styles" << QStyleFactory::keys();
        if (parser.isSet(opt_maximize)) {
          main_window.showMaximized();
        }
        for (size_t i = 0; i < parser.positionalArguments().size(); i++) {
          QString file = parser.positionalArguments()[i];
          if (!file.isEmpty()) {
            qDebug() << "opening file" << file;
            main_window.openAny(file);
          }
        }
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
