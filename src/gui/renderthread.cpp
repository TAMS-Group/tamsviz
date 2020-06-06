// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "renderthread.h"

#include "../core/bagplayer.h"
#include "../core/loader.h"
#include "../core/log.h"
#include "../core/topic.h"
#include "../core/workspace.h"

#include "../render/context.h"
#include "../render/renderer.h"
#include "../render/resource.h"
#include "../render/shader.h"

#include "glqtwrapper.h"
#include "renderwindow.h"

RenderThread::RenderThread() {
  _gl_display.reset(new GLDisplay());
  _gl_context.reset(new GLContext(_gl_display.get()));
  _gl_offscreen_surface.reset(new GLSurface(_gl_display.get()));
  setObjectName("RenderThread");
  start();
}

void RenderThread::run() {
  LOG_DEBUG("render thread started");

  // try {

  // auto *qtgl = egl2qt(gl_context->eglContext(), gl_display->eglDisplay());

  std::vector<std::function<void()>> garbage_list;
  std::mutex garbage_mutex;
  ResourceBase::setCleanupFunction([&](const std::function<void()> &garbage) {
    std::lock_guard<std::mutex> lock(garbage_mutex);
    garbage_list.push_back(garbage);
  });
  std::vector<std::shared_ptr<RenderWindowBase>> render_window_list;
  std::vector<std::shared_ptr<Display>> display_list;
  // Shader shader("package://" ROS_PACKAGE_NAME "/shaders/main.vert",
  //            "package://" ROS_PACKAGE_NAME "/shaders/main.frag");
  //{
  // GLScope gl_scope(gl_context.get(), gl_offscreen_surface.get());
  /*{
    V_GL(glEnable(GL_DEBUG_OUTPUT));
    V_GL(glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS));
    V_GL(glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0,
                               nullptr, GL_TRUE));
    V_GL(glDebugMessageCallback(
        [](GLenum source, GLenum type, GLuint id, GLenum severity,
           GLsizei length, const GLchar *message,
           const void *userParam) { LOG_INFO("GL " << message); },
        0));
  }*/
  // shader.use();
  //}
  Renderer renderer;
  RenderList render_list;
  while (true) {
    {
      std::unique_lock<std::mutex> lock(_mutex);
      if (_stop_flag) {
        break;
      }
      if (!_redraw_flag) {
        _condition.wait(lock);
        continue;
      }
      _redraw_flag = false;
    }
    render_list.clear();
    {
      GLScope gl_scope(_gl_context.get(), _gl_offscreen_surface.get());
      {
        LockScope ws;
        ws->recurse([&render_window_list,
                     &display_list](const std::shared_ptr<Object> &object) {
          if (object) {
            if (auto render_window =
                    std::dynamic_pointer_cast<RenderWindowBase>(object)) {
              render_window_list.push_back(render_window);
            }
            if (auto display = std::dynamic_pointer_cast<Display>(object)) {
              display_list.push_back(display);
            }
          }
        });
        {
          std::lock_guard<std::mutex> lock(garbage_mutex);
          for (auto &f : garbage_list) {
            f();
          }
          garbage_list.clear();
        }
        {
          RenderSyncContext render_context;
          render_context.render_list = &render_list;
          ws->document()->display()->renderSyncRecursive(render_context);
        }
        for (auto &render_window : render_window_list) {
          RenderWindowSyncContext render_context;
          render_window->renderWindowSync(render_context);
        }
      }
    }
    {
      GLScope gl_scope(_gl_context.get(), _gl_offscreen_surface.get());
      RenderAsyncContext render_context;
      render_context.render_list = &render_list;
      for (auto &display : display_list) {
        display->renderAsync(render_context);
      }
    }
    for (auto &render_window : render_window_list) {
      GLScope gl_scope(_gl_context.get(), render_window->surface());
      // V_GL(glViewport(0, 0, render_window->surface()->width(),
      //                  render_window->surface()->height()));
      RenderWindowAsyncContext render_context;
      render_context.render_list = &render_list;
      render_context.renderer = &renderer;
      render_window->renderWindowAsync(render_context);
      render_window->surface()->swap();
    }
    /*for (auto &render_window : render_window_list) {
      GLScope gl_scope(gl_context.get(), render_window->surface());
      render_window->surface()->swap();
  }*/
    {
      LockScope ws;
      {
        for (auto &display : display_list) {
          if (display.unique()) {
            auto ref = std::make_shared<std::shared_ptr<Display>>(display);
            auto lambda = [ref]() {
              LockScope ws;
              ref->reset();
            };
            ref.reset();
            display.reset();
            startOnMainThreadAsync(lambda);
          }
        }
        display_list.clear();
      }
      {
        for (auto &render_window : render_window_list) {
          if (render_window.unique()) {
            auto ref = std::make_shared<std::shared_ptr<RenderWindowBase>>(
                render_window);
            auto lambda = [ref]() {
              LockScope ws;
              ref->reset();
            };
            ref.reset();
            render_window.reset();
            startOnMainThreadAsync(lambda);
          }
        }
        render_window_list.clear();
      }
    }
  }
  ResourceBase::setCleanupFunction(nullptr);

  /* } catch (const std::exception &ex) {
     LOG_FATAL(ex.what());
     LOG_FATAL("fatal error in render thread");
     std::terminate();
 }*/
}

void RenderThread::stop() {
  LOG_DEBUG("stopping render thread");
  {
    std::unique_lock<std::mutex> lock(_mutex);
    _stop_flag = true;
    _condition.notify_all();
  }
  wait();
  LOG_DEBUG("render thread stopped");
}

RenderThread *RenderThread::instance() {
  static std::shared_ptr<RenderThread> instance = []() {
    auto instance = std::make_shared<RenderThread>();
    RenderThread *ptr = instance.get();
    {
      LockScope ws;
      ws->modified.connect(instance, [ptr]() { ptr->invalidate(); });
      ws->redraw.connect(instance, [ptr]() { ptr->invalidate(); });
    }
    LoaderThread::instance()->finished.connect(instance,
                                               [ptr]() { ptr->invalidate(); });
    TopicManager::instance()->received.connect(instance,
                                               [ptr]() { ptr->invalidate(); });
    return instance;
  }();
  return instance.get();
}
