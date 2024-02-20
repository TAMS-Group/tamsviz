// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "renderthread.h"

#include "../annotations/scene.h"
#include "../core/bagplayer.h"
#include "../core/loader.h"
#include "../core/log.h"
#include "../core/profiler.h"
#include "../core/topic.h"
#include "../core/workspace.h"
#include "../render/renderer.h"
#include "../render/resource.h"
#include "../render/shader.h"
#include "renderwindow.h"

RenderThread::RenderThread() {}

void RenderThread::invalidate() {
  // LOG_DEBUG("render thread invalidate");
  std::unique_lock<std::mutex> lock(_mutex);
  _redraw_flag = true;
  _condition.notify_all();
}

void RenderThread::invalidate(const std::function<void()> &callback) {
  {
    std::unique_lock<std::mutex> lock(_mutex);
    if (!_stop_flag) {
      _redraw_flag = true;
      _callbacks.push_back(callback);
      _condition.notify_all();
      return;
    }
  }
  callback();
}

void RenderThread::_run() {
  LOG_DEBUG("render thread started");

  QOffscreenSurface offscreen_surface;
  offscreen_surface.create();
  if (!offscreen_surface.isValid()) {
    throw std::runtime_error("failed to create offscreen surface");
  }

  QOpenGLContext offscreen_context;
  offscreen_context.setShareContext(QOpenGLContext::globalShareContext());
  if (!offscreen_context.create()) {
    throw std::runtime_error("failed to create offscreen context");
  }

  offscreen_context.makeCurrent(&offscreen_surface);

  std::vector<std::function<void()>> garbage_list;
  std::mutex garbage_mutex;
  ResourceBase::setCleanupFunction([&](const std::function<void()> &garbage) {
    std::lock_guard<std::mutex> lock(garbage_mutex);
    garbage_list.push_back(garbage);
  });
  std::vector<std::shared_ptr<RenderWindowBase>> render_window_list;
  std::vector<std::shared_ptr<Display>> display_list;
  std::vector<std::shared_ptr<SceneAnnotationBase>> scene_annotations;

  Renderer renderer;
  RenderList render_list;
  while (true) {
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::vector<std::function<void()>> callbacks;
    {
      std::unique_lock<std::mutex> lock(_mutex);
      while (true) {
        if (_stop_flag) {
          break;
        }
        if (_redraw_flag) {
          _redraw_flag = false;
          callbacks = _callbacks;
          _callbacks.clear();
          break;
        }
        _condition.wait(lock);
      }
      if (_stop_flag) {
        break;
      }
    }
    LOG_DEBUG("render");

    PROFILER();

    render_list.clear();
    {
      NoMessageScope m_scope;
      LockScope ws;
      ws->recurseObjects([&render_window_list, &display_list](
                             const std::shared_ptr<Object> &object) {
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
      {
        scene_annotations.clear();
        if (ws->player && ws->document()->timeline()) {
          auto current_time = ws->player->time();
          for (auto &track : ws->document()->timeline()->tracks()) {
            if (auto annotation_track =
                    std::dynamic_pointer_cast<AnnotationTrack>(track)) {
              if (auto branch = annotation_track->branch(ws(), false)) {
                for (auto &span : branch->spans()) {
                  if (span->start() <= current_time &&
                      span->start() + span->duration() >= current_time) {
                    for (auto &annotation : span->annotations()) {
                      if (auto scene_annotation =
                              std::dynamic_pointer_cast<SceneAnnotationBase>(
                                  annotation)) {
                        {
                          RenderSyncContext render_context;
                          render_context.render_list = &render_list;
                          scene_annotation->renderSync(render_context, track,
                                                       span);
                        }
                        scene_annotations.push_back(scene_annotation);
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
      for (auto &render_window : render_window_list) {
        RenderWindowSyncContext render_context;
        render_window->renderWindowSync(render_context);
      }
    }
    {
      RenderAsyncContext render_context;
      render_context.render_list = &render_list;
      for (auto &display : display_list) {
        display->renderAsync(render_context);
      }
      for (auto &scene_annotation : scene_annotations) {
        scene_annotation->renderAsync(render_context);
      }
    }
    // V_GL(glFlush());
    // V_GL(glFinish());
    renderer.renderShadows(render_list);
    {
      RenderViewsAsyncContext render_context;
      render_context.render_list = &render_list;
      render_context.renderer = &renderer;
      for (auto &display : display_list) {
        display->renderViewsAsync(render_context);
      }
    }
    for (auto &render_window : render_window_list) {
      RenderWindowAsyncContext render_context;
      render_context.render_list = &render_list;
      render_context.renderer = &renderer;
      render_window->renderWindowAsync(render_context);
    }
    for (auto &render_window : render_window_list) {
      startOnMainThreadAsync([render_window]() { render_window->present(); });
    }
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
    for (auto &callback : callbacks) {
      callback();
    }
    callbacks.clear();
  }
  {
    std::vector<std::function<void()>> callbacks;
    {
      std::unique_lock<std::mutex> lock(_mutex);
      callbacks = _callbacks;
      _callbacks.clear();
    }
    for (auto &c : callbacks) {
      c();
    }
    callbacks.clear();
  }
  offscreen_context.doneCurrent();
  ResourceBase::setCleanupFunction(nullptr);
}

void RenderThread::_stop() {
  if (_running) {
    _running = false;
    LOG_DEBUG("stopping render thread");
    {
      std::unique_lock<std::mutex> lock(_mutex);
      _stop_flag = true;
      _condition.notify_all();
    }
    _thread.join();
    LOG_DEBUG("render thread stopped");
  }
}

RenderThread::~RenderThread() { _stop(); }

static std::shared_ptr<RenderThread> &renderThreadInstance() {
  static std::shared_ptr<RenderThread> instance;
  return instance;
};

void RenderThread::stop() {
  if (auto instance = renderThreadInstance()) {
    instance->_stop();
  }
}

static const std::string g_topic_name_tf = "/tf";
static const std::string g_topic_name_tf_static = "/tf_static";

void RenderThread::start() {
  auto &instance = renderThreadInstance();

  instance = std::make_shared<RenderThread>();

  auto *ptr = instance.get();

  instance->_running = true;
  instance->_thread = std::thread([ptr]() { ptr->_run(); });

  {
    LockScope ws;
    ws->modified.connect(instance, [ptr]() { ptr->invalidate(); });
    GlobalEvents::instance()->redraw.connect(instance,
                                             [ptr]() { ptr->invalidate(); });
  }
  LoaderThread::instance()->started.connect(instance,
                                            [ptr]() { ptr->invalidate(); });
  LoaderThread::instance()->finished.connect(instance,
                                             [ptr]() { ptr->invalidate(); });
  TopicManager::instance()->received.connect(
      instance,
      [ptr](const Topic &topic, const std::shared_ptr<const Message> &message) {
        auto &topic_name = topic.name();
        if (topic_name != g_topic_name_tf &&
            topic_name != g_topic_name_tf_static)
          ptr->invalidate();
      });
}

RenderThread *RenderThread::instance() { return renderThreadInstance().get(); }
