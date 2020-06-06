// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include <QThread>

#include <condition_variable>
#include <memory>
#include <mutex>

class GLDisplay;
class GLContext;
class GLSurface;

class RenderThread : public QThread {
  std::unique_ptr<GLDisplay> _gl_display;
  std::unique_ptr<GLContext> _gl_context;
  std::unique_ptr<GLSurface> _gl_offscreen_surface;
  bool _redraw_flag = false;
  bool _stop_flag = false;
  std::mutex _mutex;
  std::condition_variable _condition;

protected:
  virtual void run() override;

public:
  RenderThread();
  void stop();
  static RenderThread *instance();
  GLDisplay *display() { return _gl_display.get(); }
  void invalidate() {
    std::unique_lock<std::mutex> lock(_mutex);
    _redraw_flag = true;
    _condition.notify_all();
  }
};
