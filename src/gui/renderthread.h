// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

class RenderThread {
  bool _redraw_flag = false;
  bool _stop_flag = false;
  bool _running = false;
  std::mutex _mutex;
  std::condition_variable _condition;
  std::thread _thread;

private:
  void run();

public:
  RenderThread();
  ~RenderThread();
  void stop();
  static RenderThread *instance();
  void invalidate();
};
