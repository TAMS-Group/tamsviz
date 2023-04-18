// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

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
  void _stop();
  void _run();

public:
  RenderThread();
  ~RenderThread();
  static void start();
  static void stop();
  static RenderThread *instance();
  void invalidate();
};
