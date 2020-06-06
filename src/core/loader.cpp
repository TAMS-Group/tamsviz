// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "loader.h"

void LoaderThread::start(const std::shared_ptr<void> &owner,
                         const std::function<void()> &fnc) {
  std::unique_lock<std::mutex> lock(_mutex);
  _queue.emplace_back(owner, fnc);
  _condition.notify_one();
}

void LoaderThread::cancel(const std::shared_ptr<void> &owner) {
  std::unique_lock<std::mutex> lock(_mutex);
  for (auto it = _queue.begin(); it != _queue.end();) {
    if (it->first == owner) {
      it = _queue.erase(it);
    } else {
      it++;
    }
  }
}

LoaderThread::LoaderThread() {
  _thread = std::thread([this]() {
    while (true) {
      std::function<void()> job;
      {
        std::unique_lock<std::mutex> lock(_mutex);
        while (true) {
          if (_stop) {
            return;
          }
          if (!_queue.empty()) {
            job = _queue.front().second;
            _queue.pop_front();
            break;
          }
          _condition.wait(lock);
        }
      }
      job();
      finished();
    }
  });
}

LoaderThread::~LoaderThread() {
  {
    std::unique_lock<std::mutex> lock(_mutex);
    _stop = true;
    _condition.notify_all();
  }
  _thread.join();
}

const std::shared_ptr<LoaderThread> &LoaderThread::instance() {
  static auto instance = std::make_shared<LoaderThread>();
  return instance;
}
