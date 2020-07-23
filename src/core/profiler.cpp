// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "profiler.h"

#include "log.h"

#include <algorithm>
#include <chrono>

ProfilerData ProfilerTrack::swap() {
  ProfilerData ret;
  ret.time = _time.exchange(0);
  ret.count = _count.exchange(0);
  return ret;
}

std::shared_ptr<Profiler> Profiler::instance() {
  static std::shared_ptr<Profiler> instance = std::make_shared<Profiler>();
  return instance;
}

std::shared_ptr<ProfilerTrack>
Profiler::track(const std::shared_ptr<ProfilerTrack> &track) {
  if (track) {
    std::unique_lock<std::mutex> lock(_mutex);
    _tracks.emplace_back(track);
  }
  return track;
}

std::vector<std::shared_ptr<ProfilerTrack>> Profiler::tracks() const {
  std::unique_lock<std::mutex> lock(_mutex);
  std::vector<std::shared_ptr<ProfilerTrack>> ret;
  for (auto it = _tracks.begin(); it != _tracks.end();) {
    if (auto ptr = it->lock()) {
      ret.push_back(ptr);
      it++;
    } else {
      it = _tracks.erase(it);
    }
  }
  return std::move(ret);
}

ProfilerTrack::ProfilerTrack(const std::string &name) : _name(name) {
  _time = 0;
}

ProfilerTrack::~ProfilerTrack() {}

ProfilerThread::ProfilerThread(const std::shared_ptr<Profiler> &profiler) {
  _thread = std::thread([this, profiler]() {
    auto timeout = std::chrono::steady_clock::now();
    while (true) {
      std::vector<std::shared_ptr<ProfilerTrack>> tracks;
      {
        std::unique_lock<std::mutex> lock(_mutex);
        while (true) {
          if (_exit) {
            return;
          }
          if (std::chrono::steady_clock::now() >= timeout) {
            break;
          }
          _condition.wait_until(lock, timeout);
        }
        tracks = profiler->tracks();
      }
      std::vector<std::pair<ProfilerData, std::shared_ptr<ProfilerTrack>>> data;
      for (auto &t : tracks) {
        data.emplace_back(t->swap(), t);
      }
      std::sort(
          data.begin(), data.end(),
          [](const std::pair<ProfilerData, std::shared_ptr<ProfilerTrack>> &a,
             const std::pair<ProfilerData, std::shared_ptr<ProfilerTrack>> &b) {
            return a.first.time > b.first.time;
          });
      std::stringstream stream;
      stream << "profiler\n";
      for (auto &row : data) {
        if (row.first.count > 0) {
          stream << row.first.time << " " << row.first.count << " "
                 << row.second->name() << "\n";
        }
      }
      stream << "\n";
      LOG_DEBUG(stream.str());
      timeout = std::max(timeout + std::chrono::seconds(5),
                         std::chrono::steady_clock::now());
    }
  });
}

ProfilerThread::~ProfilerThread() {
  {
    std::unique_lock<std::mutex> lock(_mutex);
    _exit = true;
    _condition.notify_all();
  }
  _thread.join();
}
