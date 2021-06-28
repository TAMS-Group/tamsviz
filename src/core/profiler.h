// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

struct ProfilerData {
  uint64_t time = 0;
  uint64_t count = 0;
};

class ProfilerTrack {
  std::string _name;
  std::atomic<int64_t> _time;
  std::atomic<int64_t> _count;

public:
  ProfilerTrack(const std::string &name);
  ProfilerTrack(const ProfilerTrack &) = delete;
  ProfilerTrack &operator=(const ProfilerTrack &) = delete;
  ~ProfilerTrack();
  const std::string &name() const { return _name; }
  inline void add(int64_t time) {
    _time += time;
    _count++;
  }
  ProfilerData swap();
};

class ProfilerScope {
  ProfilerTrack &_track;
  std::chrono::steady_clock::time_point _start;

public:
  inline ProfilerScope(ProfilerTrack &track)
      : _track(track), _start(std::chrono::steady_clock::now()) {}
  inline ~ProfilerScope() {
    _track.add(std::chrono::duration_cast<std::chrono::microseconds, int64_t>(
                   std::chrono::steady_clock::now() - _start)
                   .count());
  }
  ProfilerScope(const ProfilerScope &) = delete;
  ProfilerScope &operator=(const ProfilerScope &) = delete;
};

class Profiler {
  mutable std::mutex _mutex;
  mutable std::vector<std::weak_ptr<ProfilerTrack>> _tracks;

public:
  static std::shared_ptr<Profiler> instance();
  std::shared_ptr<ProfilerTrack>
  track(const std::shared_ptr<ProfilerTrack> &track);
  std::vector<std::shared_ptr<ProfilerTrack>> tracks() const;
};

class ProfilerThread {
  std::thread _thread;
  bool _exit = false;
  std::mutex _mutex;
  std::condition_variable _condition;

public:
  ProfilerThread(
      const std::shared_ptr<Profiler> &profiler = Profiler::instance());
  ~ProfilerThread();
};

#define PROFILER(...)                                                          \
  static std::shared_ptr<ProfilerTrack> profiler_track =                       \
      Profiler::instance()->track(std::make_shared<ProfilerTrack>(             \
          std::string() + __PRETTY_FUNCTION__ + " " __VA_ARGS__));             \
  ProfilerScope profiler_scope(*profiler_track);
