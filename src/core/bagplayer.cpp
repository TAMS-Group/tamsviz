// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "bagplayer.h"

#include "log.h"
#include "topic.h"

#include <rosbag/view.h>

#include <chrono>

class RosBagView : public rosbag::View {

public:
  RosBagView(const rosbag::Bag &bag) : rosbag::View(bag) {}
  void findMessages(
      const ros::Time &t,
      const std::function<void(const rosbag::MessageInstance &)> &callback) {
    for (auto *range : ranges_) {
      if (range->begin != range->end) {
        auto iter = std::upper_bound(range->begin, range->end, t,
                                     rosbag::IndexEntryCompare());
        if (iter == range->end) {
          iter = range->end;
          --iter;
        }
        if (iter != range->begin) {
          --iter;
        }
        auto *message = newMessageInstance(range->connection_info, *iter,
                                           *range->bag_query->bag);
        callback(*message);
        delete message;
        continue;
      }
    }
  }
  bool findTimeSpan(const std::string &topic, double t, double *start,
                    double *duration) {
    for (auto *range : ranges_) {
      if (range->connection_info->topic != topic) {
        continue;
      }
      auto iter2 = std::upper_bound(range->begin, range->end,
                                    getBeginTime() + ros::Duration(t),
                                    rosbag::IndexEntryCompare());
      if (iter2 == range->end) {
        continue;
      }
      auto iter = iter2;

      --iter;
      if (iter == range->end) {
        continue;
      }

      *start = (iter->time - getBeginTime()).toSec();
      *duration = (iter2->time - getBeginTime()).toSec() - *start;
      return true;
    }
    return false;
  }
  std::vector<std::shared_ptr<Message>>
  readMessageSamples(const std::string &topic, double start, double stop,
                     double step) {
    std::vector<std::shared_ptr<Message>> ret;
    for (auto *range : ranges_) {
      if (range->connection_info->topic != topic) {
        continue;
      }
      for (auto it = range->begin; it != range->end; it++) {
        std::unique_ptr<rosbag::MessageInstance> message(newMessageInstance(
            range->connection_info, *it, *range->bag_query->bag));
        if (auto inst = message->instantiate<Message>()) {
          inst->time(message->getTime());
          ret.push_back(std::make_shared<Message>(*inst));
        }
      }
      return ret;
    }
    return ret;
  }
  friend class RosBagIterator;
};

class RosBagIterator {
  RosBagView *_view = nullptr;
  std::multimap<ros::Time,
                std::pair<std::multiset<rosbag::IndexEntry>::const_iterator,
                          rosbag::MessageRange *>>
      _iterators;
  mutable std::unique_ptr<rosbag::MessageInstance> _message_instance;

public:
  RosBagIterator() {}
  RosBagIterator(RosBagView &view) : _view(&view) {
    for (auto *range : _view->ranges_) {
      _iterators.emplace(range->begin->time,
                         std::make_pair(range->begin, range));
    }
  }
  RosBagIterator(RosBagView &view, double time_from_start) : _view(&view) {
    for (auto *range : _view->ranges_) {
      auto iter = std::lower_bound(range->begin, range->end,
                                   _view->getBeginTime() +
                                       ros::Duration(time_from_start),
                                   rosbag::IndexEntryCompare());
      if (iter != range->end) {
        _iterators.emplace(range->begin->time, std::make_pair(iter, range));
      }
    }
  }
  void seek(double time) {}
  bool operator==(const RosBagIterator &other) {
    return _iterators == other._iterators;
  }
  bool operator!=(const RosBagIterator &other) {
    return _iterators != other._iterators;
  }
  RosBagIterator &operator++() {
    auto iter = _iterators.begin();
    auto pair = iter->second;
    _iterators.erase(iter);
    ++pair.first;
    if (pair.first != pair.second->end) {
      _iterators.emplace(pair.first->time, pair);
    }
    return *this;
  }
  const rosbag::MessageInstance &operator*() const {
    auto &pair = _iterators.begin()->second;
    _message_instance.reset(
        _view->newMessageInstance(pair.second->connection_info, *pair.first,
                                  *(pair.second->bag_query->bag)));
    return *_message_instance;
  }
};

bool BagPlayer::findMessageTimeSpan(const std::string &topic, double time,
                                    double *start, double *duration) const {
  std::unique_lock<std::mutex> lock(_view_mutex);
  return _view->findTimeSpan(topic, time, start, duration);
}

std::shared_ptr<Message>
BagPlayer::instantiate_nolock(const rosbag::MessageInstance &msg) {
  boost::shared_ptr<Message> inst = msg.instantiate<Message>();
  if (!inst) {
    return nullptr;
  }
  auto m = std::make_shared<Message>(*inst);
  m->time(msg.getTime());
  return m;
}

void BagPlayer::publish(const std::string &topic,
                        const std::shared_ptr<Message> &message) {
  if (message) {
    if (0) {
      LOG_DEBUG("bag message topic:"
                << topic
                << " time:" << (message->time() - _bag_start_time).toSec());
    }
    _topics[topic]->publish(message);
  }
}

void BagPlayer::publish(const rosbag::MessageInstance &msg) {
  std::shared_ptr<Message> m;
  {
    std::unique_lock<std::mutex> lock(_view_mutex);
    m = instantiate_nolock(msg);
  }
  publish(msg.getTopic(), m);
}

std::vector<std::shared_ptr<Message>>
BagPlayer::readMessageSamples(const std::string &topic, double start,
                              double stop, double step) {
  std::unique_lock<std::mutex> lock(_view_mutex);
  return _view->readMessageSamples(topic, start, stop, step);
}

BagPlayer::BagPlayer(const std::string &path) : _path(path) {
  LOG_DEBUG("opening bag " << path);
  _file_name = path;
  if (auto *n = std::strrchr(path.c_str(), '/')) {
    _file_name = n + 1;
  }
  _bag.open(path);
  _time_function = []() { return 0.0; };
  _view = std::make_shared<RosBagView>(_bag);
  std::vector<std::string> topic_names;
  for (auto *conn : _view->getConnections()) {
    // LOG_INFO(conn->datatype << " " << conn->topic);
    // LOG_INFO(conn->msg_def);
    _topics[conn->topic] = Topic::instance(conn->topic);
    topic_names.push_back(conn->topic);
  }
  _message_playback_scope = std::make_shared<MessagePlaybackScope>(topic_names);
  _duration = (_view->getEndTime() - _view->getBeginTime()).toSec();
  _bag_start_time = _view->getBeginTime();
  LOG_DEBUG("rosbag opened path:" << path << " duration:" << _duration);
  _thread = std::thread([this]() {
    LOG_DEBUG("bag thread started");
    while (true) {
      std::function<void()> action;
      while (true) {
        std::unique_lock<std::mutex> lock(_action_mutex);
        if (_exit) {
          LOG_DEBUG("exiting bag player thread");
          return;
        }
        if (_pending_action) {
          LOG_DEBUG("new bag player action");
          action = _pending_action;
          _pending_action = nullptr;
          break;
        }
        _action_condition.wait(lock);
      }
      LOG_DEBUG("executing bag player action");
      action();
      LOG_DEBUG("bag player action finished");
    }
    LOG_DEBUG("exiting bag thread");
  });
  rewind();
  LOG_DEBUG("bag opened");
}

double BagPlayer::time() {
  std::unique_lock<std::mutex> lock(_action_mutex);
  return _time_function();
}

void BagPlayer::startAction(const std::function<void()> &action) {
  std::unique_lock<std::mutex> lock(_action_mutex);
  _pending_action = action;
  _action_condition.notify_one();
}

void BagPlayer::stop() {
  LOG_DEBUG("bag stop");
  startAction([this]() {});
}

void BagPlayer::play() {
  LOG_DEBUG("bag play");
  startAction([this]() {
    double time_from_start = this->time();
    if (time_from_start + 1e-6 >= _duration) {
      time_from_start = 0.0;
    }
    auto playback_start_time = std::chrono::high_resolution_clock::now();
    int64_t view_start_time =
        (_bag_start_time + ros::Duration(time_from_start)).toNSec();
    {
      std::unique_lock<std::mutex> lock(_action_mutex);
      double duration = this->duration();
      _time_function = [duration, playback_start_time, time_from_start]() {
        return std::min(
            duration,
            std::chrono::duration<double>(
                std::chrono::high_resolution_clock::now() - playback_start_time)
                    .count() +
                time_from_start);
      };
    }
    RosBagIterator it;
    RosBagIterator it_end;
    {
      std::unique_lock<std::mutex> lock(_view_mutex);
      // it = _view->begin();
      // it_end = _view->end();
      it = RosBagIterator(*_view, time_from_start);
    }
    while (it != it_end) {
      std::shared_ptr<rosbag::MessageInstance> msg;
      {
        std::lock_guard<std::mutex> lock(_view_mutex);
        msg = std::make_shared<rosbag::MessageInstance>(*it);
      }
      int64_t message_time = msg->getTime().toNSec();
      auto send_time = playback_start_time +
                       std::chrono::nanoseconds(message_time - view_start_time);
      bool interrupted = false;
      {
        std::unique_lock<std::mutex> lock(_action_mutex);
        while (true) {
          auto current_time = std::chrono::high_resolution_clock::now();
          if (interrupted_nolock()) {
            interrupted = true;
            break;
          }
          if (current_time >= send_time) {
            break;
          }
          _action_condition.wait_until(lock, send_time);
        }
      }
      if (interrupted) {
        break;
      }
      publish(*msg);
      {
        std::lock_guard<std::mutex> lock(_view_mutex);
        ++it;
      }
    }
    {
      std::lock_guard<std::mutex> lock(_view_mutex);
      it = RosBagIterator();
      it_end = RosBagIterator();
    }
    {
      std::unique_lock<std::mutex> lock(_action_mutex);
      double t = _time_function();
      _time_function = [t]() { return t; };
    }
  });
}

void BagPlayer::seek(double time_from_start) {
  LOG_DEBUG("bag seek T0+" << time_from_start);
  startAction([this, time_from_start]() {
    {
      std::unique_lock<std::mutex> lock(_action_mutex);
      _time_function = [time_from_start]() { return time_from_start; };
    }
    std::vector<std::pair<std::string, std::shared_ptr<Message>>> messages;
    {
      std::lock_guard<std::mutex> lock(_view_mutex);
      auto seek_time = _view->getBeginTime() + ros::Duration(time_from_start);
      _view->findMessages(seek_time, [&](const rosbag::MessageInstance &msg) {
        if (auto instance = instantiate_nolock(msg)) {
          messages.emplace_back(msg.getTopic(), instance);
        }
      });
    }
    for (auto &m : messages) {
      publish(m.first, m.second);
    }
  });
}

void BagPlayer::rewind() {
  LOG_DEBUG("bag rewind");
  seek(0.0);
}

BagPlayer::~BagPlayer() {
  LOG_DEBUG("stopping bag player " << _path);
  {
    std::unique_lock<std::mutex> lock(_action_mutex);
    _exit = true;
    _action_condition.notify_one();
  }
  _thread.join();
  _bag.close();
  LOG_DEBUG("rosbag closed " << _path);
}

std::vector<std::string> BagPlayer::listTopics(const std::string &type_name) {
  std::lock_guard<std::mutex> lock(_view_mutex);
  std::vector<std::string> ret;
  for (auto *conn : _view->getConnections()) {
    if (conn->datatype == type_name) {
      ret.push_back(conn->topic);
    }
  }
  return ret;
}
