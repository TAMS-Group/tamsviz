// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "bagplayer.h"

#include "log.h"
#include "message.h"
#include "topic.h"
#include "workspace.h"

#include <ros/ros.h>
#include <rosbag/view.h>

#include <chrono>

class RosBagViewBase {
protected:
  rosbag::Bag _bag;
  RosBagViewBase(const std::string &filename) { _bag.open(filename); }
};

struct RosBagTopicInfo {
  std::string name, type, md5, definition;
};

class RosBagView : protected RosBagViewBase, protected rosbag::View {

  std::vector<std::pair<std::string, std::string>> _viz_topic_list;
  std::unordered_map<std::string, std::string> _topic_bag_to_viz;
  std::unordered_map<std::string, std::string> _topic_viz_to_bag;
  std::vector<RosBagTopicInfo> _topic_infos;

  std::shared_ptr<const Message> loadMessage(const rosbag::ConnectionInfo *conn,
                                             const rosbag::IndexEntry &index,
                                             const rosbag::Bag &bag) {
    auto *msg = newMessageInstance(conn, index, bag);
    boost::shared_ptr<Message> inst = msg->instantiate<Message>();
    auto m = std::make_shared<Message>(*inst);
    m->time(msg->getTime());
    delete msg;
    return m;
  }

public:
  RosBagView(const std::string &filename)
      : RosBagViewBase(filename), rosbag::View(_bag) {
    for (auto &conn : getConnections()) {
      std::string viz_name = conn->topic;
      if (viz_name.empty() || viz_name[0] != '/') {
        viz_name = "/" + viz_name;
      }
      _topic_bag_to_viz[conn->topic] = viz_name;
      _topic_viz_to_bag[viz_name] = conn->topic;
      _viz_topic_list.emplace_back(conn->datatype, viz_name);
      RosBagTopicInfo info;
      info.name = viz_name;
      info.type = conn->datatype;
      info.md5 = conn->md5sum;
      info.definition = conn->msg_def;
      _topic_infos.push_back(info);
    }
  }

  const std::vector<RosBagTopicInfo> &topicInfos() const {
    return _topic_infos;
  }

  ros::Time startTime() { return getBeginTime(); }
  ros::Time endTime() { return getEndTime(); }

  std::vector<std::pair<std::string, std::string>> topics() {
    return _viz_topic_list;
  }

  void
  findMessages(const ros::Time &t,
               const std::function<void(const std::string &,
                                        const std::shared_ptr<const Message> &)>
                   &callback) {
    for (auto *range : ranges_) {
      if (range->begin != range->end) {
        auto iter = std::upper_bound(range->begin, range->end, t,
                                     rosbag::IndexEntryCompare());
        if (iter != range->begin) {
          --iter;
        }
        if (iter == range->end) {
          iter = range->end;
          --iter;
        }
        callback(
            _topic_bag_to_viz[range->connection_info->topic],
            loadMessage(range->connection_info, *iter, *range->bag_query->bag));
        continue;
      }
    }
  }

  bool findTimeSpan(const std::string &viz_topic, double t, double *start,
                    double *duration) {
    auto &bag_topic = _topic_viz_to_bag[viz_topic];
    for (auto *range : ranges_) {
      if (range->connection_info->topic != bag_topic) {
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

  class Iterator {
    RosBagView *_view = nullptr;
    std::multimap<ros::Time,
                  std::pair<std::multiset<rosbag::IndexEntry>::const_iterator,
                            rosbag::MessageRange *>>
        _iterators;

  public:
    Iterator() {}
    Iterator(RosBagView &view) : _view(&view) {
      for (auto *range : _view->ranges_) {
        _iterators.emplace(range->begin->time,
                           std::make_pair(range->begin, range));
      }
    }
    Iterator(RosBagView &view, double time_from_start) : _view(&view) {
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
    bool operator==(const Iterator &other) {
      return _iterators == other._iterators;
    }
    bool operator!=(const Iterator &other) {
      return _iterators != other._iterators;
    }
    Iterator &operator++() {
      auto iter = _iterators.begin();
      auto pair = iter->second;
      _iterators.erase(iter);
      ++pair.first;
      if (pair.first != pair.second->end) {
        _iterators.emplace(pair.first->time, pair);
      }
      return *this;
    }
    std::pair<std::string, std::shared_ptr<const Message>> operator*() const {
      auto &pair = _iterators.begin()->second;
      return std::make_pair(
          _view->_topic_bag_to_viz[pair.second->connection_info->topic],
          _view->loadMessage(pair.second->connection_info, *pair.first,
                             *(pair.second->bag_query->bag)));
    }
  };
};

bool BagPlayer::findMessageTimeSpan(const std::string &topic, double time,
                                    double *start, double *duration) const {
  std::unique_lock<std::mutex> lock(_view_mutex);
  return _view->findTimeSpan(topic, time, start, duration);
}

void BagPlayer::publish(const std::string &topic,
                        const std::shared_ptr<const Message> &message) {
  if (message) {
    if (0) {
      LOG_DEBUG("bag message topic:"
                << topic
                << " time:" << (message->time() - _bag_start_time).toSec());
    }
    _topics[topic]->publish(message);
    if (_data->republish) {
      _republishers[topic].publish(*message);
    }
  }
}

void BagPlayer::readMessageSamples(
    const std::string &topic, double start, double stop,
    const std::function<bool(const std::shared_ptr<const Message> &)>
        &callback) {
  RosBagView::Iterator it;
  RosBagView::Iterator it_end;
  {
    std::unique_lock<std::mutex> lock(_view_mutex);
    it = RosBagView::Iterator(*_view, start);
  }
  while (it != it_end) {
    std::pair<std::string, std::shared_ptr<const Message>> p;
    {
      std::unique_lock<std::mutex> lock(_view_mutex);
      p = *it;
      if ((p.second->time() - _bag_start_time).toSec() > stop) {
        break;
      }
    }
    bool ok = callback(p.second);
    if (!ok) {
      break;
    }
    {
      std::unique_lock<std::mutex> lock(_view_mutex);
      ++it;
    }
  }
  {
    std::unique_lock<std::mutex> lock(_view_mutex);
    it = RosBagView::Iterator();
  }
}

BagPlayer::BagPlayer(const std::string &path) : _path(path) {
  LOG_DEBUG("opening bag " << path);
  ros::NodeHandle node;
  _file_name = path;
  if (auto *n = std::strrchr(path.c_str(), '/')) {
    _file_name = n + 1;
  }
  _time_function = []() { return 0.0; };
  LOG_DEBUG("creating bag view");
  _view = std::make_shared<RosBagView>(path);
  LOG_DEBUG("listing topics");
  for (auto &topic : _view->topics()) {
    LOG_DEBUG("topic " << topic.first << " " << topic.second);
    _topic_type_name_list.push_back(topic);
    _topic_names.push_back(topic.second);
    _topics[topic.second] = Topic::instance(topic.second);
  }
  for (auto &topic : _view->topicInfos()) {
    LOG_DEBUG("topic " << topic.name << " " << topic.type);
    ros::AdvertiseOptions opts(topic.name, 10, topic.md5, topic.type,
                               topic.definition);
    _republishers[topic.name] = node.advertise(opts);
  }
  LOG_DEBUG("done listing topics");
  _message_playback_scope =
      std::make_shared<MessagePlaybackScope>(_topic_names);
  LOG_DEBUG("playback scope created");
  _duration = (_view->endTime() - _view->startTime()).toSec();
  _bag_start_time = _view->startTime();
  LOG_DEBUG("rosbag opened path:" << path << " duration:" << _duration);
  _thread = std::thread([this]() {
    LOG_DEBUG("bag thread started");
    {
      auto data = _data;
      LockScope ws;
      ws->modified.connect(data, [data]() {
        data->republish = LockScope()->document()->display()->republish();
      });
      data->republish = ws->document()->display()->republish();
    }
    LOG_DEBUG("entering bag loop");
    while (true) {
      std::function<void()> action;
      {
        std::unique_lock<std::mutex> lock(_action_mutex);
        while (true) {
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
  startAction([this]() { changed(); });
}

void BagPlayer::play(const std::vector<double> &notification_times) {
  LOG_DEBUG("bag play");
  startAction([this, notification_times]() {
    _is_playing = true;
    double time_from_start = this->time();
    auto it_notify = notification_times.begin();
    while (it_notify != notification_times.end() &&
           *it_notify < time_from_start) {
      ++it_notify;
    }
    if (time_from_start + 1e-6 >= _duration) {
      time_from_start = 0.0;
    }
    auto playback_start_time = std::chrono::steady_clock::now();
    int64_t view_start_time =
        (_bag_start_time + ros::Duration(time_from_start)).toNSec();
    {
      std::unique_lock<std::mutex> lock(_action_mutex);
      double duration = this->duration();
      _time_function = [duration, playback_start_time, time_from_start]() {
        return std::min(duration, std::chrono::duration<double>(
                                      std::chrono::steady_clock::now() -
                                      playback_start_time)
                                          .count() +
                                      time_from_start);
      };
    }
    RosBagView::Iterator it;
    RosBagView::Iterator it_end;
    {
      std::unique_lock<std::mutex> lock(_view_mutex);
      it = RosBagView::Iterator(*_view, time_from_start);
    }
    while (it != it_end) {
      std::pair<std::string, std::shared_ptr<const Message>> msg;
      {
        std::lock_guard<std::mutex> lock(_view_mutex);
        msg = *it;
      }
      int64_t message_time = msg.second->time().toNSec();
      auto send_time = playback_start_time +
                       std::chrono::nanoseconds(message_time - view_start_time);
      bool interrupted = false;
      while (true) {
        changed();
        {
          std::unique_lock<std::mutex> lock(_action_mutex);
          while (true) {
            auto current_time = std::chrono::steady_clock::now();
            if (interrupted_nolock()) {
              interrupted = true;
              break;
            }
            if (current_time >= send_time) {
              break;
            }
            if (it_notify != notification_times.end()) {
              if (current_time >= playback_start_time +
                                      std::chrono::nanoseconds(
                                          int64_t(*it_notify * 1000000000.0))) {

                it_notify++;
                break;
              }
            }
            auto wakeup_time = send_time;
            if (it_notify != notification_times.end()) {
              wakeup_time =
                  std::min(send_time, playback_start_time +
                                          std::chrono::nanoseconds(int64_t(
                                              *it_notify * 1000000000.0)));
            }
            _action_condition.wait_until(lock, wakeup_time);
          }
        }
        if (interrupted) {
          break;
        }
        if (std::chrono::steady_clock::now() >= send_time) {
          break;
        }
      }
      if (interrupted) {
        break;
      }
      publish(msg.first, msg.second);
      changed();
      {
        std::lock_guard<std::mutex> lock(_view_mutex);
        ++it;
      }
    }
    {
      std::lock_guard<std::mutex> lock(_view_mutex);
      it = RosBagView::Iterator();
      it_end = RosBagView::Iterator();
    }
    {
      std::unique_lock<std::mutex> lock(_action_mutex);
      double t = _time_function();
      _time_function = [t]() { return t; };
    }
    _is_playing = false;
    changed();
  });
}

void BagPlayer::seek(double time_from_start) {
  LOG_DEBUG("bag seek T0+" << time_from_start);
  startAction([this, time_from_start]() {
    {
      std::unique_lock<std::mutex> lock(_action_mutex);
      _time_function = [time_from_start]() { return time_from_start; };
    }
    std::vector<std::pair<std::string, std::shared_ptr<const Message>>>
        messages;
    {
      std::lock_guard<std::mutex> lock(_view_mutex);
      auto seek_time = _bag_start_time + ros::Duration(time_from_start);
      _view->findMessages(seek_time,
                          [&](const std::string &topic,
                              const std::shared_ptr<const Message> &message) {
                            messages.emplace_back(topic, message);
                          });
    }
    for (auto &m : messages) {
      publish(m.first, m.second);
    }
    changed();
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
  LOG_DEBUG("rosbag join " << _path);
  _thread.join();
  LOG_DEBUG("rosbag closed " << _path);
}

std::vector<std::string> BagPlayer::listTopics(const std::string &type_name) {
  std::vector<std::string> ret;
  for (auto &p : _topic_type_name_list) {
    if (p.first == type_name) {
      ret.push_back(p.second);
    }
  }
  return ret;
}
