// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "timeseries.h"

TimeSeriesSubscriber::Impl::Impl() {
  _duration = 0;
  _thread = std::thread([this]() {
    while (true) {
      std::vector<std::shared_ptr<TimeSeriesListener>> listeners;
      double duration = 0.0;
      std::shared_ptr<BagPlayer> player;
      {
        std::unique_lock<std::mutex> lock(_mutex);
        while (true) {
          if (_stop_flag) {
            return;
          }
          if (_refresh) {
            _refresh = false;
            duration = _duration;
            player = _player;
            for (auto it = _listeners.begin(); it < _listeners.end();) {
              if (auto ptr = it->lock()) {
                listeners.push_back(ptr);
                ++it;
              } else {
                it = _listeners.erase(it);
              }
            }
            break;
          }
          _condition.wait(lock);
        }
      }
      if (player && _topic->isFromBag()) {
        double bag_time = player->time();
        player->readMessageSamples(
            _topic->name(), bag_time - duration, bag_time,
            [&](const std::shared_ptr<const Message> &message) {
              double playback_time = player->time();
              for (auto &listener : listeners) {
                listener->push(message,
                               player->startTime().toNSec() +
                                   (playback_time - duration) * 1000000000.0,
                               player->startTime().toNSec() +
                                   playback_time * 1000000000.0);
              }
              return !_stop_flag;
            });
        if (_stop_flag) {
          return;
        }
        for (auto &listener : listeners) {
          listener->commit();
        }
      }
      GlobalEvents::instance()->redraw();
    }
  });
}

TimeSeriesSubscriber::Impl::~Impl() {
  {
    std::unique_lock<std::mutex> lock(_mutex);
    _stop_flag = true;
    _condition.notify_all();
  }
  _thread.join();
}

void TimeSeriesSubscriber::Impl::handleMessage(
    const std::shared_ptr<const Message> &message) {
  PROFILER("TimeSeriesSubscriber");
  std::unique_lock<std::mutex> lock(_mutex);
  if (_player && _topic->isFromBag() && !_player->isPlaying()) {
    _refresh = true;
    _condition.notify_one();
  } else {
    if (0) {
      for (auto it = _listeners.begin(); it < _listeners.end();) {
        if (auto l = it->lock()) {
          l->push(message, message->time().toNSec() - _duration * 1000000000.0,
                  message->time().toNSec());
          l->commit();
          ++it;
        } else {
          it = _listeners.erase(it);
        }
      }
    }
    if (1) {
      std::vector<std::shared_ptr<TimeSeriesListener>> listeners;
      for (auto it = _listeners.begin(); it < _listeners.end();) {
        if (auto ptr = it->lock()) {
          listeners.push_back(ptr);
          ++it;
        } else {
          it = _listeners.erase(it);
        }
      }
      lock.unlock();
      for (auto l : listeners) {
        l->push(message, message->time().toNSec() - _duration * 1000000000.0,
                message->time().toNSec());
        l->commit();
      }
    }
  }
}

TimeSeriesSubscriber::TimeSeriesSubscriber(const std::string &topic) {
  _impl->_topic = Topic::instance(topic);
  auto *impl = _impl.get();
  {
    auto player = LockScope()->player;
    std::unique_lock<std::mutex> lock(impl->_mutex);
    impl->_player = player;
  }
  _subscriber = std::make_shared<Subscriber<Message>>(
      topic, _impl, [impl](const std::shared_ptr<const Message> &message) {
        impl->handleMessage(message);
      });
  LockScope()->modified.connect(_impl, [impl]() {
    auto player = LockScope()->player;
    std::unique_lock<std::mutex> lock(impl->_mutex);
    if (impl->_player != player) {
      if (player && impl->_topic->isFromBag()) {
        impl->_refresh = true;
      }
      impl->_player = player;
    }
  });
}

const std::string &TimeSeriesSubscriber::topic() const {
  return _impl->_topic->name();
}

const double TimeSeriesSubscriber::duration() const {
  std::unique_lock<std::mutex> lock(_impl->_mutex);
  return _impl->_duration;
}

void TimeSeriesSubscriber::duration(double duration) {
  std::unique_lock<std::mutex> lock(_impl->_mutex);
  _impl->_duration = duration;
}

void TimeSeriesSubscriber::addListener(
    const std::shared_ptr<TimeSeriesListener> &listener) {
  std::unique_lock<std::mutex> lock(_impl->_mutex);
  _impl->_listeners.emplace_back(listener);
}

const std::shared_ptr<Subscriber<Message>> &
TimeSeriesSubscriber::subscriber() const {
  return _subscriber;
}

bool TimeSeriesQuery::transform(const std::shared_ptr<const Message> &message,
                                std::pair<int64_t, double> &output) {
  PROFILER("TimeSeriesQuery");
  MessageParser parser(message);
  auto value_result = _query(parser);
  if (!value_result.isNull()) {
    output.second = value_result.toDouble();
    auto stamp_result = _stamp_query(parser);
    if (stamp_result.isTime()) {
      output.first = stamp_result.toInteger();
    } else {
      output.first = 0;
    }
    if (output.first == 0) {
      output.first = message->time().toNSec();
    }
    return true;
  } else {
    return false;
  }
}
