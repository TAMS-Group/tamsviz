// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "event.h"
#include "log.h"
#include "message.h"
#include "property.h"
#include "watcher.h"

#include <ros/ros.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

class TopicManager {
public:
  std::vector<std::string> listTopics(const std::string &type_name);
  std::vector<std::string> listTopics();
  Event<void()> received{"received"};
  static const std::shared_ptr<TopicManager> &instance();
};

class TopicRegistry;

class Topic {
  std::string _topic_name;
  ros::Subscriber _ros_subscriber;
  std::mutex _message_mutex;
  std::shared_ptr<const Message> _message_instance;
  std::atomic<size_t> _connections;
  std::shared_ptr<TopicRegistry> _registry;
  Topic(const std::string &name);
  Topic(const Topic &) = delete;
  Topic &operator=(const Topic &) = delete;
  static void _subscribe(const std::shared_ptr<Topic> &topic);
  static void _unsubscribe(const std::shared_ptr<Topic> &topic);

public:
  ~Topic();
  void countConnection(size_t d) { _connections += d; }
  void publish(const std::shared_ptr<const Message> &message);
  inline const std::string &name() const { return _topic_name; }
  std::shared_ptr<const Message> message() {
    std::unique_lock<std::mutex> lock(_message_mutex);
    return _message_instance;
  }
  static std::shared_ptr<Topic> instance(const std::string &name);
  Event<void(const std::shared_ptr<const Message> &)> received;
  Event<void()> connected;
  bool isFromBag() const;
  // static bool isFromBag(const std::string &topic);
  friend class MessagePlaybackScope;
};

class MessagePlaybackScope {
  std::vector<std::string> _topics;
  std::shared_ptr<TopicRegistry> _registry;

public:
  MessagePlaybackScope(const std::vector<std::string> &topics);
  ~MessagePlaybackScope();
  MessagePlaybackScope(const MessagePlaybackScope &) = delete;
  MessagePlaybackScope &operator=(const MessagePlaybackScope &) = delete;
};

template <class Message> class Publisher {
  std::string _topic;
  std::unique_ptr<ros::Publisher> _publisher;

public:
  Publisher() {}
  Publisher(const std::string &topic) { this->topic(topic); }
  Publisher(const Publisher &) = delete;
  Publisher &operator=(const Publisher &) = delete;
  const std::string &topic() const { return _topic; }
  void topic(const std::string &topic) {
    if (topic != _topic) {
      _topic = topic;
      if (_topic.empty()) {
        _publisher.reset();
      } else {
        static ros::NodeHandle ros_node;
        _publisher.reset(
            new ros::Publisher(ros_node.advertise<Message>(topic, 1000)));
      }
      _topic = topic;
    }
  }
  bool connected() const {
    return _publisher && _publisher->getNumSubscribers() > 0;
  }
  void publish(const Message &message) {
    if (connected()) {
      _publisher->publish(message);
    }
  }
  void publish(const std::shared_ptr<const Message> &message) {
    if (connected()) {
      _publisher->publish(*message);
    }
  }
  void publish(const std::string &topic, const Message &message) {
    this->topic(topic);
    if (connected()) {
      _publisher->publish(message);
    }
  }
  void publish(const std::string &topic,
               const std::shared_ptr<const Message> &message) {
    this->topic(topic);
    if (connected()) {
      _publisher->publish(*message);
    }
  }
};

template <class M> class Subscriber {
  struct Callback {
    std::weak_ptr<const void> receiver;
  };
  bool _visible = true;
  std::shared_ptr<Callback> _callback;
  std::shared_ptr<Topic> _topic;
  std::shared_ptr<const M> _message_instance;
  Watcher _watcher;
  Subscriber() {}
  class MessageCast {
    std::shared_ptr<const Message> _message;

  public:
    inline MessageCast(const std::shared_ptr<const Message> &message)
        : _message(message) {}
    inline operator std::shared_ptr<const Message>() { return _message; }
    template <class T> inline operator std::shared_ptr<T>() {
      return _message ? _message->instantiate<T>() : nullptr;
    }
  };
  template <class F>
  static void invoke(const F &callback,
                     const std::shared_ptr<const Message> &message) {
    if (!message) {
      return;
    }
    callback(MessageCast(message));
  }

public:
  Subscriber(const std::string &name, bool visible = true) {
    _topic = Topic::instance(name);
    _visible = visible;
    if (_visible) {
      _topic->countConnection(+1);
    }
  }
  template <class F>
  Subscriber(const std::string &name,
             const std::shared_ptr<const void> &receiver, const F &function,
             bool visible = true) {
    _topic = Topic::instance(name);
    _callback = std::make_shared<Callback>();
    _callback->receiver = receiver;
    auto *callback_ptr = _callback.get();
    _topic->received.connect(
        _callback, [callback_ptr,
                    function](const std::shared_ptr<const Message> &message) {
          if (auto ptr = callback_ptr->receiver.lock()) {
            invoke(function, message);
          }
        });
    invoke(function, _topic->message());
    _visible = visible;
    if (_visible) {
      _topic->countConnection(+1);
    }
  }
  ~Subscriber() {
    if (_visible) {
      _topic->countConnection(-1);
    }
  }
  Subscriber(const Subscriber &) = delete;
  Subscriber &operator=(const Subscriber &) = delete;
  const std::shared_ptr<Topic> &topic() const { return _topic; }
  std::shared_ptr<const M> message() {
    auto msg = _topic->message();
    if (_watcher.changed(msg)) {
      if (msg) {
        _message_instance = msg->instantiate<M>();
      } else {
        _message_instance = nullptr;
      }
      LOG_DEBUG("message instance " << msg.get() << " "
                                    << _message_instance.get());
    }
    return _message_instance;
  }
};

class NoMessageScope {
  std::shared_ptr<void> _instance;

public:
  NoMessageScope();
  ~NoMessageScope();
  NoMessageScope(const NoMessageScope &) = delete;
  NoMessageScope &operator=(const NoMessageScope &) = delete;
};

template <class Message> class TopicProperty {
private:
  std::string _topic;
  bool _used = false;
  std::shared_ptr<void> _callback_object;
  std::function<void(const std::shared_ptr<const Message> &)>
      _callback_function;
  std::shared_ptr<Subscriber<Message>> _subscriber;
  void _sync() {
    if ((_topic.empty() || !_used) && _subscriber) {
      _subscriber = nullptr;
    }
    if (!_topic.empty() && _used) {
      if (!_subscriber || _subscriber->topic()->name() != _topic) {
        if (_callback_object && _callback_function) {
          _subscriber = std::make_shared<Subscriber<Message>>(
              _topic, _callback_object, _callback_function);
        } else {
          _subscriber = std::make_shared<Subscriber<Message>>(_topic);
        }
      }
    }
  }

public:
  TopicProperty() {}
  TopicProperty(const std::string &topic) : _topic(topic) {}
  TopicProperty(const TopicProperty &other) {
    _topic = other._topic;
    _sync();
  }
  TopicProperty &operator=(const TopicProperty &other) {
    _topic = other._topic;
    _sync();
    return *this;
  }
  void connect(const std::shared_ptr<void> &callback_object,
               const std::function<void(const std::shared_ptr<const Message> &)>
                   &callback_function) {
    _used = true;
    _callback_object = callback_object;
    _callback_function = callback_function;
    _sync();
  }
  void connect() {
    _used = true;
    _sync();
  }
  const std::string &topic() const { return _topic; }
  void topic(const std::string &topic) {
    if (topic != _topic) {
      _topic = topic;
      _sync();
    }
  }
  std::shared_ptr<const Message> message() {
    _used = true;
    _sync();
    if (_subscriber) {
      return _subscriber->message();
    }
    return nullptr;
  }
  std::shared_ptr<Subscriber<Message>> subscriber() {
    _used = true;
    _sync();
    return _subscriber;
  }
};
template <class T>
inline void toString(const TopicProperty<T> &v, std::string &s) {
  s = v.topic();
}
template <class T>
inline void fromString(TopicProperty<T> &v, const std::string &s) {
  v.topic(s);
}
template <class T>
bool operator==(const TopicProperty<T> &a, const TopicProperty<T> &b) {
  return a.topic() == b.topic();
}
template <class T>
bool operator!=(const TopicProperty<T> &a, const TopicProperty<T> &b) {
  return a.topic() != b.topic();
}
template <class T> struct DefaultPropertyAttributes<TopicProperty<T>> {
  static inline void initialize(PropertyAttributes *attributes) {
    attributes->list = [](const Property &property) {
      // return std::vector<std::string>({"test", "bla"});
      return TopicManager::instance()->listTopics(
          ros::message_traits::DataType<T>::value());
    };
  }
};
