// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "event.h"
#include "log.h"
#include "type.h"

#include <ros/ros.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

class TopicManager {
public:
  std::vector<std::string> listTopics(const std::string &type_name);
  Event<void()> received{"received"};
  static const std::shared_ptr<TopicManager> &instance();
};

class MessageType {
  std::string _hash, _name, _definition;

private:
  MessageType() {}
  MessageType(const MessageType &) = delete;
  MessageType &operator=(const MessageType &) = delete;

public:
  static std::shared_ptr<MessageType> instance(const std::string &hash,
                                               const std::string &name,
                                               const std::string &definition);
  const std::string &hash() const { return _hash; }
  const std::string &name() const { return _name; }
  const std::string &definition() const { return _definition; }
};

class Message {
  std::shared_ptr<MessageType> _type;
  std::vector<uint8_t> _data;
  ros::Time _time;

public:
  template <class T> std::shared_ptr<T> instantiate() const {
    if (_data.empty() || _type == nullptr ||
        ros::message_traits::datatype<T>() != _type->name() ||
        ros::message_traits::md5sum<T>() != _type->hash()) {
      return nullptr;
    } else {
      auto m = std::make_shared<T>();
      ros::serialization::IStream s((uint8_t *)_data.data(), _data.size());
      ros::serialization::deserialize(s, *m);
      return m;
    }
  }
  template <class Stream> void read(Stream &stream) {
    _data.resize(stream.getLength());
    std::memcpy(_data.data(), stream.getData(), _data.size());
  }
  void type(const std::shared_ptr<MessageType> &type) { _type = type; }
  const ros::Time &time() const { return _time; }
  void time(const ros::Time &time) { _time = time; }
};

namespace ros {
namespace message_traits {
template <> struct MD5Sum<Message> {
  static const char *value() { return "*"; }
};
template <> struct DataType<Message> {
  static const char *value() { return "*"; }
};
} // namespace message_traits
namespace serialization {
template <> struct Serializer<Message> {
  template <typename Stream>
  inline static void read(Stream &stream, Message &m) {
    m.read(stream);
  }
};
template <> struct PreDeserialize<Message> {
  static void notify(const PreDeserializeParams<Message> &params) {
    params.message->type(MessageType::instance(
        (*params.connection_header)["md5sum"],
        (*params.connection_header)["type"],
        (*params.connection_header)["message_definition"]));
  }
};
} // namespace serialization
} // namespace ros

class Topic {
  std::string _topic_name;
  ros::Subscriber _ros_subscriber;
  std::mutex _message_mutex;
  std::shared_ptr<Message> _message_instance;
  Topic(const std::string &name);
  Topic(const Topic &) = delete;
  Topic &operator=(const Topic &) = delete;
  static void _subscribe(const std::shared_ptr<Topic> &topic);
  static void _unsubscribe(const std::shared_ptr<Topic> &topic);

public:
  ~Topic();
  void publish(const std::shared_ptr<Message> &message);
  inline const std::string &name() const { return _topic_name; }
  std::shared_ptr<Message> message() {
    std::unique_lock<std::mutex> lock(_message_mutex);
    return _message_instance;
  }
  static std::shared_ptr<Topic> instance(const std::string &name);
  Event<void(const std::shared_ptr<Message> &)> received;
  Event<void()> connected;
  friend class MessagePlaybackScope;
};

class MessagePlaybackScope {
  const std::vector<std::string> _topics;

public:
  MessagePlaybackScope(const std::vector<std::string> &topics);
  ~MessagePlaybackScope();
  MessagePlaybackScope(const MessagePlaybackScope &) = delete;
  MessagePlaybackScope &operator=(const MessagePlaybackScope &) = delete;
};

template <class M> class Subscriber {
  struct Callback {
    std::weak_ptr<void> receiver;
  };
  std::shared_ptr<Callback> _callback;
  std::shared_ptr<Topic> _topic;
  Subscriber() {}
  Subscriber(const Subscriber &) = delete;
  Subscriber &operator=(const Subscriber &) = delete;
  class MessageCast {
    std::shared_ptr<Message> _message;

  public:
    inline MessageCast(const std::shared_ptr<Message> &message)
        : _message(message) {}
    inline operator std::shared_ptr<Message>() { return _message; }
    template <class T> inline operator std::shared_ptr<T>() {
      return _message ? _message->instantiate<T>() : nullptr;
    }
  };
  template <class F>
  static void invoke(const F &callback,
                     const std::shared_ptr<Message> &message) {
    if (!message) {
      return;
    }
    callback(MessageCast(message));
  }

public:
  Subscriber(const std::string &name) { _topic = Topic::instance(name); }
  template <class T, class F>
  Subscriber(const std::string &name, const std::shared_ptr<T> &receiver,
             const F &function) {
    _topic = Topic::instance(name);
    _callback = std::make_shared<Callback>();
    _callback->receiver = receiver;
    auto *callback_ptr = _callback.get();
    _topic->received.connect(
        _callback,
        [callback_ptr, function](const std::shared_ptr<Message> &message) {
          if (auto ptr = callback_ptr->receiver.lock()) {
            invoke(function, message);
          }
        });
    invoke(function, _topic->message());
  }
  const std::shared_ptr<Topic> &topic() const { return _topic; }
  std::shared_ptr<M> message() const {
    if (auto msg = _topic->message()) {
      if (auto m = msg->instantiate<M>()) {
        return m;
      }
    }
    return nullptr;
  }
};

template <class Message> class TopicProperty {
private:
  std::string _topic;
  bool _used = false;
  std::shared_ptr<void> _callback_object;
  std::function<void(const std::shared_ptr<Message> &)> _callback_function;
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
               const std::function<void(const std::shared_ptr<Message> &)>
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
  std::shared_ptr<Message> message() {
    _used = true;
    _sync();
    if (_subscriber) {
      return _subscriber->message();
    }
    return nullptr;
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
