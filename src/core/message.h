// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "log.h"

#include <ros/ros.h>

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
  std::shared_ptr<const MessageType> _type;
  std::vector<uint8_t> _data;
  ros::Time _time;
  template <class T> std::shared_ptr<const T> instantiateImpl() const {
    if (_data.empty() || _type == nullptr ||
        ros::message_traits::datatype<T>() != _type->name() ||
        ros::message_traits::md5sum<T>() != _type->hash()) {
      return nullptr;
    } else {
      auto m = std::make_shared<T>();
      ros::serialization::IStream s((uint8_t *)_data.data(), _data.size());
      ros::serialization::deserialize(s, *m);
      return std::move(m);
    }
  }

public:
  template <class T> std::shared_ptr<const T> instantiate() const {
    return std::move(instantiateImpl<typename std::decay<T>::type>());
  }
  template <class Stream> void read(Stream &stream) {
    _data.resize(stream.getLength());
    std::memcpy(_data.data(), stream.getData(), _data.size());
  }
  void type(const std::shared_ptr<const MessageType> &type) { _type = type; }
  const std::shared_ptr<const MessageType> &type() const { return _type; }
  const ros::Time &time() const { return _time; }
  void time(const ros::Time &time) { _time = time; }
  const uint8_t *data() const { return _data.data(); }
  size_t size() const { return _data.size(); }
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
