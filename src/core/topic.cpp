// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "topic.h"

#include "workspace.h"

#include <ros/master.h>

std::shared_ptr<MessageType>
MessageType::instance(const std::string &hash, const std::string &name,
                      const std::string &definition) {
  static std::map<std::string, std::shared_ptr<MessageType>> _cache;
  static std::mutex _mutex;
  std::lock_guard<std::mutex> lock(_mutex);
  auto &instance = _cache[hash];
  if (!instance) {
    instance = std::shared_ptr<MessageType>(new MessageType());
    // LOG_DEBUG("message type " << hash << " " << name << "\n" << definition);
    instance->_hash = hash;
    instance->_name = name;
    instance->_definition = definition;
  }
  return instance;
}

std::vector<std::string>
TopicManager::listTopics(const std::string &type_name) {
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  std::vector<std::string> ret;
  for (auto &topic : topics) {
    if (topic.datatype == type_name) {
      ret.push_back(topic.name);
    }
  }
  return ret;
}

const std::shared_ptr<TopicManager> &TopicManager::instance() {
  static auto instance = std::make_shared<TopicManager>();
  return instance;
}

static std::unordered_map<std::string, std::weak_ptr<Topic>> _topic_map;
static std::mutex _topic_mutex;
static std::unordered_map<std::string, size_t> _topic_bag_counters;

void Topic::_subscribe(const std::shared_ptr<Topic> &topic) {
  static ros::NodeHandle _ros_node;
  std::weak_ptr<Topic> topic_weak = topic;
  {
    std::lock_guard<std::mutex> lock(topic->_message_mutex);
    topic->_message_instance = nullptr;
  }
  topic->connected();
  topic->_ros_subscriber = _ros_node.subscribe<Message>(
      topic->_topic_name, 100,
      boost::function<void(const Message &)>([topic_weak](const Message &msg) {
        if (auto topic = topic_weak.lock()) {
          {
            std::lock_guard<std::mutex> lock(_topic_mutex);
            if (_topic_bag_counters[topic->name()] != 0) {
              return;
            }
          }
          auto m = std::make_shared<Message>(msg);
          m->time(ros::Time::now());
          topic->publish(m);
        }
      }));
}
void Topic::_unsubscribe(const std::shared_ptr<Topic> &topic) {
  topic->_ros_subscriber = ros::Subscriber();
  {
    std::lock_guard<std::mutex> lock(topic->_message_mutex);
    topic->_message_instance = nullptr;
  }
  topic->connected();
}

MessagePlaybackScope::MessagePlaybackScope(
    const std::vector<std::string> &topics)
    : _topics(topics) {
  std::lock_guard<std::mutex> lock(_topic_mutex);
  for (auto &topic_name : _topics) {
    _topic_bag_counters[topic_name]++;
    if (_topic_bag_counters[topic_name] == 1) {
      if (auto topic_instance = _topic_map[topic_name].lock()) {
        Topic::_unsubscribe(topic_instance);
      }
    }
  }
}
MessagePlaybackScope::~MessagePlaybackScope() {
  std::lock_guard<std::mutex> lock(_topic_mutex);
  for (auto &topic_name : _topics) {
    _topic_bag_counters[topic_name]--;
    if (_topic_bag_counters[topic_name] == 0) {
      if (auto topic_instance = _topic_map[topic_name].lock()) {
        Topic::_subscribe(topic_instance);
      }
    }
  }
}

Topic::Topic(const std::string &name) : _topic_name(name) {}
Topic::~Topic() {}
std::shared_ptr<Topic> Topic::instance(const std::string &name) {
  std::lock_guard<std::mutex> lock(_topic_mutex);
  auto topic = _topic_map[name].lock();
  if (!topic) {
    topic = std::shared_ptr<Topic>(new Topic(name));
    {
      if (_topic_bag_counters[topic->name()] == 0) {
        _subscribe(topic);
      }
    }
    _topic_map[name] = topic;
  }
  return topic;
}

void Topic::publish(const std::shared_ptr<Message> &message) {
  {
    std::unique_lock<std::mutex> lock(_message_mutex);
    _message_instance = message;
  }
  received(message);
  TopicManager::instance()->received();
}
