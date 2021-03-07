// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "topic.h"

#include "workspace.h"

#include <ros/master.h>

#include <boost/thread.hpp>

struct TopicRegistry {

  std::unordered_map<std::string, std::weak_ptr<Topic>> topic_map;
  std::mutex topic_map_mutex;

  std::unordered_map<std::string, size_t> topic_bag_counters;
  std::mutex topic_bag_counter_mutex;

  static std::shared_ptr<TopicRegistry> instance() {
    static auto instance = std::make_shared<TopicRegistry>();
    return instance;
  }
};

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
  std::set<std::string> topic_set;
  {
    ros::master::V_TopicInfo ros_topics;
    ros::master::getTopics(ros_topics);
    for (auto &topic : ros_topics) {
      if (type_name.empty() || type_name == "*" ||
          topic.datatype == type_name) {
        topic_set.insert(topic.name);
      }
    }
  }
  {
    auto registry = TopicRegistry::instance();
    std::lock_guard<std::mutex> lock(registry->topic_map_mutex);
    for (auto &pair : registry->topic_map) {
      if (auto topic = pair.second.lock()) {
        auto message = topic->message();
        if (type_name.empty() || type_name == "*" ||
            message && message->type()->name() == type_name) {
          topic_set.insert(pair.first);
        }
      }
    }
  }
  std::vector<std::string> ret;
  for (auto &t : topic_set) {
    ret.push_back(t);
  }
  return ret;
}

std::vector<std::string> TopicManager::listTopics() {
  return listTopics(std::string());
}

const std::shared_ptr<TopicManager> &TopicManager::instance() {
  static auto instance = std::make_shared<TopicManager>();
  return instance;
}

void Topic::_subscribe(const std::shared_ptr<Topic> &topic) {
  static ros::NodeHandle _ros_node;
  std::weak_ptr<Topic> topic_weak = topic;
  {
    std::lock_guard<std::mutex> lock(topic->_message_mutex);
    topic->_message_instance = nullptr;
  }
  topic->connected();
  topic->_ros_subscriber = _ros_node.subscribe<Message>(
      topic->_topic_name, 5,
      boost::function<void(const Message &)>([topic_weak](const Message &msg) {
        if (auto topic = topic_weak.lock()) {
          {
            std::lock_guard<std::mutex> lock(
                topic->_registry->topic_bag_counter_mutex);
            if (topic->_registry->topic_bag_counters[topic->name()] != 0) {
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
  topic->_ros_subscriber.shutdown();
  topic->_ros_subscriber = ros::Subscriber();
  {
    std::lock_guard<std::mutex> lock(topic->_message_mutex);
    topic->_message_instance = nullptr;
  }
  topic->connected();
}

MessagePlaybackScope::MessagePlaybackScope(
    const std::vector<std::string> &topics)
    : _topics(topics), _registry(TopicRegistry::instance()) {
  std::lock_guard<std::mutex> lock(_registry->topic_map_mutex);
  for (auto &topic_name : _topics) {
    bool unsubscribe = false;
    {
      std::lock_guard<std::mutex> lock(_registry->topic_bag_counter_mutex);
      _registry->topic_bag_counters[topic_name]++;
      if (_registry->topic_bag_counters[topic_name] == 1) {
        unsubscribe = true;
      }
    }
    if (unsubscribe) {
      if (auto topic_instance = _registry->topic_map[topic_name].lock()) {
        Topic::_unsubscribe(topic_instance);
      }
    }
  }
}

MessagePlaybackScope::~MessagePlaybackScope() {
  std::lock_guard<std::mutex> lock(_registry->topic_map_mutex);
  for (auto &topic_name : _topics) {
    bool subscribe = false;
    {
      std::lock_guard<std::mutex> lock(_registry->topic_bag_counter_mutex);
      _registry->topic_bag_counters[topic_name]--;
      if (_registry->topic_bag_counters[topic_name] == 0) {
        subscribe = true;
      }
    }
    if (subscribe) {
      if (auto topic_instance = _registry->topic_map[topic_name].lock()) {
        Topic::_subscribe(topic_instance);
      }
    }
  }
}

Topic::Topic(const std::string &name)
    : _topic_name(name), _registry(TopicRegistry::instance()) {
  _connections = 0;
}
Topic::~Topic() {}
std::shared_ptr<Topic> Topic::instance(const std::string &name) {
  auto registry = TopicRegistry::instance();
  std::lock_guard<std::mutex> lock(registry->topic_map_mutex);
  auto topic = registry->topic_map[name].lock();
  if (!topic) {
    topic = std::shared_ptr<Topic>(new Topic(name));
    {
      bool sub = false;
      {
        std::lock_guard<std::mutex> lock(registry->topic_bag_counter_mutex);
        sub = (registry->topic_bag_counters[topic->name()] == 0);
      }
      if (sub) {
        _subscribe(topic);
      }
    }
    registry->topic_map[name] = topic;
  }
  return topic;
}

bool Topic::isFromBag() const {
  std::lock_guard<std::mutex> lock(_registry->topic_bag_counter_mutex);
  return _registry->topic_bag_counters[_topic_name] > 0;
}

/*
bool Topic::isFromBag(const std::string &topic) {
  auto registry = TopicRegistry::instance();
  std::lock_guard<std::mutex> lock(registry->topic_bag_counter_mutex);
  return registry->topic_bag_counters[topic] > 0;
}
*/

static std::shared_ptr<boost::shared_mutex> topicMessageMutex() {
  static auto instance = std::make_shared<boost::shared_mutex>();
  return instance;
}
void Topic::publish(const std::shared_ptr<const Message> &message) {
  auto mutex = topicMessageMutex();
  boost::shared_lock<boost::shared_mutex> lock(*mutex);
  {
    std::unique_lock<std::mutex> lock(_message_mutex);
    _message_instance = message;
  }
  received(message);
  if (_connections > 0) {
    TopicManager::instance()->received();
  }
}
NoMessageScope::NoMessageScope() {
  _instance = topicMessageMutex();
  topicMessageMutex()->lock();
}
NoMessageScope::~NoMessageScope() { topicMessageMutex()->unlock(); }
