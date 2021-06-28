// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "event.h"

#include <rosbag/bag.h>

class RosBagView;
class MessagePlaybackScope;
class Topic;
class Message;

class BagPlayer {
  std::shared_ptr<RosBagView> _view;
  std::shared_ptr<MessagePlaybackScope> _message_playback_scope;
  mutable std::mutex _view_mutex;
  double _duration = 0;
  std::string _path;
  std::string _file_name;
  ros::Time _bag_start_time;
  std::thread _thread;
  std::mutex _action_mutex;
  std::function<void()> _pending_action;
  std::function<double()> _time_function;
  std::condition_variable _action_condition;
  std::unordered_map<std::string, std::shared_ptr<Topic>> _topics;
  std::vector<std::string> _topic_names;
  std::vector<std::pair<std::string, std::string>> _topic_type_name_list;
  volatile bool _is_playing = false;
  bool _exit = false;
  void startAction(const std::function<void()> &action);
  inline bool interrupted_nolock() {
    return _pending_action != nullptr || _exit;
  }
  inline bool interrupted() {
    std::unique_lock<std::mutex> lock(_action_mutex);
    return interrupted_nolock();
  }
  // std::shared_ptr<const Message>
  // instantiate_nolock(const rosbag::MessageInstance &msg);
  // void publish(const rosbag::MessageInstance &msg);
  void publish(const std::string &topic,
               const std::shared_ptr<const Message> &message);

public:
  BagPlayer(const std::string &path);
  BagPlayer(const BagPlayer &) = delete;
  BagPlayer &operator=(const BagPlayer &) = delete;
  void stop();
  void play(const std::vector<double> &notification_times);
  void rewind();
  void seek(double time_from_start);
  double time();
  inline double duration() const { return _duration; }
  const std::string &path() const { return _path; }
  const std::string &fileName() const { return _file_name; }
  std::vector<std::string> listTopics(const std::string &type_name);
  bool findMessageTimeSpan(const std::string &topic, double time, double *start,
                           double *duration) const;
  const ros::Time &startTime() const { return _bag_start_time; }
  // std::vector<std::shared_ptr<const Message>>
  // readMessageSamples(const std::string &topic, double start, double stop);
  void readMessageSamples(
      const std::string &topic, double start, double stop,
      const std::function<bool(const std::shared_ptr<const Message> &)>
          &callback);
  bool isPlaying() const { return _is_playing; }
  ~BagPlayer();
  Event<void()> changed;
};
