// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "mparser.h"

#include <mutex>
#include <regex>

std::shared_ptr<MessageParser::Node>
MessageParser::findType(const std::string &name, const std::string &ns) {
  {
    auto it = _data->type_map.find(name);
    if (it != _data->type_map.end()) {
      return it->second;
    }
  }
  if (name == "Header") {
    return findType("std_msgs/Header");
  }
  if (!ns.empty()) {
    return findType(ns + "/" + name);
  }
  throw std::runtime_error("Unknown message type: " + name);
}

void MessageParser::parseDefinition(
    const std::shared_ptr<const MessageType> &type) {

  PROFILER("MessageParser::parseDefinition");

  if (_node && _node->hash == type->hash()) {
    return;
  }

  static std::mutex cache_mutex;
  static std::unordered_map<std::string, std::shared_ptr<Node>> cache_map;

  {
    std::lock_guard<std::mutex> lock(cache_mutex);
    auto it = cache_map.find(type->hash());
    if (it != cache_map.end()) {
      _node = it->second;
      return;
    }
  }

  LOG_INFO("parsing message description " << type->name() << " "
                                          << type->hash());

  std::vector<std::function<void()>> linker_jobs;

  registerPrimitiveType<uint8_t>("bool");
  registerPrimitiveType<int8_t>("byte");
  registerPrimitiveType<uint8_t>("char");
  registerPrimitiveType<int8_t>("int8");
  registerPrimitiveType<uint8_t>("uint8");
  registerPrimitiveType<int16_t>("int16");
  registerPrimitiveType<uint16_t>("uint16");
  registerPrimitiveType<int32_t>("int32");
  registerPrimitiveType<uint32_t>("uint32");
  registerPrimitiveType<int64_t>("int64");
  registerPrimitiveType<uint64_t>("uint64");
  registerPrimitiveType<float>("float32");
  registerPrimitiveType<double>("float64");
  registerType<TimeNode>("time");
  registerType<DurationNode>("duration");
  registerType<StringNode>("string");

  std::string multi_definition =
      "MSG: " + type->name() + "\n" + type->definition();

  static std::regex message_seperator("\\n\\s*========+\\s*",
                                      std::regex_constants::optimize);
  std::vector<std::string> message_definitions(
      std::sregex_token_iterator(multi_definition.begin(),
                                 multi_definition.end(), message_seperator, -1),
      std::sregex_token_iterator());

  std::shared_ptr<MessageNode> root_node;

  for (auto &message_type_and_definition : message_definitions) {

    static std::regex message_type_and_definition_regex(
        "\\s*MSG:\\s*(\\S+)\\s*\\n([\\s\\S]*)");
    std::smatch message_type_and_definition_match;
    if (!std::regex_match(message_type_and_definition,
                          message_type_and_definition_match,
                          message_type_and_definition_regex)) {
      throw std::runtime_error("Message definition format error");
    }
    std::string message_type_name = message_type_and_definition_match[1];
    std::string message_definition = message_type_and_definition_match[2];

    // LOG_DEBUG(message_type_name);

    auto message_node = std::make_shared<MessageNode>();
    registerType(message_type_name, message_node);

    if (!root_node) {
      root_node = message_node;
    }

    std::string message_namespace;
    {
      auto end = message_node->name.find('/');
      if (end != std::string::npos) {
        message_namespace = message_node->name.substr(0, end);
      }
    }

    std::string line;
    std::stringstream message_definition_stream(message_definition);
    while (std::getline(message_definition_stream, line)) {

      for (size_t i = 0; i < line.size(); i++) {
        if (line[i] == '#') {
          line.resize(i);
          break;
        }
      }

      std::smatch match;

      {
        static std::regex regex("\\s*", std::regex_constants::optimize);
        if (std::regex_match(line, match, regex)) {
          continue;
        }
      }

      {
        static std::regex regex("\\s*(\\S+)\\s+([^\\s=]+)\\s*\\=\\s*(.*)",
                                std::regex_constants::optimize);
        if (std::regex_match(line, match, regex)) {

          auto const_node = std::make_shared<ConstNode>();

          const_node->type_name = match[1];
          const_node->name = match[2];
          const_node->value_string = match[3];

          message_node->const_map[const_node->name] =
              message_node->const_nodes.size();
          message_node->const_nodes.push_back(const_node);

          continue;
        }
      }

      {
        static std::regex regex(
            "\\s*([^\\s\\[]+)\\s*(\\[([^\\]]*)\\])?\\s+(\\S+)\\s*",
            std::regex_constants::optimize);
        if (std::regex_match(line, match, regex)) {

          std::string field_name = match[4];
          std::string type_name = match[1];
          size_t field_index = message_node->field_nodes.size();

          message_node->field_map[field_name] = field_index;
          message_node->field_names.push_back(field_name);

          if (match[2].str().empty()) {
            linker_jobs.emplace_back([this, message_node, field_index,
                                      type_name, message_namespace]() {
              message_node->field_nodes[field_index] =
                  findType(type_name, message_namespace);
            });
            message_node->field_nodes.push_back(nullptr);
          } else {
            auto array_node = std::make_shared<ArrayNode>();
            if (!match[3].str().empty()) {
              array_node->has_fixed_element_count = true;
              array_node->fixed_element_count = std::stoull(match[3]);
            }
            linker_jobs.emplace_back([this, type_name, array_node,
                                      message_namespace]() {
              array_node->element_node = findType(type_name, message_namespace);
            });
            message_node->field_nodes.push_back(array_node);
          }

          continue;
        }
      }

      throw std::runtime_error("Malformed message definition line: " + line);
    }
  }

  for (auto &job : linker_jobs) {
    job();
  }

  root_node->init(*this);
  root_node->hash = type->hash();

  _node = root_node;

  {
    std::lock_guard<std::mutex> lock(cache_mutex);
    cache_map[type->hash()] = root_node;
  }
}

void MessageParser::parse(const std::shared_ptr<const Message> &message) {
  if (_data == nullptr || _data.use_count() > 1) {
    // LOG_DEBUG("new data");
    _data = std::make_shared<Data>();
    _data->index = std::make_shared<Index>();
  }
  _data->message = message;
  parseDefinition(message->type());
  _data_start = 0;
  _index_start = 0;
  {
    size_t p = 0;
    _data->index->clear();
    PROFILER("index");
    _node->index(*this, p);
    // LOG_DEBUG("s " << message->size() << " " << p);
  }
}

static void printIndent(std::ostream &stream, size_t indent) {
  for (size_t i = 0; i < indent; i++) {
    stream << "  ";
  }
}

static void printImpl(const MessageParser &message, std::ostream &stream,
                      size_t indent) {
  if (message.isMessage()) {
    for (size_t i = 0; i < message.size(); i++) {
      printIndent(stream, indent);
      stream << message.fieldName(i) << ": ";
      stream << "\n";
      printImpl(message[i], stream, indent + 1);
    }
    return;
  }
  if (message.isArray()) {
    for (size_t i = 0; i < message.size(); i++) {
      printIndent(stream, indent);
      stream << "-\n";
      printImpl(message[i], stream, indent + 1);
    }
    return;
  }
  printIndent(stream, indent);
  stream << message.toString() << "\n";
}

void MessageParser::print(std::ostream &stream) const {
  printImpl(*this, stream, 0);
}

std::string MessageParser::print() const {
  std::stringstream stream;
  print(stream);
  return stream.str();
}
