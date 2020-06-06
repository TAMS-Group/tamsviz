// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "serialization.h"

#include <yaml-cpp/yaml.h>

void toYAML(const Variant &v, YAML::Emitter &emitter) {
  auto type = v.type();
  if (type == typeid(std::string)) {
    emitter << v.value<std::string>();
  } else if (type == typeid(std::vector<Variant>)) {
    emitter << YAML::BeginSeq;
    for (auto &x : v.value<std::vector<Variant>>()) {
      toYAML(x, emitter);
    }
    emitter << YAML::EndSeq;
  } else if (type == typeid(std::map<std::string, Variant>)) {
    emitter << YAML::BeginMap;
    for (auto &x : v.value<std::map<std::string, Variant>>()) {
      emitter << YAML::Key << x.first << YAML::Value;
      toYAML(x.second, emitter);
    }
    emitter << YAML::EndMap;
  } else {
    throw std::runtime_error(std::string() + "unknown variant type " +
                             type.name());
  }
}

void toYAML(const Variant &v, std::ostream &stream) {
  YAML::Emitter emitter(stream);
  toYAML(v, emitter);
}
std::string toYAML(const Variant &v) {
  std::stringstream stream;
  toYAML(v, stream);
  return stream.str();
}

Variant parseYAML(const YAML::Node &yaml) {
  if (yaml.IsSequence()) {
    std::vector<Variant> ret;
    for (auto &y : yaml) {
      ret.push_back(parseYAML(y));
    }
    return Variant(ret);
  }
  if (yaml.IsMap()) {
    std::map<std::string, Variant> ret;
    for (auto &y : yaml) {
      ret[y.first.as<std::string>()] = parseYAML(y.second);
    }
    return Variant(ret);
  }
  return Variant(yaml.as<std::string>());
}

Variant parseYAML(const std::string &str) {
  YAML::Node yaml = YAML::Load(str);
  return parseYAML(yaml);
}
