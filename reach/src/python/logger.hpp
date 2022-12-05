#pragma once

#include <reach/interfaces/logger.h>
#include "utils.hpp"

namespace reach
{
Logger::Ptr LoggerFactory::create(const boost::python::dict& pyyaml_config) const
{
  return create(toYAML(pyyaml_config));
}

struct LoggerPython : Logger, boost::python::wrapper<Logger>
{
  void setMaxProgress(unsigned long max_progress) override
  {
    return call_and_handle([this, &max_progress]() { this->get_override("setMaxProgress")(max_progress); });
  }

  void printProgress(unsigned long progress) const override
  {
    return call_and_handle([this, &progress]() { this->get_override("printProgress")(progress); });
  }

  void printResults(const ReachResultSummary& results) const override
  {
    return call_and_handle([this, &results]() { this->get_override("printResults")(results); });
  }

  void print(const std::string& msg) const override
  {
    return call_and_handle([this, &msg]() { this->get_override("print")(msg); });
  }
};

struct LoggerFactoryPython : LoggerFactory, boost::python::wrapper<LoggerFactory>
{
  Logger::Ptr create(const YAML::Node& config) const override
  {
    return call_and_handle([this, &config]() -> Logger::Ptr { return this->get_override("create")(config); });
  }
};

}  // namespace reach
