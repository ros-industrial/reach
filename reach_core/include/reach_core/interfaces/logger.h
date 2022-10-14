#ifndef REACH_CORE_INTERFACES_LOGGER_H
#define REACH_CORE_INTERFACES_LOGGER_H

#include <atomic>
#include <boost/shared_ptr.hpp>
#include <string>

namespace YAML
{
class Node;
}

namespace reach
{
class StudyResults;

struct Logger
{
  using Ptr = boost::shared_ptr<Logger>;
  using ConstPtr = boost::shared_ptr<Logger>;

  virtual void setMaxProgress(unsigned long max_progress) = 0;
  virtual void printProgress(unsigned long progress) const = 0;

  virtual void printResults(const StudyResults& results) const = 0;

  virtual void print(const std::string& message) const = 0;
};

struct LoggerFactory
{
  using Ptr = boost::shared_ptr<LoggerFactory>;
  using ConstPtr = boost::shared_ptr<const LoggerFactory>;

  virtual Logger::ConstPtr create(const YAML::Node& config) const = 0;
};

}  // namespace reach

#endif  // REACH_CORE_INTERFACES_LOGGER_H
