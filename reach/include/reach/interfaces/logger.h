#ifndef reach_INTERFACES_LOGGER_H
#define reach_INTERFACES_LOGGER_H

#include <atomic>
#include <memory>
#include <string>

namespace YAML
{
class Node;
}

namespace reach
{
class ReachResultSummary;

/** @brief Interface for logging reach study status and progress */
struct Logger
{
  using Ptr = std::shared_ptr<Logger>;
  using ConstPtr = std::shared_ptr<const Logger>;

  virtual void setMaxProgress(unsigned long max_progress) = 0;
  virtual void printProgress(unsigned long progress) const = 0;

  virtual void printResults(const ReachResultSummary& results) const = 0;

  virtual void print(const std::string& message) const = 0;
};

/** @brief Plugin interface for generating logger interfaces */
struct LoggerFactory
{
  using Ptr = std::shared_ptr<LoggerFactory>;
  using ConstPtr = std::shared_ptr<const LoggerFactory>;

  virtual Logger::Ptr create(const YAML::Node& config) const = 0;

  static std::string getSection()
  {
    return LOGGER_SECTION;
  }
};

}  // namespace reach

#endif  // reach_INTERFACES_LOGGER_H
