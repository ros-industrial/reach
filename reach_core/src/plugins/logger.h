#ifndef REACH_CORE_PLUGINS_LOGGER_H
#define REACH_CORE_PLUGINS_LOGGER_H

#include <reach_core/interfaces/logger.h>
#include <mutex>

namespace boost
{
class progress_display;
}

namespace reach
{
/**
 * @brief Thread-safe logger that prints messages to the console via stdout
 */
class ConsoleLogger : public Logger
{
public:
  void setMaxProgress(unsigned long progress) override;
  void printProgress(unsigned long progress) const override;
  void printResults(const StudyResults& results) const override;
  void print(const std::string& message) const override;

protected:
  unsigned long max_progress_{ 0 };
  mutable std::mutex mutex_;
};

struct ConsoleLoggerFactory : public LoggerFactory
{
  Logger::ConstPtr create(const YAML::Node& config) const override;
};

/**
 * @brief Thread-safe logger that prints messages to the console via stdout, with boost progress bar progress logging
 */
class BoostProgressConsoleLogger : public Logger
{
public:
  BoostProgressConsoleLogger();

  void setMaxProgress(unsigned long max_progress) override;
  void printProgress(unsigned long progress) const override;
  void printResults(const StudyResults& results) const override;
  void print(const std::string& message) const override;

protected:
  mutable boost::shared_ptr<boost::progress_display> display_;
  mutable std::mutex mutex_;
};

struct BoostProgressConsoleLoggerFactory : public LoggerFactory
{
  Logger::ConstPtr create(const YAML::Node& config) const override;
};

}  // namespace reach

#endif  // REACH_CORE_PLUGINS_LOGGER_H
