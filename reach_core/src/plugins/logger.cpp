#include <reach_core/interfaces/logger.h>
#include <reach_core/reach_database.h>

#include <boost/progress.hpp>
#include <iostream>
#include <mutex>

namespace reach
{
/**
 * @brief Thread-safe logger that prints messages to the console via stdout
 */
class ConsoleLogger : public Logger
{
public:
  void setMaxProgress(unsigned long progress) override
  {
    std::lock_guard<std::mutex> lock{ mutex_ };
    max_progress_ = progress;
  }

  void printProgress(unsigned long progress) const override
  {
    static std::atomic<double> previous_pct{ 0.0 };
    const double current_pct_float = (static_cast<double>(progress) / static_cast<double>(max_progress_)) * 100.0;
    const int current_pct = static_cast<int>(current_pct_float);
    if (current_pct > previous_pct)
    {
      std::lock_guard<std::mutex> lock{ mutex_ };
      std::cout << "[" << current_pct << "%]" << std::endl;
    }
    previous_pct = current_pct;
  }

  void printResults(const StudyResults& results) const override
  {
    print(results.print());
  }

  void print(const std::string& message) const override
  {
    std::lock_guard<std::mutex> lock{ mutex_ };
    std::cout << message << std::endl;
  }

protected:
  mutable std::mutex mutex_;
  unsigned long max_progress_{ 0 };
};

struct ConsoleLoggerFactory : public LoggerFactory
{
  Logger::ConstPtr create(const YAML::Node& /*config*/) const override
  {
    return std::make_shared<ConsoleLogger>();
  }
};

/**
 * @brief Thread-safe logger that prints messages to the console via stdout, with boost progress bar progress logging
 */
class BoostProgressConsoleLogger : public Logger
{
public:
  BoostProgressConsoleLogger() : display_(nullptr)
  {
  }

  void setMaxProgress(unsigned long max_progress) override
  {
    std::lock_guard<std::mutex> lock{ mutex_ };
    display_ = boost::make_shared<boost::progress_display>(max_progress);
  }

  void printProgress(unsigned long progress) const override
  {
    std::lock_guard<std::mutex> lock{ mutex_ };
    if (progress > display_->count())
      *display_ += progress - display_->count();
  }

  void printResults(const StudyResults& results) const override
  {
    print(results.print());
  }

  void print(const std::string& message) const override
  {
    std::lock_guard<std::mutex> lock{ mutex_ };
    std::cout << message << std::endl;
  }

protected:
  mutable std::mutex mutex_;
  mutable boost::shared_ptr<boost::progress_display> display_;
};

struct BoostProgressConsoleLoggerFactory : public LoggerFactory
{
  Logger::ConstPtr create(const YAML::Node& /*config*/) const override
  {
    return std::make_shared<BoostProgressConsoleLogger>();
  }
};

}  // namespace reach

#include <reach_core/plugin_utils.h>
EXPORT_LOGGER_PLUGIN(reach::ConsoleLoggerFactory, ConsoleLogger)
EXPORT_LOGGER_PLUGIN(reach::BoostProgressConsoleLoggerFactory, BoostProgressConsoleLogger)
