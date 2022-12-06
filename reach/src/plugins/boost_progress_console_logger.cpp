#include <reach/interfaces/logger.h>
#include <reach/types.h>

#include <boost/progress.hpp>
#include <iostream>
#include <mutex>

namespace reach
{
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

  void printResults(const ReachResultSummary& results) const override
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
  Logger::Ptr create(const YAML::Node& /*config*/) const override
  {
    return std::make_shared<BoostProgressConsoleLogger>();
  }
};

}  // namespace reach

#include <reach/plugin_utils.h>
EXPORT_LOGGER_PLUGIN(reach::BoostProgressConsoleLoggerFactory, BoostProgressConsoleLogger)
