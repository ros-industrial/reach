#include <reach/interfaces/logger.h>
#include <reach/types.h>

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
  unsigned long max_progress_{ 0 };
};

struct ConsoleLoggerFactory : public LoggerFactory
{
  Logger::Ptr create(const YAML::Node& /*config*/) const override
  {
    return std::make_shared<ConsoleLogger>();
  }
};

}  // namespace reach

#include <reach/plugin_utils.h>
EXPORT_LOGGER_PLUGIN(reach::ConsoleLoggerFactory, ConsoleLogger)
