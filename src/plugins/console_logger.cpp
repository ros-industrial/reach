#include <reach/plugins/console_logger.h>
#include <reach/types.h>

#include <iostream>

namespace reach
{
void ConsoleLogger::setMaxProgress(unsigned long progress)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  max_progress_ = progress;
}

void ConsoleLogger::printProgress(unsigned long progress) const
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

void ConsoleLogger::printResults(const ReachResultSummary& results) const
{
  print(results.print());
}

void ConsoleLogger::print(const std::string& message) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  std::cout << message << std::endl;
}

Logger::Ptr ConsoleLoggerFactory::create(const YAML::Node& /*config*/) const
{
  return std::make_shared<ConsoleLogger>();
}

}  // namespace reach
